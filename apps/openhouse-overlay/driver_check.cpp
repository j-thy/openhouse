#include "platform/platform.hpp"
#include "driver_check.hpp"

#include <lh/github.hpp>
#include <lh/version.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#else
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
extern char** environ;
#endif

namespace openhouse::overlay {
namespace fs = std::filesystem;
namespace {
#ifdef _WIN32
constexpr const char* kDriver = "driver_lighthouse.dll";
constexpr const char* kInstaller = "openhouse-installer.exe";
#else
constexpr const char* kDriver = "driver_lighthouse.so";
constexpr const char* kInstaller = "openhouse-installer";
#endif
// A string only openhouse's driver contains (the installers check the same one).
constexpr const char* kMarker = "liblh session started";
constexpr const char* kReleases = "https://github.com/YusufOzmen01/openhouse/releases/latest";
const char* kTitle = "openhouse";

std::mutex g_mutex;
NoticeState g_notice;

void Log(const std::string& line) {
  const fs::path dir = lh::platform::FsPath(lh::platform::DataDirectory());
#ifdef _WIN32
  const auto pid = GetCurrentProcessId();
#else
  const auto pid = getpid();
#endif
  std::error_code ec;
  fs::create_directories(dir, ec);
  const auto t = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof stamp, "%Y-%m-%d %H:%M:%S", std::localtime(&t));
  std::ofstream(dir / "check.log", std::ios::app) << stamp << " [" << pid << "] " << line << std::endl;
}

// "tag_name": "v1.0.1" out of the release JSON (no JSON library for one field).
std::string TagName(const std::string& json) {
  const auto key = json.find("\"tag_name\"");
  if (key == std::string::npos) return {};
  const auto open = json.find('"', json.find(':', key) + 1);
  if (open == std::string::npos) return {};
  const auto close = json.find('"', open + 1);
  return close == std::string::npos ? std::string() : json.substr(open + 1, close - open - 1);
}

fs::path ExeDirectory() {
#ifdef _WIN32
  wchar_t path[MAX_PATH * 4];
  const DWORD n = GetModuleFileNameW(nullptr, path, DWORD(std::size(path)));
  return fs::path(std::wstring(path, n)).parent_path();
#else
  std::error_code ec;
  return fs::read_symlink("/proc/self/exe", ec).parent_path();
#endif
}

// The installed copy next to the overlay (SteamVR's openhouse folder), or the package's (the
// overlay started from its apps folder).
fs::path FindInstaller() {
  const fs::path here = ExeDirectory();
  std::error_code ec;
  for (const fs::path& p : {here / "package" / kInstaller, here / kInstaller, here.parent_path() / kInstaller})
    if (fs::exists(p, ec)) return p;
  return {};
}

// openhouse-installer --update / --repair, on its own: it closes SteamVR (and with it this overlay).
bool StartInstaller(const std::string& mode, std::string* error) {
  fs::path installer = FindInstaller();
  if (installer.empty()) { *error = "Run the openhouse installer again"; return false; }
#ifdef _WIN32
  // Uninstall removes the saved package, which a running installer can't do to itself on Windows:
  // a copy in the temp folder uninstalls instead.
  if (mode == "--uninstall") {
    std::error_code ec;
    const fs::path dir = fs::temp_directory_path(ec) / "openhouse-uninstall";
    fs::create_directories(dir, ec);
    fs::copy_file(installer, dir / kInstaller, fs::copy_options::overwrite_existing, ec);
    if (!ec) installer = dir / kInstaller;
  }
#endif
  Log("starting " + lh::platform::PathUtf8(installer) + " " + mode);
#ifdef _WIN32
  std::wstring command = L"\"" + installer.wstring() + L"\" " + std::wstring(mode.begin(), mode.end());
  STARTUPINFOW si{sizeof si};
  PROCESS_INFORMATION pi{};
  // Out of SteamVR's job, if it has one: closing SteamVR mustn't take the installer with it.
  BOOL ok = CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE, CREATE_BREAKAWAY_FROM_JOB, nullptr,
                           installer.parent_path().c_str(), &si, &pi);
  if (!ok) ok = CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE, 0, nullptr, installer.parent_path().c_str(), &si, &pi);
  if (!ok) { *error = "Couldn't start the installer"; return false; }
  CloseHandle(pi.hThread);
  CloseHandle(pi.hProcess);
#else
  // The desktop's libraries, not SteamVR's runtime ones (vrenv.sh), and a session of its own so
  // closing SteamVR doesn't end it.
  std::vector<std::string> env;
  for (char** e = environ; *e; ++e) {
    const std::string kv = *e;
    if (kv.rfind("LD_LIBRARY_PATH=", 0) == 0 || kv.rfind("LD_PRELOAD=", 0) == 0) continue;
    env.push_back(kv);
  }
  if (const char* system = std::getenv("SYSTEM_LD_LIBRARY_PATH"); system && *system) env.push_back(std::string("LD_LIBRARY_PATH=") + system);
  std::vector<char*> envp;
  for (auto& kv : env) envp.push_back(kv.data());
  envp.push_back(nullptr);
  const std::string path = lh::platform::PathUtf8(installer);
  char* argv[] = {const_cast<char*>(path.c_str()), const_cast<char*>(mode.c_str()), nullptr};
  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);
  pid_t pid = 0;
  const int rc = posix_spawn(&pid, path.c_str(), nullptr, &attr, argv, envp.data());
  posix_spawnattr_destroy(&attr);
  if (rc != 0) { *error = "Couldn't start the installer"; return false; }
  std::thread([pid] { int status = 0; waitpid(pid, &status, 0); }).detach();
#endif
  return true;
}

void OpenUrl(const std::string& url) {
#ifdef _WIN32
  ShellExecuteA(nullptr, "open", url.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#else
  char* argv[] = {const_cast<char*>("xdg-open"), const_cast<char*>(url.c_str()), nullptr};
  pid_t pid = 0;
  if (posix_spawnp(&pid, "xdg-open", nullptr, nullptr, argv, environ) == 0)
    std::thread([pid] { int status = 0; waitpid(pid, &status, 0); }).detach();
#endif
}

bool g_underSteamVR = false;  // the overlay under SteamVR (else --check: Monado / WiVRn)

std::string DialogText(const NoticeState& n) {
  if (n.kind == NoticeKind::Replaced) return "A SteamVR update replaced the openhouse driver.";
  return "openhouse " + n.tag + " is available (you have v" + std::string(lh::kVersion) + ").";
}
std::string ConfirmText(const NoticeState& n, NoticeAction a) {
  const std::string question = a == NoticeAction::Uninstall ? "Uninstall openhouse?"
                               : n.kind == NoticeKind::Replaced ? "Fix openhouse?" : "Update openhouse to " + n.tag + "?";
  return question + "\n\nVR will shut down. These will be closed:\n" + ClosingText(g_underSteamVR) +
         (a == NoticeAction::Uninstall ? "\n\nYour room and settings are kept." : "");
}

// ---- the desktop dialogs: Update (Fix) / Ignore, then a confirmation ----
#ifdef _WIN32
// MessageBox has no custom buttons: they're renamed while it's being created.
thread_local HHOOK g_hook;
thread_local const wchar_t* g_first;
thread_local const wchar_t* g_second;   // the middle button (three-button boxes)
thread_local const wchar_t* g_cancel;
LRESULT CALLBACK RenameButtons(int code, WPARAM wParam, LPARAM lParam) {
  if (code == HCBT_ACTIVATE) {
    const HWND box = reinterpret_cast<HWND>(wParam);
    SetDlgItemTextW(box, g_second ? IDYES : IDOK, g_first);
    if (g_second) {
      SetDlgItemTextW(box, IDNO, g_second);
      // "Uninstall openhouse" doesn't fit a standard button: widen it, keeping its centre.
      if (HWND b = GetDlgItem(box, IDNO)) {
        RECT r;
        GetWindowRect(b, &r);
        MapWindowPoints(nullptr, box, reinterpret_cast<POINT*>(&r), 2);
        const int extra = (r.right - r.left) * 3 / 4;
        MoveWindow(b, r.left - extra / 2, r.top, r.right - r.left + extra, r.bottom - r.top, TRUE);
        for (const int id : {IDYES, IDCANCEL})
          if (HWND o = GetDlgItem(box, id)) {
            RECT q;
            GetWindowRect(o, &q);
            MapWindowPoints(nullptr, box, reinterpret_cast<POINT*>(&q), 2);
            MoveWindow(o, q.left + (id == IDYES ? -extra / 2 : extra / 2), q.top, q.right - q.left, q.bottom - q.top, TRUE);
          }
        RECT w;
        GetWindowRect(box, &w);
        MoveWindow(box, w.left - extra / 2, w.top, w.right - w.left + extra, w.bottom - w.top, TRUE);
        for (const int id : {IDYES, IDNO, IDCANCEL})
          if (HWND o = GetDlgItem(box, id)) {
            RECT q;
            GetWindowRect(o, &q);
            MapWindowPoints(nullptr, box, reinterpret_cast<POINT*>(&q), 2);
            MoveWindow(o, q.left + extra / 2, q.top, q.right - q.left, q.bottom - q.top, TRUE);
          }
      }
    }
    SetDlgItemTextW(box, IDCANCEL, g_cancel);
    UnhookWindowsHookEx(g_hook);
  }
  return CallNextHookEx(g_hook, code, wParam, lParam);
}
// 0: the first button, 1: the middle one (when given), 2: cancel. In front of everything, a warning
// icon, no sound (the icon is the box's own, which plays none).
int Ask(const std::string& text, const wchar_t* first, const wchar_t* second, const wchar_t* cancel) {
  g_first = first;
  g_second = second;
  g_cancel = cancel;
  g_hook = SetWindowsHookExW(WH_CBT, RenameButtons, nullptr, GetCurrentThreadId());
  MSGBOXPARAMSA box{sizeof box};
  box.lpszText = text.c_str();
  box.lpszCaption = kTitle;
  box.dwStyle = (second ? MB_YESNOCANCEL : MB_OKCANCEL) | MB_USERICON | MB_TOPMOST | MB_SETFOREGROUND;
  box.lpszIcon = MAKEINTRESOURCEA(32515);  // IDI_WARNING, from the system (hInstance null)
  const int r = MessageBoxIndirectA(&box);
  return r == IDOK || r == IDYES ? 0 : r == IDNO ? 1 : 2;
}
NoticeAction AskUser(const NoticeState& n) {
  if (n.kind != NoticeKind::Replaced) return Ask(DialogText(n), L"Update", nullptr, L"Ignore") == 0 ? NoticeAction::Update : NoticeAction::Ignore;
  const int r = Ask(DialogText(n), L"Fix", L"Uninstall openhouse", L"Ignore");
  return r == 0 ? NoticeAction::Update : r == 1 ? NoticeAction::Uninstall : NoticeAction::Ignore;
}
bool Confirm(const NoticeState& n, NoticeAction a) {
  return Ask(ConfirmText(n, a), a == NoticeAction::Uninstall ? L"Uninstall" : n.kind == NoticeKind::Replaced ? L"Fix" : L"Update",
             nullptr, L"Cancel") == 0;
}
#else
std::string Quoted(const std::string& s) {
  std::string q = "'";
  for (char c : s) q += c == '\'' ? std::string("'\\''") : std::string(1, c);
  return q + "'";
}
// The desktop's own dialog tools, not SteamVR's libraries: SteamVR starts apps with its runtime
// first in LD_LIBRARY_PATH (vrenv.sh), and its own steamvr_msg.sh runs zenity under
// SYSTEM_LD_LIBRARY_PATH. Set for the dialog only: the overlay keeps running with its own.
std::string DesktopEnv() {
  const char* system = std::getenv("SYSTEM_LD_LIBRARY_PATH");
  return std::string("env -u LD_PRELOAD ") +
         (system && *system ? "LD_LIBRARY_PATH=" + Quoted(system) + " " : std::string("-u LD_LIBRARY_PATH "));
}
bool Have(const char* tool) { return std::system((std::string("command -v ") + tool + " >/dev/null 2>&1").c_str()) == 0; }

// 0: the first button, 1: the middle one (when given), 2: cancel. zenity, else kdialog, else a
// notification (and 2).
int Ask(const std::string& text, const std::string& first, const std::string& second, const std::string& cancel) {
  const std::string t = Quoted(kTitle), m = Quoted(text), env = DesktopEnv();
  if (Have("zenity")) {
    const auto shownAt = std::chrono::steady_clock::now();
    // The OK button exits 0; cancel and Esc 1; the extra button exits 1 and prints its label.
    const std::string command = env + "zenity --question --icon=dialog-warning --width=440 --title=" + t + " --text=" + m +
                                " --ok-label=" + Quoted(first) + " --cancel-label=" + Quoted(cancel) +
                                (second.empty() ? "" : " --extra-button=" + Quoted(second)) + " 2>/dev/null";
    if (FILE* pipe = popen(command.c_str(), "r")) {
      char out[128]{};
      const size_t got = std::fread(out, 1, sizeof out - 1, pipe);
      out[got] = 0;
      const int rc = pclose(pipe);
      Log("zenity exited with " + std::to_string(rc));
      if (!second.empty() && std::strstr(out, second.c_str())) return 1;
      if (WIFEXITED(rc) && WEXITSTATUS(rc) == 0) return 0;
      // A quick exit 1: no display, the next tool's turn.
      if (std::chrono::steady_clock::now() - shownAt > std::chrono::seconds(1)) return 2;
    }
  }
  if (Have("kdialog")) {
    const auto shownAt = std::chrono::steady_clock::now();
    const std::string command = env + "kdialog --title " + t +
                                (second.empty() ? " --warningyesno " + m + " --yes-label " + Quoted(first) + " --no-label " + Quoted(cancel)
                                                : " --warningyesnocancel " + m + " --yes-label " + Quoted(first) + " --no-label " +
                                                      Quoted(second) + " --cancel-label " + Quoted(cancel)) +
                                " >/dev/null 2>&1";
    const int rc = std::system(command.c_str());
    Log("kdialog exited with " + std::to_string(rc));
    const int code = WIFEXITED(rc) ? WEXITSTATUS(rc) : -1;
    if (code == 0) return 0;
    if (code == 1 && !second.empty()) return 1;
    if (code == 1 || code == 2 || std::chrono::steady_clock::now() - shownAt > std::chrono::seconds(1)) return 2;
  }
  const std::string note = text + "\n" + kReleases;
  if (!Have("notify-send") || std::system((env + "notify-send -u critical " + t + " " + Quoted(note) + " >/dev/null 2>&1").c_str()) != 0)
    std::fprintf(stderr, "%s: %s\n", kTitle, note.c_str());
  Log("no dialog tool: notification only");
  return 2;
}
NoticeAction AskUser(const NoticeState& n) {
  if (n.kind != NoticeKind::Replaced) return Ask(DialogText(n), "Update", "", "Ignore") == 0 ? NoticeAction::Update : NoticeAction::Ignore;
  const int r = Ask(DialogText(n), "Fix", "Uninstall openhouse", "Ignore");
  return r == 0 ? NoticeAction::Update : r == 1 ? NoticeAction::Uninstall : NoticeAction::Ignore;
}
bool Confirm(const NoticeState& n, NoticeAction a) {
  return Ask(ConfirmText(n, a), a == NoticeAction::Uninstall ? "Uninstall" : n.kind == NoticeKind::Replaced ? "Fix" : "Update", "",
             "Cancel") == 0;
}
#endif

// Until Update/Fix (confirmed) or Ignore, here or in the overlay. Cancel on the confirmation asks
// the first question again.
void Dialog() {
  for (;;) {
    const NoticeState n = Notice();
    if (n.kind == NoticeKind::None || n.handled) return;
    const NoticeAction a = AskUser(n);
    if (a == NoticeAction::Ignore) { Log("dialog: ignore"); ActOnNotice(NoticeAction::Ignore); return; }
    if (Notice().handled) return;  // dealt with in the overlay meanwhile
    if (Confirm(n, a)) { Log(a == NoticeAction::Uninstall ? "dialog: uninstall confirmed" : "dialog: confirmed"); ActOnNotice(a); return; }
    Log("dialog: cancelled");
  }
}

// A newer release on GitHub than this build.
std::string NewerRelease() {
  if (std::getenv("OPENHOUSE_NO_UPDATE_CHECK")) { Log("update check off (OPENHOUSE_NO_UPDATE_CHECK)"); return {}; }
  // OPENHOUSE_TEST_NEWER=v9.9.9 (testing): as if that release were out.
  if (const char* test = std::getenv("OPENHOUSE_TEST_NEWER"); test && *test) { Log(std::string("test release ") + test); return test; }
  const std::string tag = TagName(lh::GithubGet("/repos/YusufOzmen01/openhouse/releases/latest"));
  if (tag.empty()) { Log("update check: no answer from GitHub"); return {}; }
  const lh::UpdateState state = lh::CheckRelease(tag, lh::GithubGet);
  Log("update check: latest " + tag + ", this is " + lh::kVersion +
      (state == lh::UpdateState::Newer ? ": newer" : state == lh::UpdateState::Unknown ? ": can't compare this build" : ": up to date"));
  return state == lh::UpdateState::Newer ? tag : std::string();
}

void Set(NoticeKind kind, const std::string& tag) {
  std::lock_guard lock(g_mutex);
  if (g_notice.kind != NoticeKind::None) return;  // the first one found stays
  g_notice.kind = kind;
  g_notice.tag = tag;
}
}  // namespace

NoticeState Notice() {
  std::lock_guard lock(g_mutex);
  return g_notice;
}

std::string NoticeText() {
  const NoticeState n = Notice();
  if (n.kind == NoticeKind::None || n.handled) return {};
  if (n.kind == NoticeKind::Replaced) return "A SteamVR update replaced the openhouse driver";
  return "openhouse " + n.tag + " is available";
}

void ActOnNotice(NoticeAction action) {
  NoticeState n = Notice();
  if (action == NoticeAction::Update || action == NoticeAction::Uninstall) {
    {
      std::lock_guard lock(g_mutex);
      if (g_notice.updating) return;  // the overlay and the dialog both pressed
      g_notice.updating = true;
    }
    const bool replaced = n.kind == NoticeKind::Replaced;
    // The changelog, while the installer does the rest.
    if (action == NoticeAction::Update && !replaced)
      OpenUrl(n.tag.empty() ? std::string(kReleases) : "https://github.com/YusufOzmen01/openhouse/releases/tag/" + n.tag);
    std::string error;
    const bool ok = StartInstaller(action == NoticeAction::Uninstall ? "--uninstall" : replaced ? "--repair" : "--update", &error);
    std::lock_guard lock(g_mutex);
    g_notice.updating = ok;
    g_notice.handled = ok;
    g_notice.error = ok ? std::string() : error;
    if (!ok) Log("update: " + error);
    return;
  }
  std::lock_guard lock(g_mutex);
  g_notice.handled = true;
}

std::string ClosingText(bool underSteamVR) {
#ifdef _WIN32
  (void)underSteamVR;
  return "- Steam\n- SteamVR";
#else
  return underSteamVR ? "- SteamVR" : "- Monado / WiVRn\n- SteamVR";
#endif
}

void OfferUpdate(const std::string& tag) {
  // OPENHOUSE_TEST_REPLACED (testing): as if the driver had been replaced.
  if (std::getenv("OPENHOUSE_TEST_REPLACED")) Set(NoticeKind::Replaced, {});
  Set(NoticeKind::Newer, tag);
}

std::string JoinedPath(int argc, char** argv, int first) {
  // SteamVR hands over a manifest's "arguments" with their quotes, and may split a path with
  // spaces: the pieces are joined and the quotes dropped.
  std::string folder;
  for (int i = first; i < argc; ++i) folder += (i > first ? " " : "") + std::string(argv[i]);
  if (folder.size() >= 2 && folder.front() == '"' && folder.back() == '"') folder = folder.substr(1, folder.size() - 2);
  return folder;
}

int RunDriverCheck(const std::string& folder, bool underSteamVR) {
  g_underSteamVR = underSteamVR;
  if (folder.empty()) { Log("no driver folder given"); return 2; }
  const fs::path driver = lh::platform::FsPath(folder) / kDriver;
  Log("checking " + lh::platform::PathUtf8(driver));
  std::error_code ec;
  if (!fs::exists(lh::platform::FsPath(lh::platform::PathUtf8(driver) + ".stock"), ec)) { Log("no .stock backup: nothing to do"); return 0; }
  std::ifstream in(driver, std::ios::binary);
  if (!in) { Log("can't read the driver"); return 0; }
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (bytes.find(kMarker) != std::string::npos) {
    Log("driver is openhouse's: fine");
    const std::string tag = NewerRelease();
    if (tag.empty()) return 0;
    Set(NoticeKind::Newer, tag);
    Dialog();
    return 0;
  }
  Log("driver was replaced: telling the user");
  Set(NoticeKind::Replaced, {});
  Dialog();
  Log("done");
  return 1;
}

}  // namespace openhouse::overlay
