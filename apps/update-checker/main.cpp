// update-checker: started by SteamVR with every session (an auto-launched app the
// installer registers). If a SteamVR update replaced our driver_lighthouse with its own,
// tells the user to run the openhouse installer again. Only while the installer's
// driver_lighthouse.*.stock backup exists: after uninstalling (or renaming it by hand)
// SteamVR's driver is wanted, and this stays quiet.
//
//   update-checker DRIVER_DIR     (SteamVR/drivers/lighthouse/bin/<platform>)
//
// Then (openhouse's driver in place) asks GitHub for the latest release and, when it's
// newer than this one, offers it: "Ok" or "Open link" to the release page. No network or
// no answer within a few seconds: nothing is shown.
//
// On Linux every run is logged to ~/.local/share/openhouse/check.log.
#include <lh/github.hpp>
#include <lh/version.hpp>

#include <array>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <winhttp.h>
#include <shellapi.h>
#else
#include <cstdio>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {
#ifdef _WIN32
constexpr const char* kDriver = "driver_lighthouse.dll";
#else
constexpr const char* kDriver = "driver_lighthouse.so";
#endif
// A string only openhouse's driver contains (the installers check the same one).
constexpr const char* kMarker = "liblh session started";

const char* kTitle = "openhouse";
const char* kText =
    "A SteamVR update replaced the openhouse driver.\n\n"
    "Close SteamVR and run the openhouse installer again.";

#ifdef _WIN32
void Log(const std::string&) {}
#else
void Log(const std::string& line) {
  const char* data = std::getenv("XDG_DATA_HOME");
  const char* home = std::getenv("HOME");
  if (!data && !home) return;
  const auto t = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof stamp, "%Y-%m-%d %H:%M:%S", std::localtime(&t));
  std::ofstream(std::filesystem::path(data ? data : std::string(home) + "/.local/share") / "openhouse/check.log",
                std::ios::app)
      << stamp << " [" << getpid() << "] " << line << std::endl;
}
std::string Env(const char* name) {
  const char* v = std::getenv(name);
  return v ? v : "(unset)";
}
#endif

constexpr const char* kReleases = "https://github.com/YusufOzmen01/openhouse/releases/latest";

// "tag_name": "v1.0.1" out of the release JSON (no JSON library for one field).
std::string TagName(const std::string& json) {
  const auto key = json.find("\"tag_name\"");
  if (key == std::string::npos) return {};
  const auto open = json.find('"', json.find(':', key) + 1);
  if (open == std::string::npos) return {};
  const auto close = json.find('"', open + 1);
  return close == std::string::npos ? std::string() : json.substr(open + 1, close - open - 1);
}

#ifdef _WIN32
// MessageBox has no custom buttons: an OK/Cancel box whose Cancel is renamed while it
// is being created.
HHOOK g_hook;
LRESULT CALLBACK RenameButtons(int code, WPARAM wParam, LPARAM lParam) {
  if (code == HCBT_ACTIVATE) {
    const HWND box = reinterpret_cast<HWND>(wParam);
    SetDlgItemTextW(box, IDOK, L"Ok");
    SetDlgItemTextW(box, IDCANCEL, L"Open link");
    UnhookWindowsHookEx(g_hook);
  }
  return CallNextHookEx(g_hook, code, wParam, lParam);
}

void ShowUpdate(const std::string& text) {
  g_hook = SetWindowsHookExW(WH_CBT, RenameButtons, nullptr, GetCurrentThreadId());
  const int choice = MessageBoxA(nullptr, text.c_str(), kTitle, MB_OKCANCEL | MB_ICONWARNING | MB_TOPMOST | MB_SETFOREGROUND);
  if (choice == IDCANCEL) ShellExecuteA(nullptr, "open", kReleases, nullptr, nullptr, SW_SHOWNORMAL);
}
#endif

void Show() {
#ifdef _WIN32
  MessageBoxA(nullptr, kText, kTitle, MB_OK | MB_ICONWARNING | MB_TOPMOST | MB_SETFOREGROUND);
#else
  Log("DISPLAY=" + Env("DISPLAY") + " WAYLAND_DISPLAY=" + Env("WAYLAND_DISPLAY") +
      " DBUS_SESSION_BUS_ADDRESS=" + Env("DBUS_SESSION_BUS_ADDRESS") + " LD_LIBRARY_PATH=" + Env("LD_LIBRARY_PATH"));
  // SteamVR starts apps with its own libraries first in LD_LIBRARY_PATH (vrenv.sh). Its
  // own steamvr_msg.sh runs the desktop's zenity under SYSTEM_LD_LIBRARY_PATH; so do we.
  if (const char* system = std::getenv("SYSTEM_LD_LIBRARY_PATH"); system && *system) setenv("LD_LIBRARY_PATH", system, 1);
  else unsetenv("LD_LIBRARY_PATH");
  unsetenv("LD_PRELOAD");
  // Whatever dialog tool the desktop has; a notification at least; stderr at last.
  auto quoted = [](const std::string& s) {
    std::string q = "'";
    for (char c : s) q += c == '\'' ? std::string("'\\''") : std::string(1, c);
    return q + "'";
  };
  const std::string t = quoted(kTitle), m = quoted(kText);
  for (const std::string& command : {"zenity --warning --width=420 --title=" + t + " --text=" + m,
                                     "kdialog --title " + t + " --sorry " + m,
                                     "notify-send -u critical " + t + " " + m}) {
    const std::string tool = command.substr(0, command.find(' '));
    if (std::system(("command -v " + tool + " >/dev/null 2>&1").c_str()) != 0) continue;
    Log("showing with " + tool);
    const auto shownAt = std::chrono::steady_clock::now();
    const int rc = std::system((command + " >/dev/null 2>&1").c_str());
    Log(tool + " exited with " + std::to_string(rc));
    // Shown: it exited cleanly, or stayed up a while. zenity exits 1 both when dismissed with
    // Esc and when it can't open a display; only the second is quick, and only then does the
    // next tool get a turn (closing the warning showed it again).
    if (rc == 0 || std::chrono::steady_clock::now() - shownAt > std::chrono::seconds(1)) return;
  }
  std::fprintf(stderr, "%s: %s\n", kTitle, kText);
#endif
}

#ifndef _WIN32
// The same desktop dialog tools as Show(); "Open link" opens the release page.
std::string Quoted(const std::string& s) {
  std::string q = "'";
  for (char c : s) q += c == '\'' ? std::string("'\\''") : std::string(1, c);
  return q + "'";
}
void ShowUpdate(const std::string& text) {
  if (const char* system = std::getenv("SYSTEM_LD_LIBRARY_PATH"); system && *system) setenv("LD_LIBRARY_PATH", system, 1);
  else unsetenv("LD_LIBRARY_PATH");
  unsetenv("LD_PRELOAD");
  const std::string t = Quoted(kTitle), m = Quoted(text);
  bool open = false, shown = false;
  if (std::system("command -v zenity >/dev/null 2>&1") == 0) {
    // OK returns 0; the extra button returns 1 and prints its label.
    FILE* pipe = popen(("zenity --warning --width=420 --title=" + t + " --text=" + m +
                        " --ok-label=Ok --extra-button='Open link' 2>/dev/null").c_str(), "r");
    if (pipe) {
      char out[64]{};
      const size_t n = std::fread(out, 1, sizeof out - 1, pipe);
      out[n] = 0;
      pclose(pipe);
      open = std::strstr(out, "Open link") != nullptr;
      shown = true;
    }
  } else if (std::system("command -v kdialog >/dev/null 2>&1") == 0) {
    const int rc = std::system(("kdialog --title " + t + " --warningyesno " + m +
                                " --yes-label Ok --no-label 'Open link' >/dev/null 2>&1").c_str());
    open = WIFEXITED(rc) && WEXITSTATUS(rc) == 1;
    shown = true;
  } else if (std::system("command -v notify-send >/dev/null 2>&1") == 0) {
    shown = std::system(("notify-send " + t + " " + Quoted(text + "\n" + kReleases) + " >/dev/null 2>&1").c_str()) == 0;
  }
  if (!shown) std::fprintf(stderr, "%s: %s\n%s\n", kTitle, text.c_str(), kReleases);
  Log(std::string("update dialog: ") + (open ? "open link" : "ok"));
  if (open) std::system((std::string("xdg-open ") + kReleases + " >/dev/null 2>&1 &").c_str());
}
#endif

// A newer release on GitHub than this build: tell the user.
void CheckForUpdate() {
  if (std::getenv("OPENHOUSE_NO_UPDATE_CHECK")) { Log("update check off (OPENHOUSE_NO_UPDATE_CHECK)"); return; }
  const std::string tag = TagName(lh::GithubGet("/repos/YusufOzmen01/openhouse/releases/latest"));
  if (tag.empty()) { Log("update check: no answer from GitHub"); return; }
  const lh::UpdateState state = lh::CheckRelease(tag, lh::GithubGet);
  Log("update check: latest " + tag + ", this is " + lh::kVersion +
      (state == lh::UpdateState::Newer ? ": newer" : state == lh::UpdateState::Unknown ? ": can't compare this build" : ": up to date"));
  if (state != lh::UpdateState::Newer) return;
  ShowUpdate("openhouse " + tag + " is available (you have v" + std::string(lh::kVersion) +
             ")\n\nClose SteamVR before installing it.");
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) { Log("no driver folder given"); return 2; }
  // SteamVR hands over the manifest's "arguments" with their quotes, and may split a path
  // with spaces: join the pieces and drop the quotes.
  std::string folder = argv[1];
  for (int i = 2; i < argc; ++i) folder += std::string(" ") + argv[i];
  if (folder.size() >= 2 && folder.front() == '"' && folder.back() == '"') folder = folder.substr(1, folder.size() - 2);
  const std::filesystem::path driver = std::filesystem::path(folder) / kDriver;
  Log("checking " + driver.string());
  std::error_code ec;
  if (!std::filesystem::exists(driver.string() + ".stock", ec)) { Log("no .stock backup: nothing to do"); return 0; }
  std::ifstream in(driver, std::ios::binary);
  if (!in) { Log("can't read the driver"); return 0; }
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (bytes.find(kMarker) != std::string::npos) {
    Log("driver is openhouse's: fine");
    CheckForUpdate();
    return 0;
  }
  Log("driver was replaced: telling the user");
  Show();
  Log("done");
  return 1;
}
