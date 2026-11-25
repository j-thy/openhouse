#include "system.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shlobj.h>
#include <shellapi.h>
#include <tlhelp32.h>
#else
#include <dirent.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

namespace openhouse::installer {

fs::path Path(const std::string& utf8) {
  return fs::path(std::u8string(reinterpret_cast<const char8_t*>(utf8.data()), reinterpret_cast<const char8_t*>(utf8.data()) + utf8.size()));
}
std::string Utf8(const fs::path& p) {
  const std::u8string s = p.u8string();
  return std::string(reinterpret_cast<const char*>(s.data()), s.size());
}

namespace {
#ifdef _WIN32
std::wstring Wide(const std::string& s) {
  if (s.empty()) return {};
  const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), nullptr, 0);
  std::wstring w(size_t(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), w.data(), n);
  return w;
}
fs::path EnvPath(const wchar_t* name) {
  wchar_t buffer[MAX_PATH * 2];
  const DWORD n = GetEnvironmentVariableW(name, buffer, DWORD(std::size(buffer)));
  return n && n < std::size(buffer) ? fs::path(buffer) : fs::path();
}
#else
fs::path EnvPath(const char* name) {
  const char* v = std::getenv(name);
  return v && *v ? Path(v) : fs::path();
}
#endif
}  // namespace

fs::path HomeDirectory() {
#ifdef _WIN32
  return EnvPath(L"USERPROFILE");
#else
  return EnvPath("HOME");
#endif
}

fs::path ExeDirectory() {
#ifdef _WIN32
  wchar_t self[MAX_PATH * 2]{};
  GetModuleFileNameW(nullptr, self, DWORD(std::size(self)));
  return fs::path(self).parent_path();
#else
  std::error_code ec;
  const fs::path self = fs::read_symlink("/proc/self/exe", ec);
  return ec ? fs::current_path() : self.parent_path();
#endif
}

fs::path DataDirectory() {
#ifdef _WIN32
  return EnvPath(L"LOCALAPPDATA") / "openhouse";
#else
  const fs::path xdg = EnvPath("XDG_DATA_HOME");
  return (xdg.empty() ? HomeDirectory() / ".local" / "share" : xdg) / "openhouse";
#endif
}

fs::path ConfigDirectory() {
#ifdef _WIN32
  return EnvPath(L"APPDATA") / "lh";
#else
  const fs::path xdg = EnvPath("XDG_CONFIG_HOME");
  return (xdg.empty() ? HomeDirectory() / ".config" : xdg) / "lh";
#endif
}

bool ReadFile(const fs::path& p, std::string* out) {
  std::ifstream in(p, std::ios::binary);
  if (!in) return false;
  out->assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  return !in.bad();
}

bool WriteFileAtomic(const fs::path& p, const std::string& data) {
  std::error_code ec;
  fs::create_directories(p.parent_path(), ec);
  fs::path temp = p;
  temp += ".new";
  {
    std::ofstream out(temp, std::ios::binary | std::ios::trunc);
    out << data;
    out.close();
    if (!out) return false;
  }
#ifdef _WIN32
  // Steam or a virus scanner can hold the file for a moment: tried for a few seconds.
  for (int i = 0; i < 20; ++i) {
    if (MoveFileExW(temp.c_str(), p.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  fs::remove(temp, ec);
  return false;
#else
  fs::rename(temp, p, ec);
  return !ec;
#endif
}

void CopyOver(const fs::path& from, const fs::path& to) {
#ifdef _WIN32
  // CopyFileW, not std::filesystem::copy_file: some MinGW builds ignore overwrite_existing.
  for (int i = 0;; ++i) {
    const DWORD attrs = GetFileAttributesW(to.c_str());
    if (attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_READONLY))
      SetFileAttributesW(to.c_str(), attrs & ~DWORD(FILE_ATTRIBUTE_READONLY));
    if (CopyFileW(from.c_str(), to.c_str(), FALSE)) return;
    const DWORD err = GetLastError();
    // ERROR_USER_MAPPED_FILE: a program has the DLL loaded.
    const bool locked = err == ERROR_SHARING_VIOLATION || err == ERROR_LOCK_VIOLATION || err == ERROR_USER_MAPPED_FILE;
    if (locked && i < 20) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      continue;
    }
    // Still held: a loaded DLL can't be written over, but it can be renamed. The old copy is moved
    // aside (deleted at the next restart, or by the next install), and the new one goes in its place.
    if (locked) {
      fs::path aside = to;
      aside += ".old-" + std::to_string(GetTickCount64());
      if (MoveFileExW(to.c_str(), aside.c_str(), 0)) {
        if (CopyFileW(from.c_str(), to.c_str(), FALSE)) {
          if (!DeleteFileW(aside.c_str())) MoveFileExW(aside.c_str(), nullptr, MOVEFILE_DELAY_UNTIL_REBOOT);
          return;
        }
        MoveFileExW(aside.c_str(), to.c_str(), 0);   // put the old one back
      }
    }
    if (locked) throw std::runtime_error("Couldn't write " + Utf8(to) + ": a program still has it open. Close Steam and SteamVR completely, then try again.");
    if (err == ERROR_ACCESS_DENIED) throw std::runtime_error("Couldn't write " + Utf8(to) + ": access denied. Run the installer as administrator.");
    throw std::runtime_error("Couldn't copy " + Utf8(from) + " to " + Utf8(to) + " (error " + std::to_string(err) + ")");
  }
#else
  // A new file renamed over the old one, never written over: a running process (Steam loads
  // the driver too) keeps its old copy, where an overwritten one crashed it.
  std::error_code ec;
  fs::path temp = to;
  temp += ".new";
  fs::remove(temp, ec);
  if (fs::is_symlink(from)) {
    fs::create_symlink(fs::read_symlink(from), temp, ec);
  } else {
    fs::copy_file(from, temp, fs::copy_options::overwrite_existing, ec);
    if (!ec) fs::permissions(temp, fs::status(from).permissions(), ec);
  }
  if (!ec) fs::rename(temp, to, ec);
  if (ec) throw std::runtime_error("Couldn't copy " + Utf8(from) + " to " + Utf8(to) + ": " + ec.message());
#endif
}

void RemoveFile(const fs::path& p) {
  std::error_code ec;
  fs::remove(p, ec);
}

namespace {
fs::path SettingsFile() { return ConfigDirectory() / "installer.json"; }
nlohmann::json LoadSettings() {
  std::string text;
  if (!ReadFile(SettingsFile(), &text)) return nlohmann::json::object();
  nlohmann::json j = nlohmann::json::parse(text, nullptr, false);
  return j.is_object() ? j : nlohmann::json::object();
}
}  // namespace

std::string LoadSetting(const std::string& key) {
  const nlohmann::json j = LoadSettings();
  const auto it = j.find(key);
  return it != j.end() && it->is_string() ? it->get<std::string>() : std::string();
}

void SaveSetting(const std::string& key, const std::string& value) {
  nlohmann::json j = LoadSettings();
  j[key] = value;
  WriteFileAtomic(SettingsFile(), j.dump(2) + "\n");
}

// ---- processes ----

namespace {
#ifdef _WIN32
const wchar_t* kHosts[] = {L"steam.exe", L"steamwebhelper.exe", L"vrserver.exe", L"vrmonitor.exe", L"vrcompositor.exe",
                           L"vrdashboard.exe", L"vrwebhelper.exe", L"vrstartup.exe", L"vrservice.exe", L"monado-service.exe",
                           L"openhouse-overlay.exe", L"update-checker.exe", L"openhouse-check.exe"};
std::vector<std::pair<DWORD, std::string>> Hosts() {
  std::vector<std::pair<DWORD, std::string>> found;
  HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snap == INVALID_HANDLE_VALUE) return found;
  PROCESSENTRY32W e{};
  e.dwSize = sizeof e;
  for (BOOL ok = Process32FirstW(snap, &e); ok; ok = Process32NextW(snap, &e))
    for (const wchar_t* name : kHosts)
      if (_wcsicmp(e.szExeFile, name) == 0) found.emplace_back(e.th32ProcessID, Utf8(fs::path(e.szExeFile)));
  CloseHandle(snap);
  return found;
}
#else
const char* kHosts[] = {"vrserver", "vrmonitor", "vrcompositor", "vrdashboard", "vrwebhelper", "vrstartup", "monado-service", "wivrn-server"};
std::vector<std::pair<int, std::string>> Hosts() {
  std::vector<std::pair<int, std::string>> found;
  DIR* d = opendir("/proc");
  if (!d) return found;
  while (dirent* e = readdir(d)) {
    const int pid = std::atoi(e->d_name);
    if (pid <= 0) continue;
    std::string comm;
    if (!ReadFile(fs::path("/proc") / e->d_name / "comm", &comm)) continue;
    while (!comm.empty() && (comm.back() == '\n' || comm.back() == ' ')) comm.pop_back();
    for (const char* name : kHosts)
      if (comm == name) found.emplace_back(pid, comm);
  }
  closedir(d);
  return found;
}
#endif
}  // namespace

std::vector<std::string> RunningRuntimes() {
  std::vector<std::string> names;
  for (const auto& [pid, name] : Hosts())
    if (std::find(names.begin(), names.end(), name) == names.end()) names.push_back(name);
  return names;
}

bool CloseRuntimes() {
  for (const auto& [pid, name] : Hosts()) {
#ifdef _WIN32
    if (HANDLE h = OpenProcess(PROCESS_TERMINATE, FALSE, pid)) {
      TerminateProcess(h, 0);
      CloseHandle(h);
    }
#else
    kill(pid, SIGTERM);
#endif
  }
  for (int i = 0; i < 50 && !Hosts().empty(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(200));
#ifndef _WIN32
  for (const auto& [pid, name] : Hosts()) kill(pid, SIGKILL);
  for (int i = 0; i < 10 && !Hosts().empty(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(200));
#endif
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // file handles go a moment after the process
  return Hosts().empty();
}

int RunProcess(const std::vector<std::string>& argv, const std::function<void(const std::string&)>& line) {
  if (argv.empty()) return -1;
  std::string pending;
  auto feed = [&](const char* data, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      const char c = data[i];
      if (c == '\n' || c == '\r') {
        if (!pending.empty() && line) line(pending);
        pending.clear();
      } else {
        pending += c;
      }
    }
  };
#ifdef _WIN32
  std::wstring command;
  for (const auto& a : argv) {
    std::wstring w = Wide(a);
    command += (command.empty() ? L"" : L" ");
    command += L"\"";
    for (wchar_t c : w) { if (c == L'"') command += L'\\'; command += c; }
    command += L"\"";
  }
  SECURITY_ATTRIBUTES sa{sizeof sa, nullptr, TRUE};
  HANDLE readEnd = nullptr, writeEnd = nullptr;
  if (!CreatePipe(&readEnd, &writeEnd, &sa, 0)) return -1;
  SetHandleInformation(readEnd, HANDLE_FLAG_INHERIT, 0);
  STARTUPINFOW si{};
  si.cb = sizeof si;
  si.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
  si.wShowWindow = SW_HIDE;
  si.hStdOutput = si.hStdError = writeEnd;
  si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  PROCESS_INFORMATION pi{};
  const std::wstring dir = Wide(Utf8(Path(argv[0]).parent_path()));
  if (!CreateProcessW(nullptr, command.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                      dir.empty() ? nullptr : dir.c_str(), &si, &pi)) {
    CloseHandle(readEnd);
    CloseHandle(writeEnd);
    return -1;
  }
  CloseHandle(writeEnd);
  char buffer[512];
  DWORD n = 0;
  while (::ReadFile(readEnd, buffer, sizeof buffer, &n, nullptr) && n) feed(buffer, n);
  CloseHandle(readEnd);
  WaitForSingleObject(pi.hProcess, INFINITE);
  DWORD code = 1;
  GetExitCodeProcess(pi.hProcess, &code);
  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);
#else
  int fds[2];
  if (pipe(fds) != 0) return -1;
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, fds[1], 1);
  posix_spawn_file_actions_adddup2(&actions, fds[1], 2);
  posix_spawn_file_actions_addclose(&actions, fds[0]);
  std::vector<char*> args;
  for (const auto& a : argv) args.push_back(const_cast<char*>(a.c_str()));
  args.push_back(nullptr);
  pid_t pid = 0;
  const int rc = posix_spawnp(&pid, args[0], &actions, nullptr, args.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  close(fds[1]);
  if (rc != 0) { close(fds[0]); return -1; }
  char buffer[512];
  ssize_t n;
  while ((n = read(fds[0], buffer, sizeof buffer)) > 0) feed(buffer, size_t(n));
  close(fds[0]);
  int status = 0;
  waitpid(pid, &status, 0);
  const int code = WIFEXITED(status) ? WEXITSTATUS(status) : 1;
#endif
  if (!pending.empty() && line) line(pending);
  return int(code);
}

bool HaveProgram(const char* name) {
#ifdef _WIN32
  (void)name;
  return false;
#else
  const char* path = std::getenv("PATH");
  if (!path) return false;
  std::string p = path;
  size_t start = 0;
  while (start <= p.size()) {
    const size_t end = p.find(':', start);
    const std::string dir = p.substr(start, end == std::string::npos ? std::string::npos : end - start);
    if (!dir.empty() && access((dir + "/" + name).c_str(), X_OK) == 0) return true;
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return false;
#endif
}

std::string PickFolder(const std::string& title) {
#ifdef _WIN32
  CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
  const std::wstring wtitle = Wide(title);
  BROWSEINFOW bi{};
  bi.lpszTitle = wtitle.c_str();
  bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
  PIDLIST_ABSOLUTE item = SHBrowseForFolderW(&bi);
  if (!item) return {};
  wchar_t path[MAX_PATH * 2]{};
  const bool ok = SHGetPathFromIDListW(item, path);
  CoTaskMemFree(item);
  return ok ? Utf8(fs::path(path)) : std::string();
#else
  std::string chosen;
  auto run = [&](std::vector<std::string> argv) {
    chosen.clear();
    const int rc = RunProcess(argv, [&](const std::string& l) { if (chosen.empty() && !l.empty() && l[0] == '/') chosen = l; });
    return rc == 0 && !chosen.empty();
  };
  if (HaveProgram("zenity") && run({"zenity", "--file-selection", "--directory", "--title=" + title})) return chosen;
  if (HaveProgram("kdialog") && run({"kdialog", "--getexistingdirectory", HomeDirectory().string(), "--title", title})) return chosen;
  return {};
#endif
}

void StartApp(const fs::path& exe) {
#ifdef _WIN32
  ShellExecuteW(nullptr, L"open", exe.c_str(), nullptr, exe.parent_path().c_str(), SW_SHOWNORMAL);
#else
  std::thread([exe] { RunProcess({exe.string()}, nullptr); }).detach();
#endif
}

void OpenInFileManager(const fs::path& p) {
#ifdef _WIN32
  ShellExecuteW(nullptr, L"open", p.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#else
  std::thread([p] { RunProcess({"xdg-open", p.string()}, nullptr); }).detach();
#endif
}

}  // namespace openhouse::installer
