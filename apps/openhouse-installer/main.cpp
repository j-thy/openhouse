// openhouse-installer.exe: installs or removes openhouse in SteamVR on Windows. It
// replaces the Install-Driver.bat / openhouse-driver.ps1 pair: a .bat starting
// "powershell -ExecutionPolicy Bypass" is a common malware pattern, and Windows Defender
// flagged the package for it. Same steps as the script it replaces:
//   * finds SteamVR (openvrpaths.vrpath, else Steam's registry key; --steamvr PATH);
//   * offers to close Steam / SteamVR (they hold the driver DLL open);
//   * install: keeps SteamVR's driver as driver_lighthouse.dll.stock whenever the one in
//     place isn't openhouse's (a SteamVR update's newer one replaces an older backup),
//     copies ours over it, the settings page schema, and the update check (an app SteamVR
//     starts with every session: Steam's appconfig.json + vrappconfig);
//   * uninstall: puts the backup back, unless a SteamVR update already put its own
//     newer driver there (then the outdated backup goes instead), and removes the update
//     check's files from %LOCALAPPDATA%\openhouse, not the folder: recordings live there.
//
//   openhouse-installer [--install | --uninstall | --status] [--steamvr PATH] [--yes]
// Without an action it asks. Files are overwritten in place, retrying for a few seconds
// (a file a closed process just released can stay locked briefly).
#include <windows.h>
#include <tlhelp32.h>

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::ordered_json;

namespace {

constexpr const char* kMarker = "liblh session started";  // only openhouse's driver has it
constexpr const char* kCheckKey = "openhouse.check";
const wchar_t* kHosts[] = {L"steam.exe",       L"steamwebhelper.exe", L"vrserver.exe",  L"vrmonitor.exe",
                           L"vrcompositor.exe", L"vrdashboard.exe",    L"vrwebhelper.exe", L"vrstartup.exe",
                           L"vrservice.exe",   L"monado-service.exe"};

struct Failure : std::runtime_error {
  using std::runtime_error::runtime_error;
};

bool g_yes = false;

std::string Narrow(const std::wstring& w) {
  if (w.empty()) return {};
  const int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), int(w.size()), nullptr, 0, nullptr, nullptr);
  std::string s(size_t(n), '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.data(), int(w.size()), s.data(), n, nullptr, nullptr);
  return s;
}
std::string Show(const fs::path& p) { return Narrow(p.wstring()); }

fs::path EnvPath(const wchar_t* name) {
  wchar_t buffer[MAX_PATH * 2];
  const DWORD n = GetEnvironmentVariableW(name, buffer, DWORD(std::size(buffer)));
  return n && n < std::size(buffer) ? fs::path(buffer) : fs::path();
}

std::string ReadFile(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  // An unreadable driver read as "" looked like SteamVR's own, and uninstall deleted the
  // stock backup (#44): stop instead.
  if (!in) throw Failure("Could not read " + Show(p) + " (is SteamVR still running?)");
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void WriteText(const fs::path& p, const std::string& text) {  // UTF-8, no byte order mark
  fs::create_directories(p.parent_path());
  std::ofstream out(p, std::ios::binary | std::ios::trunc);
  out << text << "\n";
  if (!out) throw Failure("Could not write " + Show(p));
}

bool Ask(const std::string& question) {
  if (g_yes) return true;
  std::cout << question << " [y/N] " << std::flush;
  std::string answer;
  std::getline(std::cin, answer);
  return !answer.empty() && (answer[0] == 'y' || answer[0] == 'Y');
}

std::string SystemMessage(DWORD err) {
  wchar_t* text = nullptr;
  const DWORD n = FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                                 nullptr, err, 0, reinterpret_cast<wchar_t*>(&text), 0, nullptr);
  std::wstring w = n && text ? std::wstring(text, n) : std::wstring();
  if (text) LocalFree(text);
  while (!w.empty() && (w.back() == L'\r' || w.back() == L'\n' || w.back() == L' ' || w.back() == L'.')) w.pop_back();
  return Narrow(w) + " (error " + std::to_string(err) + ")";
}

// Overwrites `to`, retrying for ~5 s while it is locked. CopyFileW, not
// std::filesystem::copy_file: some MinGW libstdc++ builds (Ubuntu's GCC 13) ignore
// copy_options::overwrite_existing on Windows and fail with "File exists" whenever the
// target is there, which made every install over SteamVR's driver fail.
void CopyOver(const fs::path& from, const fs::path& to) {
  for (int i = 0;; ++i) {
    // A read-only target (e.g. a file copied from a read-only share) can't be replaced.
    const DWORD attrs = GetFileAttributesW(to.c_str());
    if (attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_READONLY))
      SetFileAttributesW(to.c_str(), attrs & ~DWORD(FILE_ATTRIBUTE_READONLY));
    if (CopyFileW(from.c_str(), to.c_str(), FALSE)) {
      const DWORD copied = GetFileAttributesW(to.c_str());
      if (copied != INVALID_FILE_ATTRIBUTES && (copied & FILE_ATTRIBUTE_READONLY))
        SetFileAttributesW(to.c_str(), copied & ~DWORD(FILE_ATTRIBUTE_READONLY));
      return;
    }
    const DWORD err = GetLastError();
    const bool locked = err == ERROR_SHARING_VIOLATION || err == ERROR_LOCK_VIOLATION;
    if (locked && i < 20) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      continue;
    }
    if (locked)
      throw Failure("Could not write " + Show(to) + " - a process still has it open. Close Steam and SteamVR completely, then run this again. (" +
                    SystemMessage(err) + ")");
    if (err == ERROR_ACCESS_DENIED)
      throw Failure("Could not write " + Show(to) + " - access denied. If SteamVR is installed in a protected folder, run this as administrator. (" +
                    SystemMessage(err) + ")");
    throw Failure("Could not copy " + Show(from) + " to " + Show(to) + ": " + SystemMessage(err));
  }
}

void RemoveFile(const fs::path& p) {
  std::error_code ec;
  fs::remove(p, ec);
}

// ---- finding SteamVR ----

ordered_json ReadJson(const fs::path& p) {
  try {
    const std::string text = ReadFile(p);
    if (text.empty()) return ordered_json::object();
    return ordered_json::parse(text);
  } catch (...) {
    return ordered_json::object();
  }
}

fs::path VrPathFile() { return EnvPath(L"LOCALAPPDATA") / "openvr" / "openvrpaths.vrpath"; }

fs::path FromUtf8(const std::string& s) {
  const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), nullptr, 0);
  std::wstring w(size_t(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), w.data(), n);
  return fs::path(w);
}

fs::path FindSteamVR() {
  const ordered_json paths = ReadJson(VrPathFile());
  if (paths.contains("runtime") && paths["runtime"].is_array())
    for (const auto& r : paths["runtime"])
      if (r.is_string() && fs::is_directory(FromUtf8(r.get<std::string>()) / "drivers" / "lighthouse" / "bin" / "win64"))
        return FromUtf8(r.get<std::string>());
  HKEY key;
  if (RegOpenKeyExW(HKEY_CURRENT_USER, L"Software\\Valve\\Steam", 0, KEY_READ, &key) == ERROR_SUCCESS) {
    wchar_t value[MAX_PATH * 2]{};
    DWORD size = sizeof value;
    const LONG r = RegQueryValueExW(key, L"SteamPath", nullptr, nullptr, reinterpret_cast<BYTE*>(value), &size);
    RegCloseKey(key);
    if (r == ERROR_SUCCESS) return fs::path(value) / "steamapps" / "common" / "SteamVR";
  }
  return {};
}

fs::path SteamConfig(const fs::path& steamvr) {
  const ordered_json paths = ReadJson(VrPathFile());
  if (paths.contains("config") && paths["config"].is_array() && !paths["config"].empty() && paths["config"][0].is_string())
    return FromUtf8(paths["config"][0].get<std::string>());
  return steamvr.parent_path().parent_path().parent_path() / "config";  // .../Steam/config
}

// ---- closing Steam / SteamVR ----

std::vector<std::pair<DWORD, std::wstring>> RunningHosts() {
  std::vector<std::pair<DWORD, std::wstring>> found;
  HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snap == INVALID_HANDLE_VALUE) return found;
  PROCESSENTRY32W e{};
  e.dwSize = sizeof e;
  for (BOOL ok = Process32FirstW(snap, &e); ok; ok = Process32NextW(snap, &e))
    for (const wchar_t* name : kHosts)
      if (_wcsicmp(e.szExeFile, name) == 0) found.emplace_back(e.th32ProcessID, e.szExeFile);
  CloseHandle(snap);
  return found;
}

void CloseHosts() {
  auto running = RunningHosts();
  if (running.empty()) return;
  std::set<std::string> names;
  for (const auto& [pid, name] : running) names.insert(Narrow(name));
  std::string list;
  for (const auto& n : names) list += (list.empty() ? "" : ", ") + n;
  std::cout << "\nSteam / SteamVR is running (" << list << "). It has the lighthouse driver open, so it\n"
               "must be closed before the driver can be changed.\n";
  if (!Ask("Close them now? Unsaved work in games is lost.")) throw Failure("Close Steam and SteamVR completely, then run this again.");
  for (const auto& [pid, name] : running)
    if (HANDLE h = OpenProcess(PROCESS_TERMINATE, FALSE, pid)) {
      TerminateProcess(h, 0);
      CloseHandle(h);
    }
  for (int i = 0; i < 50 && !RunningHosts().empty(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (!RunningHosts().empty()) throw Failure("Couldn't close Steam / SteamVR. Close them by hand (Task Manager), then run this again.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // file handles go a moment after the process
  std::cout << "Closed.\n";
}

// ---- the update check's registration ----

struct Paths {
  fs::path here, steamvr, directory, target, backup, schema, defaults, checkDir, checkManifest;
};

void SetCheckRegistration(const Paths& p, bool registerIt) {
  const fs::path config = SteamConfig(p.steamvr);
  const fs::path appconfig = config / "appconfig.json";
  ordered_json data = ordered_json::object();
  {
    // Steam's list of every registered VR app: one we can't read or parse is left alone.
    // Rewriting it from an empty object dropped all the others (#44).
    std::error_code ec;
    if (fs::exists(appconfig, ec)) {
      std::ifstream in(appconfig, std::ios::binary);
      const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      try {
        if (!in && !in.eof()) throw std::runtime_error("unreadable");
        if (!text.empty()) data = ordered_json::parse(text);
      } catch (const std::exception&) {
        std::cout << "Could not read " << Show(appconfig) << "; leaving it as it is (the update check isn't "
                  << (registerIt ? "registered" : "unregistered") << ").\n";
        return;
      }
      if (!data.is_object()) {
        std::cout << Show(appconfig) << " isn't what Steam writes; leaving it as it is.\n";
        return;
      }
    }
  }
  ordered_json kept = ordered_json::array();
  const std::string manifest = Show(p.checkManifest);
  if (data.contains("manifest_paths") && data["manifest_paths"].is_array())
    for (const auto& m : data["manifest_paths"])
      if (m.is_string() && fs::path(FromUtf8(m.get<std::string>())).lexically_normal() != p.checkManifest.lexically_normal())
        kept.push_back(m);
  if (registerIt) kept.push_back(manifest);
  data["manifest_paths"] = kept;
  WriteText(appconfig, data.dump(2));
  const fs::path app = config / "vrappconfig" / (std::string(kCheckKey) + ".vrappconfig");
  if (registerIt) WriteText(app, R"({ "autolaunch" : true, "last_launch_time" : "0" })");
  else RemoveFile(app);
}

// ---- the settings page schema ----

bool HasOurSection(const fs::path& schema) {
  std::error_code ec;
  return fs::exists(schema, ec) && ReadFile(schema).find("driver_openhouse") != std::string::npos;
}

void InstallSchema(const Paths& p) {
  const fs::path source = p.here / "settingsschema.vrsettings";
  if (!fs::exists(source)) return;
  fs::create_directories(p.schema.parent_path());
  if (fs::exists(p.schema) && !HasOurSection(p.schema)) CopyOver(p.schema, fs::path(p.schema) += ".stock");
  CopyOver(source, p.schema);
}

void RemoveSchema(const Paths& p) {
  const fs::path stock = fs::path(p.schema) += ".stock";
  if (fs::exists(p.schema) && HasOurSection(p.schema)) RemoveFile(p.schema);
  if (fs::exists(stock)) {
    // Still there: a SteamVR update replaced ours with a newer one, so the backup is older.
    if (fs::exists(p.schema)) RemoveFile(stock);
    else { CopyOver(stock, p.schema); RemoveFile(stock); }
  }
}

// ---- the settings page's defaults ----
// SteamVR's "reset to default" takes values from the driver's default.vrsettings, SteamVR's
// own file: without our section every toggle reset to off and every number to 0. One line
// is added after its opening brace and removed again on uninstall; the rest stays SteamVR's.
constexpr const char* kDefaultsLine =
    R"("driver_openhouse": { "predictWithAcceleration": true, "positionSmoothingMs": 60, "rotationSmoothingMs": 60, "hideLostDevices": false, "useSteamVRRooms": true, "record": false, "resetRoom": false, "mixedLight": false, "mixedLightControllers": true, "mixedLightTrackers": true, "mixedLightScout": true, "mixedLightPreferLh2": false },)";

void RemoveDefaults(const Paths& p);

void InstallDefaults(const Paths& p) {
  std::error_code ec;
  if (!fs::exists(p.defaults, ec)) return;
  if (ReadFile(p.defaults).find(kDefaultsLine) != std::string::npos) return;
  RemoveDefaults(p);  // an older version's line, with fewer settings
  std::string text = ReadFile(p.defaults);
  const size_t brace = text.find('{');
  if (brace == std::string::npos) return;
  text.insert(brace + 1, std::string("\n") + kDefaultsLine);
  WriteText(p.defaults, text);
}

void RemoveDefaults(const Paths& p) {
  std::error_code ec;
  if (!fs::exists(p.defaults, ec)) return;
  std::string text = ReadFile(p.defaults);
  // Any version's line: they all start the same and end at the line's end.
  const size_t at = text.find("\n\"driver_openhouse\": {");
  if (at == std::string::npos) return;
  size_t end = text.find('\n', at + 1);
  if (end == std::string::npos) end = text.size();
  else if (end > 0 && text[end - 1] == '\r') --end;  // keep the next line's break as it was
  text.erase(at, end - at);
  WriteText(p.defaults, text);
}

// ---- actions ----

void Install(const Paths& p) {
  const fs::path ours = p.here / "driver_lighthouse.dll";
  if (!fs::exists(ours)) throw Failure("The package's driver is missing: " + Show(ours));
  const bool isOurs = ReadFile(p.target).find(kMarker) != std::string::npos;
  if (!isOurs) {
    CopyOver(p.target, p.backup);
    std::cout << "Saved SteamVR's driver as " << Show(p.backup) << "\n";
  } else if (!fs::exists(p.backup)) {
    throw Failure("The installed driver is already openhouse's and there is no stock backup (" + Show(p.backup) +
                  "). Verify SteamVR's files in Steam, then install again.");
  }
  CopyOver(ours, p.target);
  if (ReadFile(p.target) != ReadFile(ours)) throw Failure("The installed driver doesn't match the package's (" + Show(p.target) + ")");
  InstallSchema(p);
  InstallDefaults(p);
  const fs::path checkExe = p.here / "apps" / "openhouse-check.exe";
  if (fs::exists(checkExe)) {
    fs::create_directories(p.checkDir);
    const fs::path installed = p.checkDir / "openhouse-check.exe";
    CopyOver(checkExe, installed);
    ordered_json app;
    app["app_key"] = kCheckKey;
    app["launch_type"] = "binary";
    app["binary_path_windows"] = Show(installed);
    app["arguments"] = "\"" + Show(p.directory) + "\"";
    app["is_dashboard_overlay"] = true;
    app["strings"]["en_us"] = {{"name", "openhouse update check"},
                               {"description", "Tells you when a SteamVR update replaced the openhouse driver"}};
    ordered_json manifest;
    manifest["source"] = "openhouse";
    manifest["applications"] = ordered_json::array({app});
    WriteText(p.checkManifest, manifest.dump(2));
    SetCheckRegistration(p, true);
  }
  std::cout << "Installed. Start SteamVR.\n";
}

void Uninstall(const Paths& p) {
  // First, so the check goes even when there's no driver backup.
  RemoveSchema(p);
  RemoveDefaults(p);
  SetCheckRegistration(p, false);
  // Only the files we put there: the folder is also the driver's data directory, with the
  // user's recordings (Record Session) in it.
  for (const char* name : {"openhouse-check.exe", "openhouse-check.vrmanifest"}) RemoveFile(p.checkDir / name);
  std::error_code ec;
  fs::remove(p.checkDir, ec);  // only goes when nothing else is in it
  if (ReadFile(p.target).find(kMarker) == std::string::npos) {
    // A SteamVR update puts its own, newer driver in place of ours. The backup is then the
    // older one: moving it back would undo the update, so it goes instead.
    RemoveFile(p.backup);
    std::cout << "SteamVR's own driver is already in place (a SteamVR update put it back). Your room and settings in "
                 "%APPDATA%\\lh are kept.\n";
    return;
  }
  if (!fs::exists(p.backup))
    throw Failure("SteamVR's driver backup isn't there (" + Show(p.backup) + "). Nothing to undo, or verify SteamVR's files in Steam.");
  CopyOver(p.backup, p.target);
  RemoveFile(p.backup);
  std::cout << "SteamVR's own driver is back. Your room and settings in %APPDATA%\\lh are kept.\n";
}

int Run(int argc, wchar_t** argv) {
  std::string action;
  fs::path steamvr;
  for (int i = 1; i < argc; ++i) {
    const std::wstring a = argv[i];
    if (a == L"--install" || a == L"install") action = "install";
    else if (a == L"--uninstall" || a == L"uninstall") action = "uninstall";
    else if (a == L"--status" || a == L"status") action = "status";
    else if (a == L"--yes" || a == L"-y") g_yes = true;
    else if (a == L"--steamvr" && i + 1 < argc) steamvr = argv[++i];
    else throw Failure("Unknown option " + Narrow(a) + ". Usage: openhouse-installer [--install | --uninstall | --status] [--steamvr PATH] [--yes]");
  }
  std::cout << "openhouse driver installer\n\n";
  if (action.empty()) {
    std::cout << "  1. Install (or update) openhouse\n  2. Uninstall openhouse (put SteamVR's driver back)\n\nChoose 1 or 2: " << std::flush;
    std::string choice;
    std::getline(std::cin, choice);
    if (choice == "1") action = "install";
    else if (choice == "2") action = "uninstall";
    else { std::cout << "Nothing changed.\n"; return 0; }
  }
  wchar_t self[MAX_PATH * 2]{};
  GetModuleFileNameW(nullptr, self, DWORD(std::size(self)));
  Paths p;
  p.here = fs::path(self).parent_path();
  p.steamvr = steamvr.empty() ? FindSteamVR() : steamvr;
  if (p.steamvr.empty()) throw Failure("SteamVR not found. Run this with --steamvr \"C:\\path\\to\\SteamVR\".");
  p.directory = p.steamvr / "drivers" / "lighthouse" / "bin" / "win64";
  p.target = p.directory / "driver_lighthouse.dll";
  p.backup = fs::path(p.target) += ".stock";
  p.schema = p.steamvr / "drivers" / "lighthouse" / "resources" / "settings" / "settingsschema.vrsettings";
  p.defaults = p.schema.parent_path() / "default.vrsettings";
  p.checkDir = EnvPath(L"LOCALAPPDATA") / "openhouse";
  p.checkManifest = p.checkDir / "openhouse-check.vrmanifest";
  if (!fs::exists(p.target)) throw Failure("Driver not found at " + Show(p.target) + ". Run this with --steamvr PATH.");
  if (action == "status") {
    std::cout << "Driver: " << Show(p.target) << "\n"
              << "openhouse installed: " << (fs::exists(p.target) && ReadFile(p.target).find(kMarker) != std::string::npos ? "yes" : "no") << "\n"
              << "Stock backup present: " << (fs::exists(p.backup) ? "yes" : "no") << "\n";
    return 0;
  }
  CloseHosts();
  if (action == "install") Install(p);
  else Uninstall(p);
  return 0;
}

}  // namespace

int wmain(int argc, wchar_t** argv) {
  SetConsoleOutputCP(CP_UTF8);
  int rc;
  try {
    rc = Run(argc, argv);
  } catch (const std::exception& e) {
    std::cout << "\nError: " << e.what() << "\n";
    rc = 1;
  }
  bool launchedByExplorer = false;  // double-clicked: keep the window open to be read
  {
    DWORD ids[2];
    launchedByExplorer = GetConsoleProcessList(ids, 2) <= 1;
  }
  if (launchedByExplorer && !g_yes) {
    std::cout << "\nPress Enter to close." << std::flush;
    std::string line;
    std::getline(std::cin, line);
  }
  return rc;
}
