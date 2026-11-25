#include "device_type.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace openhouse::installer {
namespace {

using json = nlohmann::json;
constexpr uint16_t kWandPid = 0x2012;  // the original Vive controller

std::string Lower(std::string s) {
  for (auto& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

bool Contains(const json& j, const char* key, const char* what) {
  return j.contains(key) && j.at(key).is_string() && Lower(j.at(key).get<std::string>()).find(what) != std::string::npos;
}

std::string Timestamp() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char text[32];
  std::strftime(text, sizeof text, "%Y%m%d-%H%M%S", &tm);
  return text;
}

// SteamVR's lighthouse_console for one device, driven line by line: it reads a command only
// after it has printed its "lh> " prompt, so each command waits for the prompt first.
class Console {
 public:
  ~Console() { Stop(); }

  bool Start(const fs::path& exe, const fs::path& workDir, const std::string& serial, std::string* error) {
#ifdef _WIN32
    SECURITY_ATTRIBUTES sa{sizeof sa, nullptr, TRUE};
    HANDLE inRead = nullptr, outWrite = nullptr;
    if (!CreatePipe(&inRead, &in_, &sa, 0) || !CreatePipe(&out_, &outWrite, &sa, 0)) {
      *error = "couldn't create pipes for lighthouse_console";
      return false;
    }
    SetHandleInformation(in_, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(out_, HANDLE_FLAG_INHERIT, 0);
    STARTUPINFOW si{};
    si.cb = sizeof si;
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = inRead;
    si.hStdOutput = si.hStdError = outWrite;
    std::wstring cmd = L"\"" + exe.wstring() + L"\" /serial " + Path(serial).wstring();
    PROCESS_INFORMATION pi{};
    const BOOL ok = CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                                   workDir.wstring().c_str(), &si, &pi);
    CloseHandle(inRead);
    CloseHandle(outWrite);
    if (!ok) {
      *error = "couldn't start SteamVR's lighthouse_console";
      return false;
    }
    CloseHandle(pi.hThread);
    process_ = pi.hProcess;
#else
    int in[2], out[2];
    if (pipe(in) || pipe(out)) { *error = "couldn't create pipes for lighthouse_console"; return false; }
    pid_ = fork();
    if (pid_ == 0) {
      dup2(in[0], 0);
      dup2(out[1], 1);
      dup2(out[1], 2);
      close(in[1]);
      close(out[0]);
      if (chdir(workDir.c_str()) != 0) _exit(127);
      const std::string path = exe.string();
      execl(path.c_str(), path.c_str(), "/serial", serial.c_str(), static_cast<char*>(nullptr));
      _exit(127);
    }
    close(in[0]);
    close(out[1]);
    if (pid_ < 0) { *error = "couldn't start SteamVR's lighthouse_console"; return false; }
    in_ = in[1];
    out_ = out[0];
    fcntl(out_, F_SETFL, fcntl(out_, F_GETFL) | O_NONBLOCK);
#endif
    std::string banner;
    if (!WaitPrompt(&banner, 20)) {
      *error = "lighthouse_console didn't open " + serial + LastLine(banner);
      return false;
    }
    return true;
  }

  // Sends one command; `output` is everything it printed before the next prompt.
  bool Command(const std::string& command, std::string* output, double seconds) {
    const std::string line = command + "\n";
#ifdef _WIN32
    DWORD written = 0;
    if (!::WriteFile(in_, line.data(), DWORD(line.size()), &written, nullptr)) return false;
#else
    if (write(in_, line.data(), line.size()) != ssize_t(line.size())) return false;
#endif
    return WaitPrompt(output, seconds);
  }

  void Stop() {
#ifdef _WIN32
    if (process_) {
      DWORD written = 0;
      ::WriteFile(in_, "exit\n", 5, &written, nullptr);
      if (WaitForSingleObject(process_, 3000) != WAIT_OBJECT_0) TerminateProcess(process_, 1);
      CloseHandle(process_);
      process_ = nullptr;
    }
    if (in_) { CloseHandle(in_); in_ = nullptr; }
    if (out_) { CloseHandle(out_); out_ = nullptr; }
#else
    if (pid_ > 0) {
      if (write(in_, "exit\n", 5) < 0) {}
      for (int i = 0; i < 30 && waitpid(pid_, nullptr, WNOHANG) == 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (waitpid(pid_, nullptr, WNOHANG) == 0) { kill(pid_, SIGKILL); waitpid(pid_, nullptr, 0); }
      pid_ = -1;
    }
    if (in_ >= 0) { close(in_); in_ = -1; }
    if (out_ >= 0) { close(out_); out_ = -1; }
#endif
  }

  static std::string LastLine(const std::string& text) {
    std::string t = text;
    while (!t.empty() && (t.back() == '\n' || t.back() == '\r' || t.back() == ' ')) t.pop_back();
    const size_t nl = t.find_last_of("\r\n");
    const std::string last = nl == std::string::npos ? t : t.substr(nl + 1);
    return last.empty() ? "" : " (" + last + ")";
  }

 private:
  bool WaitPrompt(std::string* out, double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    char buf[4096];
    while (std::chrono::steady_clock::now() < end) {
#ifdef _WIN32
      DWORD avail = 0;
      if (!PeekNamedPipe(out_, nullptr, 0, nullptr, &avail, nullptr)) return false;  // it exited
      if (avail) {
        DWORD n = 0;
        if (!::ReadFile(out_, buf, std::min<DWORD>(avail, sizeof buf), &n, nullptr)) return false;
        out->append(buf, n);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
#else
      pollfd p{out_, POLLIN, 0};
      if (poll(&p, 1, 20) > 0) {
        const ssize_t n = read(out_, buf, sizeof buf);
        if (n == 0) return false;
        if (n > 0) out->append(buf, size_t(n));
      }
#endif
      if (out->size() >= 4 && out->compare(out->size() - 4, 4, "lh> ") == 0) return true;
    }
    return false;
  }

#ifdef _WIN32
  HANDLE process_ = nullptr, in_ = nullptr, out_ = nullptr;
#else
  pid_t pid_ = -1;
  int in_ = -1, out_ = -1;
#endif
};

fs::path ConsoleTool(const fs::path& steamvr) {
#ifdef _WIN32
  return steamvr / "tools" / "lighthouse" / "bin" / "win64" / "lighthouse_console.exe";
#else
  return steamvr / "tools" / "lighthouse" / "bin" / "linux64" / "lighthouse_console";
#endif
}

// Opens the console for `serial`, working in the backups folder: the files it reads and writes
// are given by name only (it takes the rest of the line as the name, spaces in a path or not).
bool OpenConsole(const fs::path& steamvr, const std::string& serial, Console* console, std::string* error) {
  const fs::path tool = ConsoleTool(steamvr);
  std::error_code ec;
  if (!fs::exists(tool, ec)) { *error = "SteamVR's lighthouse_console isn't there (" + Utf8(tool) + ")"; return false; }
  fs::create_directories(DeviceConfigBackups(), ec);
  return console->Start(tool, DeviceConfigBackups(), serial, error);
}

// `after` written as edits to `text` (the config `before` came from): only the top-level string
// values that changed are replaced, so every other byte (numbers above all: a re-serialized
// calibration loses its exact digits) stays as the device had it. Anything else changed: the
// whole document, re-serialized.
std::string PatchConfig(const std::string& text, const json& before, const json& after) {
  bool simple = before.size() == after.size();
  for (auto it = after.begin(); simple && it != after.end(); ++it)
    if (!before.contains(it.key()) || (before.at(it.key()) != it.value() && (!it.value().is_string() || !before.at(it.key()).is_string())))
      simple = false;
  if (!simple) return after.dump(3);
  std::string out = text;
  for (auto it = after.begin(); it != after.end(); ++it) {
    if (before.at(it.key()) == it.value()) continue;
    const std::string key = "\"" + it.key() + "\"";
    const std::string oldValue = json(before.at(it.key())).dump(), newValue = it.value().dump();
    size_t k = out.find(key);
    size_t v = k == std::string::npos ? std::string::npos : out.find(oldValue, k + key.size());
    // Only `"key" : "old"` with nothing but whitespace and the colon between them.
    if (v != std::string::npos) {
      for (size_t i = k + key.size(); i < v; ++i)
        if (!std::isspace(static_cast<unsigned char>(out[i])) && out[i] != ':') { v = std::string::npos; break; }
    }
    if (v == std::string::npos) return after.dump(3);
    out.replace(v, oldValue.size(), newValue);
  }
  return json::parse(out, nullptr, false) == after ? out : after.dump(3);
}

std::string SafeName(const std::string& serial) {
  std::string s;
  for (char c : serial) s += std::isalnum(static_cast<unsigned char>(c)) || c == '-' ? c : '_';
  return s;
}

}  // namespace

DeviceKind KindOf(const json& config) {
  if (!config.is_object()) return DeviceKind::Unknown;
  const std::string cls = Lower(config.value("device_class", ""));
  if (cls == "hmd") return DeviceKind::Headset;
  if (Lower(config.value("tracked_controller_role", "")) == "generic_tracker") return DeviceKind::Tracker;
  if (cls == "generic_tracker") return DeviceKind::Tracker;
  if (cls == "controller") return DeviceKind::Controller;
  return DeviceKind::Unknown;
}

const char* KindName(DeviceKind kind) {
  switch (kind) {
    case DeviceKind::Controller: return "controller";
    case DeviceKind::Tracker: return "tracker";
    case DeviceKind::Headset: return "headset";
    default: return "unknown";
  }
}

bool ConvertKind(const json& config, DeviceKind to, uint16_t pid, const json* original, json* out, std::string* why) {
  const DeviceKind from = KindOf(config);
  if (from == DeviceKind::Headset) { *why = "it's a headset"; return false; }
  if (from == DeviceKind::Unknown) { *why = "its config doesn't say what it is (device_class)"; return false; }
  if (to != DeviceKind::Controller && to != DeviceKind::Tracker) { *why = "it can only be a controller or a tracker"; return false; }
  if (original && KindOf(*original) == to) {
    *out = *original;
    return true;
  }
  *out = config;
  json& j = *out;
  if (to == DeviceKind::Tracker) {
    j["device_class"] = "generic_tracker";
    if (j.contains("tracked_controller_role")) j.erase("tracked_controller_role");
    return true;
  }
  j["device_class"] = "controller";
  if (Lower(j.value("tracked_controller_role", "")) == "generic_tracker") j.erase("tracked_controller_role");
  // A wand that was made a tracker by hand usually got a tracker's model and name too: the
  // wand's own, so SteamVR draws and binds it as a Vive controller again.
  if (pid == kWandPid) {
    if (Contains(j, "render_model", "tracker")) j["render_model"] = "vr_controller_vive_1_5";
    for (const char* key : {"model_name", "model_number"})
      if (Contains(j, key, "tracker")) j[key] = "Vive. Controller MV";
  }
  return true;
}

fs::path DeviceConfigBackups() { return ConfigDirectory() / "device-configs"; }

bool ReadDeviceConfig(const fs::path& steamvr, const std::string& serial, std::string* text, std::string* error) {
  Console console;
  if (!OpenConsole(steamvr, serial, &console, error)) return false;
  const std::string name = SafeName(serial) + ".read.json";
  std::error_code ec;
  fs::remove(DeviceConfigBackups() / name, ec);
  std::string out;
  if (!console.Command("downloadconfig " + name, &out, 20)) { *error = "lighthouse_console didn't answer" + Console::LastLine(out); return false; }
  if (!ReadFile(DeviceConfigBackups() / name, text) || text->empty()) {
    *error = "couldn't read the config" + Console::LastLine(out);
    return false;
  }
  fs::remove(DeviceConfigBackups() / name, ec);
  return true;
}

bool WriteDeviceConfig(const fs::path& steamvr, const std::string& serial, const std::string& text, std::string* error) {
  const std::string name = SafeName(serial) + ".write.json";
  if (!WriteFileAtomic(DeviceConfigBackups() / name, text)) { *error = "couldn't write a temporary file"; return false; }
  Console console;
  if (!OpenConsole(steamvr, serial, &console, error)) return false;
  std::string out;
  const bool answered = console.Command("uploadconfig " + name, &out, 30);
  std::error_code ec;
  fs::remove(DeviceConfigBackups() / name, ec);
  if (!answered || out.find("Wrote the contents of") == std::string::npos) {
    *error = "the device didn't take the config" + Console::LastLine(out);
    return false;
  }
  return true;
}

bool SetDeviceKind(const fs::path& steamvr, const std::string& serial, uint16_t pid, DeviceKind to,
                   const std::function<void(const std::string&)>& log, std::string* error) {
  log("Reading the config of " + serial);
  std::string text;
  if (!ReadDeviceConfig(steamvr, serial, &text, error)) return false;
  const json config = json::parse(text, nullptr, false);
  if (!config.is_object()) { *error = "the device's config isn't valid JSON"; return false; }
  if (KindOf(config) == to) { log(serial + " already is a " + KindName(to)); return true; }

  // Backups: the first config ever seen for the device, and this one.
  const fs::path dir = DeviceConfigBackups();
  const fs::path first = dir / (SafeName(serial) + ".original.json");
  std::error_code ec;
  if (!fs::exists(first, ec) && !WriteFileAtomic(first, text)) { *error = "couldn't back up the config"; return false; }
  const fs::path before = dir / (SafeName(serial) + "-" + Timestamp() + ".json");
  if (!WriteFileAtomic(before, text)) { *error = "couldn't back up the config"; return false; }
  log("Backed up to " + Utf8(before));

  std::string original;
  json originalJson;
  const json* originalPtr = nullptr;
  if (ReadFile(first, &original)) {
    originalJson = json::parse(original, nullptr, false);
    if (originalJson.is_object()) originalPtr = &originalJson;
  }
  json converted;
  std::string why;
  if (!ConvertKind(config, to, pid, originalPtr, &converted, &why)) { *error = "can't switch it: " + why; return false; }
  // Back to the original: its text as it was; otherwise this config with only the type edited.
  const std::string out = originalPtr && converted == *originalPtr ? original : PatchConfig(text, config, converted);

  log(std::string("Writing it as a ") + KindName(to));
  if (!WriteDeviceConfig(steamvr, serial, out, error)) return false;

  log("Checking");
  std::string check;
  if (!ReadDeviceConfig(steamvr, serial, &check, error)) return false;
  const json back = json::parse(check, nullptr, false);
  if (KindOf(back) != to) {
    *error = "the device still reads as a " + std::string(KindName(KindOf(back))) + "; its old config is in " + Utf8(before);
    return false;
  }
  log(serial + " is now a " + KindName(to));
  return true;
}

}  // namespace openhouse::installer
