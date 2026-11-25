#include "lh/bug_report.hpp"

#include <zlib.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

#include "lh/version.hpp"
#include "platform/platform.hpp"

#ifdef _WIN32
#include <knownfolders.h>
#include <shlobj.h>
#else
#include <sys/utsname.h>
#endif

namespace lh {
namespace {
namespace fs = std::filesystem;

// A plain zip (deflate, data descriptors, no zip64: 4 GB per file and in total).
class ZipWriter {
 public:
  bool Open(const fs::path& path) {
    out_.open(path, std::ios::binary | std::ios::trunc);
    return bool(out_);
  }
  bool AddText(const std::string& name, const std::string& text) {
    std::istringstream in(text);
    return Add(name, in);
  }
  bool AddFile(const std::string& name, const fs::path& source) {
    std::ifstream in(source, std::ios::binary);
    return in && Add(name, in);
  }
  bool Close() {
    const uint64_t start = Offset();
    for (const auto& e : entries_) {
      U32(0x02014b50); U16(20); U16(20); U16(0x0808); U16(8); U16(time_); U16(date_);
      U32(e.crc); U32(uint32_t(e.compressed)); U32(uint32_t(e.size));
      U16(uint16_t(e.name.size())); U16(0); U16(0); U16(0); U16(0); U32(0); U32(uint32_t(e.offset));
      out_.write(e.name.data(), std::streamsize(e.name.size()));
    }
    const uint64_t size = Offset() - start;
    if (start > kMax || size > kMax) return false;
    U32(0x06054b50); U16(0); U16(0); U16(uint16_t(entries_.size())); U16(uint16_t(entries_.size()));
    U32(uint32_t(size)); U32(uint32_t(start)); U16(0);
    out_.close();
    return !out_.fail();
  }

 private:
  static constexpr uint64_t kMax = std::numeric_limits<uint32_t>::max();
  struct Entry { std::string name; uint32_t crc = 0; uint64_t size = 0, compressed = 0, offset = 0; };

  uint64_t Offset() { return uint64_t(out_.tellp()); }
  void U16(uint16_t v) { const char b[2] = {char(v), char(v >> 8)}; out_.write(b, 2); }
  void U32(uint32_t v) { U16(uint16_t(v)); U16(uint16_t(v >> 16)); }

  bool Add(const std::string& name, std::istream& in) {
    if (!time_) Stamp();
    Entry e{name, 0, 0, 0, Offset()};
    // Flag 0x0808: sizes and CRC follow the data (data descriptor), names are UTF-8.
    U32(0x04034b50); U16(20); U16(0x0808); U16(8); U16(time_); U16(date_);
    U32(0); U32(0); U32(0); U16(uint16_t(name.size())); U16(0);
    out_.write(name.data(), std::streamsize(name.size()));
    z_stream z{};
    if (deflateInit2(&z, 6, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY) != Z_OK) return false;
    std::vector<char> input(1 << 16), output(1 << 16);
    uLong crc = crc32(0, nullptr, 0);
    int flush = Z_NO_FLUSH;
    do {
      in.read(input.data(), std::streamsize(input.size()));
      const auto got = size_t(in.gcount());
      crc = crc32(crc, reinterpret_cast<const Bytef*>(input.data()), uInt(got));
      e.size += got;
      flush = in.eof() || got == 0 ? Z_FINISH : Z_NO_FLUSH;
      z.next_in = reinterpret_cast<Bytef*>(input.data());
      z.avail_in = uInt(got);
      do {
        z.next_out = reinterpret_cast<Bytef*>(output.data());
        z.avail_out = uInt(output.size());
        deflate(&z, flush);
        const size_t made = output.size() - z.avail_out;
        out_.write(output.data(), std::streamsize(made));
        e.compressed += made;
      } while (z.avail_out == 0);
    } while (flush != Z_FINISH);
    deflateEnd(&z);
    e.crc = uint32_t(crc);
    if (e.size > kMax || e.compressed > kMax || e.offset > kMax) return false;
    U32(0x08074b50); U32(e.crc); U32(uint32_t(e.compressed)); U32(uint32_t(e.size));
    entries_.push_back(std::move(e));
    return bool(out_);
  }
  void Stamp() {
    const std::time_t t = std::time(nullptr);
    const std::tm local = *std::localtime(&t);
    time_ = uint16_t(local.tm_hour << 11 | local.tm_min << 5 | local.tm_sec / 2);
    date_ = uint16_t((local.tm_year - 80) << 9 | (local.tm_mon + 1) << 5 | local.tm_mday);
  }

  std::ofstream out_;
  std::vector<Entry> entries_;
  uint16_t time_ = 0, date_ = 0;
};

std::string SystemLine() {
#ifdef _WIN32
  OSVERSIONINFOW v{};
  v.dwOSVersionInfoSize = sizeof v;
  // RtlGetVersion: GetVersionEx lies about Windows 10/11 without a manifest.
  using Fn = LONG(WINAPI*)(OSVERSIONINFOW*);
  if (const auto fn = reinterpret_cast<Fn>(GetProcAddress(GetModuleHandleW(L"ntdll.dll"), "RtlGetVersion")); fn && fn(&v) == 0)
    return "Windows " + std::to_string(v.dwMajorVersion) + "." + std::to_string(v.dwMinorVersion) + " build " +
           std::to_string(v.dwBuildNumber);
  return "Windows";
#else
  std::string name = "Linux";
  std::ifstream release("/etc/os-release");
  for (std::string line; std::getline(release, line);)
    if (line.rfind("PRETTY_NAME=", 0) == 0) {
      name = line.substr(12);
      name.erase(std::remove(name.begin(), name.end(), '"'), name.end());
    }
  utsname u{};
  if (uname(&u) == 0) name += std::string(", kernel ") + u.release;
  return name;
#endif
}

std::string Timestamp(const char* format) {
  const std::time_t t = std::time(nullptr);
  char text[64];
  std::strftime(text, sizeof text, format, std::localtime(&t));
  return text;
}

// For names in the zip (flagged UTF-8) and messages. path::string() converts to the
// Windows code page and throws for what it can't hold (nothing here may throw).
std::string Utf8(const fs::path& p) {
  const auto s = p.u8string();
  return std::string(s.begin(), s.end());
}

// The newest *.lhraw in dir (the settings page's recordings), if any.
fs::path NewestRecording(const fs::path& dir) {
  std::error_code ec;
  fs::path best;
  fs::file_time_type bestTime{};
  for (const auto& e : fs::directory_iterator(dir, ec))
    if (e.path().extension() == ".lhraw" && (best.empty() || e.last_write_time(ec) > bestTime)) {
      best = e.path();
      bestTime = e.last_write_time(ec);
    }
  return best;
}

}  // namespace

std::string BugReportDirectory() {
#ifdef _WIN32
  PWSTR path = nullptr;
  std::string documents;
  if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Documents, 0, nullptr, &path))) {
    try {
      documents = Utf8(fs::path(path));
    } catch (const std::exception&) {  // not in the code page: the home fallback below
    }
  }
  CoTaskMemFree(path);
  if (documents.empty()) documents = platform::Home() + "/Documents";
#else
  std::string documents;
  // ~/.config/user-dirs.dirs: XDG_DOCUMENTS_DIR="$HOME/Documents" (localized names too).
  const std::string config = platform::Env("XDG_CONFIG_HOME").empty() ? platform::Home() + "/.config"
                                                                       : platform::Env("XDG_CONFIG_HOME");
  std::ifstream dirs(config + "/user-dirs.dirs");
  for (std::string line; std::getline(dirs, line);)
    if (line.rfind("XDG_DOCUMENTS_DIR=", 0) == 0) {
      documents = line.substr(18);
      documents.erase(std::remove(documents.begin(), documents.end(), '"'), documents.end());
      if (documents.rfind("$HOME", 0) == 0) documents = platform::Home() + documents.substr(5);
    }
  if (documents.empty()) documents = platform::Home() + "/Documents";
#endif
  return documents + "/openhouse reports";
}

namespace {
std::string SaveBugReportOnce(const BugReportOptions& options, std::string* error) {
  auto fail = [&](const std::string& why) {
    if (error) *error = why;
    return std::string();
  };
  const fs::path dir = BugReportDirectory();
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) return fail("cannot create " + Utf8(dir) + ": " + ec.message());
  // The process id too: the driver and the viewer can both save one in the same second.
  const fs::path zip = dir / ("openhouse-report-" + Timestamp("%Y%m%d-%H%M%S") + "-" +
                              std::to_string(platform::ProcessId()) + ".zip");

  // (name in the zip, source); a missing one is listed as such in info.txt.
  std::vector<std::pair<std::string, fs::path>> files;
  const fs::path config = platform::ConfigDirectory();
  for (const char* name : {"roommap.json", "hostframe.json", "settings.json"})
    files.push_back({std::string("config/") + name, config / name});
  for (const auto& e : fs::directory_iterator(config / "preinit", ec))
    if (e.path().extension() == ".json") files.push_back({"config/preinit/" + Utf8(e.path().filename()), e.path()});
  const fs::path recording = !options.recording.empty() ? platform::FsPath(options.recording)
                                                         : NewestRecording(platform::FsPath(platform::DataDirectory()) / "recordings");
  if (!recording.empty()) files.push_back({"recording/" + Utf8(recording.filename()), recording});
  const fs::path logs = platform::FsPath(platform::SteamRoot()) / "logs";
  for (const char* name : {"vrserver.txt", "vrserver.previous.txt"})
    files.push_back({std::string("steamvr/") + name, logs / name});
  const fs::path ourLogs = platform::FsPath(platform::DataDirectory()) / "logs";
  for (const char* name : {"openhouse.txt", "openhouse.previous.txt"})
    files.push_back({std::string("logs/") + name, ourLogs / name});
  for (const auto& e : fs::directory_iterator(ourLogs, ec))   // the apps' crash records (common/crash_log.hpp)
    if (Utf8(e.path().filename()).ends_with("-crash.txt")) files.push_back({"logs/" + Utf8(e.path().filename()), e.path()});

  std::ostringstream info;
  info << "openhouse " << kVersion << "\n"
       << "saved " << Timestamp("%Y-%m-%d %H:%M:%S") << "\n"
       << "system: " << SystemLine() << "\n";
  if (recording.empty()) info << "recording: none (turn on Record session in SteamVR's openhouse settings)\n";
  else info << "recording: " << Utf8(recording.filename()) << "\n";
  std::vector<std::pair<std::string, fs::path>> present;
  for (const auto& f : files) {
    // Present if it opens: one that is there but unreadable lost the whole report.
    if (fs::is_regular_file(f.second, ec) && std::ifstream(f.second, std::ios::binary)) present.push_back(f);
    else info << "missing: " << Utf8(f.second) << "\n";
  }
  if (!options.summary.empty()) info << "\n" << options.summary << (options.summary.back() == '\n' ? "" : "\n");

  ZipWriter writer;
  if (!writer.Open(zip)) return fail("cannot write " + Utf8(zip));
  bool ok = writer.AddText("info.txt", info.str());
  if (!options.log.empty()) ok = ok && writer.AddText("openhouse-log.txt", options.log);
  for (const auto& [name, source] : present) ok = ok && writer.AddFile(name, source);
  ok = writer.Close() && ok;
  if (!ok) {
    fs::remove(zip, ec);
    return fail("writing " + Utf8(zip) + " failed (out of space, or a file over 4 GB)");
  }
  return Utf8(zip);
}
}  // namespace

// Nothing may throw out of here: both callers run it on their own thread.
std::string SaveBugReport(const BugReportOptions& options, std::string* error) {
  try {
    return SaveBugReportOnce(options, error);
  } catch (const std::exception& e) {
    if (error) *error = e.what();
  } catch (...) {
    if (error) *error = "unexpected error";
  }
  return {};
}

}  // namespace lh
