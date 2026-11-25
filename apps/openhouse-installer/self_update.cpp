#include "self_update.hpp"

#include <lh/github.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <winhttp.h>
#else
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
extern char** environ;
#endif

namespace openhouse::installer {
namespace {
// The package among a release's files (its name has changed between releases).
#ifdef _WIN32
constexpr const char* kPlatform = "windows";
constexpr const char* kSuffix = ".zip";
constexpr const char* kInstaller = "openhouse-installer.exe";
#else
constexpr const char* kPlatform = "linux";
constexpr const char* kSuffix = ".tar.gz";
constexpr const char* kInstaller = "openhouse-installer";
#endif
constexpr const char* kDownloads = "https://github.com/YusufOzmen01/openhouse/releases/download/";

// The string value after "key": at or after `from`.
std::string Value(const std::string& json, const std::string& key, size_t from = 0) {
  const auto at = json.find("\"" + key + "\"", from);
  if (at == std::string::npos) return {};
  const auto open = json.find('"', json.find(':', at) + 1);
  if (open == std::string::npos) return {};
  const auto close = json.find('"', open + 1);
  return close == std::string::npos ? std::string() : json.substr(open + 1, close - open - 1);
}

// The release's download link for this platform's package; only one of openhouse's own.
std::string AssetUrl(const std::string& json) {
  for (size_t at = json.find("\"browser_download_url\""); at != std::string::npos; at = json.find("\"browser_download_url\"", at + 1)) {
    const std::string url = Value(json, "browser_download_url", at);
    const std::string name = url.substr(url.find_last_of('/') + 1);
    if (url.rfind(kDownloads, 0) == 0 && name.find(kPlatform) != std::string::npos && name.size() > std::strlen(kSuffix) &&
        name.compare(name.size() - std::strlen(kSuffix), std::strlen(kSuffix), kSuffix) == 0)
      return url;
  }
  return {};
}

void Download(const std::string& url, const fs::path& to, const std::function<void(float, const std::string&)>& status) {
#ifdef _WIN32
  const std::string host = "github.com", path = url.substr(std::string("https://github.com").size());
  HINTERNET session = WinHttpOpen(L"openhouse", WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
  if (!session) throw std::runtime_error("Couldn't connect to GitHub");
  WinHttpSetTimeouts(session, 15000, 15000, 30000, 60000);
  HINTERNET connect = WinHttpConnect(session, L"github.com", INTERNET_DEFAULT_HTTPS_PORT, 0);
  const std::wstring wpath(path.begin(), path.end());
  // Redirects (to GitHub's file host) are followed by WinHTTP.
  HINTERNET request = connect ? WinHttpOpenRequest(connect, L"GET", wpath.c_str(), nullptr, WINHTTP_NO_REFERER,
                                                   WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE) : nullptr;
  bool ok = false;
  if (request && WinHttpSendRequest(request, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) &&
      WinHttpReceiveResponse(request, nullptr)) {
    DWORD code = 0, size = sizeof code;
    WinHttpQueryHeaders(request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &code, &size, WINHTTP_NO_HEADER_INDEX);
    DWORD total = 0;
    size = sizeof total;
    WinHttpQueryHeaders(request, WINHTTP_QUERY_CONTENT_LENGTH | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &total, &size, WINHTTP_NO_HEADER_INDEX);
    std::ofstream out(to, std::ios::binary);
    if (code == 200 && out) {
      ok = true;
      size_t got = 0;
      for (DWORD available = 0; WinHttpQueryDataAvailable(request, &available) && available;) {
        std::vector<char> chunk(available);
        DWORD read = 0;
        if (!WinHttpReadData(request, chunk.data(), available, &read) || !read) { ok = false; break; }
        out.write(chunk.data(), read);
        got += read;
        if (total) status(float(got) / float(total), "Downloading");
      }
      ok = ok && out.good() && (!total || got == total);
    }
  }
  if (request) WinHttpCloseHandle(request);
  if (connect) WinHttpCloseHandle(connect);
  WinHttpCloseHandle(session);
  if (!ok) throw std::runtime_error("The download failed. Check your connection and try again");
#else
  (void)status;
  if (!HaveProgram("curl")) throw std::runtime_error("curl is needed to download the update");
  if (RunProcess({"curl", "-fsSL", "--max-time", "600", "-o", to.string(), url}, nullptr) != 0)
    throw std::runtime_error("The download failed. Check your connection and try again");
#endif
}
}  // namespace

fs::path DownloadLatestPackage(const std::function<void(float, const std::string&)>& status, std::string* tag) {
  std::error_code ec;
  // In openhouse's own folder, not the temp folder: /tmp can be mounted noexec.
  const fs::path dir = DataDirectory() / "update";
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  if (ec) throw std::runtime_error("Couldn't make a temporary folder: " + ec.message());
  const fs::path archive = dir / (std::string("package") + kSuffix);
  const fs::path unpacked = dir / "package";
  fs::create_directories(unpacked, ec);
  if (const char* local = std::getenv("OPENHOUSE_UPDATE_PACKAGE"); local && *local) {
    fs::copy_file(Path(local), archive, fs::copy_options::overwrite_existing, ec);
    if (ec) throw std::runtime_error("Couldn't copy " + std::string(local) + ": " + ec.message());
    *tag = "local package";
  } else {
    status(0, "Finding the latest release");
    const std::string json = lh::GithubGet("/repos/YusufOzmen01/openhouse/releases/latest");
    *tag = Value(json, "tag_name");
    const std::string url = AssetUrl(json);
    if (tag->empty() || url.empty()) throw std::runtime_error("Couldn't find the latest release on GitHub. Check your connection and try again");
    status(0, "Downloading " + *tag);
    Download(url, archive, status);
  }
  status(1, "Unpacking");
#ifdef _WIN32
  // Windows 10 and newer come with tar, which reads zips.
  const std::string tar = Utf8(Path(std::getenv("SystemRoot") ? std::getenv("SystemRoot") : "C:\\Windows") / "System32" / "tar.exe");
  const int rc = RunProcess({tar, "-xf", Utf8(archive), "-C", Utf8(unpacked)}, nullptr);
#else
  const int rc = RunProcess({"tar", "-xzf", archive.string(), "-C", unpacked.string()}, nullptr);
#endif
#ifdef _WIN32
  if (rc != 0) throw std::runtime_error("Couldn't unpack the update (Windows 10 1803 or newer is needed). Download it from the releases page");
#else
  if (rc != 0) throw std::runtime_error("Couldn't unpack the update");
#endif
  // At the top or in the package's own folder.
  for (auto it = fs::recursive_directory_iterator(unpacked, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    if (it.depth() > 1) { it.disable_recursion_pending(); continue; }
    if (it->path().filename() == kInstaller && it->is_regular_file(ec)) return it->path();
  }
  throw std::runtime_error("The update has no installer in it");
}

bool StartDetached(const fs::path& exe, const std::string& argument) {
#ifdef _WIN32
  std::wstring command = L"\"" + exe.wstring() + L"\" " + Path(argument).wstring();
  STARTUPINFOW si{sizeof si};
  PROCESS_INFORMATION pi{};
  BOOL ok = CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE, CREATE_BREAKAWAY_FROM_JOB, nullptr,
                           exe.parent_path().c_str(), &si, &pi);
  if (!ok) ok = CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE, 0, nullptr, exe.parent_path().c_str(), &si, &pi);
  if (!ok) return false;
  CloseHandle(pi.hThread);
  CloseHandle(pi.hProcess);
  return true;
#else
  const std::string path = exe.string();
  char* argv[] = {const_cast<char*>(path.c_str()), const_cast<char*>(argument.c_str()), nullptr};
  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);
  pid_t pid = 0;
  const int rc = posix_spawn(&pid, path.c_str(), nullptr, &attr, argv, environ);
  posix_spawnattr_destroy(&attr);
  return rc == 0;
#endif
}

}  // namespace openhouse::installer
