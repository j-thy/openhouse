#ifndef _WIN32
#include <thread>
#include <spawn.h>
#include <sys/wait.h>
extern char** environ;
#endif
#include "update_check.hpp"

#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>

#include "lh/github.hpp"
#include "lh/version.hpp"

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#include <winhttp.h>
#endif

namespace openhouse::overlay {
namespace {
std::string TagName(const std::string& json) {
  const auto key = json.find("\"tag_name\"");
  if (key == std::string::npos) return {};
  const auto open = json.find('"', json.find(':', key) + 1);
  if (open == std::string::npos) return {};
  const auto close = json.find('"', open + 1);
  return close == std::string::npos ? std::string() : json.substr(open + 1, close - open - 1);
}

}  // namespace

UpdateCheck::~UpdateCheck() {
  if (thread_.joinable()) thread_.join();
}

bool UpdateCheck::Disabled() const { return std::getenv("OPENHOUSE_NO_UPDATE_CHECK") != nullptr; }

void UpdateCheck::Start() {
  if (busy_ || Disabled()) return;
  if (thread_.joinable()) thread_.join();
  busy_ = true;
  done_ = false;
  thread_ = std::thread([this] {
    // OPENHOUSE_TEST_NEWER=v9.9.9 (testing): as if that release were out.
    const char* test = std::getenv("OPENHOUSE_TEST_NEWER");
    const std::string tag = test && *test ? std::string(test) : TagName(lh::GithubGet("/repos/YusufOzmen01/openhouse/releases/latest"));
    {
      std::lock_guard lock(mutex_);
      latest_ = tag;
    }
    const lh::UpdateState state = test && *test ? lh::UpdateState::Newer
                                  : tag.empty() ? lh::UpdateState::Current : lh::CheckRelease(tag, lh::GithubGet);
    newer_ = state == lh::UpdateState::Newer;
    unknown_ = state == lh::UpdateState::Unknown;
    done_ = true;
    busy_ = false;
  });
}

std::string UpdateCheck::Latest() const {
  std::lock_guard lock(mutex_);
  return latest_;
}

void UpdateCheck::OpenReleasesPage() {
#ifdef _WIN32
  ShellExecuteA(nullptr, "open", kReleasesPage, nullptr, nullptr, SW_SHOWNORMAL);
#else
  std::string command = std::string("xdg-open ") + kReleasesPage + " >/dev/null 2>&1 &";
  if (std::system(command.c_str()) != 0) std::fprintf(stderr, "openhouse-overlay: %s\n", kReleasesPage);
#endif
}

void UpdateCheck::OpenLatestRelease() const {
  const std::string tag = Latest();
  if (tag.empty()) OpenReleasesPage();
  else Open("https://github.com/YusufOzmen01/openhouse/releases/tag/" + tag);
}

void UpdateCheck::Open(const std::string& target) {
#ifdef _WIN32
  const int n = MultiByteToWideChar(CP_UTF8, 0, target.c_str(), -1, nullptr, 0);
  std::wstring wide(n > 0 ? n : 1, L'\0');
  MultiByteToWideChar(CP_UTF8, 0, target.c_str(), -1, wide.data(), n);
  ShellExecuteW(nullptr, L"open", wide.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#else
  // No shell: the path goes to xdg-open as is, whatever is in it.
  char* argv[] = {const_cast<char*>("xdg-open"), const_cast<char*>(target.c_str()), nullptr};
  pid_t pid = 0;
  if (posix_spawnp(&pid, "xdg-open", nullptr, nullptr, argv, environ) == 0) {
    std::thread([pid] { int status = 0; waitpid(pid, &status, 0); }).detach();
  }
#endif
}

}  // namespace openhouse::overlay
