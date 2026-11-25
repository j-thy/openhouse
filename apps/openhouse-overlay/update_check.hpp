// Asks GitHub for the latest openhouse release, on a thread of its own (the same request as
// update-checker: api.github.com/repos/YusufOzmen01/openhouse/releases/latest). Off with
// OPENHOUSE_NO_UPDATE_CHECK, as there.
#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace openhouse::overlay {

class UpdateCheck {
 public:
  ~UpdateCheck();
  void Start();  // again once the last one finished
  bool Busy() const { return busy_; }
  bool Done() const { return done_; }
  bool Disabled() const;
  // After Done(): the latest tag ("v1.0.4"; "" if GitHub didn't answer), and whether it's newer.
  std::string Latest() const;
  bool Newer() const { return newer_; }
  // Same version, and GitHub couldn't say whether this build's commit is older (a local commit).
  bool Unknown() const { return unknown_; }

  static constexpr const char* kReleasesPage = "https://github.com/YusufOzmen01/openhouse/releases/latest";
  static void OpenReleasesPage();
  // The latest release's own page (Latest()), or the releases page before the check is done.
  void OpenLatestRelease() const;
  // Opens a folder or URL with the desktop's default app (UTF-8).
  static void Open(const std::string& target);

 private:
  std::thread thread_;
  std::atomic<bool> busy_{false}, done_{false}, newer_{false}, unknown_{false};
  mutable std::mutex mutex_;
  std::string latest_;
};

}  // namespace openhouse::overlay
