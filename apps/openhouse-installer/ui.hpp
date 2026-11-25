// openhouse-installer's window: the home screen (install, uninstall, flash, restore) and each
// guided flow. Long work (copying, flashing, looking for devices) runs on worker threads; the
// UI only draws their state.
#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "device_type.hpp"
#include "drives.hpp"
#include "driver_install.hpp"
#include "radio.hpp"

namespace openhouse::installer {

// One piece of long work, on its own thread.
class Job {
 public:
  ~Job() { Join(); }
  // devices: it talks to devices, so it waits for a scan in progress. Work that only touches files
  // (install, uninstall, downloads) doesn't: a scan of slow slots held it up for seconds.
  void Start(std::function<bool(Job&)> work, bool devices = true);
  bool Running() const { return running_; }
  bool Finished() const { return finished_; }
  bool Ok() const { return ok_; }
  void Reset() { Join(); finished_ = false; ok_ = false; std::lock_guard l(m_); log_.clear(); status_.clear(); error_.clear(); progress_ = 0; }
  // From the work:
  void Log(const std::string& line) { std::lock_guard l(m_); log_.push_back(line); }
  void Status(float progress, const std::string& text) { std::lock_guard l(m_); progress_ = progress; status_ = text; }
  void Fail(const std::string& error) { std::lock_guard l(m_); error_ = error; }
  // For the UI:
  std::vector<std::string> LogLines() const { std::lock_guard l(m_); return log_; }
  std::string StatusText() const { std::lock_guard l(m_); return status_; }
  std::string Error() const { std::lock_guard l(m_); return error_; }
  float Progress() const { std::lock_guard l(m_); return progress_; }

 private:
  void Join() { if (thread_.joinable()) thread_.join(); }
  std::thread thread_;
  std::atomic<bool> running_{false}, finished_{false}, ok_{false};
  mutable std::mutex m_;
  std::vector<std::string> log_;
  std::string status_, error_;
  float progress_ = 0;
};

// Held by scans and jobs while they talk to devices: a scan that's still finishing never talks to a
// device a job is using.
inline std::mutex g_deviceMutex;

// Runs `look` every `seconds` on a thread, keeping the latest result. Stop() doesn't wait for a
// look in progress (a wireless scan can take seconds): it finishes on its own, under g_deviceMutex.
template <class T>
class Watcher {
 public:
  ~Watcher() { Stop(); }
  void Start(std::function<T()> look, double seconds) {
    Stop();
    auto state = std::make_shared<State>();
    state_ = state;
    std::thread([state, look, seconds] {
      t_scan = true;
      while (!state->stop) {
        T value;
        {
          std::lock_guard d(g_deviceMutex);
          if (state->stop) break;
          value = look();
        }
        { std::lock_guard l(state->m); state->value = std::move(value); state->have = true; }
        for (int i = 0; i < int(seconds * 10) && !state->stop; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }).detach();
  }
  void Stop() {
    if (state_) state_->stop = true;
    state_.reset();
  }
  T Value() const {
    if (!state_) return T{};
    std::lock_guard l(state_->m);
    return state_->value;
  }
  bool Have() const { return state_ && state_->have; }

 private:
  struct State {
    std::atomic<bool> stop{false}, have{false};
    std::mutex m;
    T value{};
  };
  std::shared_ptr<State> state_;
};

class Ui {
 public:
  Ui();
  static void SetUpImGui();
  void SetLogo(uint64_t texture) { logo_ = texture; }
  void Draw(float width, float height);
  // Testing (OPENHOUSE_INSTALLER_PAGE): the page shown first. "kuroshin": its DFU instructions.
  void ShowForTest(const std::string& page);
  // --update: downloads the latest release and starts its installer with --install.
  void StartSelfUpdate();
  // --install: installs right away (closing SteamVR), then offers a dongle update if one is due.
  void StartAutoInstall();
  void StartAutoUninstall();  // --uninstall: the same, uninstalling
  bool QuitRequested() const { return quit_; }

 private:
  enum class Page {
    Home, Install, Uninstall, FlashMenu, BaseStation, DoormanMenu, Dongle, RadioMenu, RadioWired, RadioWireless,
    RestoreMenu, RestoreBase, RestoreRadioMenu, RestoreRadio, RestoreRadioWireless, DeviceType,
    DongleUpdatePick, DongleUpdate, Tools, SelfUpdate
  };
  void Go(Page page);
  void Header();
  void Footer();
  void SteamVRPrompt();
  bool RuntimesClosed();
  bool DeviceAccess();  // Linux: draws the udev notice; true when this user can open devices  // draws the "close SteamVR" notice; true when nothing is running
  void Back(Page to);
  bool BackButton(const char* label = "< Back");  // Back's button, for a step back within a page

  void Home();
  void InstallPage(bool install);
  void SelfUpdatePage();
  void DongleUpdateOffer();
  void FlashMenu();
  void DoormanMenu();
  void RadioMenu();
  void RestoreMenu();
  void BaseStationPage(bool restore);
  void DonglePage();
  void RadioWiredPage(bool restore);
  void RadioWirelessPage(bool restore);
  void RestoreRadioMenu();
  void DeviceTypePage();
  void DongleUpdatePick();
  void ToolsPage();
  void DongleUpdatePage();
  void StartDongleUpdate(const std::vector<DongleSlot>& slots);

  void RefreshStatus();

  Page page_ = Page::Home;
  uint64_t logo_ = 0;
  SteamVR steamvr_;
  DriverStatus status_;
  bool askSteamVR_ = false;
  char steamvrInput_[1024] = {};
  std::string steamvrError_;
  std::vector<std::string> running_;
  double runningChecked_ = -1, accessChecked_ = -1;
  bool haveAccess_ = true;
  Job job_, closeJob_;
  bool monadoCheck_ = true;
  bool autoInstall_ = false, autoStarted_ = false, autoClosed_ = false;
  std::atomic<bool> quit_{false};  // set by the update job
  std::string updateTag_;
  bool statusFresh_ = true;
  int installAction_ = 0;  // 0 install, 1 update, 2 reinstall
  // Whether this package is newer than the installed driver (lh::CompareBuilds: a GitHub
  // compare for two builds of one version, on a thread). Only a confirmed newer one is an update.
  bool PackageNewer();
  // openhouse was installed, but its driver isn't in place any more (a SteamVR update or
  // "verify files" put SteamVR's back): Repair, and Uninstall cleans up what's left.
  bool Replaced() const;
  int InstallAction();  // 0 install, 1 update, 2 reinstall, 3 repair
  std::string orderFor_;                                  // the installed version it's for
  std::shared_ptr<std::atomic<int>> order_;               // lh::BuildOrder, -1 while asking

  // Base station flow
  enum class BaseStep { Warning, Waiting, Ready, Writing, Replug, Verifying, Done } baseStep_ = BaseStep::Warning;
  bool confirmed10_ = false;
  std::string baseImage_;      // what's written
  std::string baseImageName_;
  bool alreadyInstalled_ = false;
  Watcher<std::vector<Drive>> baseDrives_;
  Drive baseDrive_;
  bool sawUnplug_ = false;

  // Dongle flow
  enum class DongleStep { Board, Detect, Flashing, Done } dongleStep_ = DongleStep::Board;
  int dongleBoard_ = 0;  // board picked: 0 UF2, 1 Holyiot 21017, 2 PCA10059 (DoormanDongleInfo::board)
  struct DongleSeen { std::vector<Uf2Drive> drives; std::vector<std::string> ports; std::vector<DoormanDongleInfo> dongles; };
  Watcher<DongleSeen> dongleSeen_;
  std::string dongleResult_;

  // Radio flows
  enum class RadioStep { Choose, Flashing, Done } radioStep_ = RadioStep::Choose;
  Watcher<std::vector<WiredDevice>> wired_;
  Watcher<std::vector<WirelessDevice>> wireless_;
  std::set<std::string> checked_;
  std::set<std::string> accepted_;  // unknown devices the user accepted to flash anyway
  std::vector<StockModel> stockModels_;
  std::map<std::string, int> stockChoice_;  // doorman-dongle-manager device -> stockModels_ index
  std::vector<std::pair<std::string, std::string>> results_;  // device, result
  std::atomic<int> current_{0};
  int total_ = 0;

  // Device type flow: each wired device's type, read once from its config (kindJob_).
  Job kindJob_;
  std::mutex kindsMutex_;
  std::map<std::string, std::string> kinds_;  // serial -> "controller", "tracker", "headset", or why it's unreadable
  std::string kindTarget_;                    // the device being switched

  // Dongle update (Home's "Update Dongles"): the devices' radios first, then the dongle.
  Watcher<std::vector<DoormanDongleInfo>> homeDongles_;   // Home: is any dongle on other firmware?
  bool homeWatching_ = false;
  bool testOnly_ = false;   // ShowForTest("update"): draws the page, never starts the update
  Watcher<std::vector<DongleSlot>> updateSlots_;
  DoormanDongleInfo updateDongle_;
  struct SlotResult { int state = 0; std::string text; };   // state: 0 waiting, 1 working, 2 done, 3 failed
  std::mutex updateMutex_;
  std::map<int, SlotResult> updateResults_;                // slot -> what happened (-1: the dongle)
  std::vector<DongleSlot> updateSnapshot_;                 // the slots as they were when it started
};

}  // namespace openhouse::installer
