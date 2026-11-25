// openhouse-overlay's window: one UI for both the SteamVR dashboard overlay and the desktop
// window. It talks to the running driver over its telemetry (lh/telemetry.hpp): the state
// and log it streams, and commands for settings and actions.
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <string>
#include <string_view>

#include "base_channels.hpp"
#include "log_view.hpp"
#include "telemetry_client.hpp"
#include "views.hpp"
#include "update_check.hpp"

namespace openhouse::overlay {

class Ui {
 public:
  explicit Ui(int port);
  // k: the pixel font at 13 * k px (sharp only at whole multiples), sizes to match.
  static void SetUpImGui(int k);
  void SetLogo(uint64_t texture) { logo_ = texture; }  // an ImTextureID (GL texture)
  void SetInVR(bool inVR) { inVR_ = inVR; }
  void SetNameKeyboard(std::function<bool(const char*, uint64_t)> keyboard) { nameKeyboard_ = std::move(keyboard); }
  void CommitTrackerName(std::string_view name, uint64_t edit);
  void CancelTrackerName(uint64_t edit);

  void Poll(double now);                // telemetry, replies
  void Draw(float width, float height);  // one frame, inside ImGui::NewFrame/Render
  bool QuitRequested() const { return quit_; }
  void SaveSettings() const;

 private:
  enum class Page { Overview, Devices, Bases, Dongles, Map, Settings, Debug, About };
  void Sidebar(float height);
  void Overview();
  void Devices();
  void Bases();
  void BaseChannelControls(const viewer::Base& base);
  void PollBaseChannels();
  void SetBaseChannel(BaseChannelChange change, bool automatic = false);
  void CancelBaseChannelChange(const std::string& error);
  void Dongles();
  void Map();
  void SettingsPage();
  void Debug();
  void About();
  void StatusBar();
  void UpdateCard();
  void ReplacedBanner();
  void NoticePopups();
  bool confirm_ = false, offered_ = false, uninstall_ = false;  // uninstall_: the confirmation is for uninstalling
  void FloorCalibration();  // the Room map page's "put your controllers down" countdown
  void RoomActions();      // Record / Reset room, in the sidebar

  // A command to the driver; `what` is what the status bar says about it.
  void Send(const std::string& json, const std::string& what);
  void Toast(const std::string& text, bool error = false);

  viewer::TelemetryClient client_;
  const bool haveChannelBackend_ = HaveBaseChannelBackend();
  bool autoChannels_ = false;
  BaseChannelChange channelChange_;
  bool automaticChannelChange_ = false;
  uint64_t channelConnection_ = 0, autoChannelConnection_ = 0;
  std::shared_ptr<BaseChannelLock> channelLock_;
  std::future<BaseChannelDiscovery> channelDiscovery_;
  struct ChannelWriteResult {
    std::string error;
    double completedAt;
  };
  std::future<ChannelWriteResult> channelWrite_;
  double channelConfirmationDeadline_ = 0, channelConfirmationAfter_ = 0;
  viewer::LogView logs_;
  viewer::Detail detail_ = viewer::Detail::Basic;
  UpdateCheck update_;
  Page page_ = Page::Overview;
  std::string selectedDevice_, selectedBase_, renameSerial_;
  std::array<char, 256> trackerName_{};
  std::function<bool(const char*, uint64_t)> nameKeyboard_;
  uint64_t renameConnection_ = 0, renameEdit_ = 0;
  uint64_t reportCount_ = UINT64_MAX;  // the driver's report count last seen (MAX: none yet)
  std::string reportPath_, reportError_;
  bool showReport_ = false;
  void ReportPopup();
  void CopyServerLog();  // vrserver.txt to the clipboard
  std::string logCopyResult_;
  bool showLogCopy_ = false;
  double floorAt_ = 0;  // when the floor calibration happens (0: none under way)
  std::map<uint64_t, std::string> pending_;  // command id -> what it does
  std::string toast_;
  bool toastError_ = false;
  double toastUntil_ = 0, now_ = 0;
  uint64_t logo_ = 0;
  bool inVR_ = false, quit_ = false;
};

}  // namespace openhouse::overlay
