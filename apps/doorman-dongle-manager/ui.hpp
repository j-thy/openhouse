// doorman-dongle-manager's window: the Doorman Dongles plugged in, their eight slots (what's paired,
// pairing into a slot, unpairing), the radio's live performance, its channels and firmware.
// Same look as openhouse-overlay (pixel font, sidebar pages, status bar).
#pragma once
#include <filesystem>
#include <atomic>
#include <cstdint>
#include <map>
#include <set>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "dongle.hpp"
#include "telemetry_client.hpp"

namespace openhouse::dongle {

class Ui {
 public:
  Ui();
  ~Ui();
  static void SetUpImGui(int k);
  void SetLogo(uint64_t texture) { logo_ = texture; }
  void Poll(double now);
  void Draw(float width, float height);
  void SaveSettings();

 private:
  enum class Page { Overview, Slots, Radio, Channels, Firmware, About };
  void Sidebar(float height);
  void StatusBar();
  void Overview(const DongleState& d);
  void Slots(const DongleState& d);
  void SlotCard(const DongleState& d, int slot, float width);
  void Radio(const DongleState& d);
  void Channels(const DongleState& d);
  void Firmware(const DongleState& d);
  void About();
  void NoDongle();
  void PairBar(const DongleState& d);

  std::shared_ptr<Dongle> Current();
  Dongle::Done Report(const std::string& what);
  void Toast(const std::string& text, bool error = false);
  std::string DeviceLabel(const SlotState& s) const;
  void Remember(const DongleState& d);

  DongleManager manager_;
  viewer::TelemetryClient driver_;
  Page page_ = Page::Overview;
  std::string dongle_;    // the selected dongle's serial
  int selectedSlot_ = 0;  // Radio page
  int pendingSlot_ = -1;  // slot a confirmation is about
  char rename_[64] = {};
  std::set<std::string> movedNames_;  // dongle/pairing id whose remembered name was put on the dongle
  // remembered devices, by pairing id: serial, model, nickname (settings file)
  struct Known { std::string serial, model, nickname; };
  std::map<std::string, Known> known_;
  bool knownDirty_ = false;
  // results of actions, from the dongle threads
  std::mutex resultsMutex_;
  std::vector<std::pair<std::string, bool>> results_;
  std::string toast_;
  bool toastError_ = false;
  double toastUntil_ = 0, now_ = 0;
  uint64_t logo_ = 0;
  // firmware update (UF2 drive copy), on a thread of its own
  char firmwarePath_[512] = {};
  double lastSave_ = 0;
  std::thread updater_;
  std::atomic<bool> updating_{false};
  std::mutex updateMutex_;
  std::string updateStatus_;
  bool updateError_ = false;
  void StartUpdate(std::shared_ptr<Dongle> d, const std::string& file, bool nordicDfu);
  void StartBankUpdate(std::shared_ptr<Dongle> d, const std::filesystem::path& file);
  static std::filesystem::path PackagedBankImage(const std::string& name);
};

}  // namespace openhouse::dongle
