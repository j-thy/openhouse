// SteamVR's watchdog (IVRWatchdogProvider): loaded while SteamVR is not running, it starts
// SteamVR when a device is turned on, as stock's CWatchdogDriver_Lighthouse does ("new
// wireless connection", #38). Off when Steam's own setting is: "power" /
// "autoLaunchSteamVROnButtonPress" ("Start SteamVR when a headset or controller is turned
// on" in Steam's settings).
//
// What wakes it, polled once a second:
// - a receiver (dongle, 28de:2101/2102) that had no device gets one: a controller or tracker
//   was turned on;
// - a wired device (28de:2000/2012/2022/2300) is plugged in, unless SteamVR knows it as a
//   controller or tracker (WiredPlugInWakes): those show up on a charging cable while off (#91).
//   A Vive tracker and the Bigscreen Beyond are both 28de:2300, so it goes by the serial.
// Devices already there when the watchdog starts don't wake it: SteamVR was just closed with
// them, or Steam is starting.
#include "watchdog.hpp"

#include "../platform/platform.hpp"
#include "../transport/usb/usb_transport.hpp"

#include <hidapi/hidapi.h>

#include <atomic>
#include <chrono>
#include <map>
#include <set>
#include <string>
#include <thread>

using namespace vr;

namespace lh::openvr {
namespace {

constexpr uint16_t kValve = 0x28de;

bool IsReceiver(uint16_t pid) { return pid == 0x2101 || pid == 0x2102; }
bool IsWired(uint16_t pid) { return pid == 0x2000 || pid == 0x2012 || pid == 0x2022 || pid == 0x2300; }

std::string Narrow(const wchar_t* s) {
  std::string out;
  for (; s && *s; ++s) out += *s < 0x80 ? char(*s) : '?';
  return out;
}

class Watchdog final : public IVRWatchdogProvider {
 public:
  EVRInitError Init(IVRDriverContext* context) override {
    VR_INIT_WATCHDOG_DRIVER_CONTEXT(context);
    // Steam's switch. Missing (an old Steam): on, as SteamVR's default.
    if (VRSettings()) {
      EVRSettingsError e = VRSettingsError_None;
      const bool on = VRSettings()->GetBool("power", "autoLaunchSteamVROnButtonPress", &e);
      if (e == VRSettingsError_None && !on) return VRInitError_Init_WatchdogDisabledInSettings;
    }
    if (hid_init() != 0) return VRInitError_Init_InitCanceledByUser;
    stop_ = false;
    thread_ = std::thread([this] { Main(); });
    return VRInitError_None;
  }

  void Cleanup() override {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    for (auto& [path, r] : receivers_)
      if (r.dev) hid_close(r.dev);
    receivers_.clear();
    wired_.clear();
    CleanupDriverContext();
  }

 private:
  struct Receiver {
    hid_device* dev = nullptr;
    bool connected = false;
  };

  void Main() {
    bool first = true;
    while (!stop_) {
      Poll(first);
      first = false;
      for (int i = 0; i < 10 && !stop_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void Poll(bool first) {
    std::set<std::string> seenReceivers, seenWired;
    hid_device_info* list = hid_enumerate(kValve, 0);
    for (hid_device_info* d = list; d; d = d->next) {
      if (!d->path) continue;
      const std::string path = d->path;
      if (IsReceiver(d->product_id) && d->interface_number == 0) {
        seenReceivers.insert(path);
        if (!receivers_.count(path)) {
          Receiver r;
          r.dev = hid_open_path(d->path);
          if (!r.dev) continue;
          // A receiver plugged in after start counts from its first answer, as one at start.
          if (const auto s = lh::usb::UsbTransport::ConnectedSerial(r.dev)) r.connected = *s != 0;
          receivers_[path] = r;
        }
      } else if (IsWired(d->product_id) && d->interface_number == 0) {
        seenWired.insert(path);
        if (!first && !wired_.count(path)) {
          const std::string serial = Narrow(d->serial_number);
          const std::string config =
              serial.empty() ? std::string() : lh::usb::UsbTransport::LoadSteamConfig(lh::platform::SteamConfigDirectory(), serial);
          if (WiredPlugInWakes(config)) {
            Wake(d->product_id == 0x2000 ? TrackedDeviceClass_HMD : TrackedDeviceClass_Controller);
          } else if (VRDriverLog()) {
            VRDriverLog()->Log(("lh: [INFO] watchdog: " + serial + " plugged in, a controller or tracker charging: not starting SteamVR\n").c_str());
          }
        }
      }
    }
    hid_free_enumeration(list);
    wired_ = seenWired;
    for (auto it = receivers_.begin(); it != receivers_.end();) {
      if (!seenReceivers.count(it->first)) {
        hid_close(it->second.dev);
        it = receivers_.erase(it);
        continue;
      }
      const auto serial = lh::usb::UsbTransport::ConnectedSerial(it->second.dev);
      if (serial) {
        const bool connected = *serial != 0;
        if (connected && !it->second.connected) Wake(TrackedDeviceClass_Controller);
        it->second.connected = connected;
      }
      ++it;
    }
  }

  void Wake(ETrackedDeviceClass cls) {
    // One wake-up per 10 s: SteamVR is starting, and closes the watchdog when it has.
    const auto now = std::chrono::steady_clock::now();
    if (now - lastWake_ < std::chrono::seconds(10)) return;
    lastWake_ = now;
    if (VRWatchdogHost()) VRWatchdogHost()->WatchdogWakeUp(cls);
  }

  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::map<std::string, Receiver> receivers_;  // by HID path
  std::set<std::string> wired_;
  std::chrono::steady_clock::time_point lastWake_{};
};

Watchdog g_watchdog;

}  // namespace

IVRWatchdogProvider* WatchdogProvider() { return &g_watchdog; }

}  // namespace lh::openvr
