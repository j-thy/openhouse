// USB/HID transport: finds Valve Lighthouse devices with hidapi, reads their HID
// reports on one thread (like CLighthouseUsbThread, 0x3cb310) and pushes decoded
// data into an IInputSink. Optionally tees raw reports to a RawCaptureWriter.
//
// This is the only component that touches hardware; the tracking library only
// sees IInputSink calls.
#pragma once

#include <atomic>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

#include "lh/input_sink.hpp"
#include "radio_mode.hpp"
#include "raw_capture.hpp"
#include "report_parser.hpp"

struct hid_device_;
struct hid_device_info;

namespace lh::usb {

struct DeviceCommandResult {
  DeviceId serial;
  bool powerOff = false;
  const char* status = "not-connected";  // written / expired / write-failed / not-connected
  int bytes = 0;                     // HID return count; not a motor acknowledgement
  std::array<uint8_t, 64> report{};
  int opticalMode = -1;               // >= 0: a light-mode (optical sub-protocol) command
};

struct UsbTransportOptions {
  // Where to find SteamVR's cached device configs (lhr-xxxxxxxx/config.json).
  std::string steamLighthouseConfigDir;  // empty = $HOME/.local/share/Steam/config/lighthouse
  std::string capturePath;               // non-empty: write .lhraw capture
  // Called on the reader thread after dispatch. Never called with a transport lock held.
  std::function<void(const DeviceCommandResult&)> onCommandResult;
  bool fetchConfigIfMissing = true;      // feature 0x10/0x11 + zlib; cache remains preferred
};

// Decodes a stream of raw reports (live or from a capture) into an IInputSink.
// Keeps per-device state (tick unwrapping, clock sync, range modes).
class ReportDecoder {
 public:
  explicit ReportDecoder(IInputSink* sink) : sink_(sink) {}
  void AddEndpoint(const EndpointInfo& e);
  // An unplugged endpoint: its parser state goes (the device's shared state stays, so
  // the same device on a replugged receiver continues under the same id).
  void RemoveEndpoint(uint16_t ep) { eps_.erase(ep); }
  void OnReport(const RawReport& r);
  void OnReport(double hostTime, uint16_t ep, bool last, const uint8_t* data, size_t len);
  uint64_t WirelessGeneration(uint16_t ep) const {
    auto it = eps_.find(ep);
    return it == eps_.end() ? 0 : it->second.generation;
  }
  void SetVersionNegotiation(uint16_t ep) { eps_.at(ep).versioning = true; }
  WirelessReportParser* Wireless(uint16_t ep) {
    auto it = eps_.find(ep);
    return it == eps_.end() ? nullptr : it->second.wireless.get();
  }

 private:
  struct Ep {
    EndpointInfo info;
    double lastWirelessReport = 0;
    bool connected = true, versioning = false;
    uint64_t generation = 0;
    std::unique_ptr<ImuReportParser> imu;
    std::unique_ptr<OpticalReportParser> optical;
    std::unique_ptr<ButtonsReportParser> buttons;  // EndpointKind::Buttons
    std::unique_ptr<WirelessReportParser> wireless;
  };
  void ResetWireless(Ep& ep);
  IInputSink* sink_;
  std::map<std::string, std::shared_ptr<DeviceState>> devices_;
  std::map<uint16_t, Ep> eps_;
};

class UsbTransport {
 public:
  UsbTransport(IInputSink* sink, UsbTransportOptions opts = {});
  ~UsbTransport();

  // Enumerates and opens devices, starts the reader thread.
  bool Start();
  void Stop();

  std::vector<EndpointInfo> Endpoints() const;
  bool CaptureHealthy() const { return capture_.Healthy(); }
  // Starts (a path) or stops (empty) a .lhraw capture while running; every open device is
  // written first, so the file replays like one started with the session.
  void SetCapture(const std::string& path);
  // Queued to the HID reader: never issue feature reports from host callbacks.
  void Haptic(const DeviceId& serial, double frequency, double amplitude, double duration);
  void PowerOff(const DeviceId& serial);
  // Ask a device (wireless or wired) for LH1 (0) or LH2 (1) light reports.
  void SetOpticalSubProtocol(const DeviceId& serial, int sub);

  // Wireless receivers (dongles) plugged in now, and the device connected to each
  // ("" if none). Any thread.
  struct Receiver { std::string serial, connected; uint16_t pid = 0; };
  std::vector<Receiver> Receivers() const;
  // The device a receiver was carrying has gone (wireless dropout, powered off). Until this
  // existed, `receivers_[dongle].connected` kept the old serial: SteamVR kept showing the
  // dongle as occupied, its pairing page insisted a controller was still connected, and the
  // only way to pair another was to unplug the receiver.
  void OnDeviceGone(const DeviceId& serial);
  // Puts a receiver into pairing mode for `milliseconds` (0: leaves pairing mode), as
  // stock 0x3ce9e0 / 0x3cea40: feature 0xff "ad 03 <on> <ms u16>". A receiver that has a
  // device is reopened afterwards (its device is replaced). Any thread; false if no such
  // receiver is plugged in.
  bool SetPairing(const std::string& receiver, int milliseconds);

  // The receiver with Bluetooth, for SteamVR's base station power management
  // (radio_mode.hpp). Any thread. The two below wait for the HID threads to run the command
  // (at most a couple of seconds, if the receiver goes away): not for tight loops.
  bool HasBluetoothReceiver() const;
  // Switches its radio; on: checks that it took, and goes back to Valve's mode if it didn't
  // (stock 0x2cb80). False if there is no such receiver or it failed.
  bool SetBluetoothRadio(bool on);
  // Whether its radio is in Bluetooth mode; nullopt if there is none or it didn't answer.
  std::optional<bool> BluetoothRadioOn();

  // The radio serial of the device connected to an open receiver (dongle), 0 if none, or
  // nullopt if it didn't answer. A short query, for the watchdog (watchdog.cpp).
  static std::optional<uint32_t> ConnectedSerial(hid_device_* receiver);

  // Loads a cached SteamVR config for a serial ("LHR-0704E07A"), or "".
  static std::string LoadSteamConfig(const std::string& dir, const std::string& serial);

 private:
  struct Open {
    EndpointInfo info;
    hid_device_* dev = nullptr;
    double versionRequestTime = 0;  // pending wireless protocol request
    bool disconnected = false;
    bool wantVersion = false;
    uint64_t generation = 0;
    std::string receiver;  // dongle serial for wireless devices
    int sensorType = -1;   // hardware id low byte (report 0x05), -1 unknown
    std::string path;      // hidapi path, unique while plugged in
    bool bluetooth = false;  // receiver's radio in Bluetooth mode: no device can connect
    int keepalive = -1;      // last keepalive to a receiver with no device: 1 sent, 0 failed, -1 none yet
    // A receiver can stay plugged in while another device paired to it connects (#92): its first
    // data after a disconnect or a radio gap is checked against the device it was set up for
    // (OtherDeviceConnected). lastData: host time of its last data report.
    // The check runs on the probe thread (DeviceCheck), so it never holds up this one.
    bool checkDevice = false;
    bool checkPending = false;  // a DeviceCheck is queued or running
    double lastData = 0;
    // A receiver that doesn't answer the check is asked again until checkUntil, from nextCheck.
    double checkUntil = 0, nextCheck = 0;
  };
  enum class OpenResult { Failed, Idle, Ready };
  // Opens and sets up one enumerated HID interface (Start and hotplug).
  OpenResult OpenEndpoint(const ::hid_device_info* d, Open* out);
  void Enumerate(bool startup);  // opens interfaces not open yet
  void CompleteSetup(Open& o, bool supportsVersioning);
  void RequestVersion(Open& o);
  void Run();
  void ProbeMain();  // hotplug: new interfaces, and receivers with no device connected yet
  static bool FetchRangeModes(hid_device_* dev, int8_t& gyro, int8_t& accel);
  struct DeviceInfo {
    uint32_t serial = 0;
    bool supportsVersioning = false;
  };
  // 0x3dd8a0: feature 0x83 TLV device info.
  // attempts: how long to wait for a receiver that has no device yet (each ~55 ms).
  static bool QueryDeviceInfo(hid_device_* dev, DeviceInfo* info, int attempts = 20);
  static bool RequestProtocolVersion(hid_device_* dev, int version);

#if defined(__GNUC__) || defined(__clang__)
  void Log(LogLevel level, const char* format, ...) __attribute__((format(printf, 3, 4)));
#else
  void Log(LogLevel level, const char* format, ...);
#endif
  IInputSink* sink_;
  UsbTransportOptions opts_;
  ReportDecoder decoder_;
  RawCaptureWriter capture_;
  std::vector<Open> open_;
  // Receivers whose wireless device is not connected yet (owned by the probe thread)
  // and endpoints the probe thread finished setting up (handed to the reader thread).
  std::vector<Open> idle_;
  std::vector<Open> pending_;
  std::mutex pendingMutex_;
  std::atomic<bool> havePending_{false};
  std::mutex captureMutex_;
  std::optional<std::string> captureRequest_;
  std::atomic<bool> haveCaptureRequest_{false};
  std::thread probe_;
  // Paths of every interface open (or idle) now; the reader thread drops a path when
  // its interface goes away, so a replugged receiver is opened again.
  std::set<std::string> paths_;
  std::set<std::string> failedPaths_;  // logged once (e.g. held open by SteamVR)
  std::mutex pathsMutex_;
  std::atomic<uint16_t> nextEp_{0};
  std::thread thread_;
  std::atomic<bool> stop_{false};
  int wirelessProtocol_ = 1;
  struct HapticCommand {
    std::string serial; std::array<uint8_t, 64> report; double deadline; bool powerOff = false;
    int opticalMode = -1;  // light-mode command: wired devices take another report (below)
  };
  std::mutex hapticMutex_;
  std::vector<HapticCommand> haptics_;
  struct PairCommand { std::string receiver; int milliseconds; };
  std::vector<PairCommand> pairing_;  // under hapticMutex_; read by the reader and probe threads
  // Run a pending pairing command for `o`'s receiver; true if one was sent.
  bool TakePairing(Open& o);
  // The stock keepalive to a receiver with no device, each probe pass (logged when it changes).
  void KeepAlive(Open& o);
  // #92: which device a wireless receiver has, asked by the probe thread on a handle of its own
  // (the query can take ~0.1 s and the reader thread serves every device). path: the receiver's
  // interface; serial: the device it was set up for; result: 1 another device, 0 the same,
  // -1 no answer, unset while queued.
  struct DeviceCheck {
    std::string path, serial, receiver;
    std::optional<int> result;
  };
  std::vector<std::shared_ptr<DeviceCheck>> deviceChecks_;  // under hapticMutex_
  std::unordered_map<std::string, std::shared_ptr<DeviceCheck>> runningChecks_;  // reader thread only, by path
  void RunDeviceChecks();  // probe thread
  int OtherDeviceConnected(const std::string& path, const std::string& serial, const std::string& receiver);
  // A radio command for the thread that owns the receiver. set: the mode to switch to, or
  // nullopt to read it. result: the mode set or read, nullopt if that failed.
  struct RadioCommand {
    std::string receiver;
    std::optional<RadioMode> set;
    std::promise<std::optional<RadioMode>> result;
  };
  std::vector<std::shared_ptr<RadioCommand>> radioCommands_;  // under hapticMutex_
  std::string BluetoothReceiver() const;
  std::optional<RadioMode> RunRadioCommand(const std::string& receiver, std::optional<RadioMode> set);
  void TakeRadioCommands(Open& o);  // runs the ones for `o`'s receiver, on its owner's thread
  // Configs read from the devices this session (serial -> JSON): a reconnect reuses it.
  std::mutex configMutex_;
  std::map<std::string, std::string> sessionConfigs_;
  mutable std::mutex receiversMutex_;
  std::map<std::string, Receiver> receivers_;
  void SetReceiver(const Open& o, const std::string& connected);
  void DropReceiver(const std::string& serial);
};

}  // namespace lh::usb
