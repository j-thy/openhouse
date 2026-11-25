// lh::Session — the public face of the lh driver runtime (liblh).
//
// A Session owns one tracking universe: it reads devices (live USB or a recorded
// .lhraw capture), runs one tracker per device on top of a room map that every
// device shares, decodes controller input, and keeps the room map on disk.
//
// Frontends (the 3D visualizer, lhtool, and later the SteamVR driver) only talk to
// this header. Everything here is plain data; no Eigen/transport types leak out.
//
// Thread safety: lifecycle calls (Start/Stop/destruction) are owner-thread
// operations. While running, data/control methods may be called concurrently.
// Snapshot() returns a consistent copy; PollEvents() drains the event queue.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lh/log_level.hpp"

namespace lh {

struct Pose {
  std::array<double, 3> position{};                 // meters
  std::array<double, 4> orientation{1, 0, 0, 0};    // w, x, y, z
};

// Role a device is exposed as. Taken from the device JSON `device_class`; physical
// hardware never overrides it (an Index controller configured as generic_tracker is
// a Tracker).
enum class DeviceRole { Hmd, Controller, Tracker, BaseStation };

enum class TrackingState {
  Disconnected,   // no data from the device
  Uninitialized,  // connected, never had a pose
  Acquiring,      // has a pose in its base frame; world frame not known yet
  Tracking,       // optical updates within the last 0.25 s
  Coasting,       // IMU-only for a short time (pose still valid)
  Lost,           // tracking reset; waiting for light
};

const char* ToString(DeviceRole);
const char* ToString(TrackingState);

// SteamVR ETrackingResult equivalents, for the future driver wrapper.
enum class TrackingResult : int {
  Uninitialized = 1,
  CalibratingInProgress = 100,
  CalibratingOutOfRange = 101,
  RunningOk = 200,
  RunningOutOfRange = 201,
};

struct DevicePose {
  double hostTime = 0;          // seconds, host monotonic clock of the pose sample
  bool valid = false;           // usable world pose
  TrackingState state = TrackingState::Uninitialized;
  TrackingResult result = TrackingResult::Uninitialized;
  Pose worldFromImu;            // filter origin (IMU)
  Pose worldFromHead;           // config "head" frame (HMD eyes center / controller grip-ish)
  Pose imuFromHead;             // constant per device (DriverPose qDriverFromHead*)
  std::array<double, 3> velocity{};         // world, m/s
  std::array<double, 3> angularVelocity{};  // world, rad/s
  std::array<double, 3> acceleration{};     // world, m/s^2 (0 unless reported)
  uint32_t primaryBase = 0;     // base whose frame the filter runs in
  std::vector<uint32_t> bases;  // all bases in the filter
  // Light used in the last 0.5 s, per base: frames and the mean sensors lit per frame
  // (coverage: how well each base sees the device right now).
  struct Coverage { uint32_t base = 0; int frames = 0; double sensors = 0; };
  std::vector<Coverage> coverage;
  uint64_t updates = 0;         // optical updates since last (re)acquire
  // Diagnostics: the filter's own pose (last optical/inertial update, before the
  // output stage's prediction and smoothing) and its host time.
  std::array<double, 3> filterPosition{};
  double filterTime = 0;
};

// Decoded controller and tracker inputs. A tracker power button reports systemClick.
struct ControllerInput {
  double hostTime = 0;
  // Buttons (pressed).
  bool systemClick = false, aClick = false, bClick = false, triggerClick = false;
  bool thumbstickClick = false, trackpadTouch = false;
  bool trackpadClick = false, gripClick = false, menuClick = false;  // Vive wand
  // Capacitive touch.
  bool systemTouch = false, aTouch = false, bTouch = false, triggerTouch = false, thumbstickTouch = false;
  double trigger = 0;                 // 0..1
  double thumbstickX = 0, thumbstickY = 0;  // -1..1
  double trackpadX = 0, trackpadY = 0;      // -1..1 (valid while trackpadTouch)
  double trackpadForce = 0;           // 0..1
  double gripForce = 0;               // 0..1
  // Finger proximity / curl, 0 = open .. 1 = touching.
  double fingerIndex = 0, fingerMiddle = 0, fingerRing = 0, fingerPinky = 0;
  // Mapped controllers (ControllerType::Mapped): the latest raw input record, as the
  // device's own input profile lays it out (its "input_components" bit offsets).
  std::vector<uint8_t> mapped;
};

// Which controller a Controller is (from its config); decides its inputs and profile.
// Mapped: any other maker's controller whose firmware sends "mapped input" (config
// firmware_config.mapped_input), e.g. Shiftall's FlipVR: its own input profile names
// each component's bits in the raw record.
enum class ControllerType { Index, ViveWand, Mapped };

struct DeviceInfo {
  bool IsTundraTracker() const { return manufacturer == "Tundra Labs" && model == "Tundra Tracker"; }

  std::string serial;           // "LHR-13DA81D1", or "LHB-A101B835" for bases
  DeviceRole role = DeviceRole::Tracker;
  std::string deviceClass;      // raw config device_class
  ControllerType controllerType = ControllerType::Index;  // Controller role only
  std::string model, manufacturer, renderModel, controllerRole;  // "left_hand"/"right_hand"
  std::string resourceRoot, inputProfile, registeredType;  // config resource_root, input_profile, registered_device_type
  bool connected = false;
  bool enabled = true;          // disabled devices are listed but ignored entirely
  std::optional<int> batteryPercent;
  bool charging = false;
  std::string configJson;       // full device JSON (empty for bases)
  // Sensor geometry in the IMU frame (for visualization).
  std::vector<std::array<double, 3>> sensors;
};

struct BaseStationState {
  std::string serial;           // "LHB-XXXXXXXX"
  uint32_t id = 0;
  bool seen = false;            // OOTX received this session
  bool known = false;           // pose known in the room map
  Pose worldFromBase;
  double positionVariance = 0, rotationVariance = 0;
  int mode = 0;                 // 1 A, 2 B, 3 C
  bool lh2 = false;             // a 2.0 base station (its light is LH2 words)
  double channelObservedAt = -1;  // host time of the last directly decoded LH2 OOTX
  std::string channel;          // as its display shows it: "A" "b" "c", lhbs multi "1".."4"; "" unknown
  int faults = 0;
  bool origin = false;
  bool locked = false;          // placed well: only its own accelerometer moves it now
  int observations = 0;         // keyframes behind its pose (this session or saved)
  // Learned: placed well enough to track and align against (the origin; under 5 mm;
  // locked or 40+ keyframes under 20 mm). What "settled" and every reader go by.
  bool learned = false;
  double sigma = 0;             // m, position uncertainty (total, 0 for the origin)
  static constexpr double kLearnedSigma = 0.005, kLearnedLockedSigma = 0.02;
  static constexpr int kLearnedObservations = 40;
};

struct DeviceState {
  DeviceInfo info;
  DevicePose pose;
  std::optional<ControllerInput> input;
};

// Per-device data that changes every sample (no config/geometry copies): what a
// high-rate consumer such as the SteamVR driver polls.
struct DeviceUpdate {
  std::string serial;
  DeviceRole role = DeviceRole::Tracker;
  bool connected = false, enabled = true;
  std::optional<int> batteryPercent;
  bool charging = false;
  DevicePose pose;
  std::optional<ControllerInput> input;
};

struct SessionSnapshot {
  double hostTime = 0;
  std::vector<DeviceState> devices;
  std::vector<BaseStationState> bases;
  uint64_t roomRevision = 0;
  std::string roomId;
  uint64_t originEpoch = 0;  // bumps when a knocked origin base was re-placed in the room
};

enum class SessionEventType {
  DeviceConnected, DeviceDisconnected, DeviceActivated, DeviceEnabledChanged,
  TrackingStateChanged, BaseStationSeen, BaseStationLocated, RoomChanged, Log,
};

struct SessionEvent {
  double hostTime = 0;
  SessionEventType type = SessionEventType::Log;
  std::string serial;           // device or base
  std::string text;
  LogLevel level = LogLevel::Info;
};

// Not stock: 1.0 and 2.0 base stations in one room, used together (ai/driver/mixed_stations.md).
// Devices that do switch to LH1 mode, where they see both kinds, and their 2.0 stations' hits are
// picked out of the LH1 pulses; now and then they visit LH2 mode to time the 2.0 stations again.
// Off by default: only for rooms with both kinds.
struct MixedLightOptions {
  bool enabled = false;
  // Which kinds of device read both; the others stay on 2.0 light (LH2 mode), as the headset
  // always does. Trackers are everything that isn't a headset or a controller.
  bool controllers = true, trackers = true;
  // The headset, in LH2 mode, times each 2.0 station for the others, so they visit LH2 mode half as
  // often while it covers every station they use.
  bool scout = true;
  // On (a test): a device stays in LH2 mode while the 2.0 stations light it well, and reads the 1.0
  // stations (LH1 mode) only while they don't. In LH2 mode a hit carries its own angle, and a tracker
  // lying still swayed a third as much (runtime/mixed_light.hpp, Lh2LightEnough).
  bool preferLh2 = false;
  // The devices in LH2 mode (the headset, mostly) measure each 2.0 rotor's timing wobble from its beam
  // words, and the devices in LH1 mode take it off their sorted hits of that rotor
  // (runtime/rotor_reference.hpp). It's the station's own, the same for every device that sees it.
  bool rotorReference = true;
  bool operator==(const MixedLightOptions&) const = default;
};

struct SessionOptions {
  // Input: a .lhraw capture (replay) or, when empty, live USB devices.
  std::string replayPath;
  bool replayRealtime = true;   // pace replay by recorded host time
  // Replay window, in seconds from the capture's first report (end <= 0: to the end).
  // Reports before the window are skipped, so devices start fresh at its start.
  double replayFrom = 0, replayTo = 0;
  // Replay only these devices (serials); empty: all. Others' reports are dropped
  // before decoding, so a focused replay costs only what that device costs.
  std::vector<std::string> replayOnly;
  // Replay: every device's pose written here (JSON Lines) at this rate of *capture*
  // time, from the replay thread, however fast the replay runs.
  std::string replayTracePath;
  double replayTraceRate = 100;
  // Room map persistence. Empty = $XDG_CONFIG_HOME/lh/roommap.json.
  std::string roomMapPath;
  bool saveRoomMap = true;
  // SteamVR lighthousedb.json to import rooms from when our map has none for the
  // visible bases. Empty = auto-detect; "-" = disabled.
  std::string steamDatabasePath;
  // SteamVR cached device configs (lhr-*/config.json) for live USB. Empty = auto.
  std::string steamConfigDir;
  // Devices to ignore (serials, e.g. "LHR-0704E07A"). Merged with the persisted
  // settings ($XDG_CONFIG_HOME/lh/settings.json) for live sessions.
  std::vector<std::string> disabledDevices;
  // Settings file; empty = default for live sessions, none for replays.
  std::string settingsPath;
  // Device JSON configs (*.json, e.g. from `lhtool configs`) of devices to create at start,
  // disconnected until they connect: hosts that only take devices at startup (Monado) then
  // still get one turned on later. A device whose own config differs updates its file.
  // Empty = $XDG_CONFIG_HOME/lh/preinit for live sessions, none for replays; "-" = off.
  std::string preinitDirectory;
  // Live USB: also record every raw report to this .lhraw (replayable).
  std::string capturePath;
  // 1.0 and 2.0 base stations together, from the start (SetMixedLight changes it). In a replay
  // it sorts the recorded LH1 pulses; the light modes are the recording's.
  MixedLightOptions mixedLight;
  // Replay: capture times (s, as replayFrom) at which that switch flips, as it did live.
  std::vector<double> replayMixedSwitch;
};

class Session {
 public:
  static std::unique_ptr<Session> Create(const SessionOptions& options = {});
  virtual ~Session() = default;

  virtual bool Start(std::string* error = nullptr) = 0;
  virtual void Stop() = 0;
  virtual bool Running() const = 0;
  // Replay only: true once the whole capture has been processed.
  virtual bool Finished() const = 0;

  virtual SessionSnapshot Snapshot() const = 0;
  virtual std::vector<DeviceUpdate> Updates() const = 0;
  // Monotonic host time the poses refer to (steady clock seconds for live sessions).
  virtual double HostNow() const = 0;
  virtual std::vector<SessionEvent> PollEvents() = 0;

  // Enable/disable a device by serial. Disabled devices produce no poses, input or
  // room-map contributions. Persisted in the settings file when there is one.
  virtual void SetDeviceEnabled(const std::string& serial, bool enabled) = 0;
  // Output tuning, for every device, applied at once: how long corrections take to show
  // (glide time constants, s; 0 = at once) and whether poses carry the acceleration for
  // the host to predict with. Defaults 0.06 s, 0.06 s, true.
  virtual void SetOutputTuning(double positionSmoothing, double rotationSmoothing, bool reportAcceleration) = 0;
  // Applied at once: devices that stop reading both kinds of base station go back to the
  // usual choice of light mode, and those that start go to LH2 mode to time the 2.0 stations.
  virtual void SetMixedLight(const MixedLightOptions& options) = 0;
  // What's in force: settings.json "mixedLight" (or SessionOptions) until SetMixedLight.
  virtual MixedLightOptions MixedLight() const { return {}; }
  // settings.json "hideLostDevices": a lost controller or tracker is reported as lost (the
  // host hides it) instead of held where it was last seen. Off by default.
  virtual bool HideLostDevices() const { return false; }

  // Wireless receivers (dongles), live USB only: serial, and the device paired and
  // connected to it now ("" if none).
  // Link stats are live estimates over the last second; -1 / 0 where the receiver can't tell.
  // A Doorman Dongle appears once per paired slot ("OHDG-<id>/<slot>"), like a dongle each.
  struct Receiver {
    std::string serial, connected;
    std::string kind;            // "watchman" (Valve/HTC dongle) or "openhouse" (one slot of ours)
    int slot = -1;               // openhouse slot, -1 for a Watchman dongle
    bool paired = true;          // openhouse: a device is paired to the slot (empty slots are listed too)
    bool linked = false;         // a device is talking through it now
    double kBytesPerSec = 0;     // payload from the device
    double framesPerSec = 0;     // reports (Watchman) or radio frames (openhouse)
    double lossPercent = -1;     // radio polls without a good answer (openhouse)
    double dropsPerSec = 0;      // frames lost on the dongle (openhouse)
    int rssiDongle = 0, rssiDevice = 0;   // dBm (openhouse), 0 unknown
    int devicesOnDongle = 0;     // openhouse: devices joined on the whole dongle
    std::string name;            // openhouse: the user's name for the slot's device, kept on the dongle
    bool keepsName = false;      // openhouse: the dongle keeps names (firmware 0x0025+)
  };
  virtual std::vector<Receiver> Receivers() const = 0;
  // Stores a name on an openhouse slot ("OHDG-.../3"); false if there's no such slot.
  virtual bool SetReceiverName(const std::string& receiver, const std::string& name) = 0;
  // Pairing mode on a receiver for `seconds` (0 leaves it); false if not plugged in.
  virtual bool SetPairing(const std::string& receiver, double seconds) = 0;
  // Live USB: the radio of the receiver with Bluetooth, for SteamVR's base station power
  // management. Whether there is one, switching it (false if it failed) and reading it
  // (nullopt if it didn't answer). The last two can block for a moment.
  virtual bool HasBluetoothRadio() const = 0;
  virtual bool SetBluetoothRadio(bool on) = 0;
  virtual std::optional<bool> BluetoothRadioOn() = 0;
  // Live USB: starts recording every raw report to this .lhraw (replayable with
  // `lhtool session`), or stops with an empty path. False when there is nothing to record.
  virtual bool SetCapture(const std::string& path) = 0;
  // Index only. Returns true when queued, not a hardware acknowledgement.
  // Replay/disabled devices return false. Frequency 0 selects a default.
  virtual bool Haptic(const std::string& serial, double frequency, double amplitude, double duration) = 0;
  // Queue the wireless device's hardware power-off command. Does not disable it
  // in settings; powering it on again can resume the existing session.
  virtual bool PowerOff(const std::string& serial) = 0;

  // Forget the room map (all base poses) and relearn it.
  // Forgets every room (as if the room map file were deleted) and relearns from scratch;
  // the file is rewritten. Also done when the room map file is deleted while running.
  virtual void ResetRoom() = 0;
  // Bumps on every ResetRoom (readers that hold anything room-relative drop it).
  virtual uint64_t RoomResets() const = 0;
  virtual bool SaveRoom(std::string* error = nullptr) = 0;
};

}  // namespace lh
