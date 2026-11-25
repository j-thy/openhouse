// Drop-in OpenVR driver ("lighthouse") on top of liblh.
//
// Replaces SteamVR's drivers/lighthouse/bin/linux64/driver_lighthouse.so for tracked
// devices: Index controllers (with input), trackers and base stations. Head-mounted
// displays are intentionally not exposed (no display/direct-mode support).
//
// Hosts: SteamVR (vrserver) and Monado's steamvr_lh driver. For Monado:
//   * devices are only accepted during its discovery window after Init
//     (LH_DISCOVER_WAIT_MS), so devices are registered as soon as their config is read;
//   * Activate() runs inside TrackedDeviceAdded(): no driver locks are held then;
//   * angular velocity is reported in the device (IMU) frame, like the stock driver;
//   * poses are in our room's frame (origin base, gravity up), the same every session;
//     LH_HOST_FRAME=steamvr puts them in SteamVR's raw frame of lighthousedb.json's
//     universe instead (what Monado's chaperone transform expects without an aligner);
//   * the room is published for alignment tools (room_publisher.hpp, lh/room_shm.h).
//
// Environment: LH_REPLAY=/path/input.lhraw replays a capture instead of live USB.
//   LH_CAPTURE=/path/dir records a diagnostic session while running in the host:
//   input.lhraw (raw USB, replayable with lhviz/lhtool), driver.jsonl (every pose
//   handed to the host, haptic events, session events) and lighthousedb.json.
#include <openvr_driver.h>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "device_abi.hpp"
#include "pairing_mailbox.hpp"
#include "stock_hmd.hpp"
#include "watchdog.hpp"

namespace {
#ifdef _WIN32
constexpr const char* kStockFile = "driver_lighthouse.dll.stock";
#else
constexpr const char* kStockFile = "driver_lighthouse.so.stock";
#endif
}  // namespace
#include "lh/session.hpp"
#include "hand_skeleton.hpp"
#include "runtime/mapped_input.hpp"
#include <nlohmann/json.hpp>
#include "raw_alignment.hpp"
#include "steam_rooms.hpp"
#include "room_publisher.hpp"
#include "telemetry_server.hpp"
#include "tracker_names.hpp"
#include "lh/log_level.hpp"
#include "lh/build_time.h"
#include "lh/telemetry.hpp"
#include "lh/version.hpp"
#include "lh/bug_report.hpp"
#include "host_frame.hpp"
#include "haptic_routing.hpp"
#include <set>
#include "platform/platform.hpp"
#include <ctime>
#include <filesystem>
#include <functional>
#include <fstream>
#include <sstream>
#ifdef _WIN32
#include <windows.h>
#endif
#include <deque>

namespace {

using namespace vr;


double SteadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool EqualsNoCase(const std::string& a, const char* b) {
  const size_t n = std::strlen(b);
  if (a.size() != n) return false;
  for (size_t i = 0; i < n; ++i)
    if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i]))) return false;
  return true;
}

// Buttons, axes and haptics. The stock driver asks for IVRDriverInput_005, whose first
// slots (Create/UpdateBoolean, CreateScalar, CreateHaptic) are laid out as in _004.
// Windows SteamVR accepted booleans sent through _004 but never applied them (no button
// presses, no dashboard; grip/thumbstick/trackpad touch refused as invalid or wrong type).
IVRDriverInput* ComponentInput() {
  static IVRDriverInput* input = [] {
    EVRInitError e = VRInitError_None;
    auto* v5 = static_cast<IVRDriverInput*>(VRDriverContext()->GetGenericInterface("IVRDriverInput_005", &e));
    if (v5 && e == VRInitError_None) return v5;
    return VRDriverInput();
  }();
  return input;
}

// LH_CAPTURE's driver.jsonl: what the host was given, on the session's host clock.
class Recorder {
 public:
  bool Open(const std::string& path) {
    std::lock_guard lock(mutex_);
    file_ = std::fopen(path.c_str(), "w");
    return file_ != nullptr;
  }
  void Close() {
    std::lock_guard lock(mutex_);
    if (file_) std::fclose(file_);
    file_ = nullptr;
  }
  bool On() const { return file_ != nullptr; }
  void Line(const std::string& json) {
    std::lock_guard lock(mutex_);
    if (!file_) return;
    std::fputs(json.c_str(), file_);
    std::fputc('\n', file_);
    if (++lines_ % 20 == 0) std::fflush(file_);  // a killed host keeps nearly everything
  }
  static std::string Quote(const std::string& text) {
    std::string out = "\"";
    for (char c : text) {
      if (c == '"' || c == '\\') { out += '\\'; out += c; }
      else if (uint8_t(c) < 0x20) out += ' ';
      else out += c;
    }
    return out + "\"";
  }
 private:
  std::mutex mutex_;
  std::FILE* file_ = nullptr;
  uint64_t lines_ = 0;
};
Recorder g_recorder;

// Every message goes to the viewers (they filter for themselves); the host's log gets
// Info and up unless LH_LOG_LEVEL says otherwise.
lh::openvr::TelemetryServer g_telemetry;
const lh::LogLevel g_logLevel = lh::MinimumLogLevelFromEnvironment();

// Always on, as SteamVR's vrserver.txt: <data dir>/logs/openhouse.txt, the run before as
// openhouse.previous.txt. Info and up (or what LH_LOG_LEVEL asks for, if lower), whatever the host keeps.
class LogFile {
 public:
  void Line(lh::LogLevel level, const std::string& text, const std::string& serial) {
    std::lock_guard lock(mutex_);
    if (!tried_) Open();
    if (!out_) return;
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    const int ms = int(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof stamp, "%Y-%m-%d %H:%M:%S", &tm);
    char msPart[8];
    std::snprintf(msPart, sizeof msPart, ".%03d", ms);
    out_ << stamp << msPart << " [" << lh::LogLevelName(level) << "] " << (serial.empty() ? "" : serial + ": ") << text << '\n';
    out_.flush();   // a crashed vrserver keeps every line
    if (++lines_ % 1000 == 0 && out_.tellp() > kMaxBytes) { out_.close(); Open(); }   // a run that logs without end
  }
 private:
  static constexpr std::streamoff kMaxBytes = 64ll << 20;
  void Open() {
    tried_ = true;
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path dir = fs::u8path(lh::platform::DataDirectory()) / "logs";
    fs::create_directories(dir, ec);
    fs::rename(dir / "openhouse.txt", dir / "openhouse.previous.txt", ec);
    out_.open(dir / "openhouse.txt", std::ios::out | std::ios::trunc);
  }
  std::mutex mutex_;
  std::ofstream out_;
  bool tried_ = false;
  uint64_t lines_ = 0;
};
LogFile g_logFile;

void Log(lh::LogLevel level, const std::string& text, const std::string& serial = {}, double time = SteadyNow()) {
  g_telemetry.Log(time, level, serial, text);
  if (level >= std::min(g_logLevel, lh::LogLevel::Info)) g_logFile.Line(level, text, serial);
  if (level >= g_logLevel && VRDriverLog())
    VRDriverLog()->Log(("lh: [" + std::string(lh::LogLevelName(level)) + "] " + text + "\n").c_str());
}
void Log(const std::string& text) { Log(lh::ClassifyLog(text), text); }

// A driver resource ("{shiftall}/input/flipvr_controller_vc1b_profile.json") as text, ""
// if SteamVR can't find it: another driver's resources (a resource-only driver such as
// Shiftall's) resolve through SteamVR, not a path of ours.
std::string ReadResource(const std::string& name) {
  if (name.empty() || !VRResources()) return {};
  char path[4096]{};
  VRResources()->GetResourceFullPath(name.c_str(), "", path, sizeof path);
  if (!*path) return {};
  std::ifstream in(std::filesystem::u8path(path), std::ios::binary);
  if (!in) return {};
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// A maker's status icons for a model, from its driver.vrresources "statusicons" (as the
// stock driver shows a FlipVR with Shiftall's icons).
std::vector<std::pair<ETrackedDeviceProperty, std::string>> StatusIcons(const std::string& root, const std::string& model) {
  static const std::pair<const char*, ETrackedDeviceProperty> kProps[] = {
      {"Prop_NamedIconPathDeviceOff_String", Prop_NamedIconPathDeviceOff_String},
      {"Prop_NamedIconPathDeviceSearching_String", Prop_NamedIconPathDeviceSearching_String},
      {"Prop_NamedIconPathDeviceSearchingAlert_String", Prop_NamedIconPathDeviceSearchingAlert_String},
      {"Prop_NamedIconPathDeviceReady_String", Prop_NamedIconPathDeviceReady_String},
      {"Prop_NamedIconPathDeviceReadyAlert_String", Prop_NamedIconPathDeviceReadyAlert_String},
      {"Prop_NamedIconPathDeviceNotReady_String", Prop_NamedIconPathDeviceNotReady_String},
      {"Prop_NamedIconPathDeviceStandby_String", Prop_NamedIconPathDeviceStandby_String},
      {"Prop_NamedIconPathDeviceAlertLow_String", Prop_NamedIconPathDeviceAlertLow_String}};
  std::vector<std::pair<ETrackedDeviceProperty, std::string>> out;
  if (root.empty()) return out;
  try {
    const std::string text = ReadResource("{" + root + "}/driver.vrresources");
    if (text.empty()) return out;
    const auto j = nlohmann::json::parse(text, nullptr, true, true);
    if (!j.contains("statusicons") || !j.at("statusicons").contains(model)) return out;
    const auto& icons = j.at("statusicons").at(model);
    for (const auto& [name, prop] : kProps)
      if (icons.contains(name) && icons.at(name).is_string()) out.emplace_back(prop, icons.at(name).get<std::string>());
  } catch (const std::exception&) {
  }
  return out;
}

// The process hosting the driver (vrserver, monado-service, wivrn-server, ...).
std::string HostName();

// No stock display for a lighthouse headset: LH_NO_STOCK_HMD, or WiVRn, which streams its
// own headset (a Vive left plugged in crashed wivrn-server inside the stock driver's
// headset setup). LH_STOCK_HMD=1 still forces it.
bool StockHmdUnwanted(std::string* why) {
  if (std::getenv("LH_NO_STOCK_HMD")) { if (why) *why = "LH_NO_STOCK_HMD"; return true; }
  const char* force = std::getenv("LH_STOCK_HMD");
  if (!(force && *force == '1') && HostName() == "wivrn-server") {
    if (why) *why = "WiVRn streams its own headset";
    return true;
  }
  return false;
}

std::string HostName() {
#ifdef _WIN32
  wchar_t path[MAX_PATH]{};
  GetModuleFileNameW(nullptr, path, MAX_PATH);
  return lh::platform::PathUtf8(std::filesystem::path(path).stem());
#else
  std::ifstream comm("/proc/self/comm");
  std::string name;
  std::getline(comm, name);
  return name;
#endif
}

uint64_t UniverseId(const std::string& roomId) {
  // "steamvr-1770781428" (imported SteamVR universe) or "room-<unix time>".
  const auto dash = roomId.rfind('-');
  if (dash == std::string::npos) return 0;
  return std::strtoull(roomId.c_str() + dash + 1, nullptr, 10);
}

HmdQuaternion_t Quat(const std::array<double, 4>& q) { return {q[0], q[1], q[2], q[3]}; }

// The render model name in a device's own config.
// 1) Index controllers' firmware names "{knuckles}valve_controller_knu_1_0_right", a resource
// root SteamVR no longer has (it is "indexcontroller" now): SteamVR found no model and logged
// "Could not find named path knuckles" every frame.
// 2) Vive Tracker 1.0/2.0 ("VIVE Tracker Pro MV") firmware reports "vr_tracker_vive_1_0" without
// resource root. Without "{htc}", SteamVR attempts to load it from resources/rendermodels/
// instead of drivers/htc/resources/rendermodels/, failing with VRRenderModelError_InvalidModel (301).
std::string RenderModel(const std::string& configured) {
  static const std::string kOld = "{knuckles}";
  if (configured.rfind(kOld, 0) == 0) return "{indexcontroller}" + configured.substr(kOld.size());
  if (configured == "vr_tracker_vive_1_0") return "{htc}vr_tracker_vive_1_0";
  return configured;
}

struct DeviceStatic {
  std::string serial, model, manufacturer, renderModel, controllerRole;
  lh::DeviceRole role = lh::DeviceRole::Tracker;
  std::string baseChannel;  // base stations: "A" "b" "c", lhbs multi "1".."4"
  lh::ControllerType controllerType = lh::ControllerType::Index;
  bool lh2 = false;         // base stations: a 2.0 one
  std::string resourceRoot, inputProfile, registeredType;  // trackers: the config's own (lh::DeviceInfo)
  // device_class "null": tracked as ever, but registered as TrackedDeviceClass_Invalid, so SteamVR and
  // apps don't list it, while TrackingOverrides can still use it (e.g. a tracker on the head standing
  // in for the headset). As the stock driver (0x2e8120): a class it doesn't know is Invalid.
  bool hidden = false;
  bool tundraTracker = false;
};

class Device : public ITrackedDeviceServerDriver {
 public:
  explicit Device(DeviceStatic info) : info_(std::move(info)) {
    pose_.qWorldFromDriverRotation = {1, 0, 0, 0};
    pose_.qDriverFromHeadRotation = {1, 0, 0, 0};
    pose_.qRotation = {1, 0, 0, 0};
    pose_.result = TrackingResult_Calibrating_InProgress;
    pose_.poseIsValid = false;
    pose_.deviceIsConnected = true;
  }

  // What the host is given for this device: GetPose in the host's order (device_abi.hpp).
  ITrackedDeviceServerDriver* ForHost() { return forHost_.Host(); }

  EVRInitError Activate(uint32_t objectId) override {
    active_ = false;
    lastInputTime_ = lastPoseTime_ = -1;
    basePublished_ = false;
    skeletonInput_ = {};
    lastSkeletonSent_ = -1e300;
    battery_ = -1; universe_ = 0; hadInput_ = false; autoHand_ = 0;
    std::fill(std::begin(sent_), std::end(sent_), false);
    std::fill(std::begin(handles_), std::end(handles_), k_ulInvalidInputComponentHandle);
    haptic_ = skeletonHandle_ = skeletonOtherHandle_ = k_ulInvalidInputComponentHandle;
    id_ = objectId;
    container_ = VRProperties()->TrackedDeviceToPropertyContainer(objectId);
    SetProperties();
    if (info_.role == lh::DeviceRole::Controller) CreateInputs();
    else if (info_.role == lh::DeviceRole::Tracker && (HtcTracker() || info_.tundraTracker)) CreateTrackerInputs();
    active_ = true;
    return VRInitError_None;
  }
  void Deactivate() override {
    active_ = false;
    id_ = k_unTrackedDeviceIndexInvalid;
  }
  // A device the host activated through another driver (a headset from the stock driver,
  // stock_hmd.hpp): we only publish its pose; its properties are that driver's. published:
  // also handed every pose we publish for it (on our thread, after poseMutex_ is released).
  void AttachExternal(uint32_t objectId, const std::atomic<bool>* alive, lh::openvr::PublishedPose* published = nullptr) {
    id_ = objectId;
    container_ = VRProperties()->TrackedDeviceToPropertyContainer(objectId);
    externalAlive_ = alive;
    published_ = published;
    active_ = true;
  }
  // SteamVR's idle power-off (power/turnOffControllersTimeout) calls this; the stock
  // driver powers the device off here when it can (Prop_DeviceCanPowerOff), and so do we.
  void EnterStandby() override {
    const bool canPowerOff = info_.role == lh::DeviceRole::Controller || info_.role == lh::DeviceRole::Tracker;
    Log("SteamVR put " + info_.serial + " in standby" + (canPowerOff ? ": powering it off" : " (not a device we power off)"));
    if (canPowerOff) powerOffRequested_ = true;
  }
  void* GetComponent(const char*) override { return nullptr; }
  void DebugRequest(const char*, char* response, uint32_t size) override {
    if (size) response[0] = 0;
  }
  DriverPose_t GetPose() override {
    std::lock_guard lock(poseMutex_);
    return pose_;
  }

  bool Active() const { return active_ && (!externalAlive_ || *externalAlive_); }
  bool IsHmd() const { return info_.role == lh::DeviceRole::Hmd; }
  // "left_hand" / "right_hand" for a controller, else "".
  std::string Hand() const { return info_.role == lh::DeviceRole::Controller ? info_.controllerRole : std::string(); }
  // A wand with no hand set: its hand follows where it is, as SteamVR's own driver does.
  bool AutoHandWand() const {
    return info_.role == lh::DeviceRole::Controller && info_.controllerType == lh::ControllerType::ViveWand &&
           info_.controllerRole.empty();
  }
  // 0 none yet, else TrackedControllerRole_LeftHand / _RightHand.
  int AutoHand() const { return autoHand_; }
  void SetAutoHand(int role) {
    if (role == autoHand_ || !Active()) return;
    autoHand_ = role;
    VRProperties()->SetInt32Property(container_, Prop_ControllerRoleHint_Int32,
                                     role ? role : int(TrackedControllerRole_Invalid));
    VRServerDriverHost()->VendorSpecificEvent(id_, VREvent_TrackedDeviceRoleChanged, {}, 0);
    if (role) Log(info_.serial + ": " + (role == TrackedControllerRole_LeftHand ? "left" : "right") + " hand");
  }
  std::string StringProperty(ETrackedDeviceProperty prop) const {
    if (!Active()) return {};
    char value[256] = {};
    ETrackedPropertyError error = TrackedProp_Success;
    VRProperties()->GetStringProperty(container_, prop, value, sizeof value, &error);
    return error == TrackedProp_Success ? std::string(value) : std::string();
  }
  const DeviceStatic& Info() const { return info_; }
  bool MatchesHaptic(const VREvent_t& event) const {
    return active_ && lh::openvr::MatchesHaptic(event, id_, container_, haptic_);
  }
  bool TakePowerOffRequest() { return powerOffRequested_.exchange(false); }

  void SetRawTransform(const Eigen::Isometry3d& t) { rawFromWorld_ = t; }
  // A base's channel (Prop_ModeLabel_String): known once its OOTX is, and changed with
  // its mode button.
  void SetChannel(const std::string& channel) {
    if (channel == info_.baseChannel) return;
    info_.baseChannel = channel;
    if (active_) VRProperties()->SetStringProperty(container_, Prop_ModeLabel_String, channel.c_str());
  }
  // 1.0 or 2.0 base station: model, render model and status icons to match.
  void SetLh2(bool lh2) {
    if (lh2 == info_.lh2) return;
    info_.lh2 = lh2;
    if (active_) BaseLook(container_);
  }
  void BaseLook(PropertyContainerHandle_t c) {
    auto* props = VRProperties();
    const bool two = info_.lh2;
    props->SetStringProperty(c, Prop_ManufacturerName_String, two ? "Valve" : "HTC");
    props->SetStringProperty(c, Prop_ModelNumber_String, two ? "Valve SR Imp" : "HTC V2-XD/XE");
    props->SetStringProperty(c, Prop_RenderModelName_String, two ? "lh_basestation_valve_gen2" : "lh_basestation_vive");
    const std::string icons = two ? "{lighthouse}/icons/base2_status_" : "{htc}/icons/base_status_";
    props->SetStringProperty(c, Prop_NamedIconPathDeviceOff_String, (icons + "off.png").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceSearching_String, (icons + "searching.gif").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceSearchingAlert_String, (icons + "searching_alert.gif").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceReady_String, (icons + "ready.png").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceReadyAlert_String, (icons + "ready_alert.png").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceNotReady_String, (icons + "error.png").c_str());
    props->SetStringProperty(c, Prop_NamedIconPathDeviceStandby_String, (icons + "standby.png").c_str());
  }
  void SetUniverse(uint64_t universe) {
    if (!active_ || universe == universe_) return;
    universe_ = universe;
    VRProperties()->SetUint64Property(container_, Prop_CurrentUniverseId_Uint64, universe);
  }
  // The receiver it's connected through, as the stock driver publishes it. Tools that
  // power devices off (OyasumiVR) need it: they run lighthouse_console against it.
  void SetDongle(const std::string& receiver) {
    if (!active_ || receiver == dongle_) return;
    dongle_ = receiver;
    VRProperties()->SetStringProperty(container_, Prop_ConnectedWirelessDongle_String, receiver.c_str());
  }

  // Tracked devices: called from the driver thread with the latest liblh data.
  void Update(const lh::DeviceUpdate& u, double now) {
    if (!Active()) return;
    DriverPose_t p{};
    const Eigen::Quaterniond rawRotation(rawFromWorld_.rotation());
    p.qWorldFromDriverRotation = {rawRotation.w(), rawRotation.x(), rawRotation.y(), rawRotation.z()};
    for (int i = 0; i < 3; ++i) p.vecWorldFromDriverTranslation[i] = rawFromWorld_.translation()[i];
    const auto& pose = u.pose;
    p.qDriverFromHeadRotation = Quat(pose.imuFromHead.orientation);
    for (int i = 0; i < 3; ++i) p.vecDriverFromHeadTranslation[i] = pose.imuFromHead.position[size_t(i)];
    p.qRotation = Quat(pose.worldFromImu.orientation);
    for (int i = 0; i < 3; ++i) {
      p.vecPosition[i] = pose.worldFromImu.position[size_t(i)];
      p.vecVelocity[i] = pose.velocity[size_t(i)];
      p.vecAcceleration[i] = pose.acceleration[size_t(i)];  // SteamVR predicts with it (swings curve)
    }
    // Angular velocity in the device (IMU) frame, as the stock driver reports it.
    const auto& o = pose.worldFromImu.orientation;
    const Eigen::Quaterniond q(o[0], o[1], o[2], o[3]);
    const Eigen::Vector3d w = q.conjugate() * Eigen::Vector3d(pose.angularVelocity[0], pose.angularVelocity[1],
                                                             pose.angularVelocity[2]);
    for (int i = 0; i < 3; ++i) p.vecAngularVelocity[i] = w[i];
    p.poseTimeOffset = pose.valid ? std::min(0.0, pose.hostTime - now) : 0.0;
    p.result = static_cast<ETrackingResult>(int(pose.result));
    p.poseIsValid = pose.valid;
    p.deviceIsConnected = u.connected && u.enabled;
    p.willDriftInYaw = false;
    p.shouldApplyHeadModel = false;
    bool held = false;
    if (pose.valid) {
      held_ = p;
      haveHeld_ = true;
    } else if (haveHeld_ && p.deviceIsConnected) {
      // Lost: held where it was last seen until light finds it again, reported out of range.
      // liblh's lost record is an identity pose, which a host would show at the origin.
      // SteamVR hides a controller or tracker whose pose is invalid, as with its own driver;
      // Monado and WiVRn show an invalid pose at the origin instead, so there it stays valid.
      // The headset is always held: SteamVR shows its grey tracking-lost screen otherwise.
      static const bool hostHides = HostName().rfind("vrserver", 0) == 0;
      p = held_;
      for (int i = 0; i < 3; ++i) p.vecVelocity[i] = p.vecAngularVelocity[i] = p.vecAcceleration[i] = p.vecAngularAcceleration[i] = 0;
      p.qWorldFromDriverRotation = {rawRotation.w(), rawRotation.x(), rawRotation.y(), rawRotation.z()};
      for (int i = 0; i < 3; ++i) p.vecWorldFromDriverTranslation[i] = rawFromWorld_.translation()[i];
      p.poseTimeOffset = 0;
      p.result = TrackingResult_Running_OutOfRange;
      p.poseIsValid = IsHmd() || !hostHides;
      held = p.poseIsValid;
    } else if (!p.deviceIsConnected) {
      haveHeld_ = false;  // switched off: it comes back wherever it is
    }
    const bool changed = pose.hostTime != lastPoseTime_ || p.poseIsValid != lastValid_ ||
                         p.deviceIsConnected != lastConnected_ || p.result != lastResult_ ||
                         !rawFromWorld_.matrix().isApprox(publishedRaw_.matrix(), 1e-12);
    if (changed) {
      publishedRaw_ = rawFromWorld_;
      lastPoseTime_ = pose.hostTime;
      lastValid_ = p.poseIsValid;
      lastConnected_ = p.deviceIsConnected;
      lastResult_ = p.result;
      {
        std::lock_guard lock(poseMutex_);
        pose_ = p;
      }
      VRServerDriverHost()->TrackedDevicePoseUpdated(id_, p, sizeof(p));
      if (published_) published_->Publish(p);
      Record(p, now, held);
    }
    if (u.batteryPercent && (*u.batteryPercent != battery_ || u.charging != charging_)) {
      battery_ = *u.batteryPercent;
      charging_ = u.charging;
      VRProperties()->SetFloatProperty(container_, Prop_DeviceBatteryPercentage_Float, float(battery_) / 100.0f);
      VRProperties()->SetBoolProperty(container_, Prop_DeviceIsCharging_Bool, charging_);
    }
    if (info_.role == lh::DeviceRole::Controller || u.input || hadInput_) {
      if (u.input && p.deviceIsConnected) { UpdateInputs(*u.input, now); hadInput_ = true; }
      else if (hadInput_) { lh::ControllerInput neutral; neutral.hostTime = now; UpdateInputs(neutral, now); hadInput_ = false; }
      if (info_.role == lh::DeviceRole::Controller && now - lastSkeletonSent_ >= kSkeletonInterval) SendSkeleton(now);
    }
  }

  // Base stations: static pose from the room map.
  void UpdateBase(const lh::BaseStationState& b) {
    if (!active_) return;
    DriverPose_t p{};
    const Eigen::Quaterniond rawRotation(rawFromWorld_.rotation());
    p.qWorldFromDriverRotation = {rawRotation.w(), rawRotation.x(), rawRotation.y(), rawRotation.z()};
    for (int i = 0; i < 3; ++i) p.vecWorldFromDriverTranslation[i] = rawFromWorld_.translation()[i];
    p.qDriverFromHeadRotation = {1, 0, 0, 0};
    p.qRotation = Quat(b.worldFromBase.orientation);
    for (int i = 0; i < 3; ++i) p.vecPosition[i] = b.worldFromBase.position[size_t(i)];
    // Until it's locked (or is the origin) it shows as searching (a blinking icon): its
    // place can still move. Learned but not locked yet: calibrating.
    const bool settled = b.known && (b.origin || b.locked || b.learned);  // learned: placed well enough to track with
    p.result = !b.known ? TrackingResult_Uninitialized : settled ? TrackingResult_Running_OK
                                                                 : TrackingResult_Calibrating_InProgress;
    p.poseIsValid = settled && b.seen;
    p.deviceIsConnected = b.seen;
    // Only when something changed (its place, the room frame, its state): apps that watch base
    // stations count every update as a move.
    const auto same = [](const DriverPose_t& x, const DriverPose_t& y) {
      return x.result == y.result && x.poseIsValid == y.poseIsValid && x.deviceIsConnected == y.deviceIsConnected &&
             !std::memcmp(&x.qRotation, &y.qRotation, sizeof x.qRotation) &&
             !std::memcmp(x.vecPosition, y.vecPosition, sizeof x.vecPosition) &&
             !std::memcmp(&x.qWorldFromDriverRotation, &y.qWorldFromDriverRotation, sizeof x.qWorldFromDriverRotation) &&
             !std::memcmp(x.vecWorldFromDriverTranslation, y.vecWorldFromDriverTranslation, sizeof x.vecWorldFromDriverTranslation);
    };
    if (basePublished_ && same(p, lastBase_)) return;
    basePublished_ = true;
    lastBase_ = p;
    {
      std::lock_guard lock(poseMutex_);
      pose_ = p;
    }
    VRServerDriverHost()->TrackedDevicePoseUpdated(id_, p, sizeof(p));
  }

 private:
  lh::openvr::DeviceForHost forHost_{this};

  // At most 100 Hz per device, but every change of state.
  void Record(const DriverPose_t& p, double now, bool held) {
    if (!g_recorder.On()) return;
    const bool stateChanged = p.poseIsValid != recValid_ || p.result != recResult_ || held != recHeld_ ||
                              p.deviceIsConnected != recConnected_;
    if (!stateChanged && now - recTime_ < 0.01) return;
    recTime_ = now; recValid_ = p.poseIsValid; recResult_ = p.result; recHeld_ = held; recConnected_ = p.deviceIsConnected;
    char text[512];
    std::snprintf(text, sizeof text,
                  "{\"t\":%.6f,\"pose\":%s,\"valid\":%d,\"held\":%d,\"connected\":%d,\"result\":%d,"
                  "\"p\":[%.5f,%.5f,%.5f],\"q\":[%.6f,%.6f,%.6f,%.6f],\"v\":[%.4f,%.4f,%.4f],\"w\":[%.4f,%.4f,%.4f],"
                  "\"offset\":%.4f,\"world\":[%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f]}",
                  now, Recorder::Quote(info_.serial).c_str(), p.poseIsValid, held, p.deviceIsConnected, int(p.result),
                  p.vecPosition[0], p.vecPosition[1], p.vecPosition[2], p.qRotation.w, p.qRotation.x, p.qRotation.y,
                  p.qRotation.z, p.vecVelocity[0], p.vecVelocity[1], p.vecVelocity[2], p.vecAngularVelocity[0],
                  p.vecAngularVelocity[1], p.vecAngularVelocity[2], p.poseTimeOffset, p.vecWorldFromDriverTranslation[0],
                  p.vecWorldFromDriverTranslation[1], p.vecWorldFromDriverTranslation[2], p.qWorldFromDriverRotation.w,
                  p.qWorldFromDriverRotation.x, p.qWorldFromDriverRotation.y, p.qWorldFromDriverRotation.z);
    g_recorder.Line(text);
  }

  void SetProperties() {
    auto* props = VRProperties();
    const PropertyContainerHandle_t c = container_;
    props->SetStringProperty(c, Prop_TrackingSystemName_String, "lighthouse");
    props->SetStringProperty(c, Prop_SerialNumber_String, info_.serial.c_str());
    // Tells apps it's openhouse, not SteamVR's own driver (same tracking system name).
    props->SetStringProperty(c, Prop_DriverVersion_String, (std::string("openhouse ") + lh::kVersion).c_str());
    props->SetBoolProperty(c, Prop_WillDriftInYaw_Bool, false);
    props->SetBoolProperty(c, Prop_DeviceIsWireless_Bool, info_.role != lh::DeviceRole::BaseStation);
    props->SetBoolProperty(c, Prop_HasDisplayComponent_Bool, false);
    props->SetBoolProperty(c, Prop_HasCameraComponent_Bool, false);
    props->SetBoolProperty(c, Prop_HasDriverDirectModeComponent_Bool, false);
    props->SetBoolProperty(c, Prop_HasVirtualDisplayComponent_Bool, false);
    props->SetBoolProperty(c, Prop_Firmware_UpdateAvailable_Bool, false);
    props->SetBoolProperty(c, Prop_Firmware_ManualUpdate_Bool, false);
    props->SetBoolProperty(c, Prop_CanUnifyCoordinateSystemWithHmd_Bool, true);
    props->SetBoolProperty(c, Prop_DeviceCanPowerOff_Bool,
                           info_.role == lh::DeviceRole::Controller || info_.role == lh::DeviceRole::Tracker);
    switch (info_.role) {
      case lh::DeviceRole::Controller: {
        const bool left = info_.controllerRole == "left_hand";
        if (info_.controllerType == lh::ControllerType::Mapped) {
          // Another maker's controller (Shiftall FlipVR): its own resources, profile and
          // render model, as its config names them; its inputs come from that profile.
          LoadMappedProfile();
          const std::string root = info_.resourceRoot;
          props->SetStringProperty(c, Prop_ManufacturerName_String, info_.manufacturer.c_str());
          props->SetStringProperty(c, Prop_ModelNumber_String, info_.model.c_str());
          if (!root.empty()) props->SetStringProperty(c, Prop_ResourceRoot_String, root.c_str());
          if (!mapped_.controllerType.empty())
            props->SetStringProperty(c, Prop_ControllerType_String, mapped_.controllerType.c_str());
          props->SetStringProperty(c, Prop_InputProfilePath_String, info_.inputProfile.c_str());
          if (!info_.renderModel.empty()) props->SetStringProperty(c, Prop_RenderModelName_String, info_.renderModel.c_str());
          const std::string registered =
              !info_.registeredType.empty()
                  ? info_.registeredType
                  : (root.empty() ? std::string("lighthouse") : root) + "/" +
                        (mapped_.controllerType.empty() ? std::string("controller") : mapped_.controllerType) + info_.serial;
          props->SetStringProperty(c, Prop_RegisteredDeviceType_String, registered.c_str());
          props->SetInt32Property(c, Prop_ControllerRoleHint_Int32,
                                  left ? TrackedControllerRole_LeftHand
                                       : info_.controllerRole == "right_hand" ? TrackedControllerRole_RightHand
                                                                              : TrackedControllerRole_Invalid);
          props->SetInt32Property(c, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
          props->SetInt32Property(c, Prop_ControllerHandSelectionPriority_Int32, NextHandPriority());
          lastUse_ = std::chrono::steady_clock::now();
          props->SetInt32Property(c, Prop_Axis0Type_Int32, k_eControllerAxis_Joystick);
          props->SetInt32Property(c, Prop_Axis1Type_Int32, k_eControllerAxis_Trigger);
          props->SetBoolProperty(c, Prop_DeviceProvidesBatteryStatus_Bool, true);
          props->SetBoolProperty(c, Prop_Identifiable_Bool, true);
          for (const auto& [prop, icon] : StatusIcons(root, info_.model)) props->SetStringProperty(c, prop, icon.c_str());
          break;
        }
        if (info_.controllerType == lh::ControllerType::ViveWand) {
          // As SteamVR's own wand: the htc resources (profile, render model, icons).
          props->SetStringProperty(c, Prop_ManufacturerName_String, info_.manufacturer.empty() ? "HTC" : info_.manufacturer.c_str());
          props->SetStringProperty(c, Prop_ModelNumber_String, info_.model.empty() ? "Vive. Controller MV" : info_.model.c_str());
          props->SetStringProperty(c, Prop_ResourceRoot_String, "htc");
          props->SetStringProperty(c, Prop_ControllerType_String, "vive_controller");
          props->SetStringProperty(c, Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");
          props->SetStringProperty(c, Prop_RenderModelName_String,
                                   info_.renderModel.empty() ? "vr_controller_vive_1_5" : info_.renderModel.c_str());
          props->SetStringProperty(c, Prop_RegisteredDeviceType_String, ("htc/vive_controller" + info_.serial).c_str());
          props->SetInt32Property(c, Prop_ControllerRoleHint_Int32,
                                  left ? TrackedControllerRole_LeftHand
                                       : info_.controllerRole == "right_hand" ? TrackedControllerRole_RightHand
                                                                              : TrackedControllerRole_Invalid);
          props->SetInt32Property(c, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
          props->SetInt32Property(c, Prop_ControllerHandSelectionPriority_Int32, NextHandPriority());
          lastUse_ = std::chrono::steady_clock::now();
          props->SetInt32Property(c, Prop_Axis0Type_Int32, k_eControllerAxis_TrackPad);
          props->SetInt32Property(c, Prop_Axis1Type_Int32, k_eControllerAxis_Trigger);
          props->SetBoolProperty(c, Prop_DeviceProvidesBatteryStatus_Bool, true);
          props->SetFloatProperty(c, Prop_DeviceBatteryPercentage_Float, 1.0f);
          props->SetBoolProperty(c, Prop_DeviceIsCharging_Bool, false);
          props->SetBoolProperty(c, Prop_Identifiable_Bool, true);
          props->SetStringProperty(c, Prop_NamedIconPathDeviceOff_String, "{htc}/icons/controller_status_off.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/controller_status_searching.gif");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/controller_status_searching_alert.gif");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceReady_String, "{htc}/icons/controller_status_ready.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/controller_status_ready_alert.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/controller_status_error.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/controller_status_standby.png");
          props->SetStringProperty(c, Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/controller_status_ready_low.png");
          break;
        }
        props->SetStringProperty(c, Prop_ManufacturerName_String, info_.manufacturer.empty() ? "Valve" : info_.manufacturer.c_str());
        props->SetStringProperty(c, Prop_ModelNumber_String, info_.model.c_str());
        props->SetStringProperty(c, Prop_ResourceRoot_String, "indexcontroller");
        props->SetStringProperty(c, Prop_ControllerType_String, "knuckles");
        props->SetStringProperty(c, Prop_InputProfilePath_String, "{indexcontroller}/input/index_controller_profile.json");
        props->SetStringProperty(c, Prop_RenderModelName_String,
                                 info_.renderModel.empty()
                                     ? (left ? "{indexcontroller}valve_controller_knu_1_0_left" : "{indexcontroller}valve_controller_knu_1_0_right")
                                     : info_.renderModel.c_str());
        props->SetStringProperty(c, Prop_RegisteredDeviceType_String, ("valve/index_controller" + info_.serial).c_str());
        props->SetInt32Property(c, Prop_ControllerRoleHint_Int32,
                                left ? TrackedControllerRole_LeftHand
                                     : info_.controllerRole == "right_hand" ? TrackedControllerRole_RightHand
                                                                            : TrackedControllerRole_Invalid);
        props->SetInt32Property(c, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
        // Two controllers for one hand: SteamVR's primary is the higher priority. The one
        // turned on last, then whichever is picked up and used (UpdateInputs).
        props->SetInt32Property(c, Prop_ControllerHandSelectionPriority_Int32, NextHandPriority());
        lastUse_ = std::chrono::steady_clock::now();
        props->SetInt32Property(c, Prop_Axis0Type_Int32, k_eControllerAxis_Joystick);
        props->SetInt32Property(c, Prop_Axis1Type_Int32, k_eControllerAxis_Trigger);
        props->SetInt32Property(c, Prop_Axis2Type_Int32, k_eControllerAxis_TrackPad);
        props->SetBoolProperty(c, Prop_DeviceProvidesBatteryStatus_Bool, true);
        props->SetFloatProperty(c, Prop_DeviceBatteryPercentage_Float, 1.0f);
        props->SetBoolProperty(c, Prop_DeviceIsCharging_Bool, false);
        props->SetBoolProperty(c, Prop_Identifiable_Bool, true);  // enables SteamVR's Identify button
        const std::string side = left ? "left" : "right";
        props->SetStringProperty(c, Prop_NamedIconPathDeviceOff_String, ("{indexcontroller}/icons/" + side + "_controller_status_off.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceSearching_String, ("{indexcontroller}/icons/" + side + "_controller_status_searching.gif").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceSearchingAlert_String, ("{indexcontroller}/icons/" + side + "_controller_status_searching_alert.gif").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceReady_String, ("{indexcontroller}/icons/" + side + "_controller_status_ready.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceReadyAlert_String, ("{indexcontroller}/icons/" + side + "_controller_status_ready_alert.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceNotReady_String, ("{indexcontroller}/icons/" + side + "_controller_status_error.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceStandby_String, ("{indexcontroller}/icons/" + side + "_controller_status_standby.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceAlertLow_String, ("{indexcontroller}/icons/" + side + "_controller_status_ready_low.png").c_str());
        break;
      }
      case lh::DeviceRole::BaseStation: {
        BaseLook(c);
        props->SetStringProperty(c, Prop_ResourceRoot_String, "htc");
        props->SetStringProperty(c, Prop_ModeLabel_String, info_.baseChannel.c_str());
        props->SetInt32Property(c, Prop_DeviceClass_Int32, TrackedDeviceClass_TrackingReference);
        props->SetFloatProperty(c, Prop_FieldOfViewLeftDegrees_Float, 60);
        props->SetFloatProperty(c, Prop_FieldOfViewRightDegrees_Float, 60);
        props->SetFloatProperty(c, Prop_FieldOfViewTopDegrees_Float, 60);
        props->SetFloatProperty(c, Prop_FieldOfViewBottomDegrees_Float, 60);
        props->SetFloatProperty(c, Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        props->SetFloatProperty(c, Prop_TrackingRangeMaximumMeters_Float, 7.0f);
        break;
      }
      default: {  // trackers (HTC's: inputs from CreateTrackerInputs)
        // As the stock driver (0x1cf860, 0x1d69f0): an HTC tracker gets HTC's profile and
        // type, and "vr_tracker_vive_1_0" becomes {htc}vr_tracker_vive_3_0 on a 3.0. Any
        // other maker's (Tundra Labs) keeps its own resource root, profile and icons, instead
        // of HTC's.
        const bool htc = HtcTracker();
        const std::string root = htc || info_.resourceRoot.empty() ? "htc" : info_.resourceRoot;
        std::string renderModel = info_.renderModel.empty() ? "{htc}vr_tracker_vive_1_0" : info_.renderModel;
        // (RenderModel() has already put {htc} in front of a bare name.)
        if (htc && EqualsNoCase(renderModel, "{htc}vr_tracker_vive_1_0") && info_.model.rfind("VIVE Tracker 3.0", 0) == 0)
          renderModel = "{htc}vr_tracker_vive_3_0";
        const std::string profile = htc || info_.inputProfile.empty() ? "{htc}/input/vive_tracker_profile.json" : info_.inputProfile;
        // The device's path in SteamVR ("/devices/<this>"), which saved tracker roles and
        // TrackingOverrides go by: as the stock driver names it, HTC's "htc/vive_tracker<serial>",
        // any other maker's its config's type or else "lighthouse/<serial>".
        const std::string registered = htc ? "htc/vive_tracker" + info_.serial
                                           : !info_.registeredType.empty() ? info_.registeredType
                                                                           : "lighthouse/" + info_.serial;
        props->SetStringProperty(c, Prop_ManufacturerName_String, info_.manufacturer.empty() ? "HTC" : info_.manufacturer.c_str());
        props->SetStringProperty(c, Prop_ModelNumber_String, info_.model.empty() ? "VIVE Tracker" : info_.model.c_str());
        props->SetStringProperty(c, Prop_ResourceRoot_String, root.c_str());
        props->SetStringProperty(c, Prop_ControllerType_String, "vive_tracker");  // Tundra's profile says so too
        props->SetStringProperty(c, Prop_InputProfilePath_String, profile.c_str());
        props->SetStringProperty(c, Prop_RenderModelName_String, renderModel.c_str());
        props->SetStringProperty(c, Prop_RegisteredDeviceType_String, registered.c_str());
        props->SetInt32Property(c, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);
        props->SetInt32Property(c, Prop_DeviceClass_Int32, info_.hidden ? TrackedDeviceClass_Invalid : TrackedDeviceClass_GenericTracker);
        props->SetBoolProperty(c, Prop_DeviceProvidesBatteryStatus_Bool, true);
        props->SetFloatProperty(c, Prop_DeviceBatteryPercentage_Float, 1.0f);
        props->SetBoolProperty(c, Prop_DeviceIsCharging_Bool, false);
        props->SetBoolProperty(c, Prop_Identifiable_Bool, true);  // enables SteamVR's Identify button
        const std::string icons = "{" + root + "}/icons/tracker_status_";
        props->SetStringProperty(c, Prop_NamedIconPathDeviceOff_String, (icons + "off.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceSearching_String, (icons + "searching.gif").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceSearchingAlert_String, (icons + "searching_alert.gif").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceReady_String, (icons + "ready.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceReadyAlert_String, (icons + "ready_alert.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceNotReady_String, (icons + "error.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceStandby_String, (icons + "standby.png").c_str());
        props->SetStringProperty(c, Prop_NamedIconPathDeviceAlertLow_String, (icons + "ready_low.png").c_str());
        break;
      }
    }
  }

  enum Input {
    SystemClick, SystemTouch, AClick, ATouch, BClick, BTouch, TriggerClick, TriggerTouch, TriggerValue,
    GripTouch, GripValue, GripForce, StickClick, StickTouch, StickX, StickY, PadTouch, PadX, PadY, PadForce,
    FingerIndex, FingerMiddle, FingerRing, FingerPinky, PadClick, GripClick, MenuClick, InputCount
  };

  bool HtcTracker() const { return info_.manufacturer.empty() || EqualsNoCase(info_.manufacturer, "htc"); }

  // An HTC tracker's inputs, as {htc}/input/vive_tracker_profile.json names them. The stock
  // driver (0x3f78a0, case 3) maps the wand's buttons onto them: system -> power, trackpad
  // click -> thumb; menu, grip and trigger keep their names. The pogo pins drive grip
  // (pin 3), trigger (pin 4), thumb (pin 5) and menu (pin 6).
  void CreateTrackerInputs() {
    auto* in = ComponentInput();
    const PropertyContainerHandle_t c = container_;
    auto created = [&](Input i, const char* path, EVRInputError e) {
      paths_[i] = path;
      if (e != VRInputError_None) Log(info_.serial + ": creating " + path + " failed, error " + std::to_string(int(e)));
    };
    auto b = [&](Input i, const char* path) {
      isBool_[i] = true;
      created(i, path, in->CreateBooleanComponent(c, path, &handles_[i]));
    };
    b(SystemClick, "/input/power/click");
    if (info_.tundraTracker) return;  // built-in button only
    b(MenuClick, "/input/application_menu/click");
    b(GripClick, "/input/grip/click");
    b(TriggerClick, "/input/trigger/click");
    created(TriggerValue, "/input/trigger/value",
            in->CreateScalarComponent(c, "/input/trigger/value", &handles_[TriggerValue], VRScalarType_Absolute,
                                      VRScalarUnits_NormalizedOneSided));
    b(PadClick, "/input/thumb/click");
    in->CreateHapticComponent(c, "/output/haptic", &haptic_);
  }

  void CreateInputs() {
    auto* in = ComponentInput();
    const PropertyContainerHandle_t c = container_;
    // Each component's path, and any creation error, logged (Windows SteamVR refused one
    // boolean as "wrong type" without saying which).
    auto created = [&](Input i, const char* path, EVRInputError e) {
      paths_[i] = path;
      if (e != VRInputError_None) Log(info_.serial + ": creating " + path + " failed, error " + std::to_string(int(e)));
    };
    auto b = [&](Input i, const char* path) {
      isBool_[i] = true;
      created(i, path, in->CreateBooleanComponent(c, path, &handles_[i]));
    };
    auto s1 = [&](Input i, const char* path) {
      created(i, path, in->CreateScalarComponent(c, path, &handles_[i], VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided));
    };
    auto s2 = [&](Input i, const char* path) {
      created(i, path, in->CreateScalarComponent(c, path, &handles_[i], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided));
    };
    if (info_.controllerType == lh::ControllerType::ViveWand) {
      // The wand's components, as vive_controller_profile.json names them, and a hand
      // estimated from the buttons, as the stock driver gives it: apps that animate the
      // fingers from the skeleton have nothing to go by without it.
      b(SystemClick, "/input/system/click");
      b(MenuClick, "/input/application_menu/click");
      b(GripClick, "/input/grip/click");
      b(TriggerClick, "/input/trigger/click");
      s1(TriggerValue, "/input/trigger/value");
      b(PadClick, "/input/trackpad/click");
      b(PadTouch, "/input/trackpad/touch");
      s2(PadX, "/input/trackpad/x");
      s2(PadY, "/input/trackpad/y");
      in->CreateHapticComponent(c, "/output/haptic", &haptic_);
      CreateSkeleton();
      return;
    }
    // A mapped controller (FlipVR) gets no skeleton, as with the stock driver: it has no finger
    // sensors, and apps that see one take it over their own hand poses (VRChat's gestures).
    if (info_.controllerType == lh::ControllerType::Mapped) {
      CreateMappedInputs();
      return;
    }
    CreateIndexInputs(b, s1, s2);
    CreateSkeleton();
  }

  template <class B, class S1, class S2>
  void CreateIndexInputs(B& b, S1& s1, S2& s2) {
    auto* in = ComponentInput();
    const PropertyContainerHandle_t c = container_;
    b(SystemClick, "/input/system/click");
    b(SystemTouch, "/input/system/touch");
    b(AClick, "/input/a/click");
    b(ATouch, "/input/a/touch");
    b(BClick, "/input/b/click");
    b(BTouch, "/input/b/touch");
    b(TriggerClick, "/input/trigger/click");
    b(TriggerTouch, "/input/trigger/touch");
    s1(TriggerValue, "/input/trigger/value");
    b(GripTouch, "/input/grip/touch");
    s1(GripValue, "/input/grip/value");
    s1(GripForce, "/input/grip/force");
    b(StickClick, "/input/thumbstick/click");
    b(StickTouch, "/input/thumbstick/touch");
    s2(StickX, "/input/thumbstick/x");
    s2(StickY, "/input/thumbstick/y");
    b(PadTouch, "/input/trackpad/touch");
    s2(PadX, "/input/trackpad/x");
    s2(PadY, "/input/trackpad/y");
    s1(PadForce, "/input/trackpad/force");
    s1(FingerIndex, "/input/finger/index");
    s1(FingerMiddle, "/input/finger/middle");
    s1(FingerRing, "/input/finger/ring");
    s1(FingerPinky, "/input/finger/pinky");
    in->CreateHapticComponent(c, "/output/haptic", &haptic_);
  }

  // The profile of a mapped controller (Shiftall FlipVR), for this hand.
  void LoadMappedProfile() {
    mapped_ = {};
    const std::string text = ReadResource(info_.inputProfile);
    std::string error;
    if (text.empty()) {
      error = "not found";
    } else if (lh::runtime::ParseMappedProfile(text, info_.controllerRole, &mapped_, &error)) {
      Log(info_.serial + ": mapped input profile " + info_.inputProfile + " (" + mapped_.controllerType + ", " +
          std::to_string(mapped_.components.size()) + " components)");
      return;
    }
    Log(info_.serial + ": mapped input profile " + info_.inputProfile + " unusable: " + error);
    mapped_ = {};
  }

  // Every component the profile lays out, by its own path.
  void CreateMappedInputs() {
    auto* in = ComponentInput();
    const PropertyContainerHandle_t c = container_;
    mappedInputs_.clear();
    for (const auto& m : mapped_.components) {
      if (m.kind == lh::runtime::MappedComponent::Kind::Haptic) {
        if (haptic_ == k_ulInvalidInputComponentHandle) in->CreateHapticComponent(c, m.path.c_str(), &haptic_);
        continue;
      }
      MappedInput mi;
      mi.component = &m;
      const EVRInputError e =
          m.kind == lh::runtime::MappedComponent::Kind::Bool
              ? in->CreateBooleanComponent(c, m.path.c_str(), &mi.handle)
              : in->CreateScalarComponent(c, m.path.c_str(), &mi.handle, VRScalarType_Absolute,
                                          m.twoSided ? VRScalarUnits_NormalizedTwoSided : VRScalarUnits_NormalizedOneSided);
      if (e != VRInputError_None) {
        Log(info_.serial + ": creating " + m.path + " failed, error " + std::to_string(int(e)));
        continue;
      }
      mappedInputs_.push_back(mi);
    }
  }

  void CreateSkeleton() {
    const PropertyContainerHandle_t c = container_;
    std::vector<std::string> paths;
    if (const char* p = std::getenv("LH_HAND_POSES")) paths.emplace_back(p);
    char resource[4096]{};
    if (VRResources()) VRResources()->GetResourceFullPath("{indexcontroller}/anims/vr_glove_right_poses.glb", "", resource, sizeof resource);
    if (*resource) paths.emplace_back(resource);
    paths.emplace_back(lh::platform::SteamVRDirectory() + "/drivers/indexcontroller/resources/anims/vr_glove_right_poses.glb");
    if (const char* home = std::getenv("HOME")) {
      for (const char* root : {"/.local/share/Steam/steamapps/common/SteamVR", "/.steam/steam/steamapps/common/SteamVR"})
        paths.emplace_back(std::string(home) + root + "/drivers/indexcontroller/resources/anims/vr_glove_right_poses.glb");
    }
    const bool wand = info_.controllerType == lh::ControllerType::ViveWand;
    const bool left = info_.controllerRole == "left_hand";
    // A wand's hand is known only once SteamVR picks one (no role hint): both then.
    const bool both = wand && info_.controllerRole != "left_hand" && info_.controllerRole != "right_hand";
    std::string error;
    auto create = [&](lh::openvr::HandSkeleton& skeleton, bool isLeft, VRInputComponentHandle_t* handle) {
      bool loaded = false;
      for (const auto& path : paths) if (skeleton.Load(path, isLeft, &error)) { loaded = true; break; }
      if (!loaded) { Log("skeleton unavailable: " + error); return; }
      if (!error.empty()) Log("skeleton: " + error);
      skeleton.SetIndexFromTrigger(wand);
      const auto& bones = skeleton.GripLimit();
      // Partial, as the stock Index driver reports it: each finger follows its own
      // capacitive sensor. Monado's steamvr_lh only exposes a skeleton that isn't
      // Estimated as (conforming) hand tracking; without it OpenXR apps and xrizer
      // fall back to their own estimate, middle, ring and pinky as one. A wand has no
      // finger sensors: Estimated, as the stock driver says.
      const auto result = VRDriverInput()->CreateSkeletonComponent(c, isLeft ? "/input/skeleton/left" : "/input/skeleton/right",
          isLeft ? "/skeleton/hand/left" : "/skeleton/hand/right", "/pose/raw",
          wand ? VRSkeletalTracking_Estimated : VRSkeletalTracking_Partial, bones.data(), uint32_t(bones.size()), handle);
      if (result != VRInputError_None) { *handle = 0; Log("skeleton creation failed: " + std::to_string(result)); }
    };
    create(skeleton_, both ? false : left, &skeletonHandle_);
    if (both) create(skeletonOther_, true, &skeletonOtherHandle_);
  }

  // Monado's steamvr_lh stamps each skeleton update with the controller's latest pose
  // and, for the hand-tracking joints xrizer gives VRChat, extrapolates from the newest
  // one along its velocity. Sent only with input reports (30 ms - 1.5 s apart on
  // session-post3), the hand flew along a stale velocity while the controller pose itself
  // (WayVR's clock) was fine: so the skeleton also goes out with the poses.
  // The hand's springs and fades advance here, on the host clock, with the latest
  // input: stepped only when an input report came, they moved in jumps.
  // The controller's orientation as apps get it from the last pose sent: world from driver, driver from
  // IMU, IMU from head (as stock gives its hand).
  std::optional<std::array<double, 4>> HeadOrientation() {
    DriverPose_t p;
    { std::lock_guard lock(poseMutex_); p = pose_; }
    if (!p.poseIsValid) return std::nullopt;
    const auto q = [](const HmdQuaternion_t& v) { return Eigen::Quaterniond(v.w, v.x, v.y, v.z); };
    const Eigen::Quaterniond h = q(p.qWorldFromDriverRotation) * q(p.qRotation) * q(p.qDriverFromHeadRotation);
    return std::array<double, 4>{h.w(), h.x(), h.y(), h.z()};
  }

  void SendSkeleton(double now) {
    if (!skeletonHandle_ && !skeletonOtherHandle_) return;
    const double dt = lastSkeletonSent_ < -1e299 ? 0 : now - lastSkeletonSent_;
    lastSkeletonSent_ = now;
    // A wand's buttons as the fingers they stand for: trigger -> index, grip -> the other three.
    lh::ControllerInput in = skeletonInput_;
    if (info_.controllerType == lh::ControllerType::ViveWand) {
      in.fingerIndex = float(in.trigger);
      in.triggerTouch = in.trigger > 0.05 || in.triggerClick;
      in.fingerMiddle = in.fingerRing = in.fingerPinky = in.gripClick ? 1.f : 0.f;
      in.gripForce = in.gripClick ? 0.3f : 0.f;
    }
    for (auto [skeleton, handle] : {std::pair{&skeleton_, skeletonHandle_}, std::pair{&skeletonOther_, skeletonOtherHandle_}}) {
      if (!handle) continue;
      if (const auto q = HeadOrientation()) skeleton->SetOrientation(*q);
      skeleton->Update(in, dt);
      for (bool withController : {true, false}) {
        const auto bones = skeleton->Current(withController);
        VRDriverInput()->UpdateSkeletonComponent(
            handle, withController ? VRSkeletalMotionRange_WithController : VRSkeletalMotionRange_WithoutController,
            bones.data(), uint32_t(bones.size()));
      }
    }
  }

  void UpdateInputs(const lh::ControllerInput& raw, double now) {
    if (raw.hostTime == lastInputTime_) return;
    lastInputTime_ = raw.hostTime;
    // A mapped controller's record, by name, for the hand skeleton and the in-use check.
    lh::ControllerInput named = raw;
    if (info_.controllerType == lh::ControllerType::Mapped)
      lh::runtime::MappedToControllerInput(mapped_, raw.mapped, &named);
    const lh::ControllerInput& s = named;
    if (active_ && info_.role == lh::DeviceRole::Controller &&
        (s.systemClick || s.aClick || s.bClick || s.triggerClick || s.thumbstickClick || s.trackpadClick ||
         s.gripClick || s.menuClick || s.trigger > 0.5)) {
      // Used after a minute of rest: this one is in the hand now (the other on a hip,
      // a shelf): it becomes the primary for its hand.
      const auto t = std::chrono::steady_clock::now();
      if (t - lastUse_ > std::chrono::seconds(60) && !autoHand_)
        VRProperties()->SetInt32Property(container_, Prop_ControllerHandSelectionPriority_Int32, NextHandPriority());
      lastUse_ = t;
    }
    // Clamped: the first report after activation can carry a host time from long before.
    const double offset = std::clamp(s.hostTime - now, -0.1, 0.0);
    // Squeeze: the Index reports finger proximity (middle/ring/pinky) and grip force.
    const double curl = (s.fingerMiddle + s.fingerRing + s.fingerPinky) / 3.0;
    const double grip = std::max(curl, s.gripForce);
    const double values[InputCount] = {
        double(s.systemClick), double(s.systemTouch), double(s.aClick), double(s.aTouch), double(s.bClick),
        double(s.bTouch), double(s.triggerClick), double(s.triggerTouch), s.trigger, double(curl > 0.7),
        grip, s.gripForce, double(s.thumbstickClick), double(s.thumbstickTouch), s.thumbstickX, s.thumbstickY,
        double(s.trackpadTouch), s.trackpadX, s.trackpadY, s.trackpadForce, s.fingerIndex, s.fingerMiddle,
        s.fingerRing, s.fingerPinky, double(s.trackpadClick), double(s.gripClick), double(s.menuClick)};
    auto* in = ComponentInput();
    if (skeletonHandle_ || skeletonOtherHandle_) {
      skeletonInput_ = s;
      SendSkeleton(now);
    }
    for (int i = 0; i < InputCount; ++i) {
      if (handles_[i] == k_ulInvalidInputComponentHandle || (sent_[i] && values[i] == last_[i])) continue;
      const EVRInputError e = isBool_[i] ? in->UpdateBooleanComponent(handles_[i], values[i] != 0, offset)
                                         : in->UpdateScalarComponent(handles_[i], float(values[i]), offset);
      if (e != VRInputError_None && failLogged_[i] < 5) {
        ++failLogged_[i];
        Log(info_.serial + ": updating " + (paths_[i] ? paths_[i] : "?") + " (handle " + std::to_string(handles_[i]) +
            ") value " + std::to_string(values[i]) + " offset " + std::to_string(offset) + " -> error " +
            std::to_string(int(e)));
      }
      last_[i] = values[i];
      sent_[i] = true;
    }
    for (auto& m : mappedInputs_) {
      // A neutral (disconnected) record is empty: everything released.
      const double v = lh::runtime::MappedValue(s.mapped, *m.component).value_or(0.0);
      if (m.sent && v == m.last) continue;
      const EVRInputError e = m.component->kind == lh::runtime::MappedComponent::Kind::Bool
                                  ? in->UpdateBooleanComponent(m.handle, v != 0, offset)
                                  : in->UpdateScalarComponent(m.handle, float(v), offset);
      if (e != VRInputError_None && m.failLogged < 5) {
        ++m.failLogged;
        Log(info_.serial + ": updating " + m.component->path + " value " + std::to_string(v) + " -> error " +
            std::to_string(int(e)));
      }
      m.last = v;
      m.sent = true;
    }
  }

  Eigen::Isometry3d rawFromWorld_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d publishedRaw_ = Eigen::Isometry3d::Identity();
  DriverPose_t lastBase_{};      // base stations: the pose last published
  bool basePublished_ = false;
  bool hadInput_ = false;
  DeviceStatic info_;
  std::atomic<bool> active_{false};
  const std::atomic<bool>* externalAlive_ = nullptr;  // AttachExternal: the host still has it
  lh::openvr::PublishedPose* published_ = nullptr;  // AttachExternal: handed every pose we publish
  std::atomic<bool> powerOffRequested_{false};
  std::chrono::steady_clock::time_point lastUse_{};
  int autoHand_ = 0;
  static int32_t NextHandPriority() {
    static std::atomic<int32_t> next{1000};
    return ++next;
  }
  std::atomic<uint32_t> id_{k_unTrackedDeviceIndexInvalid};
  PropertyContainerHandle_t container_ = k_ulInvalidPropertyContainer;
  std::string dongle_;  // SetDongle
  std::mutex poseMutex_;
  DriverPose_t pose_{};
  double recTime_ = -1e300;
  bool recValid_ = false, recHeld_ = false, recConnected_ = false;
  ETrackingResult recResult_ = TrackingResult_Uninitialized;
  DriverPose_t held_{};    // the last valid pose: what a device that loses tracking keeps
  bool haveHeld_ = false;
  double lastPoseTime_ = -1;
  bool lastValid_ = false, lastConnected_ = false;
  ETrackingResult lastResult_ = TrackingResult_Uninitialized;
  int battery_ = -1;
  bool charging_ = false;
  uint64_t universe_ = 0;
  VRInputComponentHandle_t handles_[InputCount] = {};
  bool isBool_[InputCount] = {};
  const char* paths_[InputCount] = {};
  int failLogged_[InputCount] = {};
  double last_[InputCount] = {};
  bool sent_[InputCount] = {};
  VRInputComponentHandle_t haptic_ = k_ulInvalidInputComponentHandle;
  // Mapped controllers (Shiftall FlipVR): the profile's components and their handles.
  struct MappedInput {
    const lh::runtime::MappedComponent* component = nullptr;
    VRInputComponentHandle_t handle = k_ulInvalidInputComponentHandle;
    double last = 0;
    bool sent = false;
    int failLogged = 0;
  };
  lh::runtime::MappedProfile mapped_;
  std::vector<MappedInput> mappedInputs_;
  double lastInputTime_ = -1;
  double lastSkeletonSent_ = -1e300;
  lh::ControllerInput skeletonInput_;
  static constexpr double kSkeletonInterval = 0.005;  // s: at most 200 Hz, with the poses
  lh::openvr::HandSkeleton skeleton_;
  VRInputComponentHandle_t skeletonHandle_ = 0;
  // A wand with no hand yet: the other hand's skeleton too (SteamVR uses the one for the hand it takes).
  lh::openvr::HandSkeleton skeletonOther_;
  VRInputComponentHandle_t skeletonOtherHandle_ = 0;
};

class Provider : public IServerTrackedDeviceProvider {
 public:
  // A throw escaping Init ends vrserver (Error 301, then safe mode blocks every driver):
  // it's logged and the driver doesn't load instead.
  EVRInitError Init(IVRDriverContext* context) override {
    try {
      return InitImpl(context);
    } catch (const std::exception& e) {
      Log(std::string("openhouse failed to start: ") + e.what());
    } catch (...) {
      Log("openhouse failed to start");
    }
    session_.reset();
    return VRInitError_Init_HmdNotFound;
  }
  EVRInitError InitImpl(IVRDriverContext* context) {
    VR_INIT_SERVER_DRIVER_CONTEXT(context);
    context_ = context;
    lh::SessionOptions options;
    if (const char* replay = std::getenv("LH_REPLAY")) options.replayPath = replay;
    if (const char* room = std::getenv("LH_ROOM_MAP")) options.roomMapPath = room;
    // SteamVR's rooms (steam_rooms.hpp): under SteamVR, by default, SteamVR's rooms are
    // imported, a room SteamVR already has Room Setup for uses that universe and its frame,
    // and our other rooms are written into lighthousedb.json as universes of their own.
    // Off ("Use SteamVR's rooms" on the settings page, LH_HOST_FRAME=openhouse, other
    // hosts): our room map is the only source of base poses and the host frame is ours.
    // LH_HOST_FRAME=steamvr pins the host frame to lighthousedb.json's anchor in any room.
    const bool steamvrHost = HostName().rfind("vrserver", 0) == 0;
    const char* hostFrame = std::getenv("LH_HOST_FRAME");
    const std::string hostFrameMode = hostFrame ? hostFrame : "";
    steamFrame_ = hostFrameMode == "steamvr";
    steamRooms_ = false;
    if (steamvrHost) MigrateSteamVRSettings();  // older versions' SteamVR settings page
    if (steamvrHost && hostFrameMode.empty()) {
      const nlohmann::json settings = ReadSettings();
      const auto on = settings.find("useSteamVRRooms");
      steamRooms_ = on == settings.end() || !on->is_boolean() || on->get<bool>();  // no value: on
    }
    if (const char* force = std::getenv("LH_STEAMVR_ROOMS"); force && *force && hostFrameMode.empty())
      steamRooms_ = std::string(force) == "1";  // any host (tests)
    replay_ = !options.replayPath.empty();
    options.steamDatabasePath = steamRooms_ ? "" : "-";
    if (const char* db = std::getenv("LH_STEAM_DB"); db && *db)
      options.steamDatabasePath = std::string(db) == "auto" ? "" : db;
    std::string captureDir;
    // Or "capture" in steamvr.vrsettings under driver_lighthouse (or driver_openhouse): on
    // Windows a new environment variable reaches SteamVR only after Steam restarts.
    // SteamVR only: older Monado's steamvr_lh (Envision's) aborts on a GetString of a
    // key missing from default.vrsettings, and Monado/WiVRn see LH_CAPTURE anyway.
    std::string captureSetting;
    if (steamvrHost) for (const char* section : {"driver_lighthouse", "driver_openhouse"}) {
      char value[1024]{};
      VRSettings()->GetString(section, "capture", value, sizeof value);
      if (*value) { captureSetting = value; break; }
    }
    const char* c = std::getenv("LH_CAPTURE");
    if (!(c && *c) && !captureSetting.empty()) c = captureSetting.c_str();
    if (c && *c) {
      // Never overwrite an earlier session: a used directory gets a numbered sibling.
      captureDir = c;
      for (int n = 2; std::filesystem::exists(captureDir); ++n) captureDir = std::string(c) + "-" + std::to_string(n);
      std::error_code ec;
      std::filesystem::create_directories(captureDir, ec);
      if (ec || !g_recorder.Open(captureDir + "/driver.jsonl")) {
        Log("LH_CAPTURE: cannot create " + captureDir);
        captureDir.clear();
      } else if (options.replayPath.empty()) {
        options.capturePath = captureDir + "/input.lhraw";  // a replay already is one
      }
    }
    // Keep the host's chaperone frame separate from our optimizable room frame.
    std::string reference;
    if (const char* p = std::getenv("LH_REFERENCE_DB")) reference = p;
    else reference = lh::platform::SteamConfigDirectory() + "/lighthousedb.json";
    std::string alignmentError;
    if (steamFrame_ && !reference.empty() && !alignment_.Load(reference, &alignmentError)) Log(alignmentError);
    if (steamRooms_) {
      referencePath_ = reference;
      std::string text;
      {
        std::ifstream f(std::filesystem::u8path(reference), std::ios::binary);
        std::ostringstream all;
        if (f) all << f.rdbuf();
        text = all.str();
      }
      std::string chaperone = lh::platform::SteamRoot() + "/config/chaperone_info.vrchap";
      if (const char* p = std::getenv("LH_CHAPERONE")) chaperone = p;
      const auto setUp = lh::openvr::ChaperoneUniverses(chaperone);
      setUpUniverses_ = setUp;
      // SteamVR's universes (an imported room stays in its own), and those with Room Setup.
      if (!text.empty()) {
        if (!alignment_.LoadText(lh::openvr::ForeignUniverses(text, AllUniverseIds(text)), &alignmentError)) Log(alignmentError);
        if (!setUpAlignment_.LoadText(lh::openvr::ForeignUniverses(text, setUp), &alignmentError)) Log(alignmentError);
      }
      std::string ids;
      for (const auto& id : setUp) ids += (ids.empty() ? "" : ", ") + id;
      Log("SteamVR's rooms: on (" + reference + "; Room Setup done for " + (ids.empty() ? std::string("no universe") : ids) + ")");
    }
    if (!steamFrame_) {
      const std::string roomMap = options.roomMapPath.empty() ? lh::platform::ConfigDirectory() + "/roommap.json"
                                                              : options.roomMapPath;
      hostFrame_.Load(lh::platform::PathUtf8(lh::platform::FsPath(roomMap).parent_path()) + "/hostframe.json");
    }
    Log(steamFrame_ ? "host frame: SteamVR's (" + reference + ")" : "host frame: our room's, pinned to its origin base");
    if (!captureDir.empty()) {
      std::error_code ec;
      std::filesystem::copy_file(reference, captureDir + "/lighthousedb.json", ec);
      Log("recording a diagnostic session to " + captureDir);
    }
    session_ = lh::Session::Create(options);
    std::string error;
    poweredOffForExit_ = false;
    if (!session_->Start(&error)) {
      Log("could not start: " + error);
      session_.reset();
      return VRInitError_Init_HmdNotFound;
    }
    ApplyTuning(true);
    Log("openhouse " + std::string(lh::kVersion) + ": liblh session started (" + std::string(options.replayPath.empty() ? "live USB" : "replay " + options.replayPath) +
        "); head-mounted displays: tracked by us, displayed through the stock driver when one is present");
    Log(std::string("input components through ") +
        (ComponentInput() != VRDriverInput() ? "IVRDriverInput_005" : IVRDriverInput_Version));
    // SteamVR's dongle pairing menu: ours owns the dongles (registered before the stock
    // driver loads, which is kept off this mailbox).
    if (options.replayPath.empty()) pairing_.Open(context_, [](const std::string& text) { Log(text); });
    {
      std::lock_guard lock(radioMutex_);
      radioSession_ = session_.get();
    }
    stock_.SetRequestHandler([this](const char* request, char* response, uint32_t size) {
      return RadioRequest(request, response, size);
    });
    // A headset on USB: bring up the stock driver for its display now, inside the host's
    // device discovery window.
    using Presence = lh::openvr::StockHmd::Presence;
    std::string which;
    std::vector<std::string> maybe;
    std::string unwanted;
    const bool noStock = StockHmdUnwanted(&unwanted);
    Presence presence = noStock ? Presence::No : lh::openvr::StockHmd::HeadsetPresent(&which, &maybe);
    if (noStock && unwanted != "LH_NO_STOCK_HMD") Log("lighthouse headsets aren't displayed here (" + unwanted + ")");
    // Wired 0x2300 devices that may be headsets (Index, Beyond) are asked by their configs even
    // when a 0x2000 one (Vive, Vive Pro) already made it a yes: with an old Vive left plugged in
    // beside an Index, the Index was hidden from the stock driver, which then found only the
    // Vive and no display for it.
    if (!maybe.empty() && (presence == Presence::Maybe || presence == Presence::Yes)) {
      // Ask the device configs (role from device_class), up to 2 s: inside Monado's 3 s.
      const bool already = presence == Presence::Yes;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
      std::set<std::string> pending(maybe.begin(), maybe.end());
      while (!pending.empty() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        for (const auto& d : session_->Snapshot().devices) {
          if (!pending.count(d.info.serial)) continue;
          pending.erase(d.info.serial);
          if (d.info.role == lh::DeviceRole::Hmd) {
            presence = Presence::Yes;
            which = already ? which + " and " + d.info.serial : d.info.serial + " (headset by its config)";
            lh::openvr::StockHmd::AllowHeadset(d.info.serial);
          }
        }
      }
      if (presence != Presence::Yes) {
        Log(pending.empty() ? "wired lighthouse devices are not headsets (by their configs)"
                            : "no config from a wired lighthouse device within 2 s; not loading the stock driver now");
        presence = Presence::No;
      }
    }
    if (presence == Presence::Yes) {
      triedStock_ = true;
      const std::string path = lh::openvr::StockHmd::DefaultPath();
      if (path.empty()) Log("headset " + which + " on USB, but SteamVR's own lighthouse driver wasn't found to display it (" + std::string(kStockFile) + " or SteamVR's drivers/lighthouse)");
      else {
        Log("headset " + which + " on USB: loading the stock driver for its display");
        stock_.Load(path, context_, [](const std::string& text) { Log(text); });
      }
    }
    if (const int port = lh::telemetry::PortFromEnvironment(); port > 0) {
      std::string telemetryError;
      if (g_telemetry.Start(port, HostName(), &telemetryError)) Log(lh::LogLevel::Debug, "viewer telemetry on 127.0.0.1:" + std::to_string(port));
      else Log(lh::LogLevel::Warn, "viewer telemetry unavailable: " + telemetryError);
    }
    stop_ = false;
    thread_ = std::thread([this] { Main(); });
    return VRInitError_None;
  }

  void Cleanup() override {
    PowerOffForExit("SteamVR exiting");  // if EnterStandby didn't already
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    if (reportThread_.joinable()) reportThread_.join();  // a report being saved finishes first
    g_telemetry.Stop();
    {
      std::lock_guard lock(radioMutex_);  // waits for a request being answered
      radioSession_ = nullptr;
    }
    if (session_) session_->Stop();
    session_.reset();
    g_recorder.Close();
    devices_.clear();
    snapshot_ = {};
    alignment_ = {};
    setUpAlignment_ = {};
    setUpUniverses_.clear();
    roomFrame_ = {};
    writtenUniverse_.clear();
    lastUniverseWrite_ = -1e300;
    roomPublisher_.Close();
    warnedRoom_.clear();
    lastSnapshot_ = -1e300;
    { std::lock_guard lock(eventMutex_); hapticEvents_.clear(); }
    loggedHaptic_.clear();
    loggedUnrouted_ = false;
    stock_.Cleanup();
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
  }

  const char* const* GetInterfaceVersions() override { return k_InterfaceVersions; }

  void RunFrame() override {
    VREvent_t event;
    while (VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
      stock_.PushEvent(event);  // the stock driver (headset display) sees the host's events too
      if (event.eventType == VREvent_Input_HapticVibration) {
        std::lock_guard lock(eventMutex_);
        if (hapticEvents_.size() < 64) hapticEvents_.push_back(event);
      }
    }
    // A headset is attached: bring up the stock driver for its display (host thread).
    if (wantStock_ && !triedStock_) {
      triedStock_ = true;
      const std::string path = lh::openvr::StockHmd::DefaultPath();
      std::string unwanted;
      if (StockHmdUnwanted(&unwanted)) Log("a head-mounted display is attached; not exposed (" + unwanted + ")");
      else if (path.empty()) Log("a head-mounted display is attached, but SteamVR's own lighthouse driver wasn't found to display it (" + std::string(kStockFile) + " or SteamVR's drivers/lighthouse)");
      else stock_.Load(path, context_, [](const std::string& text) { Log(text); });
    }
    stock_.RunFrame();
    if (session_) {
      std::string expected;
      {
        std::lock_guard lock(expectedMutex_);
        expected = expectedControllers_;
      }
      // A malformed mailbox message must not leave RunFrame: it ends vrserver.
      try {
        pairing_.Poll(*session_, expected);
      } catch (const std::exception& e) {
        Log(std::string("pairing: ignored a request: ") + e.what());
      }
    }
  }

  // The headset's expected controllers (Prop_ExpectedControllerType_String), "" if none.
  // Main thread only: it is the one that changes devices_ (RunFrame reads the copy,
  // expectedControllers_; walking the map there raced Main's inserts).
  std::string ExpectedControllerType() const {
    for (const auto& [serial, device] : devices_)
      if (device && device->IsHmd() && device->Active()) return device->StringProperty(Prop_ExpectedControllerType_String);
    return {};
  }

  // Only leaving standby reaches the stock driver: it powers the headset's panels back up
  // there (the headset's own EnterStandby, through DeviceProxy, turned them off). Without
  // it a headset that went to sleep stayed dark. Its provider EnterStandby isn't
  // forwarded: it would power off controllers it doesn't own here.
  bool ShouldBlockStandbyMode() override { return false; }
  // SteamVR calls this first when it quits, Cleanup later. The devices are powered off
  // here, as the stock driver does: if vrserver goes down on its way out, Cleanup may
  // never run.
  void EnterStandby() override {
    Log("SteamVR entering standby");
    PowerOffForExit("SteamVR entering standby");
  }

  // SteamVR's "Turn off controllers when exiting SteamVR" (power/powerOffOnExit, default
  // on): its wireless controllers and trackers, once per session, while the USB thread
  // still runs to send it.
  void PowerOffForExit(const char* why) {
    if (!session_ || poweredOffForExit_ || std::getenv("LH_NO_POWER_OFF_ON_EXIT")) return;
    bool on = true;  // the default, also when the setting can't be read
    if (VRSettings()) {
      EVRSettingsError error = VRSettingsError_None;
      const bool value = VRSettings()->GetBool("power", "powerOffOnExit", &error);
      if (error == VRSettingsError_None) on = value;
    }
    if (!on) return;
    poweredOffForExit_ = true;
    int queued = 0;
    for (const auto& d : session_->Snapshot().devices)
      if (d.info.connected && (d.info.role == lh::DeviceRole::Controller || d.info.role == lh::DeviceRole::Tracker))
        queued += session_->PowerOff(d.info.serial);
    Log(std::string(why) + ": powering off " + std::to_string(queued) + " device(s)");
    if (queued) std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  bool poweredOffForExit_ = false;
  void LeaveStandby() override {
    poweredOffForExit_ = false;  // back from standby: the next one (or the exit) powers off again
    Log("SteamVR leaving standby");
    stock_.LeaveStandby();
  }

 private:
  void Main() {
    while (!stop_) {
      const double now = session_->HostNow();
      for (auto& e : session_->PollEvents()) {
        Log(e.level, e.text, e.serial, now);
        if (g_recorder.On()) {
          char t[32];
          std::snprintf(t, sizeof t, "%.6f", now);
          g_recorder.Line(std::string("{\"t\":") + t + ",\"level\":\"" + lh::LogLevelName(e.level) + "\",\"event\":" +
                          Recorder::Quote(e.text) + "}");
        }
      }
      std::deque<VREvent_t> haptics;
      { std::lock_guard lock(eventMutex_); haptics.swap(hapticEvents_); }
      for (const auto& event : haptics) {
        bool matched = false;
        if (g_recorder.On()) {
          const auto& h = event.data.hapticVibration;
          std::string target;
          for (const auto& [serial, d] : devices_) if (d->MatchesHaptic(event)) target = serial;
          char text[320];
          std::snprintf(text, sizeof text,
                        "{\"t\":%.6f,\"haptic\":{\"device\":%u,\"container\":%llu,\"component\":%llu,\"duration\":%.6g,"
                        "\"frequency\":%.6g,\"amplitude\":%.6g,\"age\":%.4f},\"routed\":%s}",
                        now, event.trackedDeviceIndex, (unsigned long long)h.containerHandle,
                        (unsigned long long)h.componentHandle, h.fDurationSeconds, h.fFrequency, h.fAmplitude,
                        event.eventAgeSeconds, target.empty() ? "null" : Recorder::Quote(target).c_str());
          g_recorder.Line(text);
        }
        for (const auto& [serial, d] : devices_) if (d->MatchesHaptic(event)) {
          matched = true;
          const auto& h = event.data.hapticVibration;
          // Values go to the transport unchanged; HapticReport ports stock 0x3f4e50,
          // including duration 0 (one pulse, as SteamVR Identify sends every frame).
          // A zero amplitude is a no-op here (as in Monado's steamvr_lh): forwarding it
          // as a stop would replace a still-pending pulse in the transport queue.
          if (!(h.fAmplitude > 0)) continue;
          if (!loggedHaptic_.count(serial)) {
            loggedHaptic_.insert(serial);
            char details[160];
            std::snprintf(details, sizeof details, " (first event: duration=%.6g frequency=%.6g amplitude=%.6g)", h.fDurationSeconds, h.fFrequency, h.fAmplitude);
            Log("haptics routed to " + serial + details);
          }
          const bool queued = session_->Haptic(serial, h.fFrequency, h.fAmplitude, h.fDurationSeconds);
          char details[160];
          std::snprintf(details, sizeof details, " duration=%.6g frequency=%.6g amplitude=%.6g", h.fDurationSeconds, h.fFrequency, h.fAmplitude);
          Log(lh::LogLevel::Debug, "haptic " + serial + (queued ? " queued" : " ignored (replay/offline/disabled)") + details, serial);
        }
        if (!matched) {
          const auto& h = event.data.hapticVibration;
          Log(loggedUnrouted_.exchange(true) ? lh::LogLevel::Debug : lh::LogLevel::Info, "unrouted haptic: device=" + std::to_string(event.trackedDeviceIndex) + " container=" +
              std::to_string(h.containerHandle) + " component=" + std::to_string(h.componentHandle));
        }
      }
      for (const auto& [serial, d] : devices_) if (d->TakePowerOffRequest()) {
        const bool queued = session_->PowerOff(serial);
        Log("power-off " + serial + (queued ? " queued" : " ignored (replay/offline/disabled)"));
      }
      const auto updates = session_->Updates();
      // Base stations, configs and the room only change slowly.
      if (now - lastSnapshot_ > 0.25 || devices_.empty()) {
        lastSnapshot_ = now;
        snapshot_ = session_->Snapshot();
        RegisterNewDevices();
        for (const auto& d : snapshot_.devices)
          if (d.info.role == lh::DeviceRole::Hmd && d.info.enabled && d.info.connected) wantStock_ = true;
        while (const auto hmd = stock_.TakeActivated()) {
          // The stock driver's headset is up: our tracking drives its pose.
          DeviceStatic info{hmd->serial, {}, {}, {}, {}, lh::DeviceRole::Hmd, {}};
          auto device = std::make_unique<Device>(info);
          // Its GetPose returns what we publish (other drivers ask: Bigscreen's eye tracking).
          device->AttachExternal(hmd->objectId, hmd->alive, hmd->published);
          devices_[hmd->serial] = std::move(device);
          stock_.OwnPose(hmd->objectId);
          Log("head-mounted display " + hmd->serial + ": pose from our tracking");
        }
        Eigen::Isometry3d rawFromWorld;
        uint64_t universe = 0;
        if (steamRooms_) {
          SteamRoomFrame(now, &rawFromWorld, &universe);
        } else {
          const bool aligned = steamFrame_ ? alignment_.Update(snapshot_) : !snapshot_.roomId.empty();
          rawFromWorld = steamFrame_ ? alignment_.RawFromWorld() : hostFrame_.Update(snapshot_, now);
          if (steamFrame_ && !aligned && !snapshot_.roomId.empty() && snapshot_.roomId != warnedRoom_) {
            warnedRoom_ = snapshot_.roomId;
            Log("no shared reference base for room " + snapshot_.roomId + "; host room setup is required for a standing origin");
          }
          universe = UniverseId(aligned && steamFrame_ ? alignment_.RoomId() : snapshot_.roomId);
        }
        rawFromWorld = BlendFrame(rawFromWorld, universe, now);
        roomPublisher_.Publish(snapshot_, rawFromWorld, now, session_->RoomResets());
        if (const std::string line = roomPublisher_.TakeLog(); !line.empty()) Log(line);
        for (auto& [serial, d] : devices_) {
          d->SetRawTransform(rawFromWorld);
          d->SetUniverse(universe);
        }
        {
          const std::string expected = ExpectedControllerType();
          std::lock_guard lock(expectedMutex_);
          expectedControllers_ = expected;
        }
        for (const auto& b : snapshot_.bases) {
          auto found = devices_.find(b.serial);
          if (found != devices_.end()) found->second->UpdateBase(b);
        }
        std::map<std::string, std::string> dongles;  // device -> its receiver
        for (const auto& r : session_->Receivers())
          if (!r.connected.empty()) dongles[r.connected] = r.serial;
        for (auto& [serial, d] : devices_) {
          const auto r = dongles.find(serial);
          if (r != dongles.end()) d->SetDongle(r->second);
        }
      }
      if (now - lastTuning_ >= 1.0) {  // settings.json edits apply while playing
        lastTuning_ = now;
        ApplyTuning(false);
      }
      if (now - lastTelemetry_ >= 0.05) {  // viewers: ~20 Hz is plenty to watch
        lastTelemetry_ = now;
        g_telemetry.PublishState(now, snapshot_, updates, session_->Receivers(),
                                 {roomPublisher_.Settled(), roomPublisher_.Epoch(), lh::openvr::RoomSettle::kMaxSigma,
                                  session_->RoomResets()});
      }
      for (const auto& u : updates) {
        auto found = devices_.find(u.serial);
        if (found != devices_.end()) found->second->Update(u, SteadyNowFor(now));
      }
      AssignWandHands(updates);
      HandleCommands();
      if (now - lastSettings_ >= 0.25) {
        lastSettings_ = now;
        PublishSettings();
      }
      lh::platform::SleepFor(std::chrono::microseconds(500));
    }
  }

  double SteadyNowFor(double sessionNow) const {
    // Live sessions use the steady clock already; replays run on recorded host time.
    return session_ && std::getenv("LH_REPLAY") ? sessionNow : SteadyNow();
  }

  void RegisterNewDevices() {
    // SteamVR fills an empty hand slot with whichever controller comes next, also a
    // second one for the other hand (live: a right controller on a hip took the left
    // slot because the left one registered after it). So per batch: one controller per
    // free hand first (left, then right), then extra same-hand controllers, then trackers.
    std::set<std::string> handsTaken;
    for (const auto& [serial, device] : devices_)
      if (device && device->Hand().size()) handsTaken.insert(device->Hand());
    std::vector<const lh::DeviceState*> order;
    for (const auto& d : snapshot_.devices) order.push_back(&d);
    auto rank = [&](const lh::DeviceState* d) {
      if (d->info.role != lh::DeviceRole::Controller) return 3;
      if (d->info.controllerRole == "left_hand") return 0;
      if (d->info.controllerRole == "right_hand") return 1;
      return 2;
    };
    std::stable_sort(order.begin(), order.end(), [&](auto* a, auto* b) { return rank(a) < rank(b); });
    std::vector<const lh::DeviceState*> extras;
    for (auto* d : order) {
      const std::string& hand = d->info.controllerRole;
      if (d->info.role == lh::DeviceRole::Controller && !hand.empty() && !handsTaken.insert(hand).second) extras.push_back(d);
    }
    std::stable_partition(order.begin(), order.end(), [&](auto* d) { return std::find(extras.begin(), extras.end(), d) == extras.end(); });
    std::stable_partition(order.begin(), order.end(), [&](auto* d) { return d->info.role == lh::DeviceRole::Controller; });
    for (const auto* dp : order) {
      const auto& d = *dp;
      if (devices_.count(d.info.serial) || !d.info.enabled || d.info.role == lh::DeviceRole::Hmd) continue;
      DeviceStatic info{d.info.serial, d.info.model, d.info.manufacturer, RenderModel(d.info.renderModel), d.info.controllerRole,
                        d.info.role, {}, d.info.controllerType};
      info.resourceRoot = d.info.resourceRoot;
      info.inputProfile = d.info.inputProfile;
      info.registeredType = d.info.registeredType;
      info.tundraTracker = d.info.IsTundraTracker();
      info.hidden = EqualsNoCase(d.info.deviceClass, "null");
      Add(info, info.hidden ? TrackedDeviceClass_Invalid
                : d.info.role == lh::DeviceRole::Controller ? TrackedDeviceClass_Controller
                                                            : TrackedDeviceClass_GenericTracker);
    }
    for (const auto& b : snapshot_.bases) {
      if (const auto found = devices_.find(b.serial); found != devices_.end()) {
        if (!b.channel.empty()) found->second->SetChannel(b.channel);
        found->second->SetLh2(b.lh2);
        continue;
      }
      if (!b.seen) continue;
      DeviceStatic info;
      info.serial = b.serial;
      info.role = lh::DeviceRole::BaseStation;
      info.baseChannel = b.channel;
      info.lh2 = b.lh2;
      Add(info, TrackedDeviceClass_TrackingReference);
    }
  }

  void Add(const DeviceStatic& info, ETrackedDeviceClass cls) {
    auto device = std::make_unique<Device>(info);
    Device* raw = device.get();
    devices_[info.serial] = std::move(device);
    // The host may call Activate() from inside this call (Monado does).
    if (!VRServerDriverHost()->TrackedDeviceAdded(info.serial.c_str(), cls, raw->ForHost()))
      Log("host declined " + info.serial + (cls == TrackedDeviceClass_TrackingReference ? " (base station)" : ""));
    else
      Log("registered " + info.serial + (info.hidden ? " (device_class null: hidden from SteamVR's device list)" : ""));
  }

  // SteamVR's base station power management asks the headset to switch the Bluetooth
  // receiver's radio (usb/radio_mode.hpp). That receiver is ours, so ours answers, the way
  // the stock driver does (0x1eaa9). False: there is none, and the stock driver says so.
  bool RadioRequest(const char* request, char* response, uint32_t size) {
    if (!request || !response || !size) return false;
    const bool toBluetooth = !std::strcmp(request, "set_radio_mode_ble");
    const bool toValve = !std::strcmp(request, "set_radio_mode_valve");
    const bool read = !std::strcmp(request, "get_radio_mode");
    if (!toBluetooth && !toValve && !read) return false;
    std::lock_guard lock(radioMutex_);
    if (!radioSession_ || !radioSession_->HasBluetoothRadio()) return false;
    const char* answer = "0";
    if (read) {
      if (const auto on = radioSession_->BluetoothRadioOn()) answer = *on ? "B" : "V";
    } else if (radioSession_->SetBluetoothRadio(toBluetooth)) {
      answer = "1";
    }
    std::snprintf(response, size, "%s", answer);
    return true;
  }

  std::set<std::string> loggedHaptic_;
  std::atomic<bool> loggedUnrouted_{false};
  std::mutex eventMutex_;
  std::mutex expectedMutex_;
  std::string expectedControllers_;  // ExpectedControllerType(), from Main for RunFrame
  std::deque<VREvent_t> hapticEvents_;
  std::unique_ptr<lh::Session> session_;
  std::mutex radioMutex_;
  lh::Session* radioSession_ = nullptr;  // session_ while it can answer; under radioMutex_
  IVRDriverContext* context_ = nullptr;
  lh::openvr::StockHmd stock_;
  lh::openvr::PairingMailbox pairing_;
  std::atomic<bool> wantStock_{false};
  bool triedStock_ = false;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::map<std::string, std::unique_ptr<Device>> devices_;
  lh::SessionSnapshot snapshot_;
  lh::openvr::RawAlignment alignment_;
  bool steamFrame_ = false;
  // The host frame eases into a changed room frame (a base re-solved moves it a few mm or cm)
  // over ~0.3 s instead of every device and base jumping with it. A new room (universe) or a
  // jump too big to be a correction (over 0.5 m or 20 deg) is taken at once.
  Eigen::Isometry3d BlendFrame(const Eigen::Isometry3d& target, uint64_t universe, double now) {
    const Eigen::Quaterniond qt(target.rotation()), qs(blendedRaw_.rotation());
    const double move = (target.translation() - blendedRaw_.translation()).norm(), turn = qs.angularDistance(qt);
    if (!haveBlended_ || universe != blendedUniverse_ || move > 0.5 || turn > 20 * M_PI / 180) {
      blendedRaw_ = target;
    } else if (move > 1e-5 || turn > 1e-5) {
      const double a = 1 - std::exp(-std::clamp(now - blendT_, 0.0, 0.1) / 0.1);   // time constant 0.1 s: ~0.3 s to settle
      Eigen::Isometry3d b = Eigen::Isometry3d::Identity();
      b.linear() = qs.slerp(a, qt).normalized().toRotationMatrix();
      b.translation() = blendedRaw_.translation() + a * (target.translation() - blendedRaw_.translation());
      blendedRaw_ = b;
    } else {
      blendedRaw_ = target;
    }
    haveBlended_ = true;
    blendedUniverse_ = universe;
    blendT_ = now;
    return blendedRaw_;
  }
  Eigen::Isometry3d blendedRaw_ = Eigen::Isometry3d::Identity();
  bool haveBlended_ = false;
  uint64_t blendedUniverse_ = 0;
  double blendT_ = 0;

  // SteamVR's rooms (steam_rooms.hpp): the frame and universe of the active room.
  bool steamRooms_ = false;
  bool replay_ = false;
  std::string referencePath_;
  lh::openvr::RawAlignment setUpAlignment_;  // SteamVR's universes with Room Setup done
  std::set<std::string> setUpUniverses_;      // every universe with Room Setup (chaperone_info.vrchap)
  enum class RoomFrame { Undecided, Steam, Own };
  struct {
    std::string room;
    RoomFrame frame = RoomFrame::Undecided;
    double since = 0;
    Eigen::Isometry3d rawFromWorld = Eigen::Isometry3d::Identity();
    uint64_t universe = 0;
  } roomFrame_;
  std::string writtenUniverse_;  // what we last wrote into lighthousedb.json
  double lastUniverseWrite_ = -1e300;

  static std::set<std::string> AllUniverseIds(const std::string& lighthousedb) {
    std::set<std::string> ids;
    const auto j = nlohmann::json::parse(lighthousedb, nullptr, false);
    if (!j.is_object() || !j.contains("known_universes") || !j.at("known_universes").is_array()) return ids;
    for (const auto& u : j.at("known_universes")) {
      if (!u.is_object() || !u.contains("id")) continue;
      const auto& id = u.at("id");
      if (id.is_string()) ids.insert(id.get<std::string>());
      else if (id.is_number_unsigned()) ids.insert(std::to_string(id.get<uint64_t>()));
    }
    return ids;
  }

  // The host frame and universe under SteamVR's rooms. A room imported from SteamVR
  // ("steamvr-<id>") stays in that universe's frame. One of ours takes a SteamVR universe
  // that has Room Setup and shares a base with it (decided within the room's first 5 s,
  // then kept for the session: a frame never switches under the user); otherwise it is
  // its own universe, in our pinned host frame, written into lighthousedb.json.
  void SteamRoomFrame(double now, Eigen::Isometry3d* rawFromWorld, uint64_t* universe) {
    auto& f = roomFrame_;
    if (snapshot_.roomId != f.room) f = {snapshot_.roomId, RoomFrame::Undecided, now, Eigen::Isometry3d::Identity(), 0};
    const bool imported = f.room.rfind("steamvr-", 0) == 0;
    if (f.room.empty()) {
      *rawFromWorld = hostFrame_.Update(snapshot_, now);
      *universe = 0;
      return;
    }
    // Room Setup done in our room's own universe (as with 1.0.2, which had no SteamVR rooms):
    // that is the user's room. A SteamVR universe sharing a base would replace its floor and
    // tilt with another room's, re-estimated by the stock driver on every start.
    if (f.frame == RoomFrame::Undecided && !imported && setUpUniverses_.count(std::to_string(UniverseId(f.room)))) {
      f.frame = RoomFrame::Own;
      Log("room " + f.room + ": its own universe " + std::to_string(UniverseId(f.room)) + " (Room Setup done for it)");
    }
    auto& steam = imported ? alignment_ : setUpAlignment_;
    if (f.frame != RoomFrame::Own && steam.Update(snapshot_)) {
      if (f.frame == RoomFrame::Undecided)
        Log("room " + f.room + ": SteamVR's universe " + steam.RoomId().substr(8) +
            (imported ? "" : " (Room Setup from SteamVR)") + ", in SteamVR's frame");
      f.frame = RoomFrame::Steam;
      f.rawFromWorld = steam.RawFromWorld();
      f.universe = UniverseId(steam.RoomId());
    } else if (f.frame == RoomFrame::Undecided && !imported && now - f.since > 5) {
      f.frame = RoomFrame::Own;
      Log("room " + f.room + ": its own universe " + std::to_string(UniverseId(f.room)) +
          " (no SteamVR room with Room Setup shares a base station)");
    }
    if (f.frame == RoomFrame::Steam) {  // the last alignment holds while its anchor is out
      *rawFromWorld = f.rawFromWorld;
      *universe = f.universe;
    } else if (f.frame == RoomFrame::Own) {
      *rawFromWorld = hostFrame_.Update(snapshot_, now);
      *universe = UniverseId(f.room);
      WriteOwnUniverse(now, *rawFromWorld);
    } else {
      *rawFromWorld = hostFrame_.Update(snapshot_, now);
      *universe = 0;  // until decided: no universe to set up (SteamVR waits for one)
    }
  }

  // Our room as a lighthousedb.json universe: its learned bases where the host frame has
  // them. Rewritten when a base is learned or moves 1 cm / 1 deg, at most every 30 s.
  void WriteOwnUniverse(double now, const Eigen::Isometry3d& rawFromWorld) {
    // A replay writes only into a file it was given (LH_REFERENCE_DB), never SteamVR's.
    if ((replay_ && !std::getenv("LH_REFERENCE_DB")) || referencePath_.empty() || !hostFrame_.Anchored() || now - lastUniverseWrite_ < 30) return;
    std::vector<lh::openvr::UniverseBase> bases;
    std::string key = roomFrame_.room;
    for (const auto& b : snapshot_.bases) {
      if (!b.known || !b.learned) continue;
      const auto& p = b.worldFromBase;
      Eigen::Isometry3d worldFromBase = Eigen::Isometry3d::Identity();
      worldFromBase.linear() = Eigen::Quaterniond(p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]).normalized().toRotationMatrix();
      worldFromBase.translation() = Eigen::Vector3d(p.position.data());
      lh::openvr::UniverseBase u;
      u.serial = b.id;
      u.rawFromBase = rawFromWorld * worldFromBase;
      u.variance = b.sigma * b.sigma;
      bases.push_back(u);
      const Eigen::Vector3d t = u.rawFromBase.translation() * 100;  // cm
      const Eigen::Vector3d r = Eigen::AngleAxisd(u.rawFromBase.rotation()).axis() *
                                Eigen::AngleAxisd(u.rawFromBase.rotation()).angle() * 57.29577951308232;
      char text[160];
      std::snprintf(text, sizeof text, " %08X:%.0f,%.0f,%.0f:%.0f,%.0f,%.0f", b.id, t.x(), t.y(), t.z(), r.x(), r.y(), r.z());
      key += text;
    }
    if (bases.empty() || key == writtenUniverse_) return;
    lastUniverseWrite_ = now;
    std::string error;
    const std::string id = std::to_string(UniverseId(roomFrame_.room));
    if (lh::openvr::WriteUniverse(referencePath_, id, bases, &error)) {
      if (writtenUniverse_.empty())
        Log("room " + roomFrame_.room + " written into " + referencePath_ + " as universe " + id + " (" +
            std::to_string(bases.size()) + " bases)");
      writtenUniverse_ = key;
    } else {
      Log(lh::LogLevel::Warn, error);
    }
  }
  lh::openvr::HostFrame hostFrame_;
  lh::openvr::RoomPublisher roomPublisher_;
  std::string warnedRoom_;
  double lastSnapshot_ = -1e300;
  double lastTelemetry_ = -1e300;
  std::string recording_;  // the settings page's recording, while on
  // openhouse-overlay: settings it set on a host with no settings page (by SteamVR key), and
  // what ApplyTuning last applied (published to it).

  nlohmann::json applied_ = nlohmann::json::object();
  double lastSettings_ = -1e300;
  std::thread reportThread_;  // saving a bug report
  std::atomic<bool> reportDone_{true};
  // The last bug report saved (openhouse-overlay shows where): its path, or "" with the error.
  std::mutex reportMutex_;
  std::string lastReport_, lastReportError_;
  uint64_t reportCount_ = 0;

  // ---- openhouse-overlay (telemetry commands, lh/telemetry.hpp) ----
  // The settings it may change: SteamVR settings keys (the settings page's), with their type.
  enum class SettingType { Bool, Number };
  static std::optional<SettingType> OverlaySetting(const std::string& key) {
    static const std::map<std::string, SettingType> keys = {
        {"positionSmoothingMs", SettingType::Number}, {"rotationSmoothingMs", SettingType::Number},
        {"predictWithAcceleration", SettingType::Bool},
        {"mixedLight", SettingType::Bool}, {"mixedLightControllers", SettingType::Bool},
        {"mixedLightTrackers", SettingType::Bool}, {"mixedLightScout", SettingType::Bool},
        {"mixedLightPreferLh2", SettingType::Bool}, {"mixedLightRotorReference", SettingType::Bool},
        {"useSteamVRRooms", SettingType::Bool}};  // takes effect when the runtime restarts
    const auto it = keys.find(key);
    return it == keys.end() ? std::nullopt : std::optional<SettingType>(it->second);
  }

  static bool steamvrHostNow() { return HostName().rfind("vrserver", 0) == 0; }

  // Room calibration from openhouse-overlay (host_frame.hpp), with the devices' poses where
  // the runtime sees them: "floor" (this device is on the floor, `height` m above it),
  // "forward" (where it points is forward), "center" (where it is is the middle),
  // "nudgeFloor" (`metres`), "nudgeYaw" (`degrees`), "reset".
  bool Calibrate(const nlohmann::json& j, std::string* error) {
    if (steamvrHostNow() || steamFrame_) { *error = "SteamVR has its own Room Setup"; return false; }
    if (!hostFrame_.Anchored()) { *error = "the room isn't placed yet"; return false; }
    const std::string action = j.value("action", "");
    Eigen::Isometry3d c = hostFrame_.Calibration();
    auto yawAbout = [](double radians) {
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      t.linear() = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitY()).toRotationMatrix();
      return t;
    };
    if (action == "reset") c.setIdentity();
    else if (action == "floorControllers") {
      // The controllers lying on the floor: their middle sits about this high above it.
      constexpr double kControllerOnFloor = 0.04;  // m
      double sum = 0;
      int n = 0;
      for (const auto& d : snapshot_.devices) {
        if (d.info.role != lh::DeviceRole::Controller || !d.info.connected || !d.pose.valid) continue;
        const auto& p = d.pose.worldFromHead.position;
        sum += (c * hostFrame_.Pinned() * Eigen::Vector3d(p[0], p[1], p[2])).y();
        ++n;
      }
      if (!n) { *error = "no controller is tracking"; return false; }
      c.translation().y() -= sum / n - kControllerOnFloor;
    }
    else if (action == "nudgeFloor") c.translation().y() -= j.value("metres", 0.0);
    else if (action == "nudgeYaw") c = yawAbout(j.value("degrees", 0.0) * 0.017453292519943295) * c;
    else {
      const std::string serial = j.value("serial", "");
      const lh::DeviceState* device = nullptr;
      for (const auto& d : snapshot_.devices) if (d.info.serial == serial) device = &d;
      if (!device || !device->pose.valid) { *error = serial + " isn't tracking"; return false; }
      // The device where the runtime sees it now.
      const auto& p = device->pose.worldFromHead;
      Eigen::Isometry3d worldFromDevice = Eigen::Isometry3d::Identity();
      worldFromDevice.linear() = Eigen::Quaterniond(p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]).normalized().toRotationMatrix();
      worldFromDevice.translation() = Eigen::Vector3d(p.position.data());
      const Eigen::Isometry3d raw = c * hostFrame_.Pinned() * worldFromDevice;
      if (action == "floor") {
        c.translation().y() -= raw.translation().y() - j.value("height", 0.0);
      } else if (action == "center") {
        c.translation().x() -= raw.translation().x();
        c.translation().z() -= raw.translation().z();
      } else if (action == "forward") {
        // Where it points (-z), on the floor, becomes forward (-z): a turn about the middle.
        const Eigen::Vector3d f = raw.rotation() * Eigen::Vector3d(0, 0, -1);
        if (std::hypot(f.x(), f.z()) < 0.3) { *error = "point it level, not up or down"; return false; }
        c = yawAbout(std::atan2(f.x(), -f.z())) * c;
      } else {
        *error = "unknown calibration " + action;
        return false;
      }
    }
    if (!hostFrame_.SetCalibration(c)) { *error = "the room isn't placed yet"; return false; }
    Log("room calibration: " + action + (j.contains("serial") ? " from " + j.value("serial", "") : ""));
    lastSnapshot_ = -1e300;  // the devices take the new frame on the next pass
    return true;
  }

  void PublishSettings() {
    nlohmann::json j = applied_;
    j["host"] = HostName();
    j["steamvr"] = HostName().rfind("vrserver", 0) == 0;
    j["live"] = !replay_;
    j["recording"] = !recording_.empty();
    j["recordingPath"] = recording_;
    j["reportBusy"] = !reportDone_.load();
    {
      std::lock_guard lock(reportMutex_);
      j["report"] = {{"count", reportCount_}, {"path", lastReport_}, {"error", lastReportError_}};
    }
    j["steamRooms"] = steamRooms_;
    {
      // Room calibration: only in our own frame, without SteamVR's Room Setup.
      const Eigen::Isometry3d c = hostFrame_.Calibration();
      const Eigen::Matrix3d r = c.rotation();
      j["calibration"] = {{"available", !steamvrHostNow() && hostFrame_.Anchored() && !steamFrame_},
                          {"yaw", std::atan2(r(0, 2), r(0, 0)) * 57.29577951308232},
                          {"floor", -c.translation().y()}, {"x", c.translation().x()}, {"z", c.translation().z()}};
    }
    g_telemetry.SetSettings(j.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace));
  }

  // Main thread: what clients asked for since the last pass. Each gets a reply.
  // Wands with no hand set, as SteamVR's own driver: once both are seen with the
  // headset, the one to the headset's right is the right hand. Kept until a wand turns
  // off; one turned on later takes the free hand.
  void AssignWandHands(const std::vector<lh::DeviceUpdate>& updates) {
    const lh::DeviceUpdate* hmd = nullptr;
    struct Wand { Device* d; const lh::DeviceUpdate* u; };
    std::vector<Wand> wands;
    for (const auto& u : updates) {
      auto found = devices_.find(u.serial);
      if (found == devices_.end() || !found->second || !found->second->Active()) continue;
      Device* d = found->second.get();
      if (d->IsHmd()) { if (u.pose.valid) hmd = &u; continue; }
      if (!d->AutoHandWand()) continue;
      if (!u.connected || !u.enabled) { d->SetAutoHand(0); continue; }
      wands.push_back({d, &u});
    }
    if (wands.size() != 2) return;
    Wand& a = wands[0];
    Wand& b = wands[1];
    const int L = TrackedControllerRole_LeftHand, R = TrackedControllerRole_RightHand;
    if (a.d->AutoHand() && a.d->AutoHand() != b.d->AutoHand()) {
      if (!b.d->AutoHand()) b.d->SetAutoHand(a.d->AutoHand() == L ? R : L);
      return;
    }
    if (b.d->AutoHand() && !a.d->AutoHand()) { a.d->SetAutoHand(b.d->AutoHand() == L ? R : L); return; }
    if (!hmd || !a.u->pose.valid || !b.u->pose.valid) return;
    auto pos = [](const lh::DeviceUpdate& u) {
      const auto& p = u.pose.worldFromImu.position;
      return Eigen::Vector3d(p[0], p[1], p[2]);
    };
    const auto& ho = hmd->pose.worldFromImu.orientation;
    const auto& hh = hmd->pose.imuFromHead.orientation;
    const Eigen::Quaterniond worldFromHead = Eigen::Quaterniond(ho[0], ho[1], ho[2], ho[3]) *
                                             Eigen::Quaterniond(hh[0], hh[1], hh[2], hh[3]);
    const Eigen::Vector3d right = worldFromHead * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d head = pos(*hmd);
    const double da = right.dot(pos(*a.u) - head), db = right.dot(pos(*b.u) - head);
    if (std::abs(da - db) < 0.1) return;  // side by side in front: wait until they part
    a.d->SetAutoHand(da > db ? R : L);
    b.d->SetAutoHand(da > db ? L : R);
  }

  void HandleCommands() {
    for (const auto& c : g_telemetry.TakeCommands()) {
      const nlohmann::json j = nlohmann::json::parse(c.json, nullptr, false);
      if (!j.is_object() || j.value("type", "") != "command") continue;
      nlohmann::json reply = {{"type", "reply"}, {"ok", false}};
      if (j.contains("id")) reply["id"] = j.at("id");
      try {
        std::string error;
        reply["ok"] = RunCommand(j, &error);
        if (!error.empty()) reply["error"] = error;
      } catch (const std::exception& e) {
        reply["error"] = e.what();
      }
      g_telemetry.Reply(c.client, reply.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace));
    }
  }

  bool RunCommand(const nlohmann::json& j, std::string* error) {
    const std::string cmd = j.value("cmd", "");
    auto text = [&](const char* key) {
      const auto it = j.find(key);
      return it != j.end() && it->is_string() ? it->get<std::string>() : std::string();
    };
    if (!session_) { *error = "no session"; return false; }
    Log(lh::LogLevel::Debug, "overlay: " + cmd + (text("serial").empty() ? "" : " " + text("serial")));
    if (cmd == "set") {
      const std::string key = text("key");
      const auto type = OverlaySetting(key);
      const auto value = j.find("value");
      if (!type || value == j.end()) { *error = "unknown setting " + key; return false; }
      if (*type == SettingType::Bool && !value->is_boolean()) { *error = key + " takes true or false"; return false; }
      if (*type == SettingType::Number && !value->is_number()) { *error = key + " takes a number"; return false; }
      if (!UpdateSettings([&](nlohmann::json& s) { SetSetting(s, key, *value); return true; })) {
        *error = "couldn't save " + SettingsPath();
        return false;
      }
      ApplyTuning(false);
      Log("overlay set " + key + " to " + value->dump());
      return true;
    }
    if (cmd == "record") {
      const bool on = j.value("on", false);
      recordWanted_ = on;
      ApplyTuning(false);
      if (on != recording_.empty()) return true;
      *error = on ? "recording needs the USB devices (not available here)" : "couldn't stop";
      return false;
    }
    if (cmd == "calibrate") return Calibrate(j, error);
    if (cmd == "resetRoom") {
      Log("room reset from openhouse-overlay");
      session_->ResetRoom();
      return true;
    }
    if (cmd == "saveReport") {
      if (!reportDone_) { *error = "a bug report is still being saved"; return false; }
      SaveReport(recording_);
      return true;
    }
    const std::string serial = text("serial");
    if (cmd == "renameTracker") {
      const auto value = j.find("name");
      const auto name = lh::openvr::ValidateTrackerRename(snapshot_.devices, serial, value == j.end() ? nlohmann::json{} : *value, error);
      if (!name) return false;
      if (!UpdateSettings([&](nlohmann::json& settings) { return lh::openvr::SetTrackerName(settings, serial, *name); })) {
        *error = "couldn't save " + SettingsPath();
        return false;
      }
      // On a Doorman Dongle that keeps names: there too, so the name goes with the dongle.
      for (const auto& r : session_->Receivers())
        if (r.kind == "openhouse" && r.keepsName && r.connected == serial) session_->SetReceiverName(r.serial, *name);
      ApplyTuning(false);
      return true;
    }
    if (cmd == "identify") {
      // A short buzz on a controller or tracker, to tell which one it is.
      return session_->Haptic(serial, 160, 1.0, 0.4) || (*error = "no haptics for " + serial, false);
    }
    if (cmd == "powerOff") return session_->PowerOff(serial) || (*error = serial + " can't be powered off", false);
    if (cmd == "enable") {
      session_->SetDeviceEnabled(serial, j.value("enabled", true));
      return true;
    }
    if (cmd == "pair") {
      const double seconds = std::clamp(j.value("seconds", 30.0), 0.0, 300.0);
      return session_->SetPairing(text("receiver"), seconds) || (*error = "no receiver " + text("receiver"), false);
    }
    *error = "unknown command " + cmd;
    return false;
  }

  // The bug report zip (lh/bug_report.hpp) with this recording, off the host thread.
  void SaveReport(const std::string& recording) {
    if (reportThread_.joinable() && reportDone_) reportThread_.join();
    if (reportThread_.joinable()) {
      Log("a bug report is still being saved; this recording stays in " + recording);
      return;
    }
    reportDone_ = false;
    lh::BugReportOptions options;
    options.summary = ReportSummary();
    options.recording = recording;
    reportThread_ = std::thread([this, options] {
      // The USB thread closes the capture on its next pass: let it finish the file.
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      // Nothing thrown here may leave the thread: that is std::terminate, and it took
      // vrserver down with the report unwritten.
      try {
        std::string error;
        const std::string path = lh::SaveBugReport(options, &error);
        Log(path.empty() ? "bug report failed: " + error : "bug report saved: " + path);
        std::lock_guard lock(reportMutex_);
        lastReport_ = path;
        lastReportError_ = path.empty() ? error : "";
        ++reportCount_;
      } catch (const std::exception& e) {
        Log(std::string("bug report failed: ") + e.what() + "; the recording stays in " + options.recording);
      } catch (...) {
        Log("bug report failed; the recording stays in " + options.recording);
      }
      reportDone_ = true;
    });
  }

  // The devices, bases and room as they are now, for a bug report's info.txt.
  std::string ReportSummary() const {
    std::string out = "room " + (snapshot_.roomId.empty() ? std::string("(none)") : snapshot_.roomId) +
                      (roomPublisher_.Settled() ? ", settled" : ", not settled") + "\n\ndevices:\n";
    char line[256];
    for (const auto& d : snapshot_.devices) {
      std::snprintf(line, sizeof line, "  %s %s %s: %s, %s%s\n", d.info.serial.c_str(), lh::ToString(d.info.role),
                    d.info.model.c_str(), d.info.connected ? "connected" : "disconnected",
                    d.pose.valid ? "tracking" : "not tracking", d.info.enabled ? "" : ", disabled");
      out += line;
    }
    out += "\nbase stations:\n";
    for (const auto& b : snapshot_.bases) {
      std::snprintf(line, sizeof line, "  %s channel %s %s: %s%s%s, %.1f mm, %d keyframes\n", b.serial.c_str(),
                    b.channel.empty() ? "?" : b.channel.c_str(), b.lh2 ? "2.0" : "1.0", b.seen ? "seen" : "not seen",
                    !b.known ? ", not placed" : b.learned ? ", learned" : ", learning", b.locked ? ", locked" : "",
                    b.sigma * 1000, b.observations);
      out += line;
    }
    return out;
  }
  double lastTuning_ = -1e300;

  // openhouse's settings: settings.json in its config folder, the same file the session
  // reads (mixedLight, hideLostDevices, disabledDevices...). openhouse-overlay changes them
  // with "set"; a hand edit is picked up within a second (ApplyTuning runs every second).
  static std::string SettingsPath() { return lh::platform::ConfigDirectory() + "/settings.json"; }
  static nlohmann::json ReadSettings() {
    std::ifstream in(lh::platform::FsPath(SettingsPath()));
    if (!in) return nlohmann::json::object();
    nlohmann::json j = nlohmann::json::parse(in, nullptr, false);
    return j.is_object() ? j : nlohmann::json::object();
  }
  // Preserve unrelated settings and skip the write if the callback reports no change.
  bool UpdateSettings(const std::function<bool(nlohmann::json&)>& change) {
    try {
      const std::string settingsPath = SettingsPath();
      const auto path = lh::platform::FsPath(settingsPath);
      nlohmann::json j = nlohmann::json::object();
      {
        std::ifstream in(path);
        if (in) {
          j = nlohmann::json::parse(in, nullptr, false);
          // Never overwrite a file we can't read: its controllers and roles would be lost.
          if (!j.is_object()) { Log("settings " + settingsPath + " isn't valid JSON: not changed"); return false; }
        }
      }
      if (!change(j)) return true;
      std::filesystem::create_directories(path.parent_path());
      auto temp = path;
      temp += ".tmp";
      {
        std::ofstream out(temp);
        out << j.dump(2) << "\n";
        out.close();
        if (!out) { Log("cannot write " + settingsPath + ".tmp"); return false; }
      }
      lh::platform::ReplaceFile(temp, path);
      return true;
    } catch (const std::exception& e) {
      Log(std::string("saving settings failed: ") + e.what());
      return false;
    }
  }
  // The overlay's names for the mixedLight object's fields.
  static const char* MixedField(const std::string& key) {
    static const std::map<std::string, const char*> fields = {
        {"mixedLight", "enabled"}, {"mixedLightControllers", "controllers"}, {"mixedLightTrackers", "trackers"},
        {"mixedLightScout", "scout"}, {"mixedLightPreferLh2", "preferLh2"}, {"mixedLightRotorReference", "rotorReference"}};
    const auto it = fields.find(key);
    return it == fields.end() ? nullptr : it->second;
  }
  static void SetSetting(nlohmann::json& j, const std::string& key, const nlohmann::json& value) {
    if (const char* field = MixedField(key)) {
      if (!j.contains("mixedLight") || !j["mixedLight"].is_object())
        j["mixedLight"] = {{"enabled", j.contains("mixedLight") && j["mixedLight"].is_boolean() && j["mixedLight"].get<bool>()}};
      j["mixedLight"][field] = value;
    } else {
      j[key] = value;
    }
  }

  // Older versions kept these on a page in SteamVR's settings (steamvr.vrsettings, section
  // driver_openhouse). Each value set there that settings.json doesn't have yet moves over,
  // once. Read from the file itself: SteamVR's GetBool answers false with no error for a key
  // it has no default for, which would turn options off.
  void MigrateSteamVRSettings() {
    // <Steam>/config/, not SteamConfigDirectory() (<Steam>/config/lighthouse): there it was never found.
    const std::string path = lh::platform::SteamRoot() + "/config/steamvr.vrsettings";
    std::ifstream in(std::filesystem::u8path(path));
    if (!in) return;
    const nlohmann::json vr = nlohmann::json::parse(in, nullptr, false);
    if (!vr.is_object() || !vr.contains("driver_openhouse") || !vr["driver_openhouse"].is_object()) return;
    const nlohmann::json& old = vr["driver_openhouse"];
    const nlohmann::json current = ReadSettings();
    auto has = [&](const std::string& key) {
      if (const char* field = MixedField(key))
        return current.contains("mixedLight") && (current["mixedLight"].is_object() ? current["mixedLight"].contains(field) : std::string(field) == "enabled");
      return current.contains(key);
    };
    std::vector<std::string> moved;
    nlohmann::json values = nlohmann::json::object();
    for (const char* key : {"positionSmoothingMs", "rotationSmoothingMs", "predictWithAcceleration", "hideLostDevices",
                            "useSteamVRRooms", "mixedLight", "mixedLightControllers", "mixedLightTrackers",
                            "mixedLightScout", "mixedLightPreferLh2", "mixedLightRotorReference"}) {
      if (!old.contains(key) || has(key)) continue;
      const auto& v = old[key];
      if (!v.is_boolean() && !v.is_number()) continue;
      values[key] = v;
      moved.push_back(key);
    }
    if (moved.empty()) return;
    if (UpdateSettings([&](nlohmann::json& j) {
          for (const auto& [k, v] : values.items()) SetSetting(j, k, v);
          return true;
        }))
      Log("moved " + std::to_string(moved.size()) + " settings from SteamVR's settings to " + SettingsPath());
  }

  // On the driver thread: a throw from here (a filesystem call when recording starts or
  // stops) must not end it, or vrserver with it.
  void ApplyTuning(bool first) {
    try {
      ApplyTuningOnce(first);
    } catch (const std::exception& e) {
      Log(std::string("applying the settings failed: ") + e.what());
    } catch (...) {
      Log("applying the settings failed");
    }
  }
  void ApplyTuningOnce(bool first) {
    (void)first;
    if (!session_) return;
    const nlohmann::json f = ReadSettings();
    auto number = [&](const char* key, double fallback) {
      const auto it = f.find(key);
      return it != f.end() && it->is_number() ? it->get<double>() : fallback;
    };
    auto flag = [&](const nlohmann::json& j, const char* key, bool fallback) {
      const auto it = j.find(key);
      return it != j.end() && it->is_boolean() ? it->get<bool>() : fallback;
    };
    const double position = std::clamp(number("positionSmoothingMs", 60), 0.0, 1000.0) / 1000.0;
    const double rotation = std::clamp(number("rotationSmoothingMs", 60), 0.0, 1000.0) / 1000.0;
    const bool acceleration = flag(f, "predictWithAcceleration", true);
    session_->SetOutputTuning(position, rotation, acceleration);
    applied_ = {{"positionSmoothingMs", position * 1000}, {"rotationSmoothingMs", rotation * 1000},
                {"predictWithAcceleration", acceleration}, {"useSteamVRRooms", flag(f, "useSteamVRRooms", true)}};
    if (const auto names = f.find("trackerNames"); names != f.end() && names->is_object())
      applied_["trackerNames"] = *names;
    // A Doorman Dongle keeps its devices' names (firmware 0x0025+): those go with the dongle, so
    // they win over this PC's settings.
    for (const auto& r : session_->Receivers())
      if (r.kind == "openhouse" && r.keepsName && !r.connected.empty() && !r.name.empty()) {
        auto& entry = applied_["trackerNames"][r.connected];
        // A name over the dongle's 59 bytes is kept cut there: the whole one here stays.
        if (!entry.is_string() || entry.get_ref<const std::string&>().rfind(r.name, 0) != 0) entry = r.name;
      }
    // 1.0 and 2.0 base stations together (ai/driver/mixed_stations.md): settings.json
    // "mixedLight", true/false or an object (session.cpp LoadSettings has the fields).
    lh::MixedLightOptions mixed = session_->MixedLight();
    if (const auto m = f.find("mixedLight"); m != f.end()) {
      if (m->is_boolean()) {
        mixed.enabled = m->get<bool>();
      } else if (m->is_object()) {
        mixed.enabled = flag(*m, "enabled", false);
        mixed.controllers = flag(*m, "controllers", true);
        mixed.trackers = flag(*m, "trackers", true);
        mixed.scout = flag(*m, "scout", true);
        mixed.preferLh2 = flag(*m, "preferLh2", false);
        mixed.rotorReference = flag(*m, "rotorReference", true);
      }
    }
    const lh::MixedLightOptions now = session_->MixedLight();
    if (mixed.enabled != now.enabled || mixed.controllers != now.controllers || mixed.trackers != now.trackers ||
        mixed.scout != now.scout || mixed.preferLh2 != now.preferLh2 || mixed.rotorReference != now.rotorReference)
      session_->SetMixedLight(mixed);
    applied_["mixedLight"] = mixed.enabled;
    applied_["mixedLightControllers"] = mixed.controllers;
    applied_["mixedLightTrackers"] = mixed.trackers;
    applied_["mixedLightScout"] = mixed.scout;
    applied_["mixedLightPreferLh2"] = mixed.preferLh2;
    applied_["mixedLightRotorReference"] = mixed.rotorReference;
    // Record session (openhouse-overlay): raw device data to <data>/recordings while on
    // (replay with `lhtool session`); turning it off saves the bug report zip with it.
    if (recordWanted_ != !recording_.empty()) {
      if (recordWanted_) {
        const std::time_t t = std::time(nullptr);
        char name[64];
        std::strftime(name, sizeof name, "recording-%Y%m%d-%H%M%S.lhraw", std::localtime(&t));
        const std::string dir = lh::platform::DataDirectory() + "/recordings";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (!ec && session_->SetCapture(dir + "/" + name)) {
          recording_ = dir + "/" + name;
          Log("recording to " + recording_);
        } else {
          Log("cannot record to " + dir);
          recordWanted_ = false;
        }
      } else {
        session_->SetCapture("");
        Log("recording stopped: " + recording_ + "; saving the bug report");
        SaveReport(recording_);
        recording_.clear();
      }
    }
  }
  bool recordWanted_ = false;
};

Provider g_provider;

}  // namespace

#ifdef _WIN32
#define LH_DRIVER_EXPORT __declspec(dllexport)
#else
#define LH_DRIVER_EXPORT __attribute__((visibility("default")))
#endif
// The installed version, found by openhouse-installer in the file ("openhouse-version:X.Y.Z;").
// Exported, so the linker keeps it.
extern "C" LH_DRIVER_EXPORT const char openhouse_version[] = "openhouse-version:" LH_VERSION_STRING ";";
// When it was built (Unix seconds, "openhouse-build-time:N;"): the installer orders two builds of
// one version by it.
extern "C" LH_DRIVER_EXPORT const char openhouse_build_time[] = "openhouse-build-time:" LH_BUILD_TIME_STRING ";";

extern "C" LH_DRIVER_EXPORT void* HmdDriverFactory(const char* interfaceName, int* returnCode) {
  if (std::strcmp(interfaceName, IServerTrackedDeviceProvider_Version) == 0) return &g_provider;
  if (std::strcmp(interfaceName, IVRWatchdogProvider_Version) == 0) return lh::openvr::WatchdogProvider();
  if (returnCode) *returnCode = VRInitError_Init_InterfaceNotFound;
  return nullptr;
}
