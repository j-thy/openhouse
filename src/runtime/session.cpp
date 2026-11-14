// lh::Session implementation: transport -> per-device trackers + shared room map.
#include "lh/session.hpp"
#include "platform/platform.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>

#include "device_role.hpp"
#include "device_tracker.hpp"
#include "index_input.hpp"
#include "lh/input_sink.hpp"
#include "mixed_light.hpp"
#include "model_scale.hpp"
#include "room_map.hpp"
#include "room_solver.hpp"
#include "rotor_reference.hpp"
#include "tracking/disambiguator_sob.hpp"
#include "tracking/tracked_object.hpp"

#ifdef LH_RUNTIME_USB
#include "raw_capture.hpp"
#include "usb_transport.hpp"
#endif

namespace lh {

const char* ToString(DeviceRole role) {
  switch (role) {
    case DeviceRole::Hmd: return "hmd";
    case DeviceRole::Controller: return "controller";
    case DeviceRole::Tracker: return "tracker";
    case DeviceRole::BaseStation: return "base_station";
  }
  return "?";
}

const char* ToString(TrackingState state) {
  switch (state) {
    case TrackingState::Disconnected: return "disconnected";
    case TrackingState::Uninitialized: return "uninitialized";
    case TrackingState::Acquiring: return "acquiring";
    case TrackingState::Tracking: return "tracking";
    case TrackingState::Coasting: return "coasting";
    case TrackingState::Lost: return "lost";
  }
  return "?";
}

namespace runtime {
namespace {
using nlohmann::json;

double SteadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string BaseName(uint32_t serial) {
  char text[16];
  std::snprintf(text, sizeof text, "LHB-%08X", serial);
  return text;
}

std::string DefaultRoomMapPath() { return platform::ConfigDirectory() + "/roommap.json"; }
std::string DefaultSettingsPath() { return platform::ConfigDirectory() + "/settings.json"; }
std::string DefaultPreinitDirectory() { return platform::ConfigDirectory() + "/preinit"; }

std::string FirstExisting(const std::vector<std::string>& paths) {
  for (const auto& p : paths)
    if (std::filesystem::exists(p)) return p;
  return {};
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

Pose ToPose(const Eigen::Isometry3d& t) {
  Pose p;
  const Eigen::Quaterniond q(t.rotation());
  p.position = {t.translation().x(), t.translation().y(), t.translation().z()};
  p.orientation = {q.w(), q.x(), q.y(), q.z()};
  return p;
}

// Rotation with +X = plus_x, +Z = plus_z (same construction as the IMU block).
bool AxesRotation(const json& block, Eigen::Isometry3d* out) {
  if (!block.contains("plus_x") || !block.contains("plus_z")) return false;
  const auto px = block.at("plus_x").get<std::array<double, 3>>();
  const auto pz = block.at("plus_z").get<std::array<double, 3>>();
  Eigen::Vector3d x(px[0], px[1], px[2]), z(pz[0], pz[1], pz[2]);
  if (x.norm() < 1e-6 || z.norm() < 1e-6 || x.cross(z).norm() < 1e-6) return false;
  x.normalize(); z.normalize();
  Eigen::Matrix3d r;
  r.col(0) = x; r.col(1) = z.cross(x).normalized(); r.col(2) = x.cross(r.col(1)).normalized();
  *out = Eigen::Isometry3d::Identity();
  out->linear() = r;
  if (block.contains("position")) {
    const auto p = block.at("position").get<std::array<double, 3>>();
    out->translation() = Eigen::Vector3d(p[0], p[1], p[2]);
  }
  return true;
}

bool PoseArray(const json& a, Eigen::Isometry3d* out) {  // [qx qy qz qw x y z]
  if (!a.is_array() || a.size() != 7) return false;
  *out = Eigen::Isometry3d::Identity();
  out->linear() = Eigen::Quaterniond(a[3].get<double>(), a[0].get<double>(), a[1].get<double>(), a[2].get<double>())
                      .normalized().toRotationMatrix();
  out->translation() = Eigen::Vector3d(a[4].get<double>(), a[5].get<double>(), a[6].get<double>());
  return true;
}

std::string Lower(std::string s) {
  for (auto& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
  return s;
}
}  // namespace

namespace {
std::string ModelHint(const json& j) {
  return Lower(j.value("model_number", "") + " " + j.value("render_model", "") + " " + j.value("input_profile", ""));
}
bool IsIndexHint(const std::string& hint) {
  return hint.find("knuckles") != std::string::npos || hint.find("index") != std::string::npos ||
         hint.find("knu_") != std::string::npos;
}
// Vive wands: "Vive. Controller MV" (1.0), "VIVE Controller Pro MV" (2.0),
// render model vr_controller_vive_1_5. Not seen on hardware here; from their configs.
bool IsWandHint(const std::string& hint) {
  return hint.find("vive") != std::string::npos && hint.find("controller") != std::string::npos;
}
// Firmware that sends "mapped input" (raw records laid out by the device's own input
// profile, e.g. Shiftall FlipVR): firmware_config.mapped_input, as the stock driver reads it.
bool IsMappedInput(const json& j) {
  if (!j.contains("firmware_config") || !j.at("firmware_config").is_object()) return false;
  const auto& f = j.at("firmware_config");
  return f.contains("mapped_input") && f.at("mapped_input").is_boolean() && f.at("mapped_input").get<bool>() &&
         !j.value("input_profile", "").empty();
}
}  // namespace

DeviceRole RoleFromConfig(const std::string& configJson) {
  const json j = json::parse(configJson);
  const std::string cls = Lower(j.value("device_class", ""));
  if (cls == "hmd") return DeviceRole::Hmd;
  // As the stock driver: "generic_tracker" makes a controller a tracker (e.g. on a hip).
  if (Lower(j.value("tracked_controller_role", "")) == "generic_tracker") return DeviceRole::Tracker;
  if (cls == "controller") {
    const std::string hint = ModelHint(j);
    if (IsIndexHint(hint) || IsWandHint(hint) || IsMappedInput(j)) return DeviceRole::Controller;
  }
  return DeviceRole::Tracker;
}

ControllerType ControllerTypeFromConfig(const std::string& configJson) {
  const json j = json::parse(configJson);
  const std::string hint = ModelHint(j);
  if (IsIndexHint(hint)) return ControllerType::Index;
  if (IsWandHint(hint)) return ControllerType::ViveWand;
  return IsMappedInput(j) ? ControllerType::Mapped : ControllerType::Index;
}

class SessionImpl final : public Session, public InputSinkBase {
 public:
  void OnTransportLog(LogLevel level, const std::string& text) override {
    std::lock_guard lock(mutex_);
    Emit(SessionEventType::Log, "", text, level);
  }
  explicit SessionImpl(SessionOptions options) : options_(std::move(options)) {}
  ~SessionImpl() override { Stop(); }

  bool Start(std::string* error) override;
  void Stop() override;
  bool Running() const override { return running_; }
  bool Finished() const override { return finished_; }
  SessionSnapshot Snapshot() const override;
  std::vector<DeviceUpdate> Updates() const override;
  double HostNow() const override {
    std::lock_guard lock(mutex_);
    if (options_.replayPath.empty() && running_) now_ = std::max(now_, SteadyNow());
    return now_;
  }
  std::vector<SessionEvent> PollEvents() override;
  void ResetRoom() override;
  void SetDeviceEnabled(const std::string& serial, bool enabled) override;
  std::vector<Receiver> Receivers() const override {
    std::vector<Receiver> out;
    if (usb_)
      for (const auto& r : usb_->Receivers()) out.push_back({r.serial, r.connected});
    return out;
  }
  bool SetCapture(const std::string& path) override {
#ifdef LH_RUNTIME_USB
    if (!usb_) return false;
    usb_->SetCapture(path);
    return true;
#else
    (void)path;
    return false;
#endif
  }
  bool SetPairing(const std::string& receiver, double seconds) override {
    return usb_ && usb_->SetPairing(receiver, int(std::lround(seconds * 1000)));
  }
  bool HasBluetoothRadio() const override { return usb_ && usb_->HasBluetoothReceiver(); }
  bool SetBluetoothRadio(bool on) override { return usb_ && usb_->SetBluetoothRadio(on); }
  std::optional<bool> BluetoothRadioOn() override { return usb_ ? usb_->BluetoothRadioOn() : std::nullopt; }
  bool Haptic(const std::string& serial, double frequency, double amplitude, double duration) override;
  bool PowerOff(const std::string& serial) override;
  bool SaveRoom(std::string* error) override;

  // IInputSink (transport thread)
  void OnDeviceAttached(const DeviceId&, const std::string& configJson) override;
  // Device JSON -> tracker, role, input. False (and logged) if the config is unusable.
  bool SetUpDevice(const DeviceId& id, const std::string& config);
  // preinit/*.json: devices created at start, disconnected until they really connect.
  void LoadPreinit();
  void UpdatePreinit(const DeviceId& id, const std::string& config);
  void OnDeviceDetached(const DeviceId&) override;
  void OnClockOffset(const DeviceId&, double offset) override;
  void OnImu(const DeviceId&, const ImuSample&) override;
  void OnLightPulse(const DeviceId&, const LightPulse&) override;
  void OnLh2BeamWord(const DeviceId&, const Lh2BeamWord&) override;
  void OnLh2SensorWord(const DeviceId&, const Lh2SensorWord&) override;
  void OnInput(const DeviceId&, const InputReport&) override;
  void OnBattery(const DeviceId&, const BatteryReport&) override;

 private:
  struct Device {
    DeviceInfo info;
    std::unique_ptr<DeviceTracker> tracker;
    std::optional<ControllerInputDecoder> input;
    Eigen::Isometry3d imuFromHead = Eigen::Isometry3d::Identity();
    double clockOffset = 0;
    bool haveClock = false;
    bool attached = false;
    // Preinitialized and never attached yet: reported connected (uninitialized) so hosts
    // that only take devices at startup (Monado) keep it. Cleared on the first attach.
    bool preinitConnected = false;
    double lastData = -1e300;  // host time
    TrackingState lastState = TrackingState::Disconnected;
    // Light mode (live only): 0 = LH1 pulses, 1 = LH2 words; -1 not set yet.
    int lightMode = -1;
    double modeSince = 0, lastGoodLight = -1e300;  // host time
    // 1.0 and 2.0 stations together: see UpdateMixedLight.
    double lastPulse = -1e300, lastBeam = -1e300, lastTracked = -1e300, modeSent = -1e300, nextLightCheck = 0;
    double lastBeamDeviceTime = 0, lastPulseDeviceTime = 0;
    bool everLh2 = false;  // has sent a beam word this session
    // MixedLightOptions::preferLh2: since when a 2.0 station has lit it well without a break, and when one
    // last did.
    double lh2CoveredSince = -1e300, lh2CoveredLast = -1e300;
    bool lh2Covered = false;
    bool lh1Only = false;  // can't do LH2 mode (runtime::LooksLh1Only): left out of 1.0 + 2.0
    // The scout (UpdateScout): per channel, the line it follows and whose periods it follows.
    struct Scouted {
      int epoch = -1;  // the sorter's line for the rotor
      DeviceId from;
      runtime::RotorScout scout;
      bool none = false;  // no device had the rotor's period when the line was learned
    };
    std::map<int, Scouted> scouted;
    bool scoutCovered = false;  // ScoutCovers, as last logged
    // UpdateLh1Keyframes: as last set on the tracker, and when to look again.
    bool lh1Keyframes = true;
    double nextKeyframeCheck = 0;
  };
  struct Base {
    BaseConfig config;
    BaseDynamicState dynamic;
    bool seen = false;
  };

  // level: as given, else ClassifyLog(text).
  void Emit(SessionEventType type, const std::string& serial, std::string text, std::optional<LogLevel> level = {});
  void Tick() { if (options_.replayPath.empty()) now_ = std::max(now_, SteadyNow()); }
  void Touch(Device& d, double deviceTime);
  void Process(const DeviceId& id, Device& d);
  void UpdateLightMode(const DeviceId& id, Device& d);
  void UpdateLh1Keyframes(const DeviceId& id, Device& d);
  void UpdateMixedLight(const DeviceId& id, Device& d);
  void UpdateMixedRoom();
  void StartMixedLight(Device& d);
  void UpdateScout();
  bool ScoutCovers(const Device& d) const;
  void PushIdentities();
  void TakeKeyframes(Device& d);
  bool SolveDue() const;
  RoomSolveInput SolveInput();
  void ApplySolve(const RoomSolveOutput& out);
  void SolverMain();
  void LoadSettings();
  void LoadLearnedImu(const std::string& path);
  void ApplyLearnedImu(const DeviceId& id, const std::string& config, TrackedObjectModel* model);
  // SteamVR's learned per-unit IMU calibration (lighthousedb.json "known_objects"), by serial:
  // used only for a device whose own config has none (#69).
  struct LearnedImu { std::array<double, 3> accBias{}, accScale{1, 1, 1}, gyroBias{}; bool acc = false, scale = false, gyro = false; };
  std::map<DeviceId, LearnedImu> learnedImu_;
  std::set<DeviceId> learnedImuLogged_;
  // lighthouse_scale.json reference objects, from {lighthouse} and each resource_root seen.
  void ApplyModelScale(const DeviceId& id, const std::string& resourceRoot, TrackedObjectModel* model);
  std::vector<ScaleReference> scaleRefs_;
  std::set<std::string> scaleRoots_;
  void SaveSettings();
  void SetOutputTuning(double positionSmoothing, double rotationSmoothing, bool reportAcceleration) override;
  void SetMixedLight(const MixedLightOptions& options) override;
  MixedLightOptions MixedLight() const override {
    std::lock_guard lock(mutex_);
    return mixed_;
  }
  bool HideLostDevices() const override {
    std::lock_guard lock(mutex_);
    return hideLost_;
  }
  bool Enabled(const DeviceId& id) const { return !disabled_.count(id); }
  void CollectLogs(Device* d);
  void MaybeSave();
  void ResetRoomLocked(const std::string& why);
  bool LightMovedLockedBase(const RoomSolveOutput& out);
  std::map<uint32_t, int> lightStrikes_;  // LightMovedLockedBase: solves in a row that moved it
  std::map<uint32_t, double> lightCooldown_;  // and until when (now_) it doesn't again
  std::set<std::string> excludedFromRoom_;     // devices the last solve left out
  void WatchRoomFile(bool now = false);
  uint64_t RoomResets() const override { std::lock_guard lock(mutex_); return roomResets_; }
  void ReplayMain();
  DevicePose MakePose(const Device& d) const;
  double HostTime(const Device& d, double deviceTime) const { return deviceTime + d.clockOffset; }

  SessionOptions options_;
  mutable std::mutex mutex_;
  std::map<DeviceId, Device> devices_;
  std::map<uint32_t, Base> bases_;
  std::map<uint32_t, tdm::TdmDisambiguator::Identity> identities_;
  RoomMap room_;
  std::string roomPath_, settingsPath_, preinitDir_;
  std::map<std::string, std::string> preinitFiles_;
  uint64_t roomResets_ = 0;
  bool roomFileSeen_ = false;
  double lastRoomFileCheck_ = -1e300;  // serial -> preinit file
  bool explicitControllers_ = false;
  std::string leftController_, rightController_;
  std::set<std::string> disabled_;
  double smoothPosition_ = DeviceTracker::kDefaultSmoothing, smoothRotation_ = DeviceTracker::kDefaultSmoothing;
  bool reportAcceleration_ = true;
  std::map<std::string, std::string> roles_;  // settings.json "roles": serial -> tracked_controller_role
  bool persist_ = false;
  MixedLightOptions mixed_;  // SetMixedLight
  runtime::RotorReference rotorReference_;  // MixedLightOptions::rotorReference
  bool hideLost_ = false;    // settings.json "hideLostDevices"
  bool mixedActive_ = false;  // mixed_.enabled, and a 2.0 station to sort (UpdateMixedRoom)
  double mixedSince_ = 0;     // when mixedActive_ last changed
  double lastLh2Light_ = -1e300;  // the latest beam word from any device
  std::string darkRoom_;      // a room whose 2.0 stations were found off (UpdateMixedRoom)
  bool lh2Seen_ = false;      // a 2.0 station seen this session (UpdateMixedRoom)
  bool lh2Placed_ = false;    // a 2.0 station in the room in use (UpdateMixedRoom)
  double lastScout_ = -1e300;
  std::deque<SessionEvent> events_;
  mutable double now_ = 0;  // latest host time seen (live: steady clock)
  double lastSave_ = 0;
  std::atomic<bool> running_{false}, finished_{false}, stop_{false};
  std::atomic<bool> warming_{false};  // replay: before the window
  std::thread replay_;
  // Room solver.
  KeyframeStore keyframes_;
  std::map<std::string, std::shared_ptr<const TrackedObjectModel>> models_;
  bool newKeyframes_ = false;
  int addedKeyframes_ = 0;      // since the last SolveInput: the newest entries of the store
  bool syncSolve_ = false;
  double lastSolve_ = -1e300;   // session time of the last solve
  double solveInterval_ = 1.0;  // grows while solves stop moving the bases (see ApplySolve)
  std::map<uint32_t, Eigen::Isometry3d> lastSolved_;  // previous solve's bases
  double lastAdvance_ = -1e300;  // room bases gliding to solved poses
  std::condition_variable solverWake_;
  std::thread solver_;
#ifdef LH_RUNTIME_USB
  std::unique_ptr<usb::UsbTransport> usb_;
#endif
};

bool SessionImpl::Start(std::string* error) {
  if (running_) return true;
  const bool replay = !options_.replayPath.empty();
  std::string err;
  // Room map: replays use an in-memory map unless a path is given explicitly.
  roomPath_ = options_.roomMapPath.empty() && !replay ? DefaultRoomMapPath() : options_.roomMapPath;
  persist_ = options_.saveRoomMap && !roomPath_.empty();
  if (!roomPath_.empty() && !room_.Load(roomPath_, &err)) Emit(SessionEventType::Log, "", err);
  room_.SetFrozen(std::getenv("LH_EVALUATE_ROOM") != nullptr);
  settingsPath_ = options_.settingsPath.empty() && !replay ? DefaultSettingsPath() : options_.settingsPath;
  LoadSettings();  // settings.json "mixedLight" into mixed_
  if (options_.mixedLight != MixedLightOptions{}) mixed_ = options_.mixedLight;  // lhtool --mixed-light
  for (const auto& serial : options_.disabledDevices) disabled_.insert(serial);
  preinitDir_ = options_.preinitDirectory == "-" ? "" : options_.preinitDirectory.empty() && !replay ? DefaultPreinitDirectory()
                                                                                                    : options_.preinitDirectory;
  LoadPreinit();
  std::string steamDb = options_.steamDatabasePath;
  if (steamDb.empty()) {
    std::vector<std::string> candidates;
    if (replay) candidates.push_back((std::filesystem::path(options_.replayPath).parent_path() / "lighthousedb.json").string());
    else {
      candidates.push_back(platform::SteamConfigDirectory() + "/lighthousedb.json");
#ifndef _WIN32
      candidates.push_back(platform::Home() + "/.steam/steam/config/lighthouse/lighthousedb.json");
      candidates.push_back(platform::Home() + "/.local/share/Steam/config/lighthouse/lighthousedb.json");
#endif
    }
    steamDb = FirstExisting(candidates);
  }
  if (!steamDb.empty() && steamDb != "-") {
    if (!room_.ImportSteamDatabase(ReadFile(steamDb), &err)) Emit(SessionEventType::Log, "", err);
  }
  // The learned IMU values also when the room isn't taken from it (the driver passes "-"): live,
  // SteamVR's own database; a replay only reads the one it was given.
  if (!steamDb.empty() && steamDb != "-") LoadLearnedImu(steamDb);
  else if (!replay) LoadLearnedImu(platform::SteamConfigDirectory() + "/lighthousedb.json");
  {
    std::lock_guard lock(mutex_);
    for (const auto& [serial, c] : room_.KnownConfigs()) {
      bases_[serial] = {c.first, c.second, false};
      const auto frame = room_.OotxFrames().find(serial);
      if (frame != room_.OotxFrames().end()) identities_[serial] = {c.first, c.second, frame->second};
    }
    for (auto& text : room_.TakeLogs()) Emit(SessionEventType::Log, "", text);
    UpdateMixedRoom();
  }
  stop_ = false;
  finished_ = false;
  syncSolve_ = replay && !options_.replayRealtime;
  // The solver thread starts only once Start can't fail any more: a failed Start left it
  // running, and a second Start then terminated on the joinable thread (#49.7).
  if (replay) {
#ifdef LH_RUNTIME_USB
    if (!std::filesystem::exists(options_.replayPath)) {
      if (error) *error = "capture not found: " + options_.replayPath;
      return false;
    }
    if (!syncSolve_) solver_ = std::thread([this] { SolverMain(); });
    running_ = true;
    replay_ = std::thread([this] { ReplayMain(); });
#else
    if (error) *error = "built without the USB/capture layer";
    return false;
#endif
  } else {
#ifdef LH_RUNTIME_USB
    usb::UsbTransportOptions o;
    o.steamLighthouseConfigDir = options_.steamConfigDir;
    o.capturePath = options_.capturePath;
    o.onCommandResult = [this](const usb::DeviceCommandResult& result) {
      // A power-off is rare and worth seeing in the host log whatever happened to it.
      const bool routine = std::string(result.status) == "written" && !result.powerOff;
      std::lock_guard lock(mutex_);
      std::ostringstream text;
      const bool lightMode = result.opticalMode >= 0;
      text << (result.powerOff ? "power-off " : lightMode ? "light mode " : "haptic ") << result.serial << ": "
           << result.status << " (HID bytes=" << result.bytes;
      if (lightMode) text << ", mode " << result.opticalMode;
      else if (!result.powerOff) text << ", on=" << (result.report[4] | result.report[5] << 8)
                                << ", off=" << (result.report[6] | result.report[7] << 8)
                                << ", repeats=" << (result.report[8] | result.report[9] << 8);
      text << ")";
      Emit(SessionEventType::Log, result.serial, text.str(), routine ? LogLevel::Debug : LogLevel::Info);
    };
    usb_ = std::make_unique<usb::UsbTransport>(this, o);
    if (!usb_->Start()) {
      usb_.reset();
      if (error) *error = "no Lighthouse USB devices could be opened (is SteamVR running?)";
      return false;
    }
    if (!syncSolve_) solver_ = std::thread([this] { SolverMain(); });
    running_ = true;
#else
    if (error) *error = "built without the USB transport";
    return false;
#endif
  }
  if (error) error->clear();
  return true;
}

bool SessionImpl::Haptic(const std::string& serial, double frequency, double amplitude, double duration) {
#ifdef LH_RUNTIME_USB
  std::lock_guard lock(mutex_);
  const auto d = devices_.find(serial);
  if (!running_ || !usb_ || !Enabled(serial) || d == devices_.end() || !d->second.attached ||
      (d->second.info.role != DeviceRole::Controller && !d->second.input) || !std::isfinite(frequency) ||
      !std::isfinite(amplitude) || !std::isfinite(duration)) return false;
  usb_->Haptic(serial, frequency, amplitude, duration);
  return true;
#else
  return false;
#endif
}

bool SessionImpl::PowerOff(const std::string& serial) {
#ifdef LH_RUNTIME_USB
  std::lock_guard lock(mutex_);
  const auto d = devices_.find(serial);
  if (!running_ || !usb_ || !Enabled(serial) || d == devices_.end() || !d->second.attached ||
      (d->second.info.role != DeviceRole::Controller && d->second.info.role != DeviceRole::Tracker)) return false;
  usb_->PowerOff(serial);
  return true;
#else
  return false;
#endif
}

void SessionImpl::Stop() {
  stop_ = true;
#ifdef LH_RUNTIME_USB
  std::unique_ptr<usb::UsbTransport> transport;
  { std::lock_guard lock(mutex_); transport = std::move(usb_); }
  if (transport) transport->Stop();  // callbacks take mutex_: never hold it here
#endif
  if (replay_.joinable()) replay_.join();
  solverWake_.notify_all();
  if (solver_.joinable()) solver_.join();
  if (running_ && persist_) {
    std::string err;
    std::lock_guard lock(mutex_);
    if (room_.Dirty() && room_.Save(roomPath_, &err)) room_.ClearDirty();
  }
  running_ = false;
}

void SessionImpl::ReplayMain() {
#ifdef LH_RUNTIME_USB
  usb::ReportDecoder decoder(this);
  usb::RawCaptureReader reader;
  if (!reader.Open(options_.replayPath)) {
    std::lock_guard lock(mutex_);
    Emit(SessionEventType::Log, "", "cannot open capture " + options_.replayPath);
    finished_ = true;
    return;
  }
  double firstHost = 0, wallStart = 0, captureStart = -1;
  bool first = true;
  std::set<uint16_t> skipped;  // endpoints of devices outside replayOnly
  std::FILE* trace = options_.replayTracePath.empty() ? nullptr : std::fopen(options_.replayTracePath.c_str(), "w");
  double nextTrace = 0;
  size_t nextSwitch = 0;  // replayMixedSwitch
  const auto& only = options_.replayOnly;
  reader.ReadAll([&](const usb::EndpointInfo& e) {
                   if (!only.empty() && std::find(only.begin(), only.end(), e.serial) == only.end()) { skipped.insert(e.ep); return; }
                   decoder.AddEndpoint(e);
                 },
                 [&](const usb::RawReport& r) {
                   if (stop_) return;
                   if (captureStart < 0 && r.hostTime > 0) captureStart = r.hostTime;  // header-time reports carry 0
                   const double at = captureStart < 0 ? 0 : r.hostTime - captureStart;
                   if (skipped.count(r.ep)) return;
                   // Before the window: decoded (attach, protocol negotiation, clock), but
                   // no IMU or light reaches the trackers.
                   warming_ = at < options_.replayFrom;
                   if (options_.replayTo > 0 && at > options_.replayTo) { stop_ = true; return; }
                   if (options_.replayRealtime && !warming_) {  // the warm-up runs at full speed
                     if (first) { firstHost = r.hostTime; wallStart = SteadyNow(); first = false; }
                     const double wait = (r.hostTime - firstHost) - (SteadyNow() - wallStart);
                     if (wait > 0.0005) platform::SleepFor(std::chrono::microseconds(int64_t(wait * 1e6)));
                   }
                   {
                     std::lock_guard lock(mutex_);
                     now_ = std::max(now_, r.hostTime);
                   }
                   if (nextSwitch < options_.replayMixedSwitch.size() && at >= options_.replayMixedSwitch[nextSwitch]) {
                     ++nextSwitch;
                     MixedLightOptions mixed;
                     {
                       std::lock_guard lock(mutex_);
                       mixed = mixed_;
                     }
                     mixed.enabled = !mixed.enabled;
                     SetMixedLight(mixed);
                   }
                   decoder.OnReport(r);
                   if (trace && !warming_ && r.hostTime >= nextTrace) {
                     nextTrace = std::max(nextTrace + 1.0 / options_.replayTraceRate, r.hostTime);
                     for (const auto& u : Updates()) {
                       const auto& p = u.pose;
                       std::fprintf(trace,
                                    "{\"t\":%.6f,\"serial\":\"%s\",\"valid\":%d,\"state\":\"%s\",\"result\":%d,"
                                    "\"position\":[%.6f,%.6f,%.6f],\"orientation\":[%.7f,%.7f,%.7f,%.7f],\"velocity\":[%.5f,%.5f,%.5f],\"acceleration\":[%.3f,%.3f,%.3f],\"ft\":%.6f,\"fp\":[%.6f,%.6f,%.6f],\"pt\":%.6f}\n",
                                    r.hostTime, u.serial.c_str(), int(p.valid), ToString(p.state), int(p.result),
                                    p.worldFromImu.position[0], p.worldFromImu.position[1], p.worldFromImu.position[2],
                                    p.worldFromImu.orientation[0], p.worldFromImu.orientation[1], p.worldFromImu.orientation[2],
                                    p.worldFromImu.orientation[3], p.velocity[0], p.velocity[1], p.velocity[2],
                                    p.acceleration[0], p.acceleration[1], p.acceleration[2],
                                    p.filterTime, p.filterPosition[0], p.filterPosition[1], p.filterPosition[2], p.hostTime);
                     }
                   }
                 });
  if (trace) std::fclose(trace);
  finished_ = true;
#endif
}

void SessionImpl::Emit(SessionEventType type, const std::string& serial, std::string text, std::optional<LogLevel> level) {
  if (events_.size() > 4096) events_.pop_front();
  const LogLevel l = level ? *level : ClassifyLog(text);
  events_.push_back({now_, type, serial, std::move(text), l});
}

void SessionImpl::OnDeviceAttached(const DeviceId& id, const std::string& configJson) {
  std::lock_guard lock(mutex_);
  Tick();
  auto& d = devices_[id];
  const bool wasAttached = d.attached;
  const std::string config = configJson.empty() ? d.info.configJson : configJson;
  d.attached = true;
  d.preinitConnected = false;
  d.info.serial = id;
  d.lastData = now_;
  if (config.empty() || (wasAttached && d.tracker && d.info.configJson == config)) {
    if (d.tracker) Emit(SessionEventType::DeviceConnected, id, id + " connected");
    return;
  }
  // A radio reconnect is a new timestamp/protocol epoch, even when the device
  // JSON is identical; never take the duplicate-attach shortcut above for it.
  Emit(SessionEventType::Log, id, id + (wasAttached ? ": reattached with a new radio epoch" : ": attached"));
  if (SetUpDevice(id, config) && !configJson.empty()) UpdatePreinit(id, configJson);
}

bool SessionImpl::SetUpDevice(const DeviceId& id, const std::string& config) {
  auto& d = devices_[id];
  try {
    const json j = json::parse(config);
    TrackedObjectModel model;
    std::string err;
    if (!TrackedObjectModel::FromJson(config, &model, &err)) throw std::runtime_error(err);
    ApplyModelScale(id, j.value("resource_root", ""), &model);
    ApplyLearnedImu(id, config, &model);
    // A radio reconnect is a new timestamp/protocol epoch, not a duplicate attach.
    d.haveClock = false;
    d.clockOffset = 0;
    d.info.batteryPercent.reset();
    d.info.charging = false;
    d.input.reset();
    d.info.configJson = config;
    d.info.role = RoleFromConfig(config);
    d.info.controllerType = ControllerTypeFromConfig(config);
    d.info.deviceClass = j.value("device_class", "");
    d.info.model = j.value("model_number", "");
    d.info.manufacturer = j.value("manufacturer", "");
    d.info.renderModel = j.value("render_model", "");
    d.info.resourceRoot = j.value("resource_root", "");
    d.info.inputProfile = j.value("input_profile", "");
    d.info.registeredType = j.value("registered_device_type", "");
    d.info.controllerRole = j.value("tracked_controller_role", "");
    if (const auto r = roles_.find(id); r != roles_.end() && d.info.role != DeviceRole::Hmd) {
      // settings.json "roles": the values of tracked_controller_role.
      if (r->second == "generic_tracker") {
        d.info.role = DeviceRole::Tracker;
        d.info.controllerRole.clear();
      } else {
        d.info.role = DeviceRole::Controller;
        d.info.controllerRole = r->second;
      }
    } else if (explicitControllers_ && d.info.role != DeviceRole::Hmd) {
      d.info.controllerRole.clear();
      d.info.role = DeviceRole::Tracker;
      if (id == leftController_ || id == rightController_) {
        d.info.role = DeviceRole::Controller;
        d.info.controllerRole = id == leftController_ ? "left_hand" : "right_hand";
      }
    }
    d.info.sensors = model.InImuFrame().points;
    // imuFromHead = (objectFromImu)^-1 * objectFromHead.
    Eigen::Isometry3d objectFromImu = Eigen::Isometry3d::Identity();
    const auto& r = model.objectFromImuRotation;
    objectFromImu.linear() = Eigen::Quaterniond(r[0], r[1], r[2], r[3]).normalized().toRotationMatrix();
    objectFromImu.translation() = Eigen::Vector3d(model.imuPosition.data());
    Eigen::Isometry3d objectFromHead = Eigen::Isometry3d::Identity();
    if (j.contains("head")) AxesRotation(j.at("head"), &objectFromHead);
    else if (j.contains("trackref_from_head")) PoseArray(j.at("trackref_from_head"), &objectFromHead);
    // Tundra Trackers ship an identity head, so their pose (and model) came out bottom-up
    // (#40). The stock driver replaces exactly that case (in 0x1dec60): plus_x -1,0,0,
    // plus_z 0,0,-1 (180 deg about Y), position 0,0,0.001577.
    if (d.info.manufacturer == "Tundra Labs" && d.info.model == "Tundra Tracker" &&
        objectFromHead.isApprox(Eigen::Isometry3d::Identity(), 1e-9)) {
      AxesRotation(json{{"plus_x", {-1.0, 0.0, 0.0}}, {"plus_z", {0.0, 0.0, -1.0}}, {"position", {0.0, 0.0, 0.001577}}},
                   &objectFromHead);
    }
    // Knuckles EV2.0 (development Index controllers) ship a head with no position, and the
    // stock driver gives them the Index controller's (same function, from "EV2.0" in the
    // model number, "Right" for the right one): without it the model came out tilted about
    // 25 deg and a few cm off the controller (#96).
    if (d.info.model.find("Knuckles") != std::string::npos && d.info.model.find("EV2.0") != std::string::npos &&
        objectFromHead.translation().isZero(1e-9)) {
      const bool right = d.info.model.find("Right") != std::string::npos;
      AxesRotation(json{{"plus_x", {right ? 0.984808 : 0.979413, right ? 0.0217501 : -0.0217501, right ? 0.200692 : -0.200692}},
                        {"plus_z", {right ? -0.173648 : 0.173648, -0.416198, 0.892539}},
                        {"position", {right ? 0.010757 : -0.010757, 0.021781, -0.04214}}},
                   &objectFromHead);
    }
    d.imuFromHead = objectFromImu.inverse() * objectFromHead;
    model.serial = id;
    d.tracker = std::make_unique<DeviceTracker>(id, model, &room_);
    d.tracker->SetSmoothing(smoothPosition_, smoothRotation_);
    d.tracker->SetReportAcceleration(reportAcceleration_);
    d.tracker->SetHoldStill(d.info.role != DeviceRole::Hmd);  // a still head must not freeze the view
    StartMixedLight(d);
    models_[id] = d.tracker->ImuModel();
    for (const auto& [serial, b] : bases_) d.tracker->OnBaseConfig(b.config, b.dynamic);
    PushIdentities();
    // HTC's trackers report their pogo pins and power button in the wand's input format (the
    // stock driver reads them with the wand's button ids; libsurvive with the wand's bits), so
    // they get the wand's decoder. Other makers' trackers (Tundra) keep their inputs ignored.
    const bool htcTracker = d.info.role == DeviceRole::Tracker &&
                            (d.info.manufacturer.empty() || Lower(d.info.manufacturer) == "htc");
    if (d.info.role == DeviceRole::Controller) d.input.emplace(d.info.controllerType);
    else if (htcTracker) d.input.emplace(ControllerType::ViveWand);
    else d.input.reset();
    Emit(SessionEventType::DeviceActivated, id,
         id + " activated as " + ToString(d.info.role) + (d.info.model.empty() ? "" : " (" + d.info.model + ")") +
             (Enabled(id) ? "" : " [disabled]"));
    return true;
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, id, id + ": unusable device configuration: " + e.what());
    return false;
  }
}

void SessionImpl::LoadPreinit() {
  if (preinitDir_.empty()) return;
  std::error_code ec;
  std::vector<std::filesystem::path> files;
  for (const auto& e : std::filesystem::directory_iterator(preinitDir_, ec))
    if (e.path().extension() == ".json") files.push_back(e.path());
  std::sort(files.begin(), files.end());
  std::lock_guard lock(mutex_);
  for (const auto& file : files) {
    const std::string config = ReadFile(file.string());
    // The serial: a file named like one (LHR-0704E07A.json, as `lhtool configs` saves
    // them), else the config's own (wired headsets' configs say "LHR-XXXXXXXX").
    std::string id = file.stem().string();
    std::transform(id.begin(), id.end(), id.begin(), [](unsigned char c) { return char(std::toupper(c)); });
    try {
      const json j = json::parse(config);
      if (id.rfind("LHR-", 0) != 0) id = j.value("device_serial_number", "");
    } catch (const std::exception& e) {
      Emit(SessionEventType::Log, "", "preinit " + file.string() + ": " + e.what(), LogLevel::Warn);
      continue;
    }
    if (id.empty()) {
      Emit(SessionEventType::Log, "", "preinit " + file.string() + ": no serial (name the file <serial>.json)", LogLevel::Warn);
      continue;
    }
    if (devices_.count(id)) continue;
    preinitFiles_[id] = file.string();
    auto& d = devices_[id];
    d.info.serial = id;
    if (SetUpDevice(id, config)) {  // created, not attached: connected but uninitialized until it connects
      // Not a headset: a "connected" one would start the stock driver's display for nothing.
      d.preinitConnected = d.info.role != DeviceRole::Hmd;
      Emit(SessionEventType::Log, id, id + ": preinitialized from " + file.filename().string() + " (uninitialized)");
    }
  }
}

void SessionImpl::UpdatePreinit(const DeviceId& id, const std::string& config) {
  const auto file = preinitFiles_.find(id);
  if (file == preinitFiles_.end()) return;
  try {
    // Same JSON (formatting aside): keep the file as it is.
    if (json::parse(ReadFile(file->second)) == json::parse(config)) return;
    const std::string temp = file->second + ".tmp";
    {
      std::ofstream out(temp, std::ios::binary | std::ios::trunc);
      out << config;
      out.close();  // a write error can show only when the buffer goes out (#43)
      if (!out) throw std::runtime_error("cannot write " + temp);
    }
    platform::ReplaceFile(temp, file->second);  // rename doesn't replace a file on Windows
    Emit(SessionEventType::Log, id, id + ": device config differs from preinit; using the device's and updating " + file->second);
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, id, id + ": updating preinit " + file->second + ": " + e.what(), LogLevel::Warn);
  }
}

void SessionImpl::OnDeviceDetached(const DeviceId& id) {
  std::lock_guard lock(mutex_);
  Tick();
  // Free the receiver that was carrying this device. SteamVR asks which dongles are still
  // occupied before it offers a pairing, and a stale "connected" there means the dongle looks
  // busy until the receiver is physically unplugged.
  if (usb_) usb_->OnDeviceGone(id);
  auto found = devices_.find(id);
  if (found == devices_.end()) return;
  found->second.attached = false;
  if (mixedActive_) found->second.lightMode = -1;  // a device starts in LH1 mode: choose again
  Emit(SessionEventType::DeviceDisconnected, id, id + " disconnected");
}

void SessionImpl::OnClockOffset(const DeviceId& id, double offset) {
  if (!std::isfinite(offset)) return;
  std::lock_guard lock(mutex_);
  auto& d = devices_[id];
  d.clockOffset = offset;
  d.haveClock = true;
}

void SessionImpl::Touch(Device& d, double deviceTime) {
  const double host = d.haveClock ? HostTime(d, deviceTime) : now_;
  if (host > d.lastData) d.lastData = std::min(host, now_ + 1.0);
}

void SessionImpl::OnImu(const DeviceId& id, const ImuSample& sample) {
  if (warming_) return;
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.tracker) return;
  Touch(found->second, sample.time);
  found->second.tracker->OnImu(sample);
  Process(id, found->second);
  UpdateMixedRoom();
  UpdateScout();
  UpdateLightMode(id, found->second);
  UpdateLh1Keyframes(id, found->second);
}

// A device reports LH1 or LH2 light, whichever mode the host set (LH1 after power-on). In
// LH2 mode it reports nothing from LH1 bases; in LH1 mode, LH2 bases give stray pulses but
// no sync flash. Usable light: an LH1 sync-width pulse, or an LH2 beam word.
//  * Start from the room's known bases (SteamVR's database): LH2 if all are 2.0 bases.
//  * Once bases of one kind only have been seen, keep every device in that mode (a device
//    that sees nothing is occluded; re-send the mode now and then in case it was lost).
//  * Otherwise (nothing seen yet, or a mixed room), a device without usable light for
//    2.5 s tries the other mode.
//  * Not stock: a device that has sent 2.0 light stays in LH2 mode while any device sees 2.0
//    light (runtime::ChooseLightMode, which decides all of the above).
// With 1.0 and 2.0 stations used together (MixedLightOptions), UpdateMixedLight decides instead.
void SessionImpl::UpdateLightMode(const DeviceId& id, Device& d) {
  if (!usb_ || !d.attached) return;
  if (mixedActive_ && !d.lh1Only) {
    UpdateMixedLight(id, d);
    return;
  }
  if (mixedActive_) {
    // A 1.0-only device, its light ignored (OnLightPulse). Asked for LH2 mode again now and then, in case
    // it can after all and the command was lost: a beam word from it lets it back in (OnLh2BeamWord).
    constexpr double kAskAgain = 30.0;  // s
    if (now_ - d.modeSent >= kAskAgain) {
      usb_->SetOpticalSubProtocol(id, 1);
      d.lightMode = 1;
      d.modeSince = d.modeSent = now_;
    }
    return;
  }
  runtime::LightModeView view;
  for (const auto& [serial, b] : bases_) {
    const bool lh2 = b.config.model != 1;
    (lh2 ? view.knownLh2 : view.knownLh1) = true;
    if (b.seen) (lh2 ? view.seenLh2 : view.seenLh1) = true;
  }
  view.mode = d.lightMode;
  view.inMode = now_ - d.modeSince;
  view.dark = now_ - d.lastGoodLight;
  view.everLh2 = d.everLh2;
  view.sinceLh2Light = now_ - lastLh2Light_;
  const runtime::MixedLightStep step = runtime::ChooseLightMode(view);
  if (step.mode < 0) return;
  usb_->SetOpticalSubProtocol(id, step.mode);
  if (step.mode != d.lightMode)
    Emit(SessionEventType::Log, id, id + ": light mode " + (step.mode ? "LH2" : "LH1") + " (" + step.why + ")");
  d.lightMode = step.mode;
  d.modeSince = now_;
}

// runtime/mixed_light.hpp (Lh1KeyframesUsable): keeps LH1 frames that may hold unsorted 2.0 sweeps out of
// the room map. Replays too: it decides from light, not from the light mode.
void SessionImpl::UpdateLh1Keyframes(const DeviceId& id, Device& d) {
  constexpr double kEvery = 0.1;  // s
  if (!d.tracker || now_ < d.nextKeyframeCheck) return;
  d.nextKeyframeCheck = now_ + kEvery;
  runtime::Lh1KeyframeView view;
  view.sinceLh2Light = now_ - lastLh2Light_;
  for (const auto& [serial, b] : bases_) view.lh2Stations += b.seen && b.config.model != 1;
  view.sorting = d.tracker->Source().Sorting();
  const double deviceNow = std::max(d.lastPulseDeviceTime, d.lastBeamDeviceTime);
  for (const auto& rotor : d.tracker->Source().Rotors(deviceNow))
    view.knownRotors += rotor.ready && deviceNow - rotor.lastWord < runtime::kRotorFresh;
  const bool usable = runtime::Lh1KeyframesUsable(view);
  d.tracker->SetLh1Keyframes(usable);
  // runtime::ForeignSweeps (a 1.0-only device's light isn't used while 2.0 stations are lit).
  d.tracker->SetLh1SweepCheck(view.sinceLh2Light < runtime::kLh2Lit && !d.lh1Only);
  if (usable == d.lh1Keyframes) return;
  d.lh1Keyframes = usable;
  char text[160];
  std::snprintf(text, sizeof text, "%s: 1.0 light %s the room map (2.0 light %.0f s ago, sorting %s, %d of %d 2.0 rotors known)",
                id.c_str(), usable ? "goes into" : "kept out of", std::min(view.sinceLh2Light, 999.0), view.sorting ? "on" : "off",
                view.knownRotors, view.lh2Stations);
  Emit(SessionEventType::Log, id, text, LogLevel::Debug);
}

// runtime/mixed_light.hpp: LH1 mode once the rotors in view are known, LH2 mode now and then to
// time them again, and LH2 mode throughout for the kinds of device left out.
void SessionImpl::UpdateMixedLight(const DeviceId& id, Device& d) {
  constexpr double kCheckEvery = 0.05;  // s
  constexpr double kCalmSpeed = 0.5, kCalmTurn = 2.0;  // m/s, rad/s
  if (now_ < d.nextLightCheck) return;
  d.nextLightCheck = now_ + kCheckEvery;
  const auto out = d.tracker->Output();
  if (out.state == TrackingState::Tracking) d.lastTracked = now_;
  runtime::MixedLightView view;
  view.mode = d.lightMode;
  view.inMode = now_ - d.modeSince;
  view.dark = now_ - std::max(d.lightMode == 1 ? d.lastBeam : d.lastPulse, d.modeSince);
  view.sinceSent = now_ - d.modeSent;
  view.stayLh2 = !runtime::MixesLight(d.info.role, mixed_);
  for (const auto& rotor : d.tracker->Source().Rotors(d.lastBeamDeviceTime)) {
    if (rotor.ready) ++view.readyRotors;
    else if (rotor.hearing) ++view.learningRotors;
    if (rotor.ready && rotor.hearing && d.tracker->Source().ChannelSerial(rotor.channel) == 0) ++view.unnamedRotors;
  }
  view.tracking = out.state == TrackingState::Tracking;
  view.lost = now_ - d.lastTracked;
  view.calm = !out.valid || (out.velocity.norm() < kCalmSpeed && out.angularVelocity.norm() < kCalmTurn);
  view.lh1Pulses = now_ - d.lastPulse < 0.5;
  view.everLh2 = d.everLh2;
  view.lh2Unseen = !lh2Seen_;
  view.lh2Unplaced = lh2Seen_ && !lh2Placed_;
  std::vector<runtime::Lh2Light> lh2Light;
  for (const auto& c : out.coverage) {
    const auto base = bases_.find(c.base);
    if (base != bases_.end() && base->second.config.model != 1) lh2Light.push_back({c.frames, c.sensors});
  }
  const bool covered = runtime::Lh2LightEnough(lh2Light);
  if (covered && !d.lh2Covered) d.lh2CoveredSince = now_;
  if (covered) d.lh2CoveredLast = now_;
  d.lh2Covered = covered;
  view.preferLh2 = mixed_.preferLh2;
  view.lh2Covered = covered ? now_ - d.lh2CoveredSince : 0;
  view.lh2Short = covered ? 0 : now_ - d.lh2CoveredLast;
  if (runtime::LooksLh1Only(view)) {
    d.lh1Only = true;
    StartMixedLight(d);
    Emit(SessionEventType::Log, id, id + ": still sending 1.0 light in LH2 mode, and never 2.0 light: a 1.0-only device. With 2.0 "
                                         "stations lit its light can't be used, so it's ignored while 1.0 + 2.0 is on (to use "
                                         "it, switch that off and the 2.0 stations off too)", LogLevel::Warn);
    return;
  }
  const bool scouted = view.mode == 0 && ScoutCovers(d);
  const auto step = runtime::ChooseMixedLight(view, scouted ? runtime::kScoutedVisitEvery : runtime::kVisitEvery);
  if (step.mode < 0) return;
  usb_->SetOpticalSubProtocol(id, step.mode);
  d.modeSent = now_;
  if (step.mode == d.lightMode) return;
  char detail[192];
  std::snprintf(detail, sizeof detail, " after %.1f s, dark %.1f, rotors ready %d learning %d unnamed %d, %s, lost %.1f, %s%s%s",
                view.inMode, view.dark, view.readyRotors, view.learningRotors, view.unnamedRotors,
                view.tracking ? "tracking" : "not tracking", std::min(view.lost, 999.0), view.calm ? "calm" : "moving",
                scouted ? ", scouted" : "", view.lh2Unseen ? ", no 2.0 seen" : view.lh2Unplaced ? ", no 2.0 placed" : "");
  Emit(SessionEventType::Log, id, id + ": light mode " + (step.mode ? "LH2" : "LH1") + " (" + step.why + ")" + (view.mode < 0 ? "" : detail));
  d.lightMode = step.mode;
  d.modeSince = now_;
}

// The scout (runtime/mixed_light.hpp), live and in replays alike: which light a device is in comes from
// what it sends, not from what was asked of it.
void SessionImpl::UpdateScout() {
  constexpr double kEvery = 0.25;  // s
  if (now_ - lastScout_ < kEvery) return;
  lastScout_ = now_;
  auto inLh2 = [&](const Device& d) { return now_ - d.lastBeam < 0.3; };
  auto inLh1 = [&](const Device& d) { return now_ - d.lastPulse < 0.3 && now_ - d.lastBeam > 0.5; };
  for (auto& [id, d] : devices_) {
    if (!d.tracker) continue;
    if (!mixedActive_ || !mixed_.scout || !Enabled(id)) {
      for (const auto& [ch, s] : d.scouted) d.tracker->Source().SetRotorDrift(ch, 0);
      d.scouted.clear();
      d.scoutCovered = false;
      continue;
    }
    // In LH2 mode the device has real words; its lines start over at the first after a gap.
    if (!inLh1(d)) continue;
    std::set<int> seen;
    for (const auto& rotor : d.tracker->Source().Rotors(d.lastBeamDeviceTime)) {
      if (!rotor.ready) continue;
      seen.insert(rotor.channel);
      auto& s = d.scouted[rotor.channel];
      if (s.epoch != rotor.epoch) {
        // A new line, learned on the visit just over: follow a device that has the rotor's period now,
        // the headset first (it's in LH2 mode throughout).
        s = {};
        s.epoch = rotor.epoch;
        const DeviceId* from = nullptr;
        double period = 0;
        for (const auto& [eid, e] : devices_) {
          double p = 0;
          if (&e == &d || !e.tracker || !Enabled(eid) || !inLh2(e)) continue;
          if (!e.tracker->Source().RecentPeriod(rotor.channel, e.lastBeamDeviceTime, &p)) continue;
          if (!from || e.info.role == DeviceRole::Hmd) {
            from = &eid;
            period = p;
          }
        }
        if (from) {
          s.from = *from;
          s.scout = runtime::StartRotorScout(now_, period);
        } else {
          s.none = true;
        }
        continue;
      }
      if (s.none) continue;
      double p = 0;
      const double* period = nullptr;
      const auto e = devices_.find(s.from);
      if (e != devices_.end() && e->second.tracker && Enabled(s.from) && inLh2(e->second) &&
          e->second.tracker->Source().RecentPeriod(rotor.channel, e->second.lastBeamDeviceTime, &p))
        period = &p;
      d.tracker->Source().SetRotorDrift(rotor.channel, runtime::StepRotorScout(s.scout, now_, period));
    }
    for (auto it = d.scouted.begin(); it != d.scouted.end();) it = seen.count(it->first) ? std::next(it) : d.scouted.erase(it);
    const bool covered = ScoutCovers(d);
    if (covered != d.scoutCovered) {
      d.scoutCovered = covered;
      char text[160];
      std::snprintf(text, sizeof text, "%s: the scout %s every station in use: visits every %.0f s", id.c_str(),
                    covered ? "covers" : "no longer covers", covered ? runtime::kScoutedVisitEvery : runtime::kVisitEvery);
      Emit(SessionEventType::Log, id, text, LogLevel::Info);
    }
  }
}

// Whether every rotor the device is using now has a scout, so it can visit LH2 mode less often.
bool SessionImpl::ScoutCovers(const Device& d) const {
  if (!mixedActive_ || !mixed_.scout || !d.tracker) return false;
  int used = 0;
  for (const auto& rotor : d.tracker->Source().Rotors(d.lastBeamDeviceTime)) {
    if (!rotor.ready || d.lastPulseDeviceTime - rotor.lastClaim > 1.0) continue;  // not claiming hits now
    ++used;
    const auto s = d.scouted.find(rotor.channel);
    if (s == d.scouted.end() || s->second.epoch != rotor.epoch || s->second.none || s->second.scout.lost) return false;
  }
  return used > 0;
}

void SessionImpl::OnLightPulse(const DeviceId& id, const LightPulse& pulse) {
  if (warming_) return;
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.tracker) return;
  Touch(found->second, pulse.time);
  if (pulse.width >= 56e-6 && pulse.width <= 140e-6) found->second.lastGoodLight = now_;  // a sync flash
  found->second.lastPulse = now_;
  found->second.lastPulseDeviceTime = pulse.time;
  // A 1.0-only device in a room with 2.0 stations lit (runtime::LooksLh1Only): its light is mostly 2.0 hits
  // it can't tell apart, and making sense of it is costly enough, under this lock, that the other devices'
  // USB reports were dropped and the headset lost tracking (mixed_stations.md).
  if (mixedActive_ && found->second.lh1Only) return;
  found->second.tracker->OnPulse(pulse);
}

void SessionImpl::OnLh2BeamWord(const DeviceId& id, const Lh2BeamWord& word) {
  if (warming_) return;
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.tracker) return;
  Touch(found->second, word.time);
  found->second.lastGoodLight = now_;
  found->second.lastBeam = lastLh2Light_ = now_;
  found->second.lastBeamDeviceTime = word.time;
  found->second.everLh2 = true;
  if (found->second.lh1Only) {  // it can do LH2 mode after all
    found->second.lh1Only = false;
    StartMixedLight(found->second);
  }
  found->second.tracker->OnLh2BeamWord(word);
  // MixedLightOptions::rotorReference: what the word says of its rotor's timing, for the devices in LH1 mode.
  Device& d = found->second;
  if (mixedActive_ && mixed_.rotorReference && d.haveClock && word.channel >= 0 && word.channel < sob::kChannels)
    if (const uint32_t base = d.tracker->Source().ChannelSerial(word.channel))
      rotorReference_.OnWord(id, base, word.time, HostTime(d, word.time),
                             double(sob::kPeriodTicks[size_t(word.channel)]) * sob::kTickSeconds);
}

void SessionImpl::OnLh2SensorWord(const DeviceId& id, const Lh2SensorWord& word) {
  if (warming_) return;
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.tracker) return;
  Touch(found->second, word.time);
  found->second.tracker->OnLh2SensorWord(word);
}

void SessionImpl::OnInput(const DeviceId& id, const InputReport& report) {
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end() || !found->second.input) return;  // no decoder: inputs ignored (non-HTC trackers)
  auto& d = found->second;
  d.input->Apply(report, d.haveClock ? HostTime(d, report.time) : now_);
}

void SessionImpl::OnBattery(const DeviceId& id, const BatteryReport& b) {
  std::lock_guard lock(mutex_);
  Tick();
  if (!Enabled(id)) return;
  auto found = devices_.find(id);
  if (found == devices_.end()) return;
  const int percent = b.raw & 0x7f;
  const bool charging = b.raw & 0x80;
  if (percent == 0 && !charging) return;
  found->second.info.batteryPercent = percent;
  found->second.info.charging = charging;
}

void SessionImpl::Process(const DeviceId& id, Device& d) {
  if (now_ - lastAdvance_ >= 0.01) { lastAdvance_ = now_; room_.Advance(now_); }
  std::vector<std::pair<BaseConfig, BaseDynamicState>> decoded;
  d.tracker->Process(&decoded);
  tdm::TdmDisambiguator::Identity identity;
  bool newIdentity = false;
  while (d.tracker->Source().PopIdentity(&identity)) {
    room_.SetOotxFrame(identity.config.serial, identity.frame);
    auto& stored = identities_[identity.config.serial];
    newIdentity |= stored.frame != identity.frame;
    stored = identity;
  }
  if (newIdentity) PushIdentities();
  for (const auto& [config, dynamic] : decoded) {
    auto& base = bases_[config.serial];
    const bool first = !base.seen;
    if (first) {
      char text[128];
      std::snprintf(text, sizeof text, "base %08X: firmware %u, gravity %.3f %.3f %.3f", config.serial,
                    unsigned(dynamic.firmwareVersion), dynamic.accelDir[0], dynamic.accelDir[1], dynamic.accelDir[2]);
      Emit(SessionEventType::Log, "", text, LogLevel::Debug);
    }
    const bool newChannel = dynamic.sobChannel >= 0 && base.dynamic.sobChannel != dynamic.sobChannel;
    base.config = config;
    base.dynamic = dynamic;
    if (newChannel) PushIdentities();  // the other devices identify this LH2 base at once
    base.seen = true;
    room_.OnBaseSeen(config, dynamic);
    for (auto& [otherId, other] : devices_)
      if (other.tracker && otherId != id) other.tracker->OnBaseConfig(config, dynamic);
    if (first) {
      char text[96];
      std::snprintf(text, sizeof text, "%s seen by %s (mode %d)", BaseName(config.serial).c_str(), id.c_str(), dynamic.mode);
      Emit(SessionEventType::BaseStationSeen, BaseName(config.serial), text);
    }
  }
  TakeKeyframes(d);
  CollectLogs(&d);
  const auto out = d.tracker->Output();
  if (out.state != d.lastState) {
    Emit(SessionEventType::TrackingStateChanged, id,
         id + ": " + ToString(d.lastState) + " -> " + ToString(out.state));
    d.lastState = out.state;
  }
  MaybeSave();
}

void SessionImpl::TakeKeyframes(Device& d) {
  for (uint32_t base : room_.TakeRelearned()) keyframes_.DropBase(base);
  for (auto& k : d.tracker->TakeKeyframes()) {
    std::optional<Eigen::Isometry3d> pose;
    if (const RoomBase* primary = room_.Base(k->primary)) pose = primary->worldFromBase * k->primaryFromDevice;
    if (keyframes_.Add(std::move(k), pose)) { newKeyframes_ = true; ++addedKeyframes_; }
  }
  if (!newKeyframes_ || !SolveDue()) return;
  if (syncSolve_) {
    lastSolve_ = now_;
    newKeyframes_ = false;
    ApplySolve(SolveRoom(SolveInput()));
  } else {
    solverWake_.notify_one();
  }
}

bool SessionImpl::SolveDue() const {
  if (!room_.Active() || !room_.Origin()) return false;
  // Locating a seen base quickly matters most; refinement can be slower.
  bool missing = false;
  for (const auto& [serial, b] : bases_) missing |= b.seen && !room_.Base(serial);
  return now_ - lastSolve_ >= (missing ? 0.1 : solveInterval_);
}

RoomSolveInput SessionImpl::SolveInput() {
  RoomSolveInput in;
  in.keyframes = keyframes_.All();
  in.fresh = std::min(addedKeyframes_, int(in.keyframes.size()));
  for (const auto& b : room_.Bases()) in.known[b.serial] = b.worldFromBase;
  // A knocked origin is relearned against another placed base, held fixed meanwhile.
  in.origin = room_.OriginRelearning() && room_.OriginGauge() ? room_.OriginGauge() : room_.Origin();
  for (const auto& [serial, b] : bases_)
    if (b.config.cal.size() == 14) {
      std::array<double, 14> cal;
      std::copy(b.config.cal.begin(), b.config.cal.end(), cal.begin());
      in.calibration[serial] = cal;
      in.baseModels[serial] = b.config.model;
    }
  in.models = models_;
  in.refineBases = !std::getenv("LH_EVALUATE_ROOM");  // diagnostics: score a fixed room
  addedKeyframes_ = 0;
  return in;
}

void SessionImpl::ApplySolve(const RoomSolveOutput& out) {
  Emit(SessionEventType::Log, "", "room solve: " + out.summary, LogLevel::Debug);
  if (!out.ok || std::getenv("LH_EVALUATE_ROOM")) return;
  const uint32_t origin = room_.Origin();
  // A settled room: re-solving every second only burns CPU (up to 400 keyframes, two
  // passes). Back off while the solved bases agree with the room, up to 4 s (it also bounds how late a knocked base shows up); any real
  // change (or a base still being located, see SolveDue) brings it back to 1 s.
  // Compared with the previous solve, not the room map: the map moves toward solves
  // gradually, so it always trails them by a little.
  bool settled = !lastSolved_.empty();
  double worstMove = 0, worstTurn = 0;
  for (const auto& [serial, b] : out.bases) {
    if (serial == origin || b.keyframes <= 0) continue;
    const auto previous = lastSolved_.find(serial);
    if (previous == lastSolved_.end()) { settled = false; continue; }
    worstMove = std::max(worstMove, (b.worldFromBase.translation() - previous->second.translation()).norm());
    worstTurn = std::max(worstTurn, Eigen::AngleAxisd(previous->second.rotation().transpose() * b.worldFromBase.rotation()).angle());
  }
  settled = settled && worstMove < 0.003 && worstTurn < 0.15 * M_PI / 180;
  lastSolved_.clear();
  for (const auto& [serial, b] : out.bases) lastSolved_[serial] = b.worldFromBase;
  solveInterval_ = settled ? std::min(2 * solveInterval_, 4.0) : 1.0;
  {
    char text[128];
    std::snprintf(text, sizeof text, "room solve: moved %.1f mm %.3f deg since last; next in %.0f s", worstMove * 1e3,
                  worstTurn * 57.2958, solveInterval_);
    Emit(SessionEventType::Log, "", text, LogLevel::Debug);
  }
  if (LightMovedLockedBase(out)) {
    CollectLogs(nullptr);
    return;
  }
  if (room_.OriginRelearning()) {
    const auto o = out.bases.find(origin);
    if (o != out.bases.end() && o->second.keyframes > 0)
      room_.ApplyOriginSolved(o->second.worldFromBase, o->second.positionVariance, o->second.keyframes);
  } else {
    // Every other base far off at once (three or more bases): it's the origin that moved.
    // With two, a knocked origin and a knocked other look alike; only the
    // accelerometers tell them apart.
    int others = 0, farOff = 0;  // (not `far`: a macro in the Windows headers)
    bool locked = false;  // a locked room: only the origin's accelerometer says it was knocked
    for (const auto& b : room_.Bases()) locked |= RoomMap::Locked(b);
    for (const auto& [serial, b] : out.bases) {
      if (locked) break;
      if (serial == origin || b.keyframes <= 0 || !room_.Base(serial)) continue;
      ++others;
      farOff += room_.FarOff(serial, b.worldFromBase, b.positionVariance, b.rotationVariance);
    }
    if (others >= 2 && farOff == others) {
      room_.Bump(origin, "every other base appears moved by it");
      CollectLogs(nullptr);
      return;
    }
  }
  const uint32_t gauge = room_.OriginRelearning() ? room_.OriginGauge() : 0;  // held fixed by the solve
  for (const auto& [serial, b] : out.bases)
    if (serial != origin && serial != gauge && b.keyframes > 0)
      room_.ApplySolved(serial, b.worldFromBase, b.positionVariance, b.rotationVariance, b.keyframes, b.rms, b.fresh,
                          b.devices, b.span);
  CollectLogs(nullptr);
}

// A locked base is moved by nothing but its accelerometer, and a nudge under kBumpRad
// (a photo stand knocked a little) never trips that: every device then fights the stale
// pose its own way, reacquiring one after another (GitHub report: controllers and the
// headset jumping in turn between two "universes", forever). The solver re-solves every
// keyframe's device pose from the light itself, so a drifting filter can't fake this:
// solves that place a locked base clearly elsewhere, several in a row, are a move, handled
// like a knock (relearned from fresh keyframes). From one device only, its keyframes must
// span 10 s and it takes more solves (one short stretch from one spot can't tell a moved
// base from a misplaced device). A moved origin shows as every other base moved alike.
bool SessionImpl::LightMovedLockedBase(const RoomSolveOutput& out) {
  {
    std::set<std::string> now;
    for (const auto& [device, ratio] : out.excludedDevices) now.insert(device);
    if (now != excludedFromRoom_) {
      for (const auto& [device, ratio] : out.excludedDevices)
        if (!excludedFromRoom_.count(device)) {
          char text[200];
          std::snprintf(text, sizeof text, "%s: its light fits the room %.1fx worse than the other devices'; left out of the room solve until it agrees",
                        device.c_str(), ratio);
          Emit(SessionEventType::Log, device, text, LogLevel::Warn);
        }
      for (const auto& device : excludedFromRoom_)
        if (!now.count(device)) Emit(SessionEventType::Log, device, device + ": agrees with the room again; back in the room solve", LogLevel::Info);
      excludedFromRoom_ = now;
    }
  }
  // Floors: 15 mm, or 1 % of the base's distance (a base 5 m away wanders 3-5 cm between
  // solves: vrchat-01), and 0.75 deg. One relearn from the light per base a minute, so a
  // base it relocks too soon can't cycle.
  constexpr double kFloor = 0.015, kFloorPerMetre = 0.01, kRotFloor = 0.75 * M_PI / 180, kSigmas = 4;
  constexpr double kCooldown = 60;
  constexpr int kStrikes = 3, kStrikesOneDevice = 5;  // solves in a row (1-4 s apart)
  constexpr double kOneDeviceSpan = 10;              // s of one device's keyframes of the base
  const uint32_t origin = room_.Origin();
  if (room_.OriginRelearning()) return false;
  struct Move { double mm, deg; int strikes, rejected = 0, seen = 0; };
  std::map<uint32_t, Move> moved;
  int others = 0;
  for (const auto& [serial, b] : out.bases) {
    const RoomBase* rb = room_.Base(serial);
    if (!rb || serial == origin || !RoomMap::Locked(*rb)) continue;
    if (const auto c = lightCooldown_.find(serial); c != lightCooldown_.end() && now_ < c->second) continue;
    // The fit can keep the old pose by rejecting what disagrees: most keyframes that see
    // this base rejected, those without it fitting. That is all a nudge shows until the
    // window is mostly new keyframes (nudge-01: ch3's keyframes rejected 1 -> 85, then all
    // of them, while the solve moved it 2 mm).
    const int othersSeen = out.keyframesUsed + out.keyframesRejected - b.seen;
    const int othersRejected = out.keyframesRejected - b.rejected;
    // As clear from one device as from several: three solves, a quarter rejected, under
    // a tenth of the others.
    // More than one device must say so when more than one sees the base: one device's
    // wrong view must not relearn the room.
    const int needDevices = std::min(2, std::max(1, b.checkedDevices));
    // Devices that see it at all: one whose keyframes of it are all rejected is in neither
    // checkedDevices nor devices, and alone it met a one-device rule (#42).
    if (b.rejected >= 15 && b.rejected * 4 >= b.seen && othersSeen >= 10 && othersRejected * 10 <= othersSeen &&
        b.rejectingDevices >= std::min(2, std::max(1, b.seeingDevices))) {
      moved[serial] = {0, 0, kStrikes, b.rejected, b.seen};
      continue;
    }
    if (b.keyframes <= 0) continue;
    ++others;
    if (b.fresh <= 0 || b.devices < 1) continue;
    if (b.devices < 2 && b.span < kOneDeviceSpan) continue;
    const double dp = (b.worldFromBase.translation() - rb->worldFromBase.translation()).norm();
    const double dr = Eigen::AngleAxisd(rb->worldFromBase.rotation().transpose() * b.worldFromBase.rotation()).angle();
    const double sigma = std::sqrt((rb->positionVariance + b.positionVariance) / 3);
    const double rsigma = std::sqrt((rb->rotationVariance + b.rotationVariance) / 3);
    const double floor = std::max(kFloor, kFloorPerMetre * rb->worldFromBase.translation().norm());
    // The solve drifting is not enough: a room fit at 400 urad wanders 10-50 mm between
    // solves (live 2026-11-09: four false knocks at 11-51 mm, none with a keyframe
    // rejected). A base that moved also has its keyframes rejected.
    if ((dp > std::max(floor, kSigmas * sigma) || dr > std::max(kRotFloor, kSigmas * rsigma)) &&
        b.supportingDevices >= needDevices && b.rejected >= 8 && b.rejected * 8 >= b.seen)
      moved[serial] = {dp * 1e3, dr * 180 / M_PI, b.devices >= 2 ? kStrikes : kStrikesOneDevice};
  }
  // Every other base off at once (3+ bases): the origin moved, not all of them.
  int farOff = 0;
  for (const auto& [serial, m] : moved) farOff += m.rejected == 0;
  const auto originCooling = lightCooldown_.find(origin);
  const bool originCool = originCooling != lightCooldown_.end() && now_ < originCooling->second;
  if (others >= 2 && farOff == others && int(moved.size()) == others && originCool) {
    moved.clear();  // the origin was just relearned from this: nothing else is bumped in its place (#42)
  } else if (others >= 2 && farOff == others && int(moved.size()) == others) {
    int strikes = kStrikes;
    for (const auto& [serial, m] : moved) strikes = std::max(strikes, m.strikes);
    moved = {{origin, {0, 0, strikes}}};
  }
  for (auto it = lightStrikes_.begin(); it != lightStrikes_.end();)
    it = moved.count(it->first) ? std::next(it) : lightStrikes_.erase(it);
  bool bumped = false;
  for (const auto& [serial, d] : moved) {
    if (++lightStrikes_[serial] < d.strikes) continue;
    lightStrikes_.erase(serial);
    char why[160];
    if (serial == origin) std::snprintf(why, sizeof why, "the light places every other base elsewhere, %d solves running", d.strikes);
    else if (d.rejected) std::snprintf(why, sizeof why, "the light no longer fits it: %d of its %d keyframes rejected, %d solves running", d.rejected, d.seen, d.strikes);
    else std::snprintf(why, sizeof why, "the light places it %.0f mm / %.2f deg away, %d solves running", d.mm, d.deg, d.strikes);
    room_.Bump(serial, why);
    lightCooldown_[serial] = now_ + kCooldown;
    bumped = true;
  }
  return bumped;
}

void SessionImpl::SolverMain() {
  std::unique_lock lock(mutex_);
  while (!stop_) {
    solverWake_.wait_for(lock, std::chrono::milliseconds(100));
    if (stop_) break;
    Tick();
    if (!newKeyframes_ || !SolveDue()) continue;
    newKeyframes_ = false;
    lastSolve_ = now_;
    const RoomSolveInput input = SolveInput();
    const uint64_t resets = roomResets_, bumps = room_.Bumps();
    lock.unlock();
    const RoomSolveOutput out = SolveRoom(input);
    lock.lock();
    // A reset meanwhile: solved against the old room. A base bumped meanwhile: solved from
    // keyframes before the knock, it would re-place the base where it was and end the
    // relearn (#42). The next solve, from the fresh keyframes, is soon.
    if (resets == roomResets_ && bumps == room_.Bumps()) ApplySolve(out);
  }
}

void SessionImpl::PushIdentities() {
  std::vector<tdm::TdmDisambiguator::Identity> all;
  for (const auto& [serial, id] : identities_) all.push_back(id);
  std::map<int, std::pair<BaseConfig, BaseDynamicState>> channels;  // LH2: one channel per base, every device
  for (const auto& [serial, b] : bases_)
    if (b.dynamic.sobChannel >= 0 && b.config.valid && b.config.cal.size() == 14)
      channels[b.dynamic.sobChannel] = {b.config, b.dynamic};
  for (auto& [id, d] : devices_)
    if (d.tracker) {
      d.tracker->Source().SetKnownIdentities(all);
      d.tracker->Source().SetKnownSobChannels(channels);
    }
}

void SessionImpl::CollectLogs(Device* d) {
  if (d)
    for (auto& text : d->tracker->TakeLogs()) Emit(SessionEventType::Log, d->info.serial, std::move(text));
  const uint64_t before = room_.Revision();
  for (auto& text : room_.TakeLogs()) {
    const bool located = text.rfind("Located base", 0) == 0 || text.rfind("Setting ", 0) == 0;
    Emit(located ? SessionEventType::BaseStationLocated : SessionEventType::RoomChanged, "", std::move(text));
  }
  (void)before;
}

void SessionImpl::MaybeSave() {
  WatchRoomFile();
  if (!persist_ || !room_.Dirty()) return;
  const double wall = SteadyNow();
  if (wall - lastSave_ < 10.0) return;
  lastSave_ = wall;
  WatchRoomFile(true);  // never write over a deletion before it's seen
  if (!room_.Dirty()) return;
  std::string err;
  if (room_.Save(roomPath_, &err)) room_.ClearDirty();
  else Emit(SessionEventType::Log, "", err);
}

DevicePose SessionImpl::MakePose(const Device& d) const {
  DevicePose pose;
  pose.imuFromHead = ToPose(d.imuFromHead);
  if (!d.tracker) return pose;
  const auto out = d.tracker->Output();
  pose.state = out.state;
  pose.result = out.result;
  pose.valid = out.valid;
  pose.hostTime = d.haveClock ? HostTime(d, out.deviceTime) : now_;
  pose.worldFromImu = ToPose(out.worldFromImu);
  pose.worldFromHead = ToPose(out.worldFromImu * d.imuFromHead);
  pose.velocity = {out.velocity.x(), out.velocity.y(), out.velocity.z()};
  pose.angularVelocity = {out.angularVelocity.x(), out.angularVelocity.y(), out.angularVelocity.z()};
  pose.acceleration = {out.acceleration.x(), out.acceleration.y(), out.acceleration.z()};
  pose.primaryBase = out.primary;
  pose.bases = out.bases;
  pose.coverage = out.coverage;
  pose.updates = out.updates;
  pose.filterPosition = {out.filterPosition.x(), out.filterPosition.y(), out.filterPosition.z()};
  pose.filterTime = d.haveClock ? HostTime(d, out.filterTime) : now_;
  return pose;
}

SessionSnapshot SessionImpl::Snapshot() const {
  std::lock_guard lock(mutex_);
  SessionSnapshot snap;
  if (options_.replayPath.empty() && running_) now_ = std::max(now_, SteadyNow());
  snap.hostTime = now_;
  snap.roomRevision = room_.Revision();
  if (const Room* room = room_.Active()) snap.roomId = room->id;
  snap.originEpoch = room_.OriginEpoch();
  for (const auto& [id, d] : devices_) {
    if (!d.tracker) continue;  // receivers / devices without a usable tracking config
    DeviceState s;
    s.info = d.info;
    s.info.connected = d.preinitConnected || (d.attached && now_ - d.lastData < 1.0);
    s.info.enabled = Enabled(id);
    s.pose = MakePose(d);
    if (!s.info.connected || !s.info.enabled || d.preinitConnected) {
      s.pose.state = TrackingState::Disconnected;
      s.pose.valid = false;
      s.pose.result = TrackingResult::Uninitialized;
    }
    if (d.input && s.info.enabled) s.input = d.input->State();
    snap.devices.push_back(std::move(s));
  }
  std::map<uint32_t, BaseStationState> bases;
  for (const auto& [serial, b] : bases_) {
    auto& s = bases[serial];
    s.id = serial;
    s.serial = BaseName(serial);
    s.seen = b.seen;
    s.mode = b.dynamic.mode;
    // lhbs multi channels are its slot (1-4 on the display), stock ones A, b, c.
    s.lh2 = b.dynamic.sobChannel >= 0;
    if (b.dynamic.sobChannel >= 0) s.channel = std::to_string(b.dynamic.sobChannel + 1);  // LH2: 1-16
    else if (b.dynamic.multiSlot >= 0) s.channel = std::to_string(b.dynamic.multiSlot + 1);
    else if (b.dynamic.mode >= 1 && b.dynamic.mode <= 3) s.channel = std::string(1, "Abc"[b.dynamic.mode - 1]);
    s.faults = b.dynamic.faults;
  }
  for (const auto& rb : room_.Bases()) {
    auto& s = bases[rb.serial];
    s.id = rb.serial;
    s.serial = BaseName(rb.serial);
    s.known = true;
    s.worldFromBase = ToPose(rb.worldFromBase);
    s.positionVariance = rb.positionVariance;
    s.rotationVariance = rb.rotationVariance;
    s.origin = rb.origin;
    s.locked = RoomMap::Locked(rb);
    s.observations = std::max(rb.observations, rb.storedObservations);
    s.sigma = rb.origin ? 0.0 : std::sqrt(std::max(rb.positionVariance, 0.0));
    s.learned = rb.origin || s.sigma <= BaseStationState::kLearnedSigma ||
                ((s.locked || s.observations >= BaseStationState::kLearnedObservations) &&
                 s.sigma <= BaseStationState::kLearnedLockedSigma);
  }
  for (auto& [serial, s] : bases)
    if (s.seen || s.known) snap.bases.push_back(s);
  return snap;
}

std::vector<DeviceUpdate> SessionImpl::Updates() const {
  std::lock_guard lock(mutex_);
  if (options_.replayPath.empty() && running_) now_ = std::max(now_, SteadyNow());
  std::vector<DeviceUpdate> out;
  out.reserve(devices_.size());
  for (const auto& [id, d] : devices_) {
    if (!d.tracker) continue;
    DeviceUpdate u;
    u.serial = id;
    u.role = d.info.role;
    // As Snapshot(): a preinitialized device shows connected (not tracking) before it attaches (#49.6).
    u.connected = d.preinitConnected || (d.attached && now_ - d.lastData < 1.0);
    u.enabled = Enabled(id);
    u.batteryPercent = d.info.batteryPercent;
    u.charging = d.info.charging;
    u.pose = MakePose(d);
    if (!u.connected || !u.enabled || d.preinitConnected) {
      u.pose.state = TrackingState::Disconnected;
      u.pose.valid = false;
      u.pose.result = TrackingResult::Uninitialized;
    }
    if (d.input && u.enabled) u.input = d.input->State();
    out.push_back(std::move(u));
  }
  return out;
}

std::vector<SessionEvent> SessionImpl::PollEvents() {
  std::lock_guard lock(mutex_);
  std::vector<SessionEvent> out(events_.begin(), events_.end());
  events_.clear();
  return out;
}

void SessionImpl::LoadLearnedImu(const std::string& path) {
  std::ifstream in(path);
  if (!in) return;
  try {
    const json j = json::parse(in);
    if (!j.contains("known_objects") || !j.at("known_objects").is_array()) return;
    auto vec = [](const json& o, const char* key, std::array<double, 3>* out) {
      if (!o.contains(key)) return false;
      const auto& v = o.at(key);
      if (!v.is_array() || v.size() != 3) return false;
      for (size_t i = 0; i < 3; ++i) {
        if (!v[i].is_number()) return false;
        (*out)[i] = v[i].get<double>();
        if (!std::isfinite((*out)[i])) return false;
      }
      return true;
    };
    for (const auto& object : j.at("known_objects")) {
      if (!object.is_object() || !object.contains("imu") || !object.value("serialNumber", json()).is_string()) continue;
      const auto& imu = object.at("imu");
      if (!imu.is_object()) continue;
      LearnedImu l;
      l.acc = vec(imu, "acc_bias", &l.accBias);
      l.scale = vec(imu, "acc_scale", &l.accScale) &&
                std::all_of(l.accScale.begin(), l.accScale.end(), [](double v) { return v > 0.5 && v < 2.0; });
      l.gyro = vec(imu, "gyro_bias", &l.gyroBias);
      if (l.acc || l.scale || l.gyro) learnedImu_[object.at("serialNumber").get<std::string>()] = l;
    }
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, "", "lighthousedb " + path + ": learned IMU calibration not read (" + e.what() + ")");
  }
}

// Only where the device's own config has none: its values are its factory calibration.
void SessionImpl::ApplyModelScale(const DeviceId& id, const std::string& resourceRoot, TrackedObjectModel* model) {
  for (const std::string root : {std::string("lighthouse"), resourceRoot}) {
    if (root.empty() || !scaleRoots_.insert(root).second) continue;
    const std::string dir = DriverResourceDirectory(root);
    if (dir.empty()) continue;
    std::ifstream in(std::filesystem::u8path(dir) / "lighthouse_scale.json", std::ios::binary);
    if (!in) continue;
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::vector<ScaleReference> refs;
    std::string err;
    if (!ParseScaleReferences(text, &refs, &err)) {
      Emit(SessionEventType::Log, id, "{" + root + "}/lighthouse_scale.json: " + err, LogLevel::Warn);
      continue;
    }
    Emit(SessionEventType::Log, id, "{" + root + "}/lighthouse_scale.json: " + std::to_string(refs.size()) + " reference objects", LogLevel::Info);
    for (auto& r : refs) scaleRefs_.push_back(std::move(r));
  }
  const auto match = ApplyScaleReference(scaleRefs_, &model->points);
  for (const auto& name : match.mismatched)
    Emit(SessionEventType::Log, id, id + ": similar CAD but scale mismatch to reference object " + name, LogLevel::Info);
  if (match.reference) {
    char scale[32];
    std::snprintf(scale, sizeof scale, "%.4f", match.reference->scale);
    Emit(SessionEventType::Log, id, id + ": applying scale " + scale + " from reference object " + match.reference->name, LogLevel::Info);
  }
}

void SessionImpl::ApplyLearnedImu(const DeviceId& id, const std::string& config, TrackedObjectModel* model) {
  const auto found = learnedImu_.find(id);
  if (found == learnedImu_.end()) return;
  json imu;
  try {
    const json j = json::parse(config);
    if (j.contains("imu") && j.at("imu").is_object()) imu = j.at("imu");
  } catch (const std::exception&) {
    return;
  }
  const auto has = [&](const char* key) { return imu.is_object() && imu.contains(key); };
  const auto& l = found->second;
  std::string used;
  if (l.acc && !has("acc_bias")) { model->accBias = l.accBias; used += " acc_bias"; }
  if (l.scale && !has("acc_scale")) { model->accScale = l.accScale; used += " acc_scale"; }
  if (l.gyro && !has("gyro_bias")) { model->gyroBias = l.gyroBias; used += " gyro_bias"; }
  if (!used.empty() && learnedImuLogged_.insert(id).second)
    Emit(SessionEventType::Log, id, id + ": no IMU calibration in its config; using SteamVR's learned" + used, LogLevel::Info);
}

void SessionImpl::LoadSettings() {
  if (settingsPath_.empty()) return;
  std::ifstream in(settingsPath_);
  if (!in) return;
  try {
    const json j = json::parse(in);
    if (j.contains("controllers")) {
      const auto& controllers = j.at("controllers");
      if (!controllers.is_object()) throw std::runtime_error("controllers must be an object");
      const auto left = controllers.value("left", std::string{});
      const auto right = controllers.value("right", std::string{});
      if (!left.empty() && left == right) throw std::runtime_error("left and right controller serials must differ");
      leftController_ = left;
      rightController_ = right;
      explicitControllers_ = true;
    }
    for (const auto& serial : j.value("disabledDevices", std::vector<std::string>{})) disabled_.insert(serial);
    if (j.contains("mixedLight")) {
      // 1.0 and 2.0 base stations together: true/false, or
      // {"enabled": true, "controllers": true, "trackers": true, "scout": true, "preferLh2": false,
      // "rotorReference": true}. Under SteamVR the
      // driver's settings page wins once it has values (this seeds them); Monado and WiVRn use this.
      const auto& m = j.at("mixedLight");
      MixedLightOptions o;
      if (m.is_boolean()) {
        o.enabled = m.get<bool>();
      } else if (m.is_object()) {
        auto get = [&](const char* key, bool fallback) {
          if (!m.contains(key)) return fallback;
          if (!m.at(key).is_boolean()) throw std::runtime_error(std::string("mixedLight.") + key + " must be true or false");
          return m.at(key).get<bool>();
        };
        o.enabled = get("enabled", false);
        o.controllers = get("controllers", true);
        o.trackers = get("trackers", true);
        o.scout = get("scout", true);
        o.preferLh2 = get("preferLh2", false);
        o.rotorReference = get("rotorReference", true);
      } else {
        throw std::runtime_error("mixedLight must be true, false or an object");
      }
      mixed_ = o;
    }
    if (j.contains("hideLostDevices")) {
      if (!j.at("hideLostDevices").is_boolean()) throw std::runtime_error("hideLostDevices must be true or false");
      hideLost_ = j.at("hideLostDevices").get<bool>();
    }
    if (j.contains("roles")) {
      // {"LHR-...": "left_hand" | "right_hand" | "generic_tracker"}, per device; the rest
      // follow their config's tracked_controller_role.
      for (const auto& [serial, role] : j.at("roles").items()) {
        const std::string r = Lower(role.get<std::string>());
        if (r != "left_hand" && r != "right_hand" && r != "generic_tracker")
          throw std::runtime_error("roles: " + serial + ": expected left_hand, right_hand or generic_tracker");
        roles_[serial] = r;
      }
    }
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, "", "settings " + settingsPath_ + ": " + e.what());
  }
}

void SessionImpl::SetOutputTuning(double positionSmoothing, double rotationSmoothing, bool reportAcceleration) {
  std::lock_guard lock(mutex_);
  const double position = std::clamp(positionSmoothing, 0.0, 1.0), rotation = std::clamp(rotationSmoothing, 0.0, 1.0);
  if (position == smoothPosition_ && rotation == smoothRotation_ && reportAcceleration == reportAcceleration_) return;
  smoothPosition_ = position;
  smoothRotation_ = rotation;
  reportAcceleration_ = reportAcceleration;
  for (auto& [id, d] : devices_) {
    if (!d.tracker) continue;
    d.tracker->SetSmoothing(position, rotation);
    d.tracker->SetReportAcceleration(reportAcceleration);
  }
  char text[160];
  std::snprintf(text, sizeof text, "smoothing: position %.0f ms, rotation %.0f ms%s; acceleration %s", position * 1000,
                rotation * 1000, position == 0 && rotation == 0 ? " (off)" : "", reportAcceleration ? "reported" : "not reported");
  Emit(SessionEventType::Log, "", text, LogLevel::Info);
}

// A new tracker, or one whose room changed: sorting as the room needs, and in a room of both
// kinds a fresh choice of light mode, since it knows no rotors yet.
void SessionImpl::StartMixedLight(Device& d) {
  if (!d.tracker) return;
  const bool mixing = mixedActive_ && !d.lh1Only;
  // With the switch on, rotors are learned from LH2 words even before it's in use, so they're ready
  // when it is; in a 1.0-only room there are none.
  d.tracker->Source().SetSorting(mixed_.enabled && !d.lh1Only, mixing);
  d.tracker->SetMixedLightScatter(mixing ? runtime::kSortedHitScatter : 0, mixing ? runtime::kLh1HitScatter : 0,
                                  mixing ? runtime::kRotorWobble : 0, runtime::kRotorWobbleMemory);
  // MixedLightOptions::rotorReference (asked per frame, under the session's lock).
  d.tracker->SetRotorReference([this, &d](uint32_t base, double deviceTime, double* radians) {
    return mixedActive_ && mixed_.rotorReference && d.haveClock && rotorReference_.Offset(base, HostTime(d, deviceTime), radians);
  });
  if (mixing) d.lightMode = -1;
}

// Both kinds together only where there's a 2.0 station to sort: one in the room in use, or seen this
// session. Before any base is seen no room is chosen yet, and it isn't in use: SteamVR's database often
// has 2.0 stations of other rooms (the ms-a test capture's does), and in a 1.0-only room devices would
// start in LH2 mode, where 1.0 stations can't be seen, waiting 30 s for 2.0 stations to wake. In use
// there, it would also hold back lhbs multi-station detection.
void SessionImpl::UpdateMixedRoom() {
  const auto* room = room_.Active();
  uint32_t seen = 0, inRoom = 0;
  for (const auto& [serial, b] : bases_) {
    if (b.config.model == 1) continue;
    if (b.seen && !seen) seen = serial;
    if (room && room->bases.count(serial) && !inRoom) inRoom = serial;
  }
  lh2Seen_ = seen != 0;
  lh2Placed_ = inRoom != 0;
  // The room's 2.0 stations alone can be off or asleep: devices only look for their light now and then
  // (UpdateMixedLight), and with none for a while it stands down for that room (runtime::ChooseMixedRoom).
  runtime::MixedRoomView view;
  view.enabled = mixed_.enabled;
  view.lh2Seen = lh2Seen_;
  view.lh2InRoom = inRoom && room->id != darkRoom_;
  view.inUse = mixedActive_ ? now_ - mixedSince_ : -1;
  view.sinceLh2Light = now_ - lastLh2Light_;
  const auto step = runtime::ChooseMixedRoom(view);
  if (step == runtime::MixedRoomStep::RoomDark) darkRoom_ = room->id;
  const bool active = step == runtime::MixedRoomStep::InUse;
  if (active == mixedActive_) return;
  mixedActive_ = active;
  mixedSince_ = now_;
  // Starting, every device chooses again (StartMixedLight). Switched off, every device that was mixing goes
  // to LH2 mode, 2.0 light alone, and the usual choice goes on from there (a room without 2.0 stations,
  // the other way to stop, is left to the usual choice as it is). Not choosing afresh: in a room of
  // both kinds that's LH1 mode ("default"), the headset's too, which the stock driver, when it runs the
  // headset's display, switches back after about 11 s, starting its own tracking over. And not staying in
  // LH1 mode: unsorted, a device's light there is full of 2.0 hits it can't use, and the restarts it keeps
  // making cost enough under the session's lock that the headset's USB reports were dropped (its IMU from
  // 1011 to 444-867 samples/s). Live, either way, the headset lost tracking after every switch-off.
  for (auto& [id, d] : devices_) {
    const bool wasMixing = d.tracker && d.tracker->Source().Sorting();
    StartMixedLight(d);
    if (active || mixed_.enabled || !wasMixing || d.lightMode != 0 || !usb_ || !d.attached) continue;
    usb_->SetOpticalSubProtocol(id, 1);
    d.lightMode = 1;
    d.modeSince = d.modeSent = now_;
    Emit(SessionEventType::Log, id, id + ": light mode LH2 (1.0 + 2.0 switched off)");
  }
  char text[160];
  if (active)
    std::snprintf(text, sizeof text, "1.0 + 2.0 base stations together: in use (2.0 base %08X %s)", seen ? seen : inRoom,
                  seen ? "seen" : "in the room");
  else if (step == runtime::MixedRoomStep::RoomDark)
    std::snprintf(text, sizeof text, "1.0 + 2.0 base stations together: not in use (no 2.0 light for %.0f s: the room's 2.0 stations look off)",
                  runtime::kLh2RoomDark);
  else
    std::snprintf(text, sizeof text, "1.0 + 2.0 base stations together: not in use");
  Emit(SessionEventType::Log, "", text, step == runtime::MixedRoomStep::RoomDark ? LogLevel::Warn : LogLevel::Info);
}

void SessionImpl::SetMixedLight(const MixedLightOptions& options) {
  std::lock_guard lock(mutex_);
  if (options == mixed_) return;
  Tick();
  for (auto& [id, d] : devices_) {
    // Let in while in LH2 mode: as at the start of a visit, it waits for rotors still being
    // learned or named, rather than leaving at once because it has been there long.
    if (mixedActive_ && d.lightMode == 1 && runtime::MixesLight(d.info.role, options) && !runtime::MixesLight(d.info.role, mixed_))
      d.modeSince = now_;
  }
  const bool switched = options.enabled != mixed_.enabled;
  if (switched) darkRoom_.clear();  // turning it on again tries a dark room's 2.0 stations again
  mixed_ = options;
  auto onOff = [](bool on) { return on ? "on" : "off"; };
  Emit(SessionEventType::Log, "", std::string("1.0 + 2.0 base stations together: switch ") + onOff(options.enabled) + " (controllers " +
                                  onOff(options.controllers) + ", trackers " + onOff(options.trackers) + ", scout " +
                                  onOff(options.scout) + ", LH2 mode preferred " + onOff(options.preferLh2) + ", rotor reference " +
                                  onOff(options.rotorReference) + ")",
                                  LogLevel::Info);
  UpdateMixedRoom();
  if (switched)  // learning rotors starts or stops, in use or not
    for (auto& [id, d] : devices_) StartMixedLight(d);
}

void SessionImpl::SaveSettings() {
  if (settingsPath_.empty()) return;
  try {
    json j = json::object();
    {
      std::ifstream in(settingsPath_);
      if (in) j = json::parse(in);
    }
    j["disabledDevices"] = std::vector<std::string>(disabled_.begin(), disabled_.end());
    const std::filesystem::path target(settingsPath_);
    if (target.has_parent_path()) std::filesystem::create_directories(target.parent_path());
    const std::string temp = settingsPath_ + ".tmp";
    {
      std::ofstream out(temp);
      out << j.dump(2) << "\n";
      out.close();
      // A failed write must not replace the good file: its controllers and roles were lost (#43).
      if (!out) throw std::runtime_error("cannot write " + temp);
    }
    platform::ReplaceFile(temp, target);
  } catch (const std::exception& e) {
    Emit(SessionEventType::Log, "", "saving settings " + settingsPath_ + ": " + e.what());
  }
}

void SessionImpl::SetDeviceEnabled(const std::string& serial, bool enabled) {
  std::lock_guard lock(mutex_);
  if (Enabled(serial) == enabled) return;
  if (enabled) disabled_.erase(serial);
  else disabled_.insert(serial);
  auto found = devices_.find(serial);
  if (found != devices_.end() && found->second.tracker) {
    // Start clean when re-enabled; stale filter state is worse than a fresh bootstrap.
    auto& d = found->second;
    TrackedObjectModel model;
    if (TrackedObjectModel::FromJson(d.info.configJson, &model)) {
      ApplyModelScale(serial, d.info.resourceRoot, &model);
      ApplyLearnedImu(serial, d.info.configJson, &model);
      model.serial = serial;
      d.tracker = std::make_unique<DeviceTracker>(serial, model, &room_);
      d.tracker->SetSmoothing(smoothPosition_, smoothRotation_);
      d.tracker->SetReportAcceleration(reportAcceleration_);
      d.tracker->SetHoldStill(d.info.role != DeviceRole::Hmd);  // a still head must not freeze the view
      StartMixedLight(d);
      for (const auto& [base, b] : bases_) d.tracker->OnBaseConfig(b.config, b.dynamic);
      PushIdentities();
    }
    d.lastState = TrackingState::Disconnected;
  }
  Emit(SessionEventType::DeviceEnabledChanged, serial, serial + (enabled ? " enabled" : " disabled"));
  SaveSettings();
}

void SessionImpl::ResetRoom() {
  std::lock_guard lock(mutex_);
  ResetRoomLocked("reset requested");
}

void SessionImpl::ResetRoomLocked(const std::string& why) {
  // Every room goes, and with it everything learned against it: keyframes, solves in
  // flight, and each device's filter (its base frames and alt-base relationships).
  room_.ForgetAll();
  keyframes_.Clear();
  lastSolved_.clear();
  solveInterval_ = 1.0;
  newKeyframes_ = false;
  ++roomResets_;
  for (auto& [id, d] : devices_) {
    if (!d.tracker) continue;
    TrackedObjectModel model;
    if (!TrackedObjectModel::FromJson(d.info.configJson, &model)) continue;
    ApplyModelScale(id, d.info.resourceRoot, &model);
    ApplyLearnedImu(id, d.info.configJson, &model);
    model.serial = id;
    d.tracker = std::make_unique<DeviceTracker>(id, model, &room_);
    d.tracker->SetSmoothing(smoothPosition_, smoothRotation_);
    d.tracker->SetReportAcceleration(reportAcceleration_);
    d.tracker->SetHoldStill(d.info.role != DeviceRole::Hmd);  // a still head must not freeze the view
    StartMixedLight(d);
    for (const auto& [base, b] : bases_) d.tracker->OnBaseConfig(b.config, b.dynamic);
    d.lastState = TrackingState::Disconnected;
  }
  PushIdentities();
  // Seen bases are seen again: the first one becomes the new room's origin.
  for (const auto& [serial, b] : bases_)
    if (b.seen) room_.OnBaseSeen(b.config, b.dynamic);
  if (persist_) {
    std::string err;
    // Seen only once it's written: a failed write then looked like the file being
    // deleted, and reset the room again every second (#43).
    if (room_.Save(roomPath_, &err)) {
      room_.ClearDirty();
      roomFileSeen_ = true;
    } else {
      Emit(SessionEventType::Log, "", "saving the room map: " + err, LogLevel::Warn);
    }
  }
  Emit(SessionEventType::RoomChanged, "", "room reset (" + why + "): every base station is relearned");
  CollectLogs(nullptr);
}

void SessionImpl::WatchRoomFile(bool now) {
  // Deleting the room map file while running resets the room (an external switch).
  if (!persist_ || roomPath_.empty()) return;
  const double wall = SteadyNow();
  if (!now && wall - lastRoomFileCheck_ < 1.0) return;
  lastRoomFileCheck_ = wall;
  std::error_code ec;
  const bool exists = std::filesystem::exists(roomPath_, ec);
  if (ec) return;
  if (exists) roomFileSeen_ = true;
  else if (roomFileSeen_) { roomFileSeen_ = false; ResetRoomLocked("room map file deleted"); }
}

bool SessionImpl::SaveRoom(std::string* error) {
  std::lock_guard lock(mutex_);
  if (roomPath_.empty()) {
    if (error) *error = "no room map path (replay without --room)";
    return false;
  }
  if (!room_.Save(roomPath_, error)) return false;
  room_.ClearDirty();
  return true;
}

}  // namespace runtime

std::unique_ptr<Session> Session::Create(const SessionOptions& options) {
  return std::make_unique<runtime::SessionImpl>(options);
}

}  // namespace lh
