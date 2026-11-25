#include "lh/tracker.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>

#include "lighthouse_direct.hpp"
#include "tracking_source.hpp"

namespace lh {
namespace {
Eigen::Quaterniond Quaternion(const RigidPose& p) {
  return {p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]};
}
bool Valid(const RigidPose& p) {
  const auto q = Quaternion(p);
  return q.coeffs().allFinite() && q.norm() > 1e-9 &&
         std::all_of(p.position.begin(), p.position.end(), [](double v) { return std::isfinite(v); });
}
RigidPose ReadPose(const nlohmann::json& j) {
  RigidPose pose;
  if (!j.is_array() || j.size() != 7) throw std::runtime_error("base pose must be [qx,qy,qz,qw,x,y,z]");
  pose.orientation = {j[3].get<double>(), j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
  pose.position = {j[4].get<double>(), j[5].get<double>(), j[6].get<double>()};
  if (!Valid(pose)) throw std::runtime_error("non-finite or degenerate base pose");
  auto q = Quaternion(pose).normalized();
  pose.orientation = {q.w(), q.x(), q.y(), q.z()};
  return pose;
}
}  // namespace

struct Tracker::Impl {
  struct Device {
    bool attached = true;
    std::string config, error;
    DeviceTrackingInfo lastInfo;
    std::optional<TrackedObjectModel> model;
    std::unique_ptr<TrackingSource> source;
    std::unique_ptr<LighthouseDirect> direct;
    std::optional<double> latestInput, latestImu, latestPulse, lastPublished;
    std::optional<TrackingStatus> lastPublishedStatus;
    double clockOffset = 0;
  };
  explicit Impl(TrackerOptions o) : options(o) {}
  TrackerOptions options;
  mutable std::mutex mutex;
  std::map<DeviceId, Device> devices;
  std::map<uint32_t, std::pair<BaseConfig, BaseDynamicState>> bases;
  std::map<uint32_t, RigidPose> placements;
  struct ImuCal { std::array<double, 3> gyroBias, accBias, accScale; };
  struct Tilt { std::array<double, 2> angles; double variance; };
  std::map<DeviceId, ImuCal> calibrations;
  std::map<uint32_t, Tilt> tilts;
  std::deque<TrackedPose> poses;
  std::deque<std::string> logs;

  void Log(std::string text) {
    if (logs.size() == 256) logs.pop_front();
    logs.push_back(std::move(text));
  }
  void ResetFilter(const DeviceId& id, Device& d) {
    auto model = *d.model;
    const auto calibration = calibrations.find(id);
    if (calibration != calibrations.end()) {
      model.gyroBias = calibration->second.gyroBias;
      model.accBias = calibration->second.accBias; model.accScale = calibration->second.accScale;
    }
    d.direct = std::make_unique<LighthouseDirect>(std::move(model), options.opticalTimeout);
    for (const auto& [serial, base] : bases) d.direct->OnBaseConfig(base.first, base.second);
    for (const auto& [serial, tilt] : tilts) d.direct->SetRecordedTilt(serial, tilt.angles, tilt.variance);
  }
  void Reset(const DeviceId& id, Device& d) {
    d.source = std::make_unique<TrackingSource>(d.model->sensorType, tdm::TdmConfig{}, !d.model->sensorTypeKnown);
    ResetFilter(id, d);
    d.latestInput.reset(); d.latestImu.reset(); d.latestPulse.reset(); d.lastPublished.reset();
    d.lastPublishedStatus.reset();
  }
  void CheckGap(const DeviceId& id, Device& d, double time) {
    if (options.inputGapReset > 0 && d.latestInput && time - *d.latestInput > options.inputGapReset) {
      Reset(id, d); Log(id + ": restarting source/filter after input gap");
    }
    if (!d.latestInput || time > *d.latestInput) d.latestInput = time;
  }
  void Publish(const DeviceId& id, Device& d, std::optional<TrackingStatus> status = {}) {
    auto pose = d.direct->Pose();
    if (pose && status) pose->status = *status;
    if (!pose || (d.lastPublished && pose->time == *d.lastPublished && d.lastPublishedStatus == pose->status)) return;
    d.lastPublished = pose->time; d.lastPublishedStatus = pose->status;
    pose->device = id; pose->hostTime = pose->time + d.clockOffset;
    const auto placement = placements.find(pose->primaryBase);
    if (placement != placements.end()) {
      pose->worldSpace = true;
      const auto q = Quaternion(placement->second).normalized();
      const Eigen::Vector3d p = q * Eigen::Vector3d(pose->pose.position.data()) + Eigen::Vector3d(placement->second.position.data());
      const Eigen::Vector3d v = q * Eigen::Vector3d(pose->velocity.data());
      const auto orientation = (q * Quaternion(pose->pose)).normalized();
      std::copy(p.data(), p.data() + 3, pose->pose.position.begin());
      std::copy(v.data(), v.data() + 3, pose->velocity.begin());
      pose->pose.orientation = {orientation.w(), orientation.x(), orientation.y(), orientation.z()};
    }
    if (poses.size() == 1024) poses.pop_front();
    poses.push_back(std::move(*pose));
  }
  void Drain(const DeviceId& id, Device& d) {
    BaseConfig config; BaseDynamicState dynamic;
    while (d.source->PopBaseConfig(&config, &dynamic)) {
      // Kept and shared, as DeviceTracker does: a reset filter (ResetFilter replays `bases`)
      // and the other devices know the base at once, instead of waiting ~20 s for its OOTX
      // to decode again.
      bases[config.serial] = {config, dynamic};
      for (auto& [otherId, other] : devices)
        if (other.direct) other.direct->OnBaseConfig(config, dynamic);
      Log(id + ": OOTX base " + std::to_string(config.serial));
    }
    ImuSample imu;
    while (d.source->PopImu(&imu)) d.direct->OnImu(imu);
    OpticalFrame frame;
    while (d.source->PopFrame(&frame)) {
      if (d.direct->OnFrame(frame)) Publish(id, d);
    }
    if (d.latestImu) d.direct->AdvanceInertial(*d.latestImu - 0.02);
    Publish(id, d);
    std::string log;
    while (d.source->PopLog(&log)) Log(id + ": " + log);
    tdm::TdmEvent event;
    while (d.source->PopEvent(&event)) Log(id + ": TDM event " + std::to_string(event.type));
  }
};

Tracker::Tracker(const TrackerOptions& options) : impl_(std::make_unique<Impl>(options)) {
  if (!std::isfinite(options.inputGapReset) || !std::isfinite(options.opticalTimeout) || options.opticalTimeout <= 0)
    throw std::invalid_argument("invalid tracker timeouts");
  for (const auto& [id, type] : options.sensorTypes)
    if (type < 0 || type > 11) throw std::invalid_argument("invalid sensor hardware type");
}
Tracker::~Tracker() = default;

void Tracker::OnDeviceAttached(const DeviceId& id, const std::string& config) {
  std::lock_guard lock(impl_->mutex);
  auto& device = impl_->devices[id];
  device.attached = true;
  if (device.direct && (config.empty() || device.config == config)) return;
  if (config.empty()) { device.error = "no device configuration"; return; }
  TrackedObjectModel model;
  if (!TrackedObjectModel::FromJson(config, &model, &device.error)) {
    device.direct.reset(); device.source.reset(); device.model.reset();
    device.config.clear(); device.lastInfo = {};
    impl_->Log(id + ": " + device.error); return;
  }
  const auto sensorType = impl_->options.sensorTypes.find(id);
  if (sensorType != impl_->options.sensorTypes.end()) { model.sensorType = sensorType->second; model.sensorTypeKnown = true; }
  device.model = std::move(model); device.config = config; device.error.clear();
  impl_->Reset(id, device);
}

void Tracker::OnDeviceDetached(const DeviceId& id) {
  std::lock_guard lock(impl_->mutex);
  const auto found = impl_->devices.find(id);
  if (found == impl_->devices.end()) return;
  found->second.attached = false;
  if (found->second.direct) {
    impl_->Publish(id, found->second, TrackingStatus::Lost);
    found->second.lastInfo = found->second.direct->Info();
  }
  found->second.lastInfo.status = TrackingStatus::Lost;
  found->second.source.reset(); found->second.direct.reset();
  impl_->Log(id + ": detached");
}

void Tracker::OnClockOffset(const DeviceId& id, double offset) {
  if (!std::isfinite(offset)) return;
  std::lock_guard lock(impl_->mutex);
  impl_->devices[id].clockOffset = offset;
}

void Tracker::OnImu(const DeviceId& id, const ImuSample& sample) {
  if (!std::isfinite(sample.time)) return;
  for (double v : sample.accel) if (!std::isfinite(v)) return;
  for (double v : sample.gyro) if (!std::isfinite(v)) return;
  std::lock_guard lock(impl_->mutex);
  auto found = impl_->devices.find(id);
  if (found == impl_->devices.end() || !found->second.direct || !found->second.attached) return;
  auto& d = found->second;
  if (d.latestImu && sample.time < *d.latestImu) { impl_->Log(id + ": out-of-order IMU ignored"); return; }
  impl_->CheckGap(id, d, sample.time);
  d.latestImu = sample.time;
  d.source->OnImu(sample);
  impl_->Drain(id, d);
}

void Tracker::OnLightPulse(const DeviceId& id, const LightPulse& pulse) {
  if (!std::isfinite(pulse.time) || !std::isfinite(pulse.width) || pulse.width < 0 || pulse.sensor < 0 || pulse.sensor >= 32) return;
  std::lock_guard lock(impl_->mutex);
  auto found = impl_->devices.find(id);
  if (found == impl_->devices.end() || !found->second.direct || !found->second.attached) return;
  auto& d = found->second;
  // Wired optical reports group entries by sensor, not globally by timestamp.
  // Preserve callback order just as the original source does; TDM handles it.
  impl_->CheckGap(id, d, pulse.time);
  if (!d.latestPulse || pulse.time > *d.latestPulse) d.latestPulse = pulse.time;
  d.source->OnLightPulse(pulse);
  impl_->Drain(id, d);
}

void Tracker::OnLh2BeamWord(const DeviceId& id, const Lh2BeamWord& word) {
  if (!std::isfinite(word.time)) return;
  std::lock_guard lock(impl_->mutex);
  auto found = impl_->devices.find(id);
  if (found == impl_->devices.end() || !found->second.direct || !found->second.attached) return;
  auto& d = found->second;
  impl_->CheckGap(id, d, word.time);
  if (!d.latestPulse || word.time > *d.latestPulse) d.latestPulse = word.time;
  d.source->OnLh2BeamWord(word);
  impl_->Drain(id, d);
}

void Tracker::OnLh2SensorWord(const DeviceId& id, const Lh2SensorWord& word) {
  if (!std::isfinite(word.time)) return;
  std::lock_guard lock(impl_->mutex);
  auto found = impl_->devices.find(id);
  if (found == impl_->devices.end() || !found->second.direct || !found->second.attached) return;
  auto& d = found->second;
  impl_->CheckGap(id, d, word.time);
  d.source->OnLh2SensorWord(word);
}

void Tracker::SetBaseConfig(const BaseConfig& base, const BaseDynamicState& dynamic) {
  if (!base.valid || !base.serial || base.cal.size() != 14 || (base.hardwareVersion > 9 && base.model == 1) ||
      !std::all_of(base.cal.begin(), base.cal.end(), [](double v) { return std::isfinite(v); })) return;
  std::lock_guard lock(impl_->mutex);
  impl_->bases[base.serial] = {base, dynamic};
  for (auto& [id, d] : impl_->devices) if (d.direct) d.direct->OnBaseConfig(base, dynamic);
}

void Tracker::SetBasePose(uint32_t serial, const RigidPose& pose) {
  if (!serial || !Valid(pose)) throw std::invalid_argument("invalid base pose");
  RigidPose normalized = pose;
  const auto q = Quaternion(pose).normalized();
  normalized.orientation = {q.w(), q.x(), q.y(), q.z()};
  std::lock_guard lock(impl_->mutex);
  impl_->placements[serial] = normalized;
}

bool Tracker::LoadBasePoses(const std::string& text, std::string* error) {
  try {
    const auto j = nlohmann::json::parse(text);
    // Parse/validate first, so malformed input cannot partially change the session.
    std::map<uint32_t, RigidPose> placements;
    std::vector<std::pair<BaseConfig, BaseDynamicState>> configs;
    std::map<DeviceId, Impl::ImuCal> calibrations;
    std::map<uint32_t, Impl::Tilt> tilts;
    if (j.contains("known_objects")) {
      for (const auto& object : j.at("known_objects")) {
        if (!object.contains("imu")) continue;
        const auto& imu = object.at("imu");
        Impl::ImuCal cal{imu.at("gyro_bias").get<std::array<double, 3>>(),
                        imu.at("acc_bias").get<std::array<double, 3>>(),
                        imu.at("acc_scale").get<std::array<double, 3>>()};
        for (int i = 0; i < 3; ++i)
          if (!std::isfinite(cal.gyroBias[size_t(i)]) || !std::isfinite(cal.accBias[size_t(i)]) ||
              !std::isfinite(cal.accScale[size_t(i)]) || cal.accScale[size_t(i)] <= 0)
            throw std::runtime_error("invalid learned IMU calibration");
        calibrations[object.at("serialNumber").get<std::string>()] = cal;
      }
    }
    if (j.contains("known_universes")) {
      // Import the first universe only; never merge incompatible coordinate frames.
      const auto& universes = j.at("known_universes");
      if (!universes.empty()) {
        for (const auto& b : universes.at(0).at("base_stations")) {
          const auto& target = b.at("target_pose");
          if (target.value("target_serial_number", 0u) != 0) continue;
          placements[b.at("base_serial_number").get<uint32_t>()] = ReadPose(target.at("pose"));
        }
      }
      for (const auto& b : j.at("base_stations")) {
        const auto& c = b.at("config");
        BaseConfig config;
        config.valid = true; config.serial = c.at("serialNumber").get<uint32_t>();
        config.cal = c.at("baseCalibration").get<std::vector<double>>();
        config.hardwareVersion = c.at("modelId").get<uint8_t>();
        config.ootxVersion = c.at("ootxVersion").get<uint8_t>();
        config.model = ModelForHardware(config.hardwareVersion);
        BaseDynamicState dynamic;
        if (b.contains("dynamic_states") && !b.at("dynamic_states").empty()) {
          const auto& state = b.at("dynamic_states").back().at("dynamic_state");
          if (state.contains("gravity_vector")) dynamic.accelDir = state.at("gravity_vector").get<std::array<float, 3>>();
          const auto& last = b.at("dynamic_states").back();
          if (last.contains("tilt")) {
            const auto& tilt = last.at("tilt");
            Impl::Tilt value{{tilt.at("pitch").get<double>(), tilt.at("roll").get<double>()}, tilt.at("variance").get<double>()};
            if (!std::isfinite(value.angles[0]) || !std::isfinite(value.angles[1]) ||
                !std::isfinite(value.variance) || value.variance < 0) throw std::runtime_error("invalid recorded tilt");
            tilts[config.serial] = value;
          }
        }
        if (!config.serial || config.cal.size() != 14) throw std::runtime_error("bad base calibration");
        for (double v : config.cal) if (!std::isfinite(v)) throw std::runtime_error("non-finite calibration");
        configs.push_back({std::move(config), dynamic});
      }
    } else {
      // Minimal standalone format. No SteamVR installation is required.
      for (const auto& b : j.at("bases"))
        placements[b.at("serial").get<uint32_t>()] = ReadPose(b.at("pose"));
    }
    for (const auto& [serial, pose] : placements)
      if (!serial || !Valid(pose)) throw std::runtime_error("invalid base serial/placement");
    std::lock_guard lock(impl_->mutex);
    for (const auto& [serial, pose] : placements) impl_->placements[serial] = pose;
    for (const auto& c : configs) impl_->bases[c.first.serial] = c;
    for (const auto& [id, cal] : calibrations) impl_->calibrations[id] = cal;
    for (const auto& [serial, tilt] : tilts) impl_->tilts[serial] = tilt;
    for (auto& [id, d] : impl_->devices) {
      if (!d.direct) continue;
      if (d.direct->Info().primaryBase == 0) impl_->ResetFilter(id, d);
      else {
        for (const auto& c : configs) d.direct->OnBaseConfig(c.first, c.second);
        for (const auto& [serial, tilt] : tilts) d.direct->SetRecordedTilt(serial, tilt.angles, tilt.variance);
      }
    }
    if (error) error->clear();
    return true;
  } catch (const std::exception& e) {
    if (error) *error = e.what();
    return false;
  }
}

std::vector<TrackedPose> Tracker::PollPoses() {
  std::lock_guard lock(impl_->mutex);
  std::vector<TrackedPose> result(impl_->poses.begin(), impl_->poses.end());
  impl_->poses.clear(); return result;
}
std::vector<DeviceTrackingInfo> Tracker::Devices() const {
  std::lock_guard lock(impl_->mutex);
  std::vector<DeviceTrackingInfo> result;
  for (const auto& [id, d] : impl_->devices) {
    DeviceTrackingInfo info = d.direct ? d.direct->Info() : d.lastInfo;
    info.device = id; info.attached = d.attached; info.error = d.error;
    result.push_back(std::move(info));
  }
  return result;
}
std::vector<std::string> Tracker::PollLogs() {
  std::lock_guard lock(impl_->mutex);
  std::vector<std::string> result(impl_->logs.begin(), impl_->logs.end());
  impl_->logs.clear(); return result;
}

}  // namespace lh
