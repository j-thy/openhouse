#include "room_map.hpp"
#include "platform/platform.hpp"

#include <cstdlib>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

namespace lh::runtime {
namespace {
using nlohmann::json;

std::string Hex(uint32_t serial) {
  char text[16];
  std::snprintf(text, sizeof text, "%08X", serial);
  return text;
}

json PoseJson(const Eigen::Isometry3d& t) {
  const Eigen::Quaterniond q(t.rotation());
  return {{"position", {t.translation().x(), t.translation().y(), t.translation().z()}},
          {"orientation", {q.w(), q.x(), q.y(), q.z()}}};
}

Eigen::Isometry3d PoseFromJson(const json& j) {
  const auto p = j.at("position").get<std::array<double, 3>>();
  const auto o = j.at("orientation").get<std::array<double, 4>>();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::Quaterniond(o[0], o[1], o[2], o[3]).normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
  if (!t.matrix().allFinite()) throw std::runtime_error("non-finite pose");
  return t;
}

json ConfigJson(const BaseConfig& c, const BaseDynamicState& d) {
  return {{"serial", c.serial}, {"calibration", c.cal}, {"hardwareVersion", c.hardwareVersion},
          {"ootxVersion", c.ootxVersion}, {"gravity", d.accelDir}, {"resetCount", d.sysUnlockCount}, {"mode", d.mode}};
}

double Angle(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return Eigen::AngleAxisd(a.transpose() * b).angle();
}
}  // namespace

Eigen::Vector3d UpFromTilt(double pitch, double roll) {
  const Eigen::Vector3d v(pitch, 0, roll);
  const double angle = v.norm();
  if (angle < 1e-12) return Eigen::Vector3d::UnitY();
  return Eigen::AngleAxisd(angle, v / angle) * Eigen::Vector3d::UnitY();
}

std::array<double, 2> TiltFromUp(const Eigen::Vector3d& up) {
  const Eigen::AngleAxisd aa(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(), up.normalized()));
  const Eigen::Vector3d v = aa.axis() * aa.angle();
  return {v.x(), v.z()};
}

Eigen::Matrix3d Leveling(double pitch, double roll, int method) {
  const Eigen::Vector3d u = UpFromTilt(pitch, roll);
  if (method == 1) return Eigen::Quaterniond::FromTwoVectors(u, Eigen::Vector3d::UnitY()).toRotationMatrix();
  if (method != 0) return Eigen::Matrix3d::Identity();
  // Method 0: world Y = up, world Z = global Z projected off the up axis.
  Eigen::Vector3d z = Eigen::Vector3d::UnitZ() - u.z() * u;
  if (z.norm() < 1e-9) z = Eigen::Vector3d::UnitX() - u.x() * u;
  z.normalize();
  const Eigen::Vector3d x = u.cross(z).normalized();
  Eigen::Matrix3d rows;
  rows.row(0) = x; rows.row(1) = u; rows.row(2) = z;
  return rows;
}

double RoomMap::Now() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

bool RoomMap::Load(const std::string& path, std::string* error) {
  try {
    std::ifstream in(platform::FsPath(path));
    if (!in) { if (error) error->clear(); return true; }
    const json j = json::parse(in);
    std::vector<Room> rooms;
    for (const auto& r : j.at("rooms")) {
      Room room;
      room.id = r.at("id").get<std::string>();
      room.source = r.value("source", "lh");
      room.updated = r.value("updated", 0.0);
      for (const auto& b : r.at("bases")) {
        RoomBase base;
        base.serial = b.at("serial").get<uint32_t>();
        base.worldFromBase = PoseFromJson(b.at("pose"));
        base.positionVariance = b.value("positionVariance", 1e-4);
        base.rotationVariance = b.value("rotationVariance", 1e-4);
        base.observations = b.value("observations", 0);
        base.storedObservations = base.observations;
        base.origin = b.value("origin", false);
        // A base the room solver placed from many keyframes in an earlier session:
        // the next session's first solves (a keyframe or two) refine it gently instead
        // of replacing it at once.
        base.solved = !base.origin && base.observations >= 30;
        if (b.contains("resetCount")) base.resetCount = b.at("resetCount").get<int>();
        room.bases[base.serial] = base;
      }
      // Learned with other device model scales (1.0.2 and before applied none: a Vive headset
      // 0.4 % small): its bases are off by up to a centimetre or so, and locked there. They're
      // kept, with the weight of a short session and 1 cm of uncertainty, unlocked: the room
      // solver places them again from this session's keyframes, gliding, in the same room
      // (same universe, same Room Setup), instead of fighting every device's light.
      room.modelScale = r.value("modelScale", 0);
      if (room.modelScale != Room::kModelScale) {
        // Still learned (BaseStationState: 40 keyframes under 20 mm), so it keeps showing
        // and tracking as one; only no longer locked. 30 keyframes at 17 mm made every such
        // base "learning" again: searching in SteamVR and missing from Room Setup until relearned.
        constexpr int kEvidence = 40;  // BaseStationState::kLearnedObservations
        for (auto& [serial, base] : room.bases) {
          if (base.origin) continue;
          base.observations = std::min(base.observations, kEvidence);
          base.storedObservations = base.observations;
          base.positionVariance = std::max(base.positionVariance, 0.01 * 0.01);
          base.rotationVariance = std::max(base.rotationVariance, 0.003 * 0.003);
        }
        if (room.modelScale == 0 && !room.bases.empty())
          Log("Room " + room.id + " was learned without the devices' model scales; refining its bases again");
        room.modelScale = Room::kModelScale;
      }
      rooms.push_back(std::move(room));
    }
    std::map<uint32_t, std::pair<BaseConfig, BaseDynamicState>> configs;
    std::map<uint32_t, std::vector<uint8_t>> frames;  // kept only if the whole file loads
    if (j.contains("baseStations")) {
      for (const auto& c : j.at("baseStations")) {
        BaseConfig config;
        BaseDynamicState dynamic;
        config.valid = true;
        config.serial = c.at("serial").get<uint32_t>();
        config.cal = c.at("calibration").get<std::vector<double>>();
        config.hardwareVersion = c.value("hardwareVersion", uint8_t(0));
        config.ootxVersion = c.value("ootxVersion", uint8_t(0));
        config.model = ModelForHardware(config.hardwareVersion);
        if (c.contains("gravity")) dynamic.accelDir = c.at("gravity").get<std::array<float, 3>>();
        dynamic.sysUnlockCount = c.value("resetCount", uint8_t(0));
        dynamic.mode = c.value("mode", 0);
        if (config.cal.size() == 14) configs[config.serial] = {config, dynamic};
        if (c.contains("ootxFrame")) {
          std::vector<uint8_t> frame;
          for (char ch : c.at("ootxFrame").get<std::string>()) frame.push_back(ch == '1' ? 1 : ch == '0' ? 0 : 0xff);
          frames[config.serial] = frame;
        }
      }
    }
    rooms_ = std::move(rooms);
    for (auto& [serial, c] : configs) configs_[serial] = c;
    for (auto& [serial, f] : frames) frames_[serial] = std::move(f);
    active_ = -1;
    if (error) error->clear();
    return true;
  } catch (const std::exception& e) {
    // Moved aside, or the next save replaced it with an empty map and every stored
    // room was lost. Its rooms can still be taken back from that copy by hand.
    const std::string kept = path + ".bad-" + std::to_string(std::time(nullptr));
    std::error_code ec;
    std::filesystem::rename(path, kept, ec);
    if (error)
      *error = std::string("room map ") + path + ": " + e.what() +
               (ec ? std::string(" (could not move it aside: ") + ec.message() + ")" : "; kept as " + kept + ", starting a new map");
    return false;
  }
}

bool RoomMap::Save(const std::string& path, std::string* error) const {
  try {
    json rooms = json::array();
    for (const auto& room : rooms_) {
      json bases = json::array();
      for (const auto& [serial, b] : room.bases) {
        json jb = {{"serial", serial}, {"pose", PoseJson(b.worldFromBase)},
                   {"positionVariance", b.positionVariance}, {"rotationVariance", b.rotationVariance},
                   {"observations", b.observations}, {"origin", b.origin}};
        if (b.resetCount) jb["resetCount"] = *b.resetCount;
        bases.push_back(jb);
      }
      rooms.push_back({{"id", room.id}, {"source", room.source}, {"updated", room.updated}, {"modelScale", room.modelScale}, {"bases", bases}});
    }
    json configs = json::array();
    for (const auto& [serial, c] : configs_) {
      json jc = ConfigJson(c.first, c.second);
      if (const auto kept = keptGravity_.find(serial); kept != keptGravity_.end()) jc["gravity"] = kept->second;
      const auto frame = frames_.find(serial);
      if (frame != frames_.end()) {
        std::string bits;
        for (uint8_t b : frame->second) bits.push_back(b == 1 ? '1' : b == 0 ? '0' : '.');
        jc["ootxFrame"] = bits;
      }
      configs.push_back(jc);
    }
    const json j = {{"version", 1}, {"rooms", rooms}, {"baseStations", configs}};
    const std::filesystem::path target(path);
    if (target.has_parent_path()) std::filesystem::create_directories(target.parent_path());
    const std::string temp = path + ".tmp";
    {
      std::ofstream out(platform::FsPath(temp));
      out << j.dump(2) << "\n";
      out.close();  // a write error can show only when the buffer goes out
      if (!out) throw std::runtime_error("write failed");
    }
    platform::ReplaceFile(temp, target);
    if (error) error->clear();
    return true;
  } catch (const std::exception& e) {
    if (error) *error = std::string("saving room map ") + path + ": " + e.what();
    return false;
  }
}

bool RoomMap::ImportSteamDatabase(const std::string& text, std::string* error) {
  try {
    const json j = json::parse(text);
    std::vector<Room> imported;
    if (j.contains("known_universes")) {
      for (const auto& u : j.at("known_universes")) {
        // Our own rooms, written there for SteamVR (src/openvr/steam_rooms.hpp): our map has them.
        if (u.value("openhouse", false)) continue;
        Room room;
        room.source = "steamvr";
        room.id = "steamvr-" + (u.at("id").is_string() ? u.at("id").get<std::string>()
                                                         : std::to_string(u.at("id").get<uint64_t>()));
        if (u.contains("lastChaperoneCommit")) {
          const auto& c = u.at("lastChaperoneCommit");
          room.updated = c.is_string() ? std::stod(c.get<std::string>()) : c.get<double>();
        }
        const auto& tilt = u.at("tilt");
        const Eigen::Matrix3d level =
            Leveling(tilt.at("pitch").get<double>(), tilt.at("roll").get<double>(), u.value("tilt_to_pose_method", 0));
        for (const auto& b : u.at("base_stations")) {
          const auto& target = b.at("target_pose");
          if (target.value("target_serial_number", 0u) != 0) continue;  // not rooted
          const auto p = target.at("pose").get<std::array<double, 7>>();  // qx qy qz qw x y z
          Eigen::Isometry3d baseFromGlobal = Eigen::Isometry3d::Identity();
          baseFromGlobal.linear() = Eigen::Quaterniond(p[3], p[0], p[1], p[2]).normalized().toRotationMatrix();
          baseFromGlobal.translation() = Eigen::Vector3d(p[4], p[5], p[6]);
          Eigen::Isometry3d worldFromGlobal = Eigen::Isometry3d::Identity();
          worldFromGlobal.linear() = level;
          RoomBase base;
          base.serial = b.at("base_serial_number").get<uint32_t>();
          base.worldFromBase = worldFromGlobal * baseFromGlobal.inverse();
          const double variance = target.value("variance", 0.0);
          base.positionVariance = std::max(variance, 1e-6);
          base.rotationVariance = std::max(variance, 1e-6);
          base.origin = variance == 0.0 && p[4] == 0 && p[5] == 0 && p[6] == 0;
          base.observations = 1;
          room.bases[base.serial] = base;
        }
        if (!room.bases.empty()) imported.push_back(std::move(room));
      }
    }
    std::map<uint32_t, std::pair<BaseConfig, BaseDynamicState>> configs;
    if (j.contains("base_stations")) {
      for (const auto& b : j.at("base_stations")) {
        const auto& c = b.at("config");
        BaseConfig config;
        BaseDynamicState dynamic;
        config.valid = true;
        config.serial = c.at("serialNumber").get<uint32_t>();
        config.cal = c.at("baseCalibration").get<std::vector<double>>();
        config.hardwareVersion = c.value("modelId", uint8_t(0));
        config.ootxVersion = c.value("ootxVersion", uint8_t(0));
        config.model = ModelForHardware(config.hardwareVersion);
        if (b.contains("dynamic_states") && !b.at("dynamic_states").empty()) {
          const auto& state = b.at("dynamic_states").back().at("dynamic_state");
          if (state.contains("gravity_vector")) dynamic.accelDir = state.at("gravity_vector").get<std::array<float, 3>>();
          dynamic.sysUnlockCount = state.value("reset_count", uint8_t(0));
          dynamic.mode = state.value("basestation_mode", 0);
          if (state.contains("sobChannel")) dynamic.sobChannel = state.at("sobChannel").get<int>();  // 0-based (OOTX agrees)
        }
        if (config.cal.size() == 14) configs[config.serial] = {config, dynamic};
      }
    }
    for (auto& room : imported) {
      bool exists = false;
      for (const auto& r : rooms_) exists |= r.id == room.id;
      if (!exists) {
        Log("Imported room " + room.id + " (" + std::to_string(room.bases.size()) + " bases) from SteamVR");
        rooms_.push_back(std::move(room));
      }
    }
    for (auto& [serial, c] : configs) configs_.emplace(serial, c);
    if (error) error->clear();
    return true;
  } catch (const std::exception& e) {
    if (error) *error = std::string("lighthousedb: ") + e.what();
    return false;
  }
}

// A base (or the origin) power cycled since the room was stored: its pose is kept, with
// the weight of a few seconds of keyframes and at least 1 cm of uncertainty.
bool RoomMap::Locked(const RoomBase& b) {
  // Placed well, not only often: a base locked while still uncertain was never refined
  // again, so it never counted as learned (BaseStationState: 20 mm once locked) and the
  // room never settled (2026-11-08: D7B1AD76 locked at 50 mm, alignment tools waited
  // forever). Until it's within that, the solver keeps refining it.
  constexpr double kLockVariance = 0.02 * 0.02;  // BaseStationState::kLearnedLockedSigma squared
  return !b.origin && !b.relearn && std::max(b.observations, b.storedObservations) >= 100 &&
         b.positionVariance <= kLockVariance;
}

void RoomMap::Suspect(RoomBase& b) {
  if (Locked(b)) return;  // asleep and woken (new reset count) is not moved: its accelerometer tells
  constexpr int kSuspectEvidence = 30;
  b.observations = std::min(b.observations, kSuspectEvidence);
  b.positionVariance = std::max(b.positionVariance, 3 * 0.01 * 0.01);
  b.rotationVariance = std::max(b.rotationVariance, 3 * 0.003 * 0.003);
}

// A base that may have moved a little: Suspect's loosening, locked or not, so the room solver can move it.
// One that didn't move is solved back where it was and locks again with fresh keyframes.
void RoomMap::Loosen(RoomBase& b) {
  constexpr int kLoosenedEvidence = 30;
  b.observations = std::min(b.observations, kLoosenedEvidence);
  b.storedObservations = std::min(b.storedObservations, kLoosenedEvidence);
  b.positionVariance = std::max(b.positionVariance, 3 * 0.01 * 0.01);
  b.rotationVariance = std::max(b.rotationVariance, 3 * 0.003 * 0.003);
  Changed();
}

void RoomMap::OnBaseSeen(const BaseConfig& config, const BaseDynamicState& dynamic) {
  const uint32_t serial = config.serial;
  if (Conflicted(serial)) return;
  const auto previousConfig = configs_.find(serial);
  if (previousConfig == configs_.end() || previousConfig->second.first.cal != config.cal) dirty_ = true;
  const bool firstSighting = !seenResetCount_.count(serial);
  std::optional<Eigen::Vector3d> saved;  // the gravity this base had when the room was saved
  if (firstSighting && previousConfig != configs_.end()) {
    const auto& g = previousConfig->second.second.accelDir;
    if (Eigen::Vector3d(g[0], g[1], g[2]).norm() > 0.5) {
      saved = Eigen::Vector3d(g[0], g[1], g[2]).normalized();
      if (!keptGravity_.count(serial)) keptGravity_[serial] = g;
    }
  }
  configs_[serial] = {config, dynamic};
  const int resetCount = dynamic.sysUnlockCount;
  seenResetCount_[serial] = resetCount;
  CheckAccel(serial, dynamic, saved ? &*saved : nullptr);

  if (!firstSighting) return;
  // Choose the room: keep the active one if it has this base or other visible bases.
  auto visibleIn = [&](const Room& room) {
    int n = 0;
    for (const auto& [s, c] : seenResetCount_) n += room.bases.count(s) != 0;
    return n;
  };
  if (active_ >= 0 && (rooms_[size_t(active_)].bases.count(serial) || visibleIn(rooms_[size_t(active_)]) > 0)) {
    // stay
  } else {
    int best = -1;
    for (size_t i = 0; i < rooms_.size(); ++i) {
      if (!rooms_[i].bases.count(serial)) continue;
      if (best < 0 || visibleIn(rooms_[i]) > visibleIn(rooms_[size_t(best)]) ||
          (visibleIn(rooms_[i]) == visibleIn(rooms_[size_t(best)]) && rooms_[i].updated > rooms_[size_t(best)].updated))
        best = int(i);
    }
    if (best >= 0) {
      active_ = best;
      Log("Using room " + rooms_[size_t(best)].id + " (base " + Hex(serial) + ")");
      ++revision_;
    } else if (active_ < 0) {
      Room room;
      room.source = "lh";
      room.id = "room-" + std::to_string(int64_t(Now()));
      room.updated = Now();
      rooms_.push_back(room);
      active_ = int(rooms_.size()) - 1;
      Log("Created new room " + room.id + " (no stored room contains base " + Hex(serial) + ")");
      Changed();
    }
  }
  if (auto* room = ActiveRoom()) {
    auto found = room->bases.find(serial);
    if (found != room->bases.end() && found->second.origin && !frozen_ && LevelOrigin(*room, found->second, dynamic)) {
      Room fresh;
      fresh.source = "lh";
      fresh.id = "room-" + std::to_string(int64_t(Now()));
      fresh.updated = Now();
      rooms_.push_back(fresh);
      active_ = int(rooms_.size()) - 1;
      solvedTarget_.clear();
      keptGravity_.clear();  // a new room: its bases are learned where they are now
      Log("Created new room " + fresh.id);
      Changed();
    }
  }
  // The reset count (OOTX sys_unlock_count) counts rotor locks since power-up: it
  // changes on every power cycle or wake from standby, moved or not. Nothing is
  // thrown away for it; the stored pose keeps little weight, so the room solver
  // follows a nudge within the session, and a real move (ApplySolved) is retaken.
  if (auto* room = ActiveRoom()) {
    auto found = room->bases.find(serial);
    if (found != room->bases.end() && found->second.resetCount && *found->second.resetCount != resetCount &&
        !found->second.origin) {
      Suspect(found->second);
      found->second.resetCount = resetCount;
      Changed();
    }
  }
}

// Once a session, when the origin is first seen: its own accelerometer (OOTX) against
// the room's level. A stored room (imported from SteamVR, or saved by an earlier
// session) keeps the tilt the origin had then. More than kStaleRad off, the origin was
// turned since (remounted, stood upright on a desk, ...) and the stored layout around
// it is stale too: true, and the caller learns a new room. A new reset count on the
// origin means the others' places relative to it are suspect: they are relearned.
bool RoomMap::LevelOrigin(Room& room, RoomBase& origin, const BaseDynamicState& dynamic) {
  // Below this the base's accelerometer is not better than the stored level: Valve's
  // accelerometers are off by a few degrees (live: 2.6 deg, undone by the devices'
  // own gravity a second later, a 17 cm jump of every device). Gross moves only.
  constexpr double kStaleRad = 8.0 * 0.017453292519943295;
  const int resetCount = dynamic.sysUnlockCount;
  if (origin.resetCount && *origin.resetCount != resetCount) {  // power cycled: see OnBaseSeen
    for (auto& [s, b] : room.bases)
      if (!b.origin) Suspect(b);
    Changed();
  }
  origin.resetCount = resetCount;
  const Eigen::Vector3d up(dynamic.accelDir[0], dynamic.accelDir[1], dynamic.accelDir[2]);
  if (up.norm() < 0.5) return false;  // no accelerometer reading
  const Eigen::Vector3d upWorld = origin.worldFromBase.rotation() * up.normalized();
  const double off = std::acos(std::clamp(upWorld.y(), -1.0, 1.0));
  if (off <= kStaleRad) return false;
  // Live, lhbs: A101B835 upright one day, tilted 50-60 deg on others. Re-levelling
  // kept the stale layout, and every device fought it (the headset reset every 4 s and
  // flew between bases). The stored room is kept, unused.
  char text[200];
  std::snprintf(text, sizeof text, "Room %s is stale: origin %08X turned %.1f deg since it was saved; learning a new room",
                room.id.c_str(), origin.serial, off * 57.29577951308232);
  Log(text);
  return true;
}

void RoomMap::SetOotxFrame(uint32_t serial, const std::vector<uint8_t>& frame) {
  auto& stored = frames_[serial];
  if (stored != frame) { stored = frame; dirty_ = true; }
}

void RoomMap::SetConflictedBases(std::set<uint32_t> bases) {
  if (bases == conflictedBases_) return;
  for (uint32_t serial : bases) solvedTarget_.erase(serial);
  if (bases.contains(Origin())) levelTarget_.reset();
  conflictedBases_ = std::move(bases);
  ++revision_;
  ++bumps_;  // Discard room solves that used observations from before the conflict.
}

const RoomBase* RoomMap::Base(uint32_t serial) const {
  const Room* room = Active();
  if (!room || Conflicted(serial)) return nullptr;
  const auto found = room->bases.find(serial);
  return found == room->bases.end() ? nullptr : &found->second;
}

std::vector<RoomBase> RoomMap::Bases() const {
  std::vector<RoomBase> out;
  if (const Room* room = Active())
    for (const auto& [serial, base] : room->bases)
      if (!Conflicted(serial)) out.push_back(base);
  return out;
}

bool RoomMap::Empty() const {
  const Room* room = Active();
  return !room || room->bases.empty();
}

bool RoomMap::EstablishOrigin(uint32_t serial, const std::array<double, 2>& tilt, double tiltVariance) {
  Room* room = ActiveRoom();
  if (!room || !room->bases.empty() || Conflicted(serial)) return false;
  RoomBase base;
  base.serial = serial;
  keptGravity_.erase(serial);
  base.origin = true;
  base.worldFromBase = Eigen::Isometry3d::Identity();
  base.worldFromBase.linear() = Leveling(tilt[0], tilt[1], 0);
  base.positionVariance = 0;
  base.rotationVariance = tiltVariance;
  base.observations = 1;
  if (seenResetCount_.count(serial)) base.resetCount = seenResetCount_[serial];
  room->bases[serial] = base;
  char text[160];
  std::snprintf(text, sizeof text, "Setting %08X as the origin of room %s (pitch %.2f deg roll %.2f deg)", serial,
                room->id.c_str(), tilt[0] * 57.29577951308232, tilt[1] * 57.29577951308232);
  Log(text);
  Changed();
  return true;
}

void RoomMap::Observe(const std::string& device, uint32_t primary, uint32_t alt,
                      const Eigen::Isometry3d& altFromPrimary, double positionVariance, double rotationVariance,
                      double time) {
  Room* room = ActiveRoom();
  if (!room || primary == alt || frozen_ || Conflicted(primary) || Conflicted(alt)) return;
  const std::string key = device + ":" + Hex(primary) + ":" + Hex(alt);
  auto last = lastObservation_.find(key);
  if (last != lastObservation_.end() && time - last->second < 0.5 && time >= last->second) return;
  lastObservation_[key] = time;

  auto p = room->bases.find(primary);
  auto a = room->bases.find(alt);
  const bool havePrimary = p != room->bases.end(), haveAlt = a != room->bases.end();
  if (!havePrimary && !haveAlt) return;
  auto addBase = [&](uint32_t serial, const Eigen::Isometry3d& pose, double pv, double rv, uint32_t from) {
    RoomBase base;
    base.serial = serial;
    keptGravity_.erase(serial);
    base.worldFromBase = pose;
    base.positionVariance = pv;
    base.rotationVariance = rv;
    base.observations = 1;
    if (seenResetCount_.count(serial)) base.resetCount = seenResetCount_[serial];
    room->bases[serial] = base;
    char text[200];
    std::snprintf(text, sizeof text, "Located base %08X from %08X via %s (%.2fm from origin)", serial, from,
                  device.c_str(), pose.translation().norm());
    Log(text);
    Changed();
  };
  if (havePrimary && !haveAlt) {
    addBase(alt, p->second.worldFromBase * altFromPrimary.inverse(), p->second.positionVariance + positionVariance,
            p->second.rotationVariance + rotationVariance, primary);
    return;
  }
  if (!havePrimary && haveAlt) {
    addBase(primary, a->second.worldFromBase * altFromPrimary, a->second.positionVariance + positionVariance,
            a->second.rotationVariance + rotationVariance, alt);
    return;
  }
  // Both located: refine the one that is not the origin and is less certain.
  RoomBase* target;
  Eigen::Isometry3d measured;
  double mpv, mrv;
  const bool updateAlt = !a->second.origin && (p->second.origin || a->second.positionVariance >= p->second.positionVariance);
  if (updateAlt) {
    target = &a->second;
    measured = p->second.worldFromBase * altFromPrimary.inverse();
    mpv = p->second.positionVariance + positionVariance;
    mrv = p->second.rotationVariance + rotationVariance;
  } else {
    if (p->second.origin) return;
    target = &p->second;
    measured = a->second.worldFromBase * altFromPrimary;
    mpv = a->second.positionVariance + positionVariance;
    mrv = a->second.rotationVariance + rotationVariance;
  }
  if (Locked(*target)) return;
  const double dp = (measured.translation() - target->worldFromBase.translation()).norm();
  const double dr = Angle(measured.rotation(), target->worldFromBase.rotation());
  const bool uncertain = target->positionVariance > 0.01;
  if (!uncertain && (dp > 0.10 || dr > 0.0523599)) {
    if (++target->inconsistent >= 10) {
      char text[200];
      std::snprintf(text, sizeof text, "Base %08X disagrees with the room map by %.0fmm / %.1f deg; relearning it (solver re-solves it)",
                    target->serial, dp * 1000, dr * 57.29577951308232);
      Log(text);
      // One device's filter (a drifting or occluded one: live, a headset 4-5 deg off)
      // must not rewrite the room on its own: the base stays, uncertain, and the
      // solver places it again from every device's fresh keyframes, gliding there.
      target->positionVariance = std::max(target->positionVariance, 0.02);
      target->rotationVariance = std::max(target->rotationVariance, 0.0523599 * 0.0523599);
      target->inconsistent = 0;
      target->farSolves = 0;
      // Still the solver's: with solved cleared, the next observation from this same
      // device blended the base half way to its view.
      relearned_.push_back(target->serial);
      Changed();
    }
    return;
  }
  target->inconsistent = 0;
  if (target->solved) return;  // the room solver owns this pose
  const double kp = std::clamp(target->positionVariance / (target->positionVariance + mpv), 0.002, 0.5);
  const double kr = std::clamp(target->rotationVariance / (target->rotationVariance + mrv), 0.002, 0.5);
  const Eigen::Quaterniond q0(target->worldFromBase.rotation()), q1(measured.rotation());
  target->worldFromBase.translation() += kp * (measured.translation() - target->worldFromBase.translation());
  target->worldFromBase.linear() = q0.slerp(kr, q1).normalized().toRotationMatrix();
  target->positionVariance = std::max(target->positionVariance * mpv / (target->positionVariance + mpv), 1e-7);
  target->rotationVariance = std::max(target->rotationVariance * mrv / (target->rotationVariance + mrv), 1e-7);
  ++target->observations;
  if (seenResetCount_.count(target->serial)) target->resetCount = seenResetCount_[target->serial];
  Changed();
}

uint32_t RoomMap::Origin() const {
  if (const Room* room = Active())
    for (const auto& [serial, b] : room->bases)
      if (b.origin) return serial;
  return 0;
}

namespace {
constexpr double kMoveFloor = 0.05;                     // m: closer than this is never a move
constexpr double kMoveRotFloor = 0.5 * 0.017453292519943295;
constexpr double kMoveSigmas = 5.0;
constexpr double kRelearnVariance = 3 * 0.02 * 0.02;    // a solve this confident re-places a bumped base
constexpr double kBumpRad = 1.2 * 0.017453292519943295; // accelerometer turn: 2-3 counts of its 1/127
// Against the gravity saved by an earlier session: the reference followed the base's thermal drift
// while it ran warm, and a base just woken from standby reads cold. A 1.3 deg difference on the first
// frames re-learned a base at every first start (back after a SteamVR restart); a base moved while
// nothing ran turns far more.
constexpr double kBumpSavedRad = 2.5 * 0.017453292519943295;
}  // namespace

bool RoomMap::FarOff(uint32_t serial, const Eigen::Isometry3d& pose, double pv, double rv) const {
  const RoomBase* b = Base(serial);
  if (!b || b->origin) return false;
  const auto target = solvedTarget_.find(serial);
  const Eigen::Isometry3d& now = target != solvedTarget_.end() ? target->second : b->worldFromBase;
  const double dp = (now.translation() - pose.translation()).norm();
  const double dr = Eigen::AngleAxisd(now.rotation().transpose() * pose.rotation()).angle();
  const double sigma = std::sqrt((b->positionVariance + pv) / 3), rsigma = std::sqrt((b->rotationVariance + rv) / 3);
  return dp > std::max(kMoveFloor, kMoveSigmas * sigma) || dr > std::max(kMoveRotFloor, kMoveSigmas * rsigma);
}

// Every decoded OOTX frame carries the base's gravity direction (lhbs refreshes it in
// the frame; stock only at power-up). Turned by more than kBumpRad, twice running (two
// frames, or two devices decoding one): knocked.
void RoomMap::CheckAccel(uint32_t serial, const BaseDynamicState& dynamic, const Eigen::Vector3d* saved) {
  const Eigen::Vector3d up(dynamic.accelDir[0], dynamic.accelDir[1], dynamic.accelDir[2]);
  if (dynamic.cached || up.norm() < 0.5) return;  // a recalled frame: an earlier session's gravity
  AccelTrack& a = accel_[serial];
  const Eigen::Vector3d u = up.normalized();
  auto deg = [](double rad) { return rad * 57.29577951308232; };
  if (!a.has) {
    // Against the saved gravity first: a base moved while nothing ran is bumped on its
    // first live frames (twice running, as any bump).
    a.ref = saved ? *saved : u;
    a.has = true;
    a.fromSaved = saved != nullptr;
    if (!saved) return;
  }
  const double angle = std::acos(std::clamp(a.ref.dot(u), -1.0, 1.0));
  if (angle <= (a.fromSaved ? kBumpSavedRad : kBumpRad)) {
    if (a.fromSaved) {   // this session's own reference from here on
      // Between kBumpRad and kBumpSavedRad a base woken cold and one moved a little look the same (a 2.0
      // base moved 35 mm turned 1.4 deg, and nothing else caught it). Not relearned for it: loosened, so
      // the room solver follows it if it moved, and its saved level is kept for the next session.
      if (angle > kBumpRad) {
        char text[200];
        std::snprintf(text, sizeof text, "Base %08X: level %.1f deg off the saved one (woken cold, or moved a little): loosened",
                      serial, deg(angle));
        Log(text);
        if (Room* room = ActiveRoom()) {
          auto found = room->bases.find(serial);
          if (found != room->bases.end() && !found->second.origin) Loosen(found->second);
        }
      } else {
        keptGravity_.erase(serial);   // where it was saved: this session's reading goes with the room
      }
      a.ref = u;
      a.fromSaved = false;
    }
    a.strikes = 0;
    a.ref = (0.9 * a.ref + 0.1 * u).normalized();  // follows a slow (thermal) drift
    return;
  }
  if (++a.strikes < 2) return;
  a.ref = u;
  a.fromSaved = false;
  a.strikes = 0;
  char text[160];
  std::snprintf(text, sizeof text, "its accelerometer turned %.1f deg", deg(angle));
  Bump(serial, text);
}

void RoomMap::Refine(uint32_t serial, const std::string& why) {
  Room* room = ActiveRoom();
  if (!room || frozen_) return;
  auto found = room->bases.find(serial);
  if (found == room->bases.end() || found->second.origin || found->second.relearn) return;
  Loosen(found->second);
  char text[200];
  std::snprintf(text, sizeof text, "Base %08X not placed well yet (%s): refining it", serial, why.c_str());
  Log(text);
}

void RoomMap::Bump(uint32_t serial, const std::string& why) {
  Room* room = ActiveRoom();
  if (!room || frozen_) return;
  auto found = room->bases.find(serial);
  if (found == room->bases.end()) return;
  ++bumps_;  // a solve already running was made from keyframes before this
  RoomBase& b = found->second;
  if (b.origin) {
    if (!OriginGauge()) { Log("Origin base " + Hex(serial) + " knocked (" + why + "), but no other placed base to relearn it against"); return; }
    if (originRelearn_) return;
    originRelearn_ = true;
    char text[200];
    std::snprintf(text, sizeof text, "Origin base %08X knocked (%s): relearning its place against base %08X", serial,
                  why.c_str(), OriginGauge());
    Log(text);
  } else {
    if (b.relearn) return;
    b.relearn = true;
    b.positionVariance = std::max(b.positionVariance, 3 * 0.05 * 0.05);
    b.rotationVariance = std::max(b.rotationVariance, 3 * 0.03 * 0.03);
    b.observations = 0;
    b.inconsistent = 0;
    b.farSolves = 0;
    solvedTarget_.erase(serial);
    char text[200];
    std::snprintf(text, sizeof text, "Base %08X knocked (%s): relearning its pose", serial, why.c_str());
    Log(text);
  }
  relearned_.push_back(serial);  // keyframes from before the knock are stale
  Changed();
}

uint32_t RoomMap::OriginGauge() const {
  const Room* room = Active();
  if (!room) return 0;
  uint32_t best = 0;
  double bestVariance = 1e300;
  for (const auto& [serial, b] : room->bases)
    if (!b.origin && b.solved && !b.relearn && b.positionVariance < bestVariance) { best = serial; bestVariance = b.positionVariance; }
  return best;
}

void RoomMap::ApplyOriginSolved(const Eigen::Isometry3d& pose, double pv, int keyframes) {
  Room* room = ActiveRoom();
  if (!room || !originRelearn_ || !pose.matrix().allFinite() || pv >= kRelearnVariance) return;
  for (auto& [serial, b] : room->bases) {
    if (!b.origin) continue;
    const double dp = (b.worldFromBase.translation() - pose.translation()).norm();
    const double dr = Eigen::AngleAxisd(b.worldFromBase.rotation().transpose() * pose.rotation()).angle();
    char text[200];
    std::snprintf(text, sizeof text, "Room solver: origin base %08X re-placed after a bump: %.1f mm, %.2f deg (%d keyframes, sigma %.1f mm)",
                  serial, dp * 1000, dr * 57.29577951308232, keyframes, std::sqrt(pv / 3) * 1000);
    Log(text);
    b.worldFromBase = pose;
    keptGravity_.erase(serial);
    originRelearn_ = false;
    ++originEpoch_;
    ++b.epoch;
    Changed();
    return;
  }
}

void RoomMap::ApplySolved(uint32_t serial, const Eigen::Isometry3d& pose, double pv, double rv, int keyframes,
                          double rms, int fresh, int devices, double span) {
  constexpr int kMaxEvidence = 100000;
  // 1 mm per axis: what model errors leave, and what keeps a settled room following a
  // base that really shifted a little (a 1 cm shift is half taken in a few minutes).
  constexpr double kFloorVariance = 3 * 0.001 * 0.001;
  Room* room = ActiveRoom();
  if (!room || !pose.matrix().allFinite() || frozen_ || keyframes <= 0 || Conflicted(serial)) return;
  auto found = room->bases.find(serial);
  if (found != room->bases.end() && found->second.origin) return;
  if (fresh < 0) fresh = keyframes;
  char text[220];
  bool place = false;
  // A base is first placed from at least this many keyframes. From one (taken mid-swing
  // in a new room) it was 1.3 m off, devices took it from the map and flew off, and it
  // was moved 1.4 m and 25 deg a few seconds later. Learning a room from scratch,
  // fastmotionindex's jumps > 5 cm 92 -> 28, spin-01's 258 -> 49 (12; 5-10 also helped);
  // vrchat-01 and ms4-01 unchanged.
  if (found == room->bases.end() && keyframes < kMinLocateKeyframes) return;
  if (found == room->bases.end()) {
    RoomBase base;
    base.serial = serial;
    if (seenResetCount_.count(serial)) base.resetCount = seenResetCount_[serial];
    found = room->bases.emplace(serial, base).first;
    std::snprintf(text, sizeof text, "Located base %08X by the room solver (%.2fm from origin, %d keyframes, rms %.0f urad)",
                  serial, pose.translation().norm(), keyframes, rms * 1e6);
    Log(text);
    place = true;
  }
  auto& b = found->second;
  const Eigen::Isometry3d now = solvedTarget_.count(serial) ? solvedTarget_[serial] : b.worldFromBase;
  const double dp = (now.translation() - pose.translation()).norm();
  const double dr = Eigen::AngleAxisd(now.rotation().transpose() * pose.rotation()).angle();
  // Locked (placed well: 100+ keyframes, under 20 mm) used to be frozen: one locked at 19 mm
  // stayed 2 cm off for good. It keeps its lock (no device can relearn it, a move is the
  // session's to find: Session::LightMovedLockedBase), and still takes the refinement below
  // from solves that agree with it, at half the weight and only from more than one device's
  // view (or plenty of keyframes): a well-placed one barely moves, a loosely locked one
  // settles toward where the room's light puts it, over this session and the next.
  const bool locked = Locked(b);
  if (locked && (fresh == 0 || (devices < 2 && keyframes < 80) || FarOff(serial, pose, pv, rv))) return;
  if (!locked) {
  if (place) {
    b.worldFromBase = pose;
    solvedTarget_.erase(serial);
    b.positionVariance = std::max(pv, kFloorVariance);
    b.rotationVariance = std::max(rv, 1e-9);
    b.observations = keyframes;
    b.solved = true;
    b.farSolves = 0;
    Changed();
    return;
  }
  if (!b.solved) {
    // An imported or filter-observed pose the solver hasn't confirmed: 20 cm uncertain
    // (SteamVR's own variance claims far less; its room was 7-19 cm off), so the first
    // solves take over (at 1-5 cm, vrchat-01's imported 7 cm error lingered for
    // minutes, devices rejecting a fifth of that base's light).
    b.positionVariance = std::max(b.positionVariance, 3 * 0.2 * 0.2);
    b.rotationVariance = std::max(b.rotationVariance, 3 * 0.05 * 0.05);
    b.observations = std::min(b.observations, 30);
    b.solved = true;
    Changed();
  }
  // Bumped: the first confident solve from fresh keyframes is the base's new place.
  // From enough of the room to stand on: live, re-places from 1-8 keyframes locked
  // again at once and the next knock followed.
  if (b.relearn && pv < kRelearnVariance && keyframes >= 40 && (devices >= 2 || keyframes >= 80)) {
    std::snprintf(text, sizeof text, "Room solver: base %08X re-placed after a bump: %.1f mm, %.2f deg (%d keyframes, sigma %.1f mm)",
                  serial, dp * 1000, dr * 57.29577951308232, keyframes, std::sqrt(pv / 3) * 1000);
    Log(text);
    b.worldFromBase = pose;
    solvedTarget_.erase(serial);
    b.positionVariance = std::max(pv, kFloorVariance);
    b.rotationVariance = std::max(rv, 1e-9);
    b.observations = keyframes;
    b.storedObservations = 0;  // earlier sessions' evidence is for the old place: lock again on new solves only
    b.farSolves = 0;
    b.relearn = false;
    keptGravity_.erase(serial);  // learned where it is now: its gravity now goes with it
    ++b.epoch;
    Changed();
    return;
  }
  // Moved? Far outside what both the room and this solve allow, in place or in turn
  // (a knocked base mostly turns on its mount: 1 deg is 9 cm at 5 m).
  if (FarOff(serial, pose, pv, rv)) {
    if (++b.farSolves < 3) return;
    // A knock (20 cm, 3 deg) is taken at once: light against the old place is wrong.
    // Less (an imported room's error, a slow drift) glides there, as every other solve:
    // a step moves every device on this base at once (live: a 10 cm head shift).
    const bool knock = dp > 0.20 || dr > 0.0523599;
    std::snprintf(text, sizeof text, "Room solver: base %08X moved by %.1f mm, %.2f deg (%d keyframes, rms %.0f urad, sigma %.1f mm)%s",
                  serial, dp * 1000, dr * 57.29577951308232, keyframes, rms * 1e6, std::sqrt(pv / 3) * 1000,
                  knock ? "" : "; gliding there");
    Log(text);
    keptGravity_.erase(serial);
    if (knock) {
      b.worldFromBase = pose;
      solvedTarget_.erase(serial);
    } else {
      solvedTarget_[serial] = pose;
    }
    b.positionVariance = std::max(pv, kFloorVariance);
    b.rotationVariance = std::max(rv, 1e-9);
    b.observations = keyframes;
    b.storedObservations = 0;  // earlier sessions' evidence is for the old place, as after a bump
    b.farSolves = 0;
    ++b.epoch;
    relearned_.push_back(serial);  // keyframes from before the move are stale
    Changed();
    return;
  }
  b.farSolves = 0;
  }  // !locked
  if (fresh <= 0) return;
  // Information-weighted: the fresh share of this window is new evidence, worth
  // fresh / keyframes of the window's information; the room's own is its variance's.
  // An early solve from a few keyframes (sigma 2 cm) moves a settled room hardly at all.
  const double roomInfo = 1.0 / std::max(b.positionVariance, kFloorVariance);
  const double newInfo = (locked ? 0.5 : 1.0) * double(std::min(fresh, keyframes)) / double(keyframes) / std::max(pv, 1e-12);
  const double gain = newInfo / (roomInfo + newInfo);
  Eigen::Isometry3d target = now;
  target.translation() += gain * (pose.translation() - now.translation());
  const Eigen::AngleAxisd turn(pose.rotation() * now.rotation().transpose());
  target.linear() = Eigen::AngleAxisd(gain * turn.angle(), turn.axis()).toRotationMatrix() * now.rotation();
  b.observations = std::min(b.observations + fresh, kMaxEvidence);
  b.positionVariance = std::max(1.0 / (roomInfo + newInfo), kFloorVariance);
  b.rotationVariance = std::max(b.rotationVariance * (1 - gain), 1e-9);
  if (gain * dp < 0.0002 && gain * dr < 0.0001) {  // too small to glide: kept for the next one
    Changed();
    return;
  }
  // Refinements glide in (Advance) so devices tracking from this base never see a step.
  solvedTarget_[serial] = target;
  Changed();
  if (gain * dp > 0.01) {
    std::snprintf(text, sizeof text, "Room solver: base %08X refined by %.1f mm (%d keyframes, %d new, sigma %.1f mm)",
                  serial, gain * dp * 1000, keyframes, fresh, std::sqrt(b.positionVariance / 3) * 1000);
    Log(text);
  }
}

// The origin's tilt from a device's filter (its IMU's gravity against the base). The
// first estimate of a session levels the room at once (during acquisition). Later,
// better estimates only move the target: the room turns toward it at kLevelSlew, so a
// correction is never a step. A 1 deg step of the whole room moves a hand 5 m from
// the origin by 9 cm in one frame; the live session did that mid-game.
void RoomMap::Advance(double time) {
  constexpr double kMetersPerSecond = 0.02, kRadiansPerSecond = 0.5 * 0.017453292519943295;
  const double dt = std::isfinite(time - advanceTime_) ? std::clamp(time - advanceTime_, 0.0, 0.1) : 0.0;  // first call: NaN
  advanceTime_ = time;
  Room* room = ActiveRoom();
  if (!room || dt <= 0) return;
  for (auto it = solvedTarget_.begin(); it != solvedTarget_.end();) {
    auto found = room->bases.find(it->first);
    if (found == room->bases.end() || found->second.origin) { it = solvedTarget_.erase(it); continue; }
    Eigen::Isometry3d& pose = found->second.worldFromBase;
    const Eigen::Vector3d dp = it->second.translation() - pose.translation();
    const Eigen::AngleAxisd dr(it->second.rotation() * pose.rotation().transpose());
    const double kp = dp.norm() > 0 ? std::min(1.0, kMetersPerSecond * dt / dp.norm()) : 1.0;
    const double kr = dr.angle() > 0 ? std::min(1.0, kRadiansPerSecond * dt / dr.angle()) : 1.0;
    pose.translation() += kp * dp;
    pose.linear() = Eigen::AngleAxisd(kr * dr.angle(), dr.axis()).toRotationMatrix() * pose.rotation();
    Changed();
    if (kp >= 1 && kr >= 1) it = solvedTarget_.erase(it); else ++it;
  }
}

bool RoomMap::RefineOriginTilt(uint32_t serial, const std::array<double, 2>& tilt, double tiltVariance, double time) {
  constexpr double kLevelSlew = 0.05 * 0.017453292519943295, kFirstLevelSlew = 2.0 * 0.017453292519943295;  // rad/s
  Room* room = ActiveRoom();
  if (!room || frozen_ || Conflicted(serial)) return false;
  auto found = room->bases.find(serial);
  if (found == room->bases.end() || !found->second.origin) return false;
  RoomBase& origin = found->second;
  if (tiltVariance < 0.5 * origin.rotationVariance) {
    levelTarget_ = Leveling(tilt[0], tilt[1], 0);
    origin.rotationVariance = tiltVariance;
    const double angle = Eigen::AngleAxisd(*levelTarget_ * origin.worldFromBase.rotation().transpose()).angle();
    if (angle > 0.0017) {
      char text[200];
      std::snprintf(text, sizeof text, "Re-levelling room %s from %08X tilt by %.2f deg%s", room->id.c_str(), serial,
                    angle * 57.29577951308232, levelled_ ? " (gradually)" : "");
      Log(text);
    }
  }
  if (!levelTarget_) return false;
  const Eigen::Matrix3d full = *levelTarget_ * origin.worldFromBase.rotation().transpose();
  const Eigen::AngleAxisd aa(full);
  const double dt = std::isfinite(time - levelTime_) ? std::clamp(time - levelTime_, 0.0, 0.1) : 0.0;  // first call: NaN
  levelTime_ = time;
  if (aa.angle() < 1e-7) { levelled_ = true; return false; }
  // The first level of a session goes quickly (it is usually seconds into
  // acquisition), but still never as one step.
  const double step = std::min(aa.angle(), (levelled_ ? kLevelSlew : kFirstLevelSlew) * dt);
  if (step <= 0) return false;
  const Eigen::Matrix3d delta = Eigen::AngleAxisd(step, aa.axis()).toRotationMatrix();
  for (auto& [s, b] : room->bases) {
    b.worldFromBase.linear() = delta * b.worldFromBase.rotation();
    b.worldFromBase.translation() = delta * b.worldFromBase.translation();
  }
  // Glide targets turn with the room: left behind, the next solve (in the turned room)
  // differed from them by the turn (8 cm at 5 m for 2.5 deg) and was taken for a move.
  for (auto& [s, target] : solvedTarget_) {
    target.linear() = delta * target.rotation();
    target.translation() = delta * target.translation();
  }
  if (step == aa.angle()) levelled_ = true;
  Changed();
  return true;
}

void RoomMap::ResetActive() {
  if (Room* room = ActiveRoom()) {
    room->bases.clear();
    Log("Room " + room->id + " reset; relearning base stations");
    Changed();
  }
}

void RoomMap::ForgetAll() {
  rooms_.clear();
  active_ = -1;
  seenResetCount_.clear();
  lastObservation_.clear();
  relearned_.clear();
  levelTarget_.reset();
  levelTime_ = advanceTime_ = std::numeric_limits<double>::quiet_NaN();
  solvedTarget_.clear();
  levelled_ = false;
  accel_.clear();
  keptGravity_.clear();
  originRelearn_ = false;
  ++originEpoch_;
  Log("Room map forgotten; learning a new room");
  ++revision_;
  dirty_ = true;
}

std::vector<std::string> RoomMap::TakeLogs() {
  std::vector<std::string> out;
  out.swap(logs_);
  return out;
}

}  // namespace lh::runtime
