// Room map shared by every tracked device in a session.
//
// The stock driver lets each device bootstrap and track base stations on its own,
// and throws that work away whenever the device resets. Here the base station
// poses live in one map that all devices read and refine:
//   * a device that (re)acquires on any located base gets a world pose at once;
//   * located bases are handed to a device's filter as known alternate bases;
//   * converged alternate-base estimates from every device refine the map;
//   * the map survives device resets and is persisted between sessions.
//
// World frame: Y up (gravity), origin at the origin base. Several rooms can be
// stored; the active one is chosen by the bases that are visible.
#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <map>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "lh/optical.hpp"

namespace lh::runtime {

struct RoomBase {
  uint32_t serial = 0;
  Eigen::Isometry3d worldFromBase = Eigen::Isometry3d::Identity();
  double positionVariance = 0;  // m^2, trace
  double rotationVariance = 0;  // rad^2, trace
  int observations = 0;           // keyframes of evidence behind the pose (room solver)
  int storedObservations = 0;     // as loaded: earlier sessions' evidence, kept through Suspect
  bool origin = false;
  std::optional<int> resetCount;  // OOTX sys_unlock_count when located
  int inconsistent = 0;           // consecutive observations disagreeing with the map (Observe)
  int farSolves = 0;              // consecutive room solves far off the map (ApplySolved)
  bool solved = false;            // pose owned by the room solver (no filter blending)
  bool relearn = false;           // bumped this session: the next confident solve is taken whole
  uint32_t epoch = 0;             // bumps each time the base is re-placed (moved, knocked): devices resync
};

struct Room {
  std::string id;
  std::string source;             // "lh" or "steamvr"
  std::map<uint32_t, RoomBase> bases;
  double updated = 0;             // unix seconds, for choosing between rooms
  // Device model scales (lighthouse_scale.json, as the stock driver applies them) the room was
  // learned with: 0 = before openhouse applied them. kModelScale bumps when they change.
  static constexpr int kModelScale = 1;
  int modelScale = kModelScale;  // new rooms and SteamVR's (the stock driver applies them)
};

// Tilt in the stock representation: base-frame "up" = exp([pitch, 0, roll]) * +Y.
std::array<double, 2> TiltFromUp(const Eigen::Vector3d& upInBase);
Eigen::Vector3d UpFromTilt(double pitch, double roll);
// World-from-global levelling rotation for a universe tilt (0x353ab0).
Eigen::Matrix3d Leveling(double pitch, double roll, int method);

class RoomMap {
 public:
  static constexpr int kMinLocateKeyframes = 12;  // keyframes before the solver first places a base
  // Persistence (own format). Missing file = empty map, not an error.
  bool Load(const std::string& path, std::string* error = nullptr);
  bool Save(const std::string& path, std::string* error = nullptr) const;
  // Adds rooms from a SteamVR lighthousedb.json (rooted base poses only) and the
  // base calibrations it contains. Existing rooms with the same id are kept.
  bool ImportSteamDatabase(const std::string& json, std::string* error = nullptr);

  // A base became visible (OOTX). May switch/create the active room.
  void OnBaseSeen(const BaseConfig& config, const BaseDynamicState& dynamic);
  // Base calibrations remembered from earlier sessions / SteamVR.
  const std::map<uint32_t, std::pair<BaseConfig, BaseDynamicState>>& KnownConfigs() const { return configs_; }
  // OOTX frames (data bits) per base, for fast identification.
  void SetOotxFrame(uint32_t serial, const std::vector<uint8_t>& frame);
  const std::map<uint32_t, std::vector<uint8_t>>& OotxFrames() const { return frames_; }

  // Keep stored poses while excluding stations whose optical channel is ambiguous.
  void SetConflictedBases(std::set<uint32_t> bases);
  bool Conflicted(uint32_t serial) const { return conflictedBases_.contains(serial); }
  const RoomBase* Base(uint32_t serial) const;
  std::vector<RoomBase> Bases() const;
  bool Empty() const;
  // Locked: placed well (100+ keyframes, this session or saved), and not bumped. Only a
  // bump unlocks it: its accelerometer, or room solves that keep placing it elsewhere
  // (Session::LightMovedLockedBase). A single device's filter never moves it.
  static bool Locked(const RoomBase& base);
  const Room* Active() const { return active_ < 0 ? nullptr : &rooms_[size_t(active_)]; }
  uint64_t Revision() const { return revision_; }
  bool Dirty() const { return dirty_; }
  void ClearDirty() { dirty_ = false; }

  // Starts an empty active room at `serial`, levelled by the base tilt.
  bool EstablishOrigin(uint32_t serial, const std::array<double, 2>& tilt, double tiltVariance);
  // A device measured altFromPrimary (alt base pose relative to primary).
  void Observe(const std::string& device, uint32_t primary, uint32_t alt, const Eigen::Isometry3d& altFromPrimary,
               double positionVariance, double rotationVariance, double time);
  // Room solver result for a base (never the origin). The solver sees a sliding window
  // of keyframes, so its answers wander by about their own spread; the room keeps a
  // running estimate instead: a solve adds the information of its `fresh` keyframes
  // (new since the last solve; < 0: all of them) and moves the base by that share. A solve far
  // outside both uncertainties three times running is a moved base: taken whole.
  void ApplySolved(uint32_t serial, const Eigen::Isometry3d& worldFromBase, double positionVariance,
                   double rotationVariance, int keyframes, double rms, int fresh = -1, int devices = 2,
                   double span = 1e9);
  // A better tilt for the origin base (e.g. after a device calibrated on it):
  // re-levels the whole room once the variance is clearly lower.
  // Moves bases toward their latest room-solver poses, rate-limited (session time).
  void Advance(double time);
  bool RefineOriginTilt(uint32_t serial, const std::array<double, 2>& tilt, double tiltVariance, double time = 0);
  uint32_t Origin() const;
  // A base was knocked (its accelerometer turned, or the solves say so): its pose is
  // relearned from fresh keyframes. The origin is relearned against another placed
  // base (OriginGauge), so the room, and what devices report in it, stay put.
  void Bump(uint32_t serial, const std::string& why);
  // A placed base that is still being learned and that the light disagrees with: kept placed
  // with its keyframes, freed for the solver to refine (unlocked), not relearned from scratch.
  void Refine(uint32_t serial, const std::string& why);
  bool OriginRelearning() const { return originRelearn_; }
  uint64_t Bumps() const { return bumps_; }  // Movement and channel conflicts invalidate in-flight solves.
  uint32_t OriginGauge() const;  // the base held fixed meanwhile; 0 if none
  void ApplyOriginSolved(const Eigen::Isometry3d& worldFromOrigin, double positionVariance, int keyframes);
  uint64_t OriginEpoch() const { return originEpoch_; }  // bumps when the origin was re-placed
  // A solve for a base far outside what the room and the solve allow (a moved base).
  bool FarOff(uint32_t serial, const Eigen::Isometry3d& worldFromBase, double positionVariance,
              double rotationVariance) const;
  // Bases whose pose was thrown away (moved); keyframes using them are stale.
  std::vector<uint32_t> TakeRelearned() { std::vector<uint32_t> out; out.swap(relearned_); return out; }
  // Drops all base poses of the active room.
  void ResetActive();
  // Drops every room and all session learning (as a fresh start without a file); base
  // calibrations and OOTX frames are kept. The next OnBaseSeen starts a new room.
  void ForgetAll();

  std::vector<std::string> TakeLogs();
  // Diagnostics: a frozen room is never learned from or refined.
  void SetFrozen(bool frozen) { frozen_ = frozen; }

 private:
  Room* ActiveRoom() { return active_ < 0 ? nullptr : &rooms_[size_t(active_)]; }
  void Log(std::string text) { logs_.push_back(std::move(text)); }
  // The origin seen for the first time this session: the room re-levelled from its
  // accelerometer if the stored level disagrees, the others relearned if it was reset
  bool LevelOrigin(Room& room, RoomBase& origin, const BaseDynamicState& dynamic);  // true: room stale
  static void Suspect(RoomBase& base);
  void Loosen(RoomBase& base);
  void CheckAccel(uint32_t serial, const BaseDynamicState& dynamic, const Eigen::Vector3d* saved = nullptr);
  void Changed() { ++revision_; dirty_ = true; if (auto* r = ActiveRoom()) r->updated = Now(); }
  static double Now();

  std::vector<Room> rooms_;
  int active_ = -1;
  std::map<uint32_t, std::pair<BaseConfig, BaseDynamicState>> configs_;
  std::map<uint32_t, std::vector<uint8_t>> frames_;
  std::map<uint32_t, int> seenResetCount_;  // this session
  std::set<uint32_t> conflictedBases_;
  std::map<std::string, double> lastObservation_;
  uint64_t revision_ = 0;
  bool dirty_ = false;
  std::vector<std::string> logs_;
  std::vector<uint32_t> relearned_;
  bool frozen_ = false;
  std::optional<Eigen::Matrix3d> levelTarget_;  // where RefineOriginTilt is turning the room
  double levelTime_ = std::numeric_limits<double>::quiet_NaN(), advanceTime_ = std::numeric_limits<double>::quiet_NaN();
  std::map<uint32_t, Eigen::Isometry3d> solvedTarget_;
  bool levelled_ = false;  // the session's first level is applied at once
  struct AccelTrack {
    Eigen::Vector3d ref = Eigen::Vector3d::Zero();
    bool has = false;
    bool fromSaved = false;  // ref is still an earlier session's gravity (looser threshold)
    int strikes = 0;
  };
  std::map<uint32_t, AccelTrack> accel_;  // this session
  // The gravity a base had when its pose was learned, kept in the saved room until this session re-places
  // the base or reads it close to that level. Saving a moved base's new reading with its old pose would
  // hide the move from every later session.
  std::map<uint32_t, std::array<float, 3>> keptGravity_;
  bool originRelearn_ = false;
  uint64_t bumps_ = 0;
  uint64_t originEpoch_ = 0;
};

}  // namespace lh::runtime
