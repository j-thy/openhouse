// Publishes the driver's room (base poses in the host's raw frame, settled, epoch) for
// tools that align the lighthouse frame to something else; see include/lh/room_shm.h.
#pragma once
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

#include "lh/room_shm.h"
#include <set>

#include "lh/session.hpp"

namespace lh::openvr {

// When the room has stopped moving, and when a settled room is a different one.
class RoomSettle {
 public:
  static constexpr double kSettleTime = 3.0;     // s without a base moving
  static constexpr double kStill = 0.005;        // m a base may wander meanwhile (a solver glide is slower)
  static constexpr double kStillRad = 0.15 * 0.017453292519943295;
  static constexpr double kNewEpoch = 0.01;      // m: a settled room this different is new
  static constexpr double kNewEpochRad = 0.3 * 0.017453292519943295;
  static constexpr double kMaxSigma = 0.005;     // m
  // A locked base (placed from enough evidence; only its own accelerometer moves it) is
  // as placed as it gets: it counts up to this.
  static constexpr double kLockedMaxSigma = BaseStationState::kLearnedLockedSigma;  // m

  struct Base {
    std::string serial; bool visible, placed; Eigen::Isometry3d rawFromBase; double sigma; bool locked = false;
    bool learned = false;  // BaseStationState::learned
    bool Good() const { return placed && learned; }
  };
  // Returns whether the room is settled now; epoch() bumps on a new settled room.
  bool Update(const std::vector<Base>& bases, double now);
  // The room was reset: unsettled, and the next settled room is a new epoch.
  void Restart() { window_.clear(); settledPoses_.clear(); settled_ = false; since_ = -1; ++epoch_; }
  uint64_t epoch() const { return epoch_; }

 private:
  static bool Close(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double m, double rad);
  std::map<std::string, Eigen::Isometry3d> window_;  // poses when the still window began
  double since_ = -1;
  std::map<std::string, Eigen::Isometry3d> settledPoses_;
  bool settled_ = false;
  uint64_t epoch_ = 0;
};

class RoomPublisher {
 public:
  RoomPublisher() = default;
  RoomPublisher(const RoomPublisher&) = delete;
  RoomPublisher& operator=(const RoomPublisher&) = delete;
  ~RoomPublisher() { Close(); }
  void Close();  // readers fall back to their usual route
  void Publish(const SessionSnapshot& snapshot, const Eigen::Isometry3d& rawFromWorld, double now,
               unsigned long long roomResets = 0);

  // A line for the driver log when the room's settled state or set of placed bases
  // changed (empty otherwise).
  std::string TakeLog() { std::string out; out.swap(pendingLog_); return out; }
  bool Settled() const { return settled_; }
  unsigned long long Epoch() const { return settle_.epoch(); }

 private:
  bool Open();
  RoomSettle settle_;
  std::string lastLogKey_, pendingLog_;
  unsigned long long roomResets_ = 0;
  bool settled_ = false;
  openhouse_room* shm_ = nullptr;
  void* map_ = nullptr;  // Windows: the file mapping's handle
  bool failed_ = false;
  std::string path_;
};

}  // namespace lh::openvr
