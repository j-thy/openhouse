// Head-mounted displays through the original driver ("hybrid"): our driver tracks every
// device, but a headset's display (distortion, projection, direct mode, IPD, proximity,
// camera, audio) comes from the user's installed stock driver_lighthouse, loaded at run
// time from the copy the installer kept (driver_lighthouse.so.stock). Nothing of it is
// shipped with this project.
//
// The stock provider gets a driver context whose IVRServerDriverHost is a proxy:
//  * TrackedDeviceAdded: only head-mounted displays reach the real host; everything else
//    the stock driver finds is ours already and is swallowed (reported as added).
//  * TrackedDevicePoseUpdated for its headset: dropped once ours publishes the pose (our
//    tracking, our room frame); every other call passes through.
//  * PollNextEvent: the host's events reach both drivers (ours polls the host and hands
//    the stock driver a copy), so neither steals the other's.
// Its IVRProperties is a proxy too: the stock driver's writes of the headset's universe
// are dropped (ours sets it); every other property passes.
// Of the stock provider's standby calls only LeaveStandby is forwarded (it turns the
// headset's panels back on); EnterStandby would power off controllers it doesn't own
// here. The headset's own device calls are forwarded, except GetPose (PublishedPose).
#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <openvr_driver.h>

namespace lh::openvr {

// The same pose with the head offset folded in: qRotation is the head's rotation and vecPosition
// the head's position (qDriverFromHead becomes the identity), so worldFromDriver * pose *
// driverFromHead is unchanged: a caller that applies qDriverFromHead and one that ignores it get
// the same head pose. That needs the rotation and the position folded together; folding only the
// rotation would misplace the head for callers that apply the offset. For callers that take
// qRotation for the head's rotation and ignore qDriverFromHeadRotation: Bigscreen's eye tracking
// driver does, and a Beyond's IMU sits half a turn about (nearly) +Y from its head frame, so with
// the IMU's rotation that driver saw every nod and roll of the head reversed, and fast nods made it
// hand SteamVR a gaze tens of degrees off (yaw was unaffected). The stock driver hands over the
// IMU's rotation; we don't.
// Velocities follow: linear of the head's origin, angular in the head frame.
inline vr::DriverPose_t InHeadFrame(const vr::DriverPose_t& p) {
  using Q = vr::HmdQuaternion_t;
  const auto mul = [](const Q& a, const Q& b) {
    return Q{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
             a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
  };
  const auto rotate = [&mul](const Q& q, const double v[3], double out[3]) {
    const Q r = mul(mul(q, Q{0, v[0], v[1], v[2]}), Q{q.w, -q.x, -q.y, -q.z});
    out[0] = r.x; out[1] = r.y; out[2] = r.z;
  };
  const auto cross = [](const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1]; out[1] = a[2] * b[0] - a[0] * b[2]; out[2] = a[0] * b[1] - a[1] * b[0];
  };
  vr::DriverPose_t out = p;
  const Q headFromImu{p.qDriverFromHeadRotation.w, -p.qDriverFromHeadRotation.x, -p.qDriverFromHeadRotation.y,
                      -p.qDriverFromHeadRotation.z};
  double arm[3];  // IMU origin to head origin, driver frame
  rotate(p.qRotation, p.vecDriverFromHeadTranslation, arm);
  double spin[3], swing[3], swung[3];  // angular velocity in the driver frame; its effect at the head
  rotate(p.qRotation, p.vecAngularVelocity, spin);
  cross(spin, arm, swing);
  cross(spin, swing, swung);
  out.qRotation = mul(p.qRotation, p.qDriverFromHeadRotation);
  out.qDriverFromHeadRotation = Q{1, 0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    out.vecPosition[i] = p.vecPosition[i] + arm[i];
    out.vecDriverFromHeadTranslation[i] = 0;
    out.vecVelocity[i] = p.vecVelocity[i] + swing[i];
    out.vecAcceleration[i] = p.vecAcceleration[i] + swung[i];
  }
  rotate(headFromImu, p.vecAngularVelocity, out.vecAngularVelocity);
  rotate(headFromImu, p.vecAngularAcceleration, out.vecAngularAcceleration);
  return out;
}

// What the headset's GetPose returns when our driver tracks it through the stock driver's device,
// for other drivers that ask (Bigscreen's eye tracking driver does): the last pose ours published,
// as InHeadFrame. The stock driver doesn't track the headset, so its own pose is a placeholder;
// it's asked only until ours has published one. Forgotten whenever the host activates or
// deactivates the device, and closed for good on cleanup.
class PublishedPose {
 public:
  // Activate (true) / Deactivate (false): forget the last pose either way.
  void SetActive(bool active) {
    std::lock_guard lock(mutex_);
    active_ = active && !closed_;
    pose_.reset();
  }
  // From our driver's thread, after each pose it publishes. Ignored while not active.
  void Publish(const vr::DriverPose_t& pose) {
    std::lock_guard lock(mutex_);
    if (active_) pose_ = pose;
  }
  // GetPose: the last pose ours published, in the head frame. Before there is one, stock() (the
  // stock driver's GetPose), called under the lock so Close can't happen meanwhile. After Close, an
  // invalid, disconnected pose.
  template <class Stock>
  vr::DriverPose_t Get(Stock stock) const {
    std::lock_guard lock(mutex_);
    if (closed_) return Closed();
    if (pose_) return InHeadFrame(*pose_);
    return stock();
  }
  // Before the stock provider's Cleanup, after which its device may be gone: other drivers can
  // still call GetPose from their own threads until the host unloads them. Waits for a stock()
  // call in progress.
  void Close() {
    std::lock_guard lock(mutex_);
    closed_ = true;
    active_ = false;
    pose_.reset();
  }

 private:
  static vr::DriverPose_t Closed() {
    vr::DriverPose_t p{};
    p.qRotation = p.qWorldFromDriverRotation = p.qDriverFromHeadRotation = {1, 0, 0, 0};
    p.result = vr::TrackingResult_Uninitialized;
    p.poseIsValid = false;
    p.deviceIsConnected = false;
    return p;
  }
  mutable std::mutex mutex_;
  bool active_ = false;
  bool closed_ = false;
  std::optional<vr::DriverPose_t> pose_;
};

class StockHmd {
 public:
  using LogFn = std::function<void(const std::string&)>;
  // Loads the stock driver and runs its provider's Init. path: the stock library.
  bool Load(const std::string& path, vr::IVRDriverContext* realContext, LogFn log);
  bool Loaded() const { return provider_ != nullptr; }
  void Cleanup();

  // Host thread: our RunFrame calls these.
  void PushEvent(const vr::VREvent_t& event);
  void RunFrame();
  // The host's LeaveStandby on our provider: the stock driver turns the headset's panels
  // back on in it.
  void LeaveStandby();

  // A headset the stock driver added and the host activated: (serial, object id). alive
  // stays valid for the process's life and turns false when the host deactivates it.
  // published: where our driver hands each pose it publishes for it (same life as alive).
  struct Activated { std::string serial; uint32_t objectId; const std::atomic<bool>* alive; PublishedPose* published; };
  std::optional<Activated> TakeActivated();
  // Our driver publishes this device's pose from now on: drop the stock driver's.
  void OwnPose(uint32_t objectId);
  // IVRDebug requests to the headset go through this first; true if it answered them.
  // Call before Load. For what the stock driver can't do without the receivers.
  using RequestHandler = std::function<bool(const char* request, char* response, uint32_t size)>;
  void SetRequestHandler(RequestHandler handler) { requestHandler_ = std::move(handler); }

  // The stock library next to ours (or LH_STOCK_DRIVER), "" if none.
  static std::string DefaultPath();
  // Lighthouse headsets on USB right now. Hosts accept devices only for a few seconds
  // after Init (Monado: 3 s), so the stock driver is loaded at Init when one is.
  //  * Yes: 28de:2000 (Vive / Vive Pro / Index headset board).
  //  * Maybe: a 28de:2300 that isn't a controller. Wired 0x2300 devices name themselves
  //    by hardware family: a Bigscreen Beyond says "Tundra Tracker". Its device config
  //    decides (the session's role from device_class); `maybe` gets their serials.
  enum class Presence { No, Maybe, Yes };
  static Presence HeadsetPresent(std::string* which = nullptr, std::vector<std::string>* maybe = nullptr);
  // A wired 28de:2300 headset (Bigscreen Beyond) the stock driver may open; call before
  // Load. On Linux the stock driver never sees any other lighthouse device: dongles and
  // other wired 0x2300 devices are left out of its udev scans (it would otherwise drive
  // them too: in LH2 rooms every tracker then paused its IMU stream ~30 ms once a
  // second). LH_STOCK_ALL_DEVICES=1 turns this off.
  static void AllowHeadset(const std::string& serial);

 private:
  class Host;
  class LogFilter;
  class Settings;
  class Properties;
  class Context;
  class DeviceProxy;
  friend class Host;
  friend class DeviceProxy;

  void* library_ = nullptr;
  vr::IServerTrackedDeviceProvider* provider_ = nullptr;
  vr::IVRDriverContext* real_ = nullptr;
  vr::IVRServerDriverHost* realHost_ = nullptr;
  Host* host_ = nullptr;
  Settings* settings_ = nullptr;
  Properties* properties_ = nullptr;
  LogFilter* logProxy_ = nullptr;
  Context* context_ = nullptr;
  LogFn log_;
  RequestHandler requestHandler_;

  std::mutex mutex_;
  std::deque<vr::VREvent_t> events_;
  std::deque<Activated> activated_;
  std::set<uint32_t> owned_;
  std::set<int> vendorEvents_;  // types seen (logged once)
  std::deque<DeviceProxy*> devices_;
};

}  // namespace lh::openvr
