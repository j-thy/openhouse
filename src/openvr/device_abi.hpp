// SteamVR, its stock driver and nearly every other Windows driver are built with MSVC. A MinGW
// build of ours follows GCC's C++ ABI, which agrees with MSVC on OpenVR's interfaces except for a
// virtual that returns a struct by value: MSVC passes the hidden result pointer after `this`, GCC
// before it. ITrackedDeviceServerDriver::GetPose() is the only such method we implement or call.
// SteamVR itself never calls it, but other drivers do: Bigscreen's eye tracking driver calls it on
// the headset and crashed vrserver (our GetPose took its result buffer for the device).
//
// So under MinGW the host is given a DeviceForHost, whose GetPose takes its arguments in MSVC's
// order, and an MSVC-built device's GetPose (the stock driver's) is called through
// GetPoseFromOtherDriver. Elsewhere both are the plain calls.
#pragma once

#include <openvr_driver.h>

namespace lh::openvr {

#if defined(_WIN32) && !defined(_MSC_VER)
inline constexpr bool kMsvcGetPoseOrder = true;
// x64 has one calling convention for member functions and plain functions alike, which the raw
// slot call below relies on; 32-bit MSVC passes `this` in ECX (thiscall).
static_assert(sizeof(void*) == 8, "MSVC's GetPose order is only mirrored for 64-bit Windows");

// ITrackedDeviceServerDriver as MSVC lays it out and calls it (no virtual destructor, six slots):
// GetPose takes the result pointer after `this` and returns it.
class MsvcDeviceDriver {
 public:
  virtual vr::EVRInitError Activate(uint32_t objectId) = 0;
  virtual void Deactivate() = 0;
  virtual void EnterStandby() = 0;
  virtual void* GetComponent(const char* name) = 0;
  virtual void DebugRequest(const char* request, char* response, uint32_t size) = 0;
  virtual vr::DriverPose_t* GetPose(vr::DriverPose_t* out) = 0;
};

// One of our devices as the host sees it.
class DeviceForHost final : public MsvcDeviceDriver {
 public:
  explicit DeviceForHost(vr::ITrackedDeviceServerDriver* device) : device_(device) {}
  vr::EVRInitError Activate(uint32_t objectId) override { return device_->Activate(objectId); }
  void Deactivate() override { device_->Deactivate(); }
  void EnterStandby() override { device_->EnterStandby(); }
  void* GetComponent(const char* name) override { return device_->GetComponent(name); }
  void DebugRequest(const char* request, char* response, uint32_t size) override {
    device_->DebugRequest(request, response, size);
  }
  vr::DriverPose_t* GetPose(vr::DriverPose_t* out) override {
    *out = device_->GetPose();
    return out;
  }
  // What to pass to IVRServerDriverHost::TrackedDeviceAdded: the same vtable layout.
  vr::ITrackedDeviceServerDriver* Host() {
    return reinterpret_cast<vr::ITrackedDeviceServerDriver*>(static_cast<MsvcDeviceDriver*>(this));
  }

 private:
  vr::ITrackedDeviceServerDriver* device_;
};

// Through the device's vtable slot 5 directly, not a virtual call on MsvcDeviceDriver: the device
// isn't one of ours, and a compiler that can see every implementation of that class (were it ever
// in an anonymous namespace, or with LTO) may devirtualize the call to DeviceForHost::GetPose with
// no check, which would treat the other driver's object as ours.
inline vr::DriverPose_t GetPoseFromOtherDriver(vr::ITrackedDeviceServerDriver* device) {
  using MsvcGetPose = vr::DriverPose_t* (*)(void* self, vr::DriverPose_t* out);
  vr::DriverPose_t pose;
  (*reinterpret_cast<MsvcGetPose**>(device))[5](device, &pose);
  return pose;
}
#else
inline constexpr bool kMsvcGetPoseOrder = false;

class DeviceForHost {
 public:
  explicit DeviceForHost(vr::ITrackedDeviceServerDriver* device) : device_(device) {}
  vr::ITrackedDeviceServerDriver* Host() { return device_; }

 private:
  vr::ITrackedDeviceServerDriver* device_;
};

inline vr::DriverPose_t GetPoseFromOtherDriver(vr::ITrackedDeviceServerDriver* device) { return device->GetPose(); }
#endif

}  // namespace lh::openvr
