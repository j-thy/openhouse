#include "stock_host.hpp"
#include <string>
#include <iterator>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <thread>

#include <fstream>

#include <nlohmann/json.hpp>

#include "platform/platform.hpp"

using namespace vr;

namespace openhouse::compare {
namespace {

// SteamVR's defaults (resources/settings/default.vrsettings and the driver's own
// drivers/lighthouse/resources/settings/default.vrsettings, e.g. its disambiguation
// mode), as SteamVR and Monado serve them; writes are kept in memory only.
class Settings : public IVRSettings {
 public:
  void Load(const std::filesystem::path& file) {
    std::ifstream in(file);
    const nlohmann::json j = nlohmann::json::parse(in, nullptr, false);
    if (!j.is_object()) return;
    std::lock_guard lock(mutex_);
    for (const auto& [section, values] : j.items())
      if (values.is_object()) for (const auto& [key, value] : values.items()) values_[section][key] = value;
  }
  const char* GetSettingsErrorNameFromEnum(EVRSettingsError) override { return "error"; }
  void SetBool(const char* s, const char* k, bool v, EVRSettingsError* e) override { Set(s, k, v, e); }
  void SetInt32(const char* s, const char* k, int32_t v, EVRSettingsError* e) override { Set(s, k, v, e); }
  void SetFloat(const char* s, const char* k, float v, EVRSettingsError* e) override { Set(s, k, v, e); }
  void SetString(const char* s, const char* k, const char* v, EVRSettingsError* e) override { Set(s, k, std::string(v ? v : ""), e); }
  bool GetBool(const char* s, const char* k, EVRSettingsError* e) override {
    const auto v = Get(s, k, e);
    return v.is_boolean() ? v.get<bool>() : v.is_number() ? v.get<double>() != 0 : false;
  }
  int32_t GetInt32(const char* s, const char* k, EVRSettingsError* e) override {
    const auto v = Get(s, k, e);
    return v.is_number() ? int32_t(v.get<double>()) : v.is_boolean() ? int32_t(v.get<bool>()) : 0;
  }
  float GetFloat(const char* s, const char* k, EVRSettingsError* e) override {
    const auto v = Get(s, k, e);
    return v.is_number() ? v.get<float>() : 0;
  }
  void GetString(const char* s, const char* k, char* out, uint32_t n, EVRSettingsError* e) override {
    const auto v = Get(s, k, e);
    if (n) std::snprintf(out, n, "%s", v.is_string() ? v.get<std::string>().c_str() : "");
  }
  void RemoveSection(const char* s, EVRSettingsError* e) override {
    { std::lock_guard lock(mutex_); values_.erase(s ? s : ""); }
    if (e) *e = VRSettingsError_None;
  }
  void RemoveKeyInSection(const char* s, const char* k, EVRSettingsError* e) override {
    { std::lock_guard lock(mutex_); if (values_.contains(s ? s : "")) values_[s].erase(k ? k : ""); }
    if (e) *e = VRSettingsError_None;
  }

 private:
  template <typename T>
  void Set(const char* s, const char* k, const T& v, EVRSettingsError* e) {
    { std::lock_guard lock(mutex_); values_[s ? s : ""][k ? k : ""] = v; }
    if (e) *e = VRSettingsError_None;
  }
  nlohmann::json Get(const char* s, const char* k, EVRSettingsError* e) {
    std::lock_guard lock(mutex_);
    const std::string section = s ? s : "", key = k ? k : "";
    if (values_.contains(section) && values_[section].contains(key)) {
      if (e) *e = VRSettingsError_None;
      return values_[section][key];
    }
    if (e) *e = VRSettingsError_UnsetSettingHasNoDefault;
    return nullptr;
  }
  std::mutex mutex_;
  nlohmann::json values_ = nlohmann::json::object();
};

class Manager : public IVRDriverManager {
 public:
  uint32_t GetDriverCount() const override { return 1; }
  uint32_t GetDriverName(DriverId_t, char* v, uint32_t n) override { if (n) std::snprintf(v, n, "lighthouse"); return 11; }
  DriverHandle_t GetDriverHandle(const char*) override { return 1; }
  bool IsEnabled(DriverId_t) const override { return true; }
};

class Resources : public IVRResources {
 public:
  uint32_t LoadSharedResource(const char*, char*, uint32_t) override { return 0; }
  uint32_t GetResourceFullPath(const char*, const char*, char* v, uint32_t n) override { if (n) v[0] = 0; return 0; }
};

// "IVRPaths_002" (openvr_capi.h has its table): device paths for properties. The stock
// driver uses it unchecked when it activates a device. Writes are accepted, reads find
// nothing, and every path gets a handle.
struct PathRead { uint64_t path; void* buffer; uint32_t size; PropertyTypeTag_t tag; uint32_t required; ETrackedPropertyError error; const char* name; };
struct PathWrite { uint64_t path; EPropertyWriteType type; ETrackedPropertyError setError; void* buffer; uint32_t size;
                   PropertyTypeTag_t tag; ETrackedPropertyError error; const char* name; bool postEvents, changed; };
class Paths {
 public:
  virtual ETrackedPropertyError ReadPathBatch(PropertyContainerHandle_t, PathRead* batch, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) batch[i].error = TrackedProp_UnknownProperty;
    return TrackedProp_Success;
  }
  virtual ETrackedPropertyError WritePathBatch(PropertyContainerHandle_t, PathWrite* batch, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) batch[i].error = batch[i].setError = TrackedProp_Success;
    return TrackedProp_Success;
  }
  virtual ETrackedPropertyError StringToHandle(uint64_t* handle, const char* path) {
    std::lock_guard lock(mutex_);
    const std::string p = path ? path : "";
    auto it = std::find(names_.begin(), names_.end(), p);
    if (it == names_.end()) it = names_.insert(names_.end(), p);
    *handle = uint64_t(it - names_.begin()) + 1;
    return TrackedProp_Success;
  }
  virtual ETrackedPropertyError HandleToString(uint64_t handle, char* buffer, uint32_t size, uint32_t* used) {
    std::lock_guard lock(mutex_);
    if (handle == 0 || handle > names_.size()) return TrackedProp_InvalidOperation;
    const std::string& p = names_[size_t(handle - 1)];
    if (used) *used = uint32_t(p.size() + 1);
    if (size < p.size() + 1) return TrackedProp_BufferTooSmall;
    std::memcpy(buffer, p.c_str(), p.size() + 1);
    return TrackedProp_Success;
  }
  virtual ~Paths() = default;

 private:
  std::mutex mutex_;
  std::vector<std::string> names_;
};

// Private SteamVR interfaces the stock driver calls without checking: empty virtual
// functions, as Monado's steamvr_lh serves them (and the test host, mock_host.cpp).
struct ServerStub {
#define LH_STUB(n) virtual void Unknown##n() {}
  LH_STUB(0) LH_STUB(1) LH_STUB(2) LH_STUB(3) LH_STUB(4) LH_STUB(5) LH_STUB(6) LH_STUB(7) LH_STUB(8) LH_STUB(9)
  LH_STUB(10) LH_STUB(11) LH_STUB(12) LH_STUB(13) LH_STUB(14) LH_STUB(15) LH_STUB(16) LH_STUB(17) LH_STUB(18) LH_STUB(19)
  LH_STUB(20) LH_STUB(21) LH_STUB(22) LH_STUB(23) LH_STUB(24) LH_STUB(25) LH_STUB(26) LH_STUB(27) LH_STUB(28) LH_STUB(29)
  LH_STUB(30) LH_STUB(31) LH_STUB(32) LH_STUB(33) LH_STUB(34) LH_STUB(35) LH_STUB(36) LH_STUB(37) LH_STUB(38) LH_STUB(39)
  LH_STUB(40) LH_STUB(41) LH_STUB(42) LH_STUB(43) LH_STUB(44) LH_STUB(45) LH_STUB(46) LH_STUB(47) LH_STUB(48) LH_STUB(49)
#undef LH_STUB
  virtual ~ServerStub() = default;
};
struct BlockQueueStub {
#define LH_STUB(n) virtual int Unknown##n() { return 1; }
  LH_STUB(0) LH_STUB(1) LH_STUB(2) LH_STUB(3) LH_STUB(4) LH_STUB(5) LH_STUB(6) LH_STUB(7) LH_STUB(8) LH_STUB(9)
  LH_STUB(10) LH_STUB(11) LH_STUB(12) LH_STUB(13) LH_STUB(14) LH_STUB(15)
#undef LH_STUB
};

Eigen::Isometry3d WorldPose(const DriverPose_t& p) {
  Eigen::Isometry3d worldFromDriver = Eigen::Isometry3d::Identity(), driverFromImu = Eigen::Isometry3d::Identity();
  const auto& w = p.qWorldFromDriverRotation;
  worldFromDriver.linear() = Eigen::Quaterniond(w.w, w.x, w.y, w.z).normalized().toRotationMatrix();
  worldFromDriver.translation() = Eigen::Vector3d(p.vecWorldFromDriverTranslation);
  const auto& q = p.qRotation;
  driverFromImu.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  driverFromImu.translation() = Eigen::Vector3d(p.vecPosition);
  return worldFromDriver * driverFromImu;
}

}  // namespace

struct StockHost::Impl : public IVRServerDriverHost, public IVRDriverInput, public IVRProperties, public IVRDriverLog,
                         public IVRDriverContext {
  mutable std::mutex mutex;
  std::vector<StockDevice> devices;  // index = object id
  Settings settings;
  Manager manager;
  Resources resources;
  ServerStub server;
  BlockQueueStub blockQueue;
  Paths paths;
  IServerTrackedDeviceProvider* provider = nullptr;
  std::thread thread;
  std::atomic<bool> stop{false};
#ifdef _WIN32
  HMODULE lib = nullptr;
#else
  void* lib = nullptr;
#endif

  // IVRDriverContext
  void* GetGenericInterface(const char* version, EVRInitError* err) override {
    if (err) *err = VRInitError_None;
    if (!std::strcmp(version, IVRServerDriverHost_Version)) return static_cast<IVRServerDriverHost*>(this);
    if (!std::strncmp(version, "IVRDriverInput_", 15)) return static_cast<IVRDriverInput*>(this);
    if (!std::strcmp(version, IVRProperties_Version)) return static_cast<IVRProperties*>(this);
    if (!std::strcmp(version, IVRDriverLog_Version)) return static_cast<IVRDriverLog*>(this);
    if (!std::strcmp(version, IVRSettings_Version)) return &settings;
    if (!std::strcmp(version, IVRDriverManager_Version)) return &manager;
    if (!std::strcmp(version, IVRResources_Version)) return &resources;
    if (!std::strcmp(version, "IVRServerInternal_XXX") || !std::strcmp(version, "IVRServer_XXX")) return &server;
    if (!std::strncmp(version, "IVRBlockQueue_", 14)) return &blockQueue;
    if (!std::strcmp(version, "IVRPaths_002")) return &paths;
    if (std::getenv("LH_COMPARE_STOCK_LOG")) std::fprintf(stderr, "steamvr: (host) no interface %s\n", version);
    if (err) *err = VRInitError_Init_InterfaceNotFound;
    return nullptr;
  }
  DriverHandle_t GetDriverHandle() override { return 1; }

  // IVRServerDriverHost
  bool TrackedDeviceAdded(const char* serial, ETrackedDeviceClass cls, ITrackedDeviceServerDriver* driver) override {
    // Headsets need SteamVR's display and compositor internals, which this host doesn't
    // have (their Activate crashes without them): declined, like a host without a display.
    if (cls == TrackedDeviceClass_HMD) return false;
    uint32_t id;
    {
      std::lock_guard lock(mutex);
      id = uint32_t(devices.size());
      StockDevice d;
      d.serial = serial;
      d.cls = cls;
      devices.push_back(d);
    }
    // Like Monado: activated at once, without our lock held.
    return driver->Activate(id) == VRInitError_None;
  }
  void TrackedDevicePoseUpdated(uint32_t which, const DriverPose_t& pose, uint32_t size) override {
    if (size != sizeof(DriverPose_t)) return;
    std::lock_guard lock(mutex);
    if (which >= devices.size()) return;
    auto& d = devices[which];
    d.connected = pose.deviceIsConnected;
    d.valid = pose.poseIsValid;
    d.result = pose.result;
    if (pose.poseIsValid) d.world = WorldPose(pose);
    ++d.updates;
  }
  void VsyncEvent(double) override {}
  void VendorSpecificEvent(uint32_t, EVREventType, const VREvent_Data_t&, double) override {}
  bool IsExiting() override { return stop; }
  bool PollNextEvent(VREvent_t*, uint32_t) override { return false; }
  void GetRawTrackedDevicePoses(float, TrackedDevicePose_t*, uint32_t) override {}
  void RequestRestart(const char*, const char*, const char*, const char*) override {}
  uint32_t GetFrameTimings(Compositor_FrameTiming*, uint32_t) override { return 0; }
  void SetDisplayEyeToHead(uint32_t, const HmdMatrix34_t&, const HmdMatrix34_t&) override {}
  void SetDisplayProjectionRaw(uint32_t, const HmdRect2_t&, const HmdRect2_t&) override {}
  void SetRecommendedRenderTargetSize(uint32_t, uint32_t, uint32_t) override {}

  // IVRDriverInput: components are accepted and ignored.
  VRInputComponentHandle_t next = 1;
  EVRInputError Add(VRInputComponentHandle_t* h) { std::lock_guard lock(mutex); *h = next++; return VRInputError_None; }
  EVRInputError CreateBooleanComponent(PropertyContainerHandle_t, const char*, VRInputComponentHandle_t* h) override { return Add(h); }
  EVRInputError UpdateBooleanComponent(VRInputComponentHandle_t, bool, double) override { return VRInputError_None; }
  EVRInputError CreateScalarComponent(PropertyContainerHandle_t, const char*, VRInputComponentHandle_t* h, EVRScalarType, EVRScalarUnits) override { return Add(h); }
  EVRInputError UpdateScalarComponent(VRInputComponentHandle_t, float, double) override { return VRInputError_None; }
  EVRInputError CreateHapticComponent(PropertyContainerHandle_t, const char*, VRInputComponentHandle_t* h) override { return Add(h); }
  EVRInputError CreateSkeletonComponent(PropertyContainerHandle_t, const char*, const char*, const char*, EVRSkeletalTrackingLevel,
                                        const VRBoneTransform_t*, uint32_t, VRInputComponentHandle_t* h) override { return Add(h); }
  EVRInputError UpdateSkeletonComponent(VRInputComponentHandle_t, EVRSkeletalMotionRange, const VRBoneTransform_t*, uint32_t) override {
    return VRInputError_None;
  }
  EVRInputError CreatePoseComponent(PropertyContainerHandle_t, const char*, VRInputComponentHandle_t* h) override { return Add(h); }
  EVRInputError UpdatePoseComponent(VRInputComponentHandle_t, const HmdMatrix34_t*, double) override { return VRInputError_None; }
  EVRInputError CreateEyeTrackingComponent(PropertyContainerHandle_t, const char*, VRInputComponentHandle_t* h) override { return Add(h); }
  EVRInputError UpdateEyeTrackingComponent(VRInputComponentHandle_t, const VREyeTrackingData_t*, double) override { return VRInputError_None; }

  // IVRProperties: writes succeed, reads find nothing.
  ETrackedPropertyError ReadPropertyBatch(PropertyContainerHandle_t, PropertyRead_t* batch, uint32_t n) override {
    for (uint32_t i = 0; i < n; ++i) batch[i].eError = TrackedProp_UnknownProperty;
    return TrackedProp_Success;
  }
  ETrackedPropertyError WritePropertyBatch(PropertyContainerHandle_t, PropertyWrite_t* batch, uint32_t n) override {
    for (uint32_t i = 0; i < n; ++i) batch[i].eError = TrackedProp_Success;
    return TrackedProp_Success;
  }
  const char* GetPropErrorNameFromEnum(ETrackedPropertyError) override { return "error"; }
  PropertyContainerHandle_t TrackedDeviceToPropertyContainer(TrackedDeviceIndex_t d) override { return 100 + d; }

  // IVRDriverLog: LH_COMPARE_STOCK_LOG=1 prints the stock driver's log.
  void Log(const char* text) override {
    static const bool print = std::getenv("LH_COMPARE_STOCK_LOG") != nullptr;
    if (print) std::fprintf(stderr, "steamvr: %s", text);
  }
};

StockHost::StockHost() : impl_(std::make_unique<Impl>()) {}
StockHost::~StockHost() { Stop(); }

std::string StockHost::FindDriver() {
  if (const char* p = std::getenv("LH_STOCK_DRIVER"); p && *p) return p;
#ifdef _WIN32
  const auto dir = std::filesystem::path(lh::platform::SteamVRDirectory()) / "drivers/lighthouse/bin/win64";
  const char *stock = "driver_lighthouse.dll.stock", *original = "driver_lighthouse.dll";
#else
  const auto dir = std::filesystem::path(lh::platform::SteamVRDirectory()) / "drivers/lighthouse/bin/linux64";
  const char *stock = "driver_lighthouse.so.stock", *original = "driver_lighthouse.so";
#endif
  std::error_code ec;
  if (std::filesystem::exists(dir / stock, ec)) return (dir / stock).string();
  // No installer copy: the installed one is stock unless it's ours, which carries the line it
  // logs on start (the installers and update-checker test the same; lh-dependencies.json,
  // tested before, is never in SteamVR's folder).
  if (!std::filesystem::exists(dir / original, ec)) return "";
  std::ifstream in(dir / original, std::ios::binary);
  if (!in) return "";
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (bytes.find("liblh session started") != std::string::npos) return "";
  return (dir / original).string();
}

bool StockHost::Start(const std::string& path, std::string* error) {
  using Factory = void* (*)(const char*, int*);
  Impl& h = *impl_;
  // bin/<platform>/driver_lighthouse.* -> drivers/lighthouse -> SteamVR
  const auto driverDir = std::filesystem::path(path).parent_path().parent_path().parent_path();
  h.settings.Load(driverDir.parent_path().parent_path() / "resources/settings/default.vrsettings");
  h.settings.Load(driverDir / "resources/settings/default.vrsettings");
#ifdef _WIN32
  // Its dependencies sit next to it (bin/win64), as when SteamVR loads it.
  SetDllDirectoryW(std::filesystem::path(path).parent_path().c_str());
  h.lib = LoadLibraryW(std::filesystem::path(path).c_str());
  if (!h.lib) { if (error) *error = "cannot load " + path; return false; }
  auto factory = reinterpret_cast<Factory>(GetProcAddress(h.lib, "HmdDriverFactory"));
#else
  h.lib = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!h.lib) { if (error) *error = dlerror(); return false; }
  auto factory = reinterpret_cast<Factory>(dlsym(h.lib, "HmdDriverFactory"));
#endif
  if (!factory) { if (error) *error = path + " has no HmdDriverFactory"; return false; }
  int err = 0;
  h.provider = static_cast<IServerTrackedDeviceProvider*>(factory(IServerTrackedDeviceProvider_Version, &err));
  if (!h.provider) { if (error) *error = "the stock driver refused " + std::string(IServerTrackedDeviceProvider_Version); return false; }
  if (h.provider->Init(&h) != VRInitError_None) {
    h.provider = nullptr;
    if (error) *error = "the stock driver's Init failed";
    return false;
  }
  h.thread = std::thread([&h] {
    while (!h.stop) {
      h.provider->RunFrame();
      std::this_thread::sleep_for(std::chrono::milliseconds(11));  // ~90 Hz, like a host's frame loop
    }
  });
  return true;
}

void StockHost::Stop() {
  Impl& h = *impl_;
  h.stop = true;
  if (h.thread.joinable()) h.thread.join();
  if (h.provider) h.provider->Cleanup();
  h.provider = nullptr;
  // The library stays loaded: its threads may still be winding down.
}

std::vector<StockDevice> StockHost::Devices() const {
  std::lock_guard lock(impl_->mutex);
  return impl_->devices;
}

}  // namespace openhouse::compare
