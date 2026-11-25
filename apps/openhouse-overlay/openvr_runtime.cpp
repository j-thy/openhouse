// SteamVR's own client library, loaded at run time from the SteamVR install (never shipped
// with openhouse): openvr_api.dll / libopenvr_api.so from <runtime>/bin/<platform>, the
// runtime named in openvrpaths.vrpath. The OpenVR header's inline helpers (VR_Init,
// vr::VROverlay(), ...) call the VR_* functions below, which forward to that library.
#include "platform/platform.hpp"
#include "openvr_runtime.hpp"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace openhouse::overlay {
namespace {
namespace fs = std::filesystem;

struct Api {
  void* library = nullptr;
  uint32_t (*initInternal2)(vr::EVRInitError*, vr::EVRApplicationType, const char*) = nullptr;
  void (*shutdownInternal)() = nullptr;
  bool (*isHmdPresent)() = nullptr;
  bool (*isRuntimeInstalled)() = nullptr;
  bool (*getRuntimePath)(char*, uint32_t, uint32_t*) = nullptr;
  const char* (*errorSymbol)(vr::EVRInitError) = nullptr;
  const char* (*errorDescription)(vr::EVRInitError) = nullptr;
  void* (*getGenericInterface)(const char*, vr::EVRInitError*) = nullptr;
  bool (*isInterfaceVersionValid)(const char*) = nullptr;
  uint32_t (*getInitToken)() = nullptr;
};
Api g_api;
std::string g_error;

fs::path VrPathFile() {
#ifdef _WIN32
  const std::string local = lh::platform::Env("LOCALAPPDATA");
  return local.empty() ? fs::path() : lh::platform::FsPath(local) / "openvr" / "openvrpaths.vrpath";
#else
  if (const std::string xdg = lh::platform::Env("XDG_CONFIG_HOME"); !xdg.empty()) return lh::platform::FsPath(xdg) / "openvr" / "openvrpaths.vrpath";
  const std::string home = lh::platform::Env("HOME");
  return home.empty() ? fs::path() : lh::platform::FsPath(home) / ".config" / "openvr" / "openvrpaths.vrpath";
#endif
}

void* Symbol(const char* name) {
#ifdef _WIN32
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(g_api.library), name));
#else
  return dlsym(g_api.library, name);
#endif
}
}  // namespace

std::string SteamVRRuntime() {
  std::ifstream in(VrPathFile());
  const auto j = nlohmann::json::parse(in, nullptr, false);
  if (!j.is_object() || !j.contains("runtime") || !j.at("runtime").is_array()) return {};
  for (const auto& r : j.at("runtime"))
    if (r.is_string()) return r.get<std::string>();
  return {};
}

std::string SteamVRLogDirectory() {
  std::ifstream in(VrPathFile());
  const auto j = nlohmann::json::parse(in, nullptr, false);
  if (!j.is_object() || !j.contains("log") || !j.at("log").is_array()) return {};
  for (const auto& r : j.at("log"))
    if (r.is_string()) return r.get<std::string>();
  return {};
}

bool LoadOpenVR() {
  static std::once_flag once;
  static bool ok = false;
  std::call_once(once, [] {
    const std::string runtime = SteamVRRuntime();
    if (runtime.empty()) { g_error = "SteamVR isn't installed (no runtime in " + lh::platform::PathUtf8(VrPathFile()) + ")"; return; }
#ifdef _WIN32
    const fs::path lib = fs::u8path(runtime) / "bin" / "win64" / "openvr_api.dll";
    g_api.library = LoadLibraryW(lib.wstring().c_str());
#else
    const fs::path lib = lh::platform::FsPath(runtime) / "bin" / "linux64" / "libopenvr_api.so";
    g_api.library = dlopen(lib.c_str(), RTLD_NOW | RTLD_LOCAL);
#endif
    if (!g_api.library) { g_error = "can't load " + lh::platform::PathUtf8(lib); return; }
    auto get = [](auto*& fn, const char* name) { fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(Symbol(name)); return fn != nullptr; };
    ok = get(g_api.initInternal2, "VR_InitInternal2") && get(g_api.shutdownInternal, "VR_ShutdownInternal") &&
         get(g_api.isHmdPresent, "VR_IsHmdPresent") && get(g_api.isRuntimeInstalled, "VR_IsRuntimeInstalled") &&
         get(g_api.getRuntimePath, "VR_GetRuntimePath") && get(g_api.errorSymbol, "VR_GetVRInitErrorAsSymbol") &&
         get(g_api.errorDescription, "VR_GetVRInitErrorAsEnglishDescription") &&
         get(g_api.getGenericInterface, "VR_GetGenericInterface") &&
         get(g_api.isInterfaceVersionValid, "VR_IsInterfaceVersionValid") && get(g_api.getInitToken, "VR_GetInitToken");
    if (!ok) g_error = lh::platform::PathUtf8(lib) + " lacks an OpenVR function";
  });
  return ok;
}

const std::string& OpenVRError() { return g_error; }

bool SteamVRRunning() {
  if (!LoadOpenVR()) return false;
  // A background app never starts SteamVR: it only connects to a running one.
  vr::EVRInitError error = vr::VRInitError_None;
  g_api.initInternal2(&error, vr::VRApplication_Background, nullptr);
  if (error != vr::VRInitError_None) return false;
  g_api.shutdownInternal();
  return true;
}

}  // namespace openhouse::overlay

// ---- the functions the OpenVR header's inline helpers call ----
using openhouse::overlay::g_api;
using openhouse::overlay::LoadOpenVR;

VR_INTERFACE uint32_t VR_CALLTYPE VR_InitInternal2(vr::EVRInitError* error, vr::EVRApplicationType type, const char* startup) {
  if (!LoadOpenVR()) { if (error) *error = vr::VRInitError_Init_InstallationNotFound; return 0; }
  return g_api.initInternal2(error, type, startup);
}
VR_INTERFACE void VR_CALLTYPE VR_ShutdownInternal() { if (LoadOpenVR()) g_api.shutdownInternal(); }
VR_INTERFACE bool VR_CALLTYPE VR_IsHmdPresent() { return LoadOpenVR() && g_api.isHmdPresent(); }
VR_INTERFACE bool VR_CALLTYPE VR_IsRuntimeInstalled() { return LoadOpenVR() && g_api.isRuntimeInstalled(); }
VR_INTERFACE bool VR_GetRuntimePath(char* buffer, uint32_t size, uint32_t* required) {
  return LoadOpenVR() && g_api.getRuntimePath(buffer, size, required);
}
VR_INTERFACE const char* VR_CALLTYPE VR_GetVRInitErrorAsSymbol(vr::EVRInitError error) {
  return LoadOpenVR() ? g_api.errorSymbol(error) : "VRInitError_Init_InstallationNotFound";
}
VR_INTERFACE const char* VR_CALLTYPE VR_GetVRInitErrorAsEnglishDescription(vr::EVRInitError error) {
  return LoadOpenVR() ? g_api.errorDescription(error) : "SteamVR's OpenVR library not found";
}
VR_INTERFACE void* VR_CALLTYPE VR_GetGenericInterface(const char* version, vr::EVRInitError* error) {
  if (!LoadOpenVR()) { if (error) *error = vr::VRInitError_Init_InstallationNotFound; return nullptr; }
  return g_api.getGenericInterface(version, error);
}
VR_INTERFACE bool VR_CALLTYPE VR_IsInterfaceVersionValid(const char* version) {
  return LoadOpenVR() && g_api.isInterfaceVersionValid(version);
}
VR_INTERFACE uint32_t VR_CALLTYPE VR_GetInitToken() { return LoadOpenVR() ? g_api.getInitToken() : 0; }
