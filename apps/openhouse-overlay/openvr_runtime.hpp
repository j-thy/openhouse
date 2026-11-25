// SteamVR's OpenVR client library, loaded from the user's SteamVR install at run time
// (openvr_runtime.cpp): openhouse ships none of it. Include this instead of <openvr.h>.
#pragma once
#ifndef OPENVR_BUILD_STATIC
#define OPENVR_BUILD_STATIC  // VR_* are ours (forwarders), not imported from a DLL
#endif
#include <openvr.h>

#include <string>

namespace openhouse::overlay {

// SteamVR's runtime folder from openvrpaths.vrpath ("" if SteamVR isn't installed).
std::string SteamVRRuntime();
// SteamVR's log folder from openvrpaths.vrpath (vrserver.txt is in it); "" if none.
std::string SteamVRLogDirectory();
// Loads <runtime>/bin/<platform>/(lib)openvr_api once; false (OpenVRError()) if it can't.
bool LoadOpenVR();
const std::string& OpenVRError();
// Whether SteamVR is running now (a background connection that never starts it).
bool SteamVRRunning();

}  // namespace openhouse::overlay
