// Device type: whether a tracker/controller is a controller or a tracker, as stored in its own
// config (the JSON in the device, which SteamVR, openhouse and any dongle read). Switching it
// is how a Vive wand on a hip, an ankle or a prop becomes a tracker, and back.
//
// The config is read and written with SteamVR's own lighthouse_console (downloadconfig /
// uploadconfig), over USB. The radio firmware never touches it: a device keeps its type across
// a Doorman or stock radio flash. Every config is backed up before it's changed, and the first
// one seen for a device is kept as <serial>.original.json.
#pragma once
#include <cstdint>
#include <functional>
#include <string>

#include <nlohmann/json.hpp>

#include "system.hpp"

namespace openhouse::installer {

enum class DeviceKind { Unknown, Controller, Tracker, Headset };
DeviceKind KindOf(const nlohmann::json& config);
const char* KindName(DeviceKind kind);

// `config` turned into `to`. When `original` (the device's first config openhouse saw) is
// already of kind `to`, it's used whole: switching back restores exactly what the device had.
// Otherwise only what decides the type changes: device_class, and a role that would contradict
// it. A Vive wand (USB id 0x2012) made a controller also gets the wand's model back if a
// tracker's was put in by hand. False (and why) for headsets and configs of unknown type.
bool ConvertKind(const nlohmann::json& config, DeviceKind to, uint16_t pid, const nlohmann::json* original,
                 nlohmann::json* out, std::string* why);

fs::path DeviceConfigBackups();  // ConfigDirectory()/device-configs

// One wired device's config, through SteamVR's lighthouse_console.
bool ReadDeviceConfig(const fs::path& steamvr, const std::string& serial, std::string* json, std::string* error);
bool WriteDeviceConfig(const fs::path& steamvr, const std::string& serial, const std::string& json, std::string* error);

// Reads, backs up, converts, writes and reads back to check. `log` gets each step.
bool SetDeviceKind(const fs::path& steamvr, const std::string& serial, uint16_t pid, DeviceKind to,
                   const std::function<void(const std::string&)>& log, std::string* error);

}  // namespace openhouse::installer
