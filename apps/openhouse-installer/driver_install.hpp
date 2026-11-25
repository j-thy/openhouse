// Installing openhouse into SteamVR and taking it out again (Windows and Linux).
//
// Install keeps SteamVR's own driver as driver_lighthouse.<ext>.stock whenever the one in place
// isn't openhouse's (a SteamVR update's newer one replaces an older backup), puts ours in its
// place, and registers openhouse-overlay (which also runs the update check) as an app SteamVR starts with each
// session (Steam's appconfig.json + vrappconfig). Uninstall puts SteamVR's driver back, unless a
// SteamVR update already did. Your room and settings are kept either way.
#pragma once
#include <functional>
#include <string>

#include "system.hpp"

namespace openhouse::installer {

struct SteamVR {
  fs::path root;          // .../steamapps/common/SteamVR ("" when not found)
  bool saved = false;     // the folder the user picked (installer.json), not a default one
};
SteamVR FindSteamVR();
bool IsSteamVRFolder(const fs::path& root);
bool FlatpakSteam(const fs::path& root);  // Linux: inside Flatpak Steam's folder (not fully tested)
void RememberSteamVR(const fs::path& root);  // installer.json, so the next start finds it

struct DriverStatus {
  bool driverFound = false;    // SteamVR's lighthouse driver file is there
  bool openhouse = false;      // and it's openhouse's
  std::string version;         // openhouse's version in it ("" for one older than the marker)
  long long buildTime = 0;     // when that driver was built (Unix seconds), 0 for one older than the marker
  bool backup = false;         // SteamVR's own driver is kept next to it
};
DriverStatus ReadStatus(const fs::path& steamvr);

struct InstallOptions {
  bool monadoCheck = false;  // Linux: the update check for Monado / WiVRn too (systemd user service)
};
// Both throw std::runtime_error with a message for the user; `log` gets each step.
void InstallDriver(const fs::path& steamvr, const InstallOptions& options, const std::function<void(const std::string&)>& log);
void UninstallDriver(const fs::path& steamvr, const std::function<void(const std::string&)>& log);
bool MonadoCheckInstalled();

// Linux: the udev rule that lets the logged-in user open Valve/HTC lighthouse devices and the
// Doorman Dongle (/etc/udev/rules.d/60-openhouse.rules). Installing or removing it asks for the
// password through pkexec. Always true on Windows.
bool DeviceAccessInstalled();
bool InstallDeviceAccess(std::string* error);
bool RemoveDeviceAccess(std::string* error);

// The package's driver (next to the installer) and its version.
bool PackageHasDriver();

}  // namespace openhouse::installer
