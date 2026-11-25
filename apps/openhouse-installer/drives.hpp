// The USB drives the installer flashes through: a 1.0 base station's bootloader ("CRP DISABLD",
// one 128 KiB firmware.bin that is its flash), and a dongle's UF2 bootloader (INFO_UF2.TXT;
// copying a .uf2 file onto it flashes the dongle).
//
// On Linux an unmounted drive is mounted with udisks (no root), as a file manager would.
#pragma once
#include <string>
#include <vector>

#include "system.hpp"

namespace openhouse::installer {

struct Drive {
  fs::path mount;      // where its files are
  std::string name;    // what the user sees: "E:" (Windows), "sdb" (Linux)
  std::string device;  // Linux: /dev/sdb (for unmounting); Windows: ""
};

// ---- 1.0 base station ----
std::vector<Drive> FindBaseStations();     // bootloader drives with a 128 KiB firmware.bin
bool ReadBaseStation(const Drive& d, std::string* image);  // the whole 128 KiB
// Writes `image` over firmware.bin in place, sector by sector, straight to the device (a copy
// or a file-manager drag can scramble the flash). `progress` 0..1.
bool WriteBaseStation(const Drive& d, const std::string& image, std::string* error, const std::function<void(float)>& progress);
// What's wrong with a base station image, or "" when it's fine to write.
std::string CheckBaseStationImage(const std::string& image);

// ---- UF2 bootloaders (dongles) ----
struct Uf2Drive : Drive {
  std::string board;          // INFO_UF2.TXT "Board-ID"
  std::string model;          // INFO_UF2.TXT "Model"
  bool softDevice = false;    // a SoftDevice is in flash (the "_sd" firmware goes after it)
  bool softDeviceKnown = false;
};
std::vector<Uf2Drive> FindUf2Drives();
bool CopyUf2(const Uf2Drive& d, const fs::path& file, std::string* error, const std::function<void(float)>& progress);

void Eject(const Drive& d);  // Linux: unmount; Windows: nothing (the device resets itself)

}  // namespace openhouse::installer
