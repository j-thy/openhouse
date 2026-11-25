// SteamVR room compatibility: our rooms as SteamVR universes.
//
// SteamVR keeps Room Setup (the chaperone: standing origin, play area) per universe in
// chaperone_info.vrchap, and launches Room Setup when the universe a device reports has
// none ("calibration state 201"). Our own rooms' universes ("room-<unix time>") existed
// only in our room map, never in lighthousedb.json, and SteamVR asked for Room Setup on
// every start, even with the user's old SteamVR room set up for the same base stations.
//
// So, by default under SteamVR:
//  - a room that shares a base station with a SteamVR universe that has Room Setup done
//    (in chaperone_info.vrchap) uses that universe: its ID and its frame (RawAlignment),
//    and the old Room Setup keeps working;
//  - any other room of ours is written into lighthousedb.json as a universe of its own,
//    its bases in the host frame we report, so a Room Setup done on it has a universe
//    to belong to, and tools reading lighthousedb.json see where the bases are.
// Universes we write carry "openhouse": true: they're our rooms, never imported back as
// SteamVR's, and the only ones we ever rewrite. SteamVR's own universes are never changed.
#pragma once
#include <Eigen/Geometry>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace lh::openvr {

// Universe IDs with Room Setup in a chaperone_info.vrchap (empty when unreadable).
std::set<std::string> ChaperoneUniverses(const std::string& vrchapPath);

// lighthousedb.json text reduced to the universes in `keep` that we didn't write: what
// RawAlignment may align to. The base calibrations are kept whole.
std::string ForeignUniverses(const std::string& lighthousedb, const std::set<std::string>& keep);

struct UniverseBase {
  uint32_t serial = 0;
  Eigen::Isometry3d rawFromBase = Eigen::Isometry3d::Identity();
  double variance = 0;  // m^2
};

// Writes (or replaces) our universe `id` in the lighthousedb.json at `path`, keeping
// everything else in the file; saves `path`.openhouse-backup first if there is none yet.
// The file is replaced in one step (temp file + rename).
bool WriteUniverse(const std::string& path, const std::string& id, const std::vector<UniverseBase>& bases,
                   std::string* error);

}  // namespace lh::openvr
