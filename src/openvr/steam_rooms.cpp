#include "steam_rooms.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>

#include "platform/platform.hpp"

namespace lh::openvr {
namespace {
using json = nlohmann::json;

std::string IdText(const json& id) {
  if (id.is_string()) return id.get<std::string>();
  if (id.is_number_unsigned()) return std::to_string(id.get<uint64_t>());
  if (id.is_number_integer()) return std::to_string(id.get<int64_t>());
  return {};
}

bool ReadText(const std::string& path, std::string* text) {
  std::ifstream f(std::filesystem::u8path(path), std::ios::binary);
  if (!f) return false;
  std::ostringstream s;
  s << f.rdbuf();
  *text = s.str();
  return true;
}
}  // namespace

std::set<std::string> ChaperoneUniverses(const std::string& vrchapPath) {
  std::set<std::string> ids;
  std::string text;
  if (!ReadText(vrchapPath, &text)) return ids;
  const json j = json::parse(text, nullptr, false);
  if (!j.is_object() || !j.contains("universes") || !j.at("universes").is_array()) return ids;
  for (const auto& u : j.at("universes"))
    if (u.is_object() && u.contains("universeID")) {
      const std::string id = IdText(u.at("universeID"));
      if (!id.empty()) ids.insert(id);
    }
  return ids;
}

std::string ForeignUniverses(const std::string& lighthousedb, const std::set<std::string>& keep) {
  json j = json::parse(lighthousedb, nullptr, false);
  if (!j.is_object()) return {};
  if (j.contains("known_universes") && j.at("known_universes").is_array()) {
    json kept = json::array();
    for (const auto& u : j.at("known_universes")) {
      if (!u.is_object() || u.value("openhouse", false) || !u.contains("id")) continue;
      if (keep.count(IdText(u.at("id")))) kept.push_back(u);
    }
    j["known_universes"] = kept;
  }
  return j.dump();
}

bool WriteUniverse(const std::string& path, const std::string& id, const std::vector<UniverseBase>& bases,
                   std::string* error) {
  try {
    namespace fs = std::filesystem;
    const fs::path target = fs::u8path(path);
    json j = json::object();
    std::string text;
    if (ReadText(path, &text)) {
      j = json::parse(text);  // a file we can't read whole is left alone (throws)
      if (!j.is_object()) throw std::runtime_error("not a JSON object");
      const fs::path backup = fs::u8path(path + ".openhouse-backup");
      std::error_code ec;
      if (!fs::exists(backup, ec)) fs::copy_file(target, backup, ec);
    } else if (std::error_code ec; fs::exists(target, ec)) {
      // There but unreadable (on Windows, held open by SteamVR mid-write): writing now would
      // replace SteamVR's whole database with only our universe. Try again next time.
      throw std::runtime_error("exists but can't be read now");
    } else if (!fs::is_directory(target.parent_path())) {
      throw std::runtime_error("no SteamVR config directory");
    }
    if (!j.contains("known_universes") || !j.at("known_universes").is_array()) j["known_universes"] = json::array();
    for (const char* key : {"base_stations", "known_objects"})
      if (!j.contains(key)) j[key] = json::array();

    json universe = {{"id", id}, {"openhouse", true},
                     {"tilt", {{"pitch", 0.0}, {"roll", 0.0}, {"variance", 0.0}}},
                     {"tilt_to_pose_method", 0}};
    json list = json::array();
    for (const auto& b : bases) {
      // SteamVR's pose: baseFromGlobal as qx qy qz qw x y z; untilted, global is the raw frame.
      const Eigen::Isometry3d baseFromRaw = b.rawFromBase.inverse();
      const Eigen::Quaterniond q(baseFromRaw.rotation());
      const Eigen::Vector3d t = baseFromRaw.translation();
      list.push_back({{"base_serial_number", b.serial},
                      {"target_pose", {{"pose", {q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z()}},
                                       {"target_serial_number", 0},
                                       {"variance", b.variance}}}});
    }
    universe["base_stations"] = list;

    auto& universes = j["known_universes"];
    bool replaced = false;
    for (auto& u : universes) {
      if (!u.is_object() || !u.contains("id") || IdText(u.at("id")) != id) continue;
      if (!u.value("openhouse", false)) throw std::runtime_error("universe " + id + " is SteamVR's own");
      for (const char* key : {"lastChaperoneCommit"})  // SteamVR's, if it added any
        if (u.contains(key)) universe[key] = u.at(key);
      u = universe;
      replaced = true;
    }
    if (!replaced) universes.push_back(universe);
    if (j.contains("revision") && j.at("revision").is_number_integer()) j["revision"] = j.at("revision").get<int64_t>() + 1;

    const std::string temp = path + ".openhouse-tmp";
    {
      std::ofstream out(fs::u8path(temp), std::ios::binary);
      out << j.dump(3) << "\n";
      out.close();
      if (!out) throw std::runtime_error("write failed");
    }
    platform::ReplaceFile(fs::u8path(temp), target);
    if (error) error->clear();
    return true;
  } catch (const std::exception& e) {
    if (error) *error = "lighthousedb " + path + ": " + e.what();
    return false;
  }
}

}  // namespace lh::openvr
