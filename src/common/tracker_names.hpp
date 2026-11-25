#pragma once

#include <algorithm>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

#include "lh/telemetry.hpp"

namespace lh::telemetry {

inline std::string_view TrimTrackerName(std::string_view name) {
  const auto first = name.find_first_not_of(' ');
  if (first == std::string_view::npos) return {};
  return name.substr(first, name.find_last_not_of(' ') - first + 1);
}

inline bool IsTrackerName(std::string_view name) {
  return name.size() <= kTrackerNameMaxBytes &&
         std::none_of(name.begin(), name.end(), [](unsigned char c) { return c < 32 || c == 127; });
}

inline std::string TrackerDisplayName(const nlohmann::json& state, std::string_view serial) {
  const auto settings = state.find("settings");
  if (settings == state.end() || !settings->is_object()) return {};
  const auto names = settings->find("trackerNames");
  if (names == settings->end() || !names->is_object()) return {};
  const auto name = names->find(serial);
  if (name == names->end() || !name->is_string()) return {};
  const auto& value = name->get_ref<const std::string&>();
  return IsTrackerName(value) ? value : std::string{};
}

}  // namespace lh::telemetry
