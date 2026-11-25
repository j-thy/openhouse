#pragma once

#include <array>
#include <bitset>
#include <map>
#include <set>
#include <utility>

#include "tracking/disambiguator_sob.hpp"

namespace lh::runtime {

struct SessionBase {
  BaseConfig config;
  BaseDynamicState dynamic;
  bool seen = false;
  double channelObservedAt = -1;

  bool HasDirectChannel(int channel) const {
    return seen && !dynamic.cached && dynamic.sobChannel == channel;
  }

  bool Accepts(const BaseDynamicState& report) const {
    return report.sobChannel < 0 || !report.cached || !seen || dynamic.cached;
  }
};

struct SobChannels {
  std::map<int, std::pair<BaseConfig, BaseDynamicState>> known;
  std::bitset<sob::kChannels> blocked;
  std::set<uint32_t> conflictedBases;
  std::array<std::pair<uint32_t, uint32_t>, sob::kChannels> firstConflict{};
};

inline SobChannels ResolveSobChannels(const std::map<uint32_t, SessionBase>& bases) {
  SobChannels result;
  decltype(result.known) recalled;
  std::bitset<sob::kChannels> duplicateRecalls;
  for (const auto& [serial, base] : bases) {
    const int ch = base.dynamic.sobChannel;
    if (ch < 0 || ch >= sob::kChannels || !base.config.valid || base.config.cal.size() != 14) continue;
    if (!base.HasDirectChannel(ch)) {
      if (!recalled.try_emplace(ch, base.config, base.dynamic).second) duplicateRecalls.set(ch);
      continue;
    }
    const auto [first, added] = result.known.try_emplace(ch, base.config, base.dynamic);
    if (added) continue;
    const uint32_t other = first->second.first.serial;
    if (!result.blocked[ch]) result.firstConflict[ch] = {other, serial};
    result.blocked.set(ch);
    result.conflictedBases.insert(other);
    result.conflictedBases.insert(serial);
  }
  for (int ch = 0; ch < sob::kChannels; ++ch)
    if (result.blocked[ch]) result.known.erase(ch);
  for (const auto& [ch, base] : recalled)
    if (!result.blocked[ch] && !duplicateRecalls[ch]) result.known.try_emplace(ch, base);
  return result;
}

}  // namespace lh::runtime
