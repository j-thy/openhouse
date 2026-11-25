#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "telemetry_client.hpp"

namespace openhouse::overlay {

struct BaseChannelChange {
  std::string serial;
  int from = 0, to = 0;
  bool operator==(const BaseChannelChange&) const = default;
};

int BaseChannel(const viewer::Base& base);
std::vector<int> FreeBaseChannels(const std::vector<viewer::Base>& bases);
std::vector<BaseChannelChange> ResolveBaseChannels(const viewer::State& state);
bool CanChangeBaseChannel(const viewer::State& state, const BaseChannelChange& change, bool automatic);
bool BaseChannelObserved(const viewer::State& state, const BaseChannelChange& change, double after);
bool HaveBaseChannelBackend();
class BaseChannelLock;
struct BaseChannelDiscovery {
  std::function<std::string()> write;
  std::string error;
  std::shared_ptr<BaseChannelLock> lock{};
};

// Discovery and the returned operation run on workers. The UI revalidates the
// live channel assignment before scheduling the write.
BaseChannelDiscovery DiscoverBaseChannel(const BaseChannelChange& change);

}  // namespace openhouse::overlay
