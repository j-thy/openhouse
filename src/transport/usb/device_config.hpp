// Read the standard Lighthouse compressed configuration feature stream.
// Injectable feature reader keeps protocol tests independent of hardware/hidapi.
#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace lh::usb {
using FeatureReader = std::function<int(uint8_t* report, size_t capacity)>;
// restartOnLoss: a read that got no answer may still have taken a chunk (a request that reached the
// device, answer lost: Doorman), so the stream starts over instead of asking again. Off: the read is
// retried as stock does (a Watchman radio not ready yet fails without taking one).
bool ReadDeviceConfig(const FeatureReader& read, std::string* json, std::string* error = nullptr, bool restartOnLoss = false);
}  // namespace lh::usb
