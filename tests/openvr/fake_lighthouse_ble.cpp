#include "lighthouse_ble_abi.hpp"

#include <cstdint>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C"
#endif

using openhouse::overlay::ble_abi::Result;

namespace {
int scenario = 0, writes = 0;
const char* error = "";
Result Text(const char** out, const char* value) {
  auto* copy = static_cast<char*>(std::malloc(std::strlen(value) + 1));
  if (!copy) return static_cast<Result>(-7);
  std::memcpy(copy, value, std::strlen(value) + 1);
  *out = copy;
  return Result{};
}
}  // namespace

EXPORT void lh_fake_ble_scenario(int value) { scenario = value; writes = 0; error = ""; }
EXPORT int lh_fake_ble_writes() { return writes; }
EXPORT uint32_t lighthouse_abi_version() { return 1; }
EXPORT Result lighthouse_init() { return Result{}; }
EXPORT Result lighthouse_discover_adapters(uint64_t* handles, uint32_t* count) {
  handles[0] = 1; handles[1] = 2;
  *count = 2;
  return Result{};
}
EXPORT Result lighthouse_scan(uint64_t adapter, uint32_t, const char* const*, uint32_t, uint64_t* handles, uint32_t* count) {
  if (adapter == 2 && scenario == 2) { error = "simulated scan failure"; return static_cast<Result>(-4); }
  handles[0] = adapter == 1 ? 11 : 21;
  *count = 1;
  return Result{};
}
EXPORT Result lighthouse_peripheral_name(uint64_t handle, const char** out) {
  if (scenario == 3) return Text(out, "LHB-ABCDEF010");
  return Text(out, handle == 11 || scenario == 1 ? "LHB-ABCDEF01" : "Heart rate monitor");
}
EXPORT Result lighthouse_peripheral_id(uint64_t handle, const char** out) {
  return Text(out, handle == 11 ? "first" : "second");
}
EXPORT Result lighthouse_set_channel(uint64_t adapter, uint64_t peripheral, uint32_t channel,
                                  const char*, uint32_t attempts, uint32_t) {
  ++writes;
  if (adapter == 1 && peripheral == 11 && channel == 4 && attempts == 1 && scenario != 4) return Result{};
  error = "simulated BLE write failure";
  return static_cast<Result>(-5);
}
EXPORT Result lighthouse_release_handle(uint64_t) { return Result{}; }
EXPORT void lighthouse_free_string(char* value) { std::free(value); }
EXPORT const char* lighthouse_last_error() { return error; }
