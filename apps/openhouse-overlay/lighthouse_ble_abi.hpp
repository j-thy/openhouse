#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <type_traits>

namespace openhouse::overlay::ble_abi {

inline constexpr uint32_t kVersion = 1;
#ifdef LH_CHECK_LIGHTHOUSE_ABI
#include "lh/lighthouse_abi.h"
using Result = LighthouseError;
static_assert(LIGHTHOUSE_ABI_VERSION == kVersion);
static_assert(std::is_same_v<Handle, uint64_t>);
static_assert(std::is_same_v<std::underlying_type_t<Result>, int32_t>);
#else
using Handle = uint64_t;
enum class Result : int32_t {};
#endif

}  // namespace openhouse::overlay::ble_abi
