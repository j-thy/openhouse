#include "base_channels.hpp"
#include "lighthouse_ble_abi.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <charconv>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "platform/platform.hpp"

#ifndef _WIN32
#include <cerrno>
#include <dlfcn.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace openhouse::overlay {

class BaseChannelLock {
 public:
  BaseChannelLock() = default;
  BaseChannelLock(const BaseChannelLock&) = delete;
  BaseChannelLock& operator=(const BaseChannelLock&) = delete;

  std::string Open() {
#ifdef _WIN32
    pipe_ = CreateNamedPipeW(LR"(\\.\pipe\OpenHouse.BaseChannels)", PIPE_ACCESS_INBOUND | FILE_FLAG_FIRST_PIPE_INSTANCE,
                             PIPE_TYPE_BYTE | PIPE_REJECT_REMOTE_CLIENTS, 1, 0, 0, 0, nullptr);
    if (pipe_ != INVALID_HANDLE_VALUE) return {};
    const auto error = GetLastError();
    const bool busy = error == ERROR_ACCESS_DENIED || error == ERROR_PIPE_BUSY;
#else
    socket_ = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (socket_ < 0) return "Could not open the base station channel lock";
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    // Linux abstract socket names are independent of paths and disappear when closed.
    constexpr char name[] = "\0openhouse-base-channels";
    std::memcpy(address.sun_path, name, sizeof(name) - 1);
    constexpr auto length = offsetof(sockaddr_un, sun_path) + sizeof(name) - 1;
    if (bind(socket_, reinterpret_cast<const sockaddr*>(&address), length) == 0) return {};
    const bool busy = errno == EADDRINUSE;
#endif
    return busy ? "Another overlay is changing a base station channel" : "Could not lock base station channel changes";
  }

  ~BaseChannelLock() {
#ifdef _WIN32
    if (pipe_ != INVALID_HANDLE_VALUE) CloseHandle(pipe_);
#else
    if (socket_ >= 0) close(socket_);
#endif
  }

 private:
#ifdef _WIN32
  HANDLE pipe_ = INVALID_HANDLE_VALUE;
#else
  int socket_ = -1;
#endif
};

namespace {

auto BackendPath() {
  const auto override = lh::platform::Env("OPENHOUSE_LIGHTHOUSE_BLE");
#ifdef _WIN32
  constexpr const char* filename = "lighthouse.dll";
#else
  constexpr const char* filename = "liblighthouse.so";
#endif
  if (!override.empty()) return lh::platform::FsPath(override);
  // Next to the overlay first: the packages ship it there (apps/), and the installer puts it
  // next to the installed overlay, which is the data directory.
  std::error_code ec;
#ifdef _WIN32
  wchar_t exe[MAX_PATH * 4];
  const DWORD n = GetModuleFileNameW(nullptr, exe, DWORD(std::size(exe)));
  const auto here = std::filesystem::path(std::wstring(exe, n)).parent_path() / filename;
#else
  const auto here = std::filesystem::read_symlink("/proc/self/exe", ec).parent_path() / filename;
#endif
  if (std::filesystem::is_regular_file(here, ec)) return here;
  return lh::platform::FsPath(lh::platform::DataDirectory() + "/" + filename);
}

constexpr int kChannels = 16;
using ble_abi::Handle;
using ble_abi::Result;

// The optional ShayBox/Lighthouse 2.1 C ABI. Keep the module loaded after it starts
// its async runtime so its worker threads can finish safely on process exit.
struct Ble {
#ifdef _WIN32
  HMODULE module = nullptr;
#else
  void* module = nullptr;
#endif
  uint32_t (*version)() = nullptr;
  Result (*init)() = nullptr;
  Result (*adapters)(Handle*, uint32_t*) = nullptr;
  Result (*scan)(Handle, uint32_t, const char* const*, uint32_t, Handle*, uint32_t*) = nullptr;
  Result (*name)(Handle, const char**) = nullptr;
  Result (*id)(Handle, const char**) = nullptr;
  Result (*set)(Handle, Handle, uint32_t, const char*, uint32_t, uint32_t) = nullptr;
  Result (*release)(Handle) = nullptr;
  void (*freeString)(char*) = nullptr;
  const char* (*lastError)() = nullptr;

  template<class T>
  void Load(T& fn, const char* symbol) {
#ifdef _WIN32
    fn = reinterpret_cast<T>(GetProcAddress(module, symbol));
#else
    fn = reinterpret_cast<T>(dlsym(module, symbol));
#endif
  }

  Ble() {
    const auto path = std::filesystem::absolute(BackendPath());
#ifdef _WIN32
    module = LoadLibraryExW(path.c_str(), nullptr, LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_SYSTEM32);
#else
    module = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
#endif
    if (!module) return;
#ifdef LH_CHECK_LIGHTHOUSE_ABI
#define LH_LOAD_BLE(member, symbol) \
    static_assert(std::is_same_v<decltype(member), decltype(&ble_abi::symbol)>); \
    Load(member, #symbol)
#else
#define LH_LOAD_BLE(member, symbol) Load(member, #symbol)
#endif
    LH_LOAD_BLE(version, lighthouse_abi_version);
    LH_LOAD_BLE(init, lighthouse_init);
    LH_LOAD_BLE(adapters, lighthouse_discover_adapters);
    LH_LOAD_BLE(scan, lighthouse_scan);
    LH_LOAD_BLE(name, lighthouse_peripheral_name);
    LH_LOAD_BLE(id, lighthouse_peripheral_id);
    LH_LOAD_BLE(set, lighthouse_set_channel);
    LH_LOAD_BLE(release, lighthouse_release_handle);
    LH_LOAD_BLE(freeString, lighthouse_free_string);
    LH_LOAD_BLE(lastError, lighthouse_last_error);
#undef LH_LOAD_BLE
  }

  bool Ready() const {
    return version && version() == ble_abi::kVersion &&
           init && adapters && scan && name && id && set && release && freeString && lastError;
  }

  std::string Error(const char* fallback) const {
    const char* error = lastError();
    return error && *error ? error : fallback;
  }
};

template <size_t N>
struct Handles {
  const Ble& api;
  std::array<Handle, N> values{};
  uint32_t count = N;
  ~Handles() { for (Handle handle : values) if (handle) api.release(handle); }
  Handle Take(Handle handle) {
    const auto it = std::find(values.begin(), values.end(), handle);
    return it == values.end() ? 0 : std::exchange(*it, 0);
  }
};

struct Target {
  const Ble& api;
  std::shared_ptr<BaseChannelLock> lock;
  Handle adapter = 0, peripheral = 0;
  ~Target() {
    if (peripheral) api.release(peripheral);
    if (adapter) api.release(adapter);
  }
};

bool RecentObservation(const viewer::Base& base, double now) {
  return base.seen && base.channelObservedAt >= 0 && base.channelObservedAt <= now &&
         now - base.channelObservedAt <= 30.0;
}
}  // namespace

int BaseChannel(const viewer::Base& base) {
  if (!base.lh2) return 0;
  int number = 0;
  const char* end = base.channel.data() + base.channel.size();
  const auto parsed = std::from_chars(base.channel.data(), end, number);
  return parsed.ec == std::errc{} && parsed.ptr == end && number >= 1 && number <= kChannels ? number : 0;
}

std::vector<int> FreeBaseChannels(const std::vector<viewer::Base>& bases) {
  std::bitset<kChannels + 1> used;
  for (const auto& base : bases) used.set(BaseChannel(base));
  std::vector<int> free;
  for (int ch = 1; ch <= kChannels; ++ch)
    if (!used[ch]) free.push_back(ch);
  return free;
}

std::vector<BaseChannelChange> ResolveBaseChannels(const viewer::State& state) {
  const auto& bases = state.bases;
  std::array<std::vector<std::string>, kChannels + 1> owners;
  for (const auto& base : bases)
    if (const int ch = BaseChannel(base); ch && RecentObservation(base, state.time))
      owners[ch].push_back(base.serial);
  const auto free = FreeBaseChannels(bases);
  std::vector<BaseChannelChange> changes;
  for (int ch = 1; ch <= kChannels; ++ch) {
    auto& serials = owners[ch];
    std::sort(serials.begin(), serials.end());
    for (size_t n = 1; n < serials.size() && changes.size() < free.size(); ++n)
      changes.push_back({serials[n], ch, free[changes.size()]});
  }
  return changes;
}

bool CanChangeBaseChannel(const viewer::State& state, const BaseChannelChange& change, bool automatic) {
  if (automatic) {
    const auto plan = ResolveBaseChannels(state);
    return !plan.empty() && plan.front() == change;
  }
  const auto free = FreeBaseChannels(state.bases);
  return std::find(free.begin(), free.end(), change.to) != free.end() &&
         std::any_of(state.bases.begin(), state.bases.end(), [&](const viewer::Base& base) {
           return base.serial == change.serial && change.from > 0 && BaseChannel(base) == change.from &&
                  RecentObservation(base, state.time);
         });
}

bool BaseChannelObserved(const viewer::State& state, const BaseChannelChange& change, double after) {
  return std::any_of(state.bases.begin(), state.bases.end(), [&](const viewer::Base& base) {
    return base.serial == change.serial && BaseChannel(base) == change.to &&
           RecentObservation(base, state.time) && base.channelObservedAt > after;
  });
}

bool HaveBaseChannelBackend() {
  std::error_code error;
  return std::filesystem::is_regular_file(BackendPath(), error);
}

BaseChannelDiscovery DiscoverBaseChannel(const BaseChannelChange& change) {
  if (change.to < 1 || change.to > kChannels || change.serial.size() != 12 ||
      !change.serial.starts_with("LHB-") ||
      !std::all_of(change.serial.begin() + 4, change.serial.end(), [](unsigned char c) { return std::isxdigit(c); }))
    return {{}, "Invalid base serial or channel"};
  auto lock = std::make_shared<BaseChannelLock>();
  if (auto error = lock->Open(); !error.empty()) return {{}, std::move(error)};
  const static Ble api;
  if (!api.Ready()) return {{}, "Lighthouse BLE 2.1 library missing or incompatible"};
  if (api.init() != Result{}) return {{}, api.Error("Bluetooth initialization failed")};
  Handles<8> adapters{api};
  if (api.adapters(adapters.values.data(), &adapters.count) != Result{}) return {{}, api.Error("No BLE adapter")};
  if (adapters.count == adapters.values.size()) return {{}, "Too many BLE adapters to verify the target"};
  std::shared_ptr<Target> target;
  std::string targetId;
  for (Handle adapter : adapters.values) {
    if (!adapter) continue;
    Handles<128> peripherals{api};
    if (api.scan(adapter, 4, nullptr, 0, peripherals.values.data(), &peripherals.count) != Result{})
      return {{}, api.Error("Bluetooth scan failed")};
    if (peripherals.count == peripherals.values.size()) return {{}, "Bluetooth scan results reached their limit"};
    for (Handle peripheral : peripherals.values) {
      if (!peripheral) continue;
      const char* name = nullptr;
      if (api.name(peripheral, &name) != Result{} || !name) continue;
      const bool matches = change.serial == name;
      api.freeString(const_cast<char*>(name));
      if (!matches) continue;
      const char* id = nullptr;
      if (api.id(peripheral, &id) != Result{} || !id) return {{}, api.Error("Could not verify BLE device identity")};
      const std::string foundId(id);
      api.freeString(const_cast<char*>(id));
      if (foundId.empty()) return {{}, "BLE device identity is empty"};
      if (target) {
        if (foundId != targetId) return {{}, "Multiple BLE devices advertise " + change.serial};
        continue;
      }
      target = std::make_shared<Target>(api, lock);
      target->adapter = adapters.Take(adapter);
      target->peripheral = peripherals.Take(peripheral);
      if (!target->adapter || !target->peripheral) return {{}, "Lost the verified BLE device handle"};
      targetId = foundId;
    }
  }
  if (!target) return {{}, "Base station not found over BLE: " + change.serial};
  return {[target = std::move(target), channel = uint32_t(change.to)] {
    return target->api.set(target->adapter, target->peripheral, channel, nullptr, 1, 0) == Result{}
               ? std::string{} : target->api.Error("Channel write failed");
  }, {}, std::move(lock)};
}

}  // namespace openhouse::overlay
