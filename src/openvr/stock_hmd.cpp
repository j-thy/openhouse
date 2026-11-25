#include "stock_hmd.hpp"

#include "device_abi.hpp"
#include "pairing_mailbox.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cwchar>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

#include <nlohmann/json.hpp>

#include <hidapi/hidapi.h>

#include <cstring>

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
extern "C" {
#include <hidsdi.h>
}
#else
#include <dlfcn.h>
#include <elf.h>
#include <link.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace lh::openvr {

using namespace vr;

namespace {
std::mutex g_openMutex;
std::set<std::string> g_allowedHeadsets;  // wired 0x2300 serials, upper case
std::set<std::string> g_deniedLogged;
std::function<void(const std::string&)> g_openLog;

std::string Upper(std::string s) {
  for (auto& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
  return s;
}

// Logs once per device that the stock driver doesn't get to see.
void LogHidden(const std::string& device) {
  std::function<void(const std::string&)> log;
  {
    std::lock_guard lock(g_openMutex);
    if (g_deniedLogged.insert(device).second) log = g_openLog;
  }
  if (log) log("stock driver: hiding " + device + " from it (not the headset)");
}
}  // namespace

#ifndef _WIN32
namespace {

// /dev/hidrawN -> whether the stock driver may open it.
bool StockMayOpen(const char* path) {
  if (!path || std::strncmp(path, "/dev/hidraw", 11) != 0) return true;
  std::ifstream uevent(std::string("/sys/class/hidraw/") + (path + 5) + "/device/uevent");
  unsigned bus = 0, vid = 0, pid = 0;
  std::string uniq, line;
  while (std::getline(uevent, line)) {
    if (line.rfind("HID_ID=", 0) == 0) std::sscanf(line.c_str() + 7, "%x:%x:%x", &bus, &vid, &pid);
    else if (line.rfind("HID_UNIQ=", 0) == 0) uniq = Upper(line.substr(9));
  }
  if (vid != 0x28de) return true;
  if (pid == 0x2101 || pid == 0x2102) return false;  // dongles: ours only
  if (pid != 0x2300) return true;
  std::lock_guard lock(g_openMutex);
  return g_allowedHeadsets.count(uniq) > 0 || g_allowedHeadsets.count("*") > 0;
}

// The stock driver finds devices only by enumerating udev (no hotplug monitor; it
// rescans), so hiding entries there keeps it away from them. Refusing its open() of a
// device it already found is not an option: it crashes on the failure.
struct udev_list_entry;
using GetNext = udev_list_entry* (*)(udev_list_entry*);
using GetName = const char* (*)(udev_list_entry*);
using GetFirst = udev_list_entry* (*)(void*);
GetNext g_realNext = nullptr;
GetFirst g_realFirst = nullptr;
GetName g_name = nullptr;

bool Hidden(udev_list_entry* entry) {
  const char* syspath = g_name ? g_name(entry) : nullptr;
  if (!syspath) return false;
  const char* slash = std::strrchr(syspath, '/');
  const std::string node = slash ? slash + 1 : syspath;
  if (node.rfind("hidraw", 0) != 0) return false;
  const std::string dev = "/dev/" + node;
  if (StockMayOpen(dev.c_str())) return false;
  LogHidden(dev);
  return true;
}

udev_list_entry* SkipHidden(udev_list_entry* entry) {
  while (entry && Hidden(entry)) entry = g_realNext(entry);
  return entry;
}
udev_list_entry* StockNext(udev_list_entry* entry) { return SkipHidden(g_realNext(entry)); }
udev_list_entry* StockFirst(void* enumerate) { return SkipHidden(g_realFirst(enumerate)); }

// Points the library's import `name` (its GOT slot; RTLD_NOW has bound it) at `hook`;
// returns the original target, or nullptr.
void* HookImport(void* library, const char* name, void* hook) {
  link_map* map = nullptr;
  if (dlinfo(library, RTLD_DI_LINKMAP, &map) != 0 || !map) return nullptr;
  const auto base = static_cast<uintptr_t>(map->l_addr);
  auto at = [base](uintptr_t p) { return p < base ? p + base : p; };  // ld.so may have relocated it
  const ElfW(Rela)* rela = nullptr;
  size_t relaSize = 0;
  const ElfW(Sym)* symbols = nullptr;
  const char* strings = nullptr;
  for (const ElfW(Dyn)* d = map->l_ld; d->d_tag != DT_NULL; ++d) {
    if (d->d_tag == DT_JMPREL) rela = reinterpret_cast<const ElfW(Rela)*>(at(d->d_un.d_ptr));
    else if (d->d_tag == DT_PLTRELSZ) relaSize = d->d_un.d_val;
    else if (d->d_tag == DT_SYMTAB) symbols = reinterpret_cast<const ElfW(Sym)*>(at(d->d_un.d_ptr));
    else if (d->d_tag == DT_STRTAB) strings = reinterpret_cast<const char*>(at(d->d_un.d_ptr));
  }
  if (!rela || !symbols || !strings) return nullptr;
  for (size_t i = 0; i < relaSize / sizeof(ElfW(Rela)); ++i) {
    const auto& r = rela[i];
    if (ELF64_R_TYPE(r.r_info) != R_X86_64_JUMP_SLOT) continue;
    if (std::strcmp(strings + symbols[ELF64_R_SYM(r.r_info)].st_name, name) != 0) continue;
    auto* slot = reinterpret_cast<void**>(base + r.r_offset);
    const long page = sysconf(_SC_PAGESIZE);
    const auto pageStart = reinterpret_cast<uintptr_t>(slot) & ~uintptr_t(page - 1);
    // The page's protection as mapped: RELRO leaves it read-only, but the same page
    // can hold writable data too (the stock driver's .data starts on it).
    int prot = PROT_READ;
    {
      std::ifstream maps("/proc/self/maps");
      std::string line;
      while (std::getline(maps, line)) {
        unsigned long lo = 0, hi = 0;
        char perms[5] = {};
        if (std::sscanf(line.c_str(), "%lx-%lx %4s", &lo, &hi, perms) != 3 || pageStart < lo || pageStart >= hi) continue;
        prot = (perms[0] == 'r' ? PROT_READ : 0) | (perms[1] == 'w' ? PROT_WRITE : 0) | (perms[2] == 'x' ? PROT_EXEC : 0);
        break;
      }
    }
    void* start = reinterpret_cast<void*>(pageStart);
    if (!(prot & PROT_WRITE) && mprotect(start, size_t(page), prot | PROT_WRITE) != 0) return nullptr;
    void* original = *slot;
    *slot = hook;
    if (!(prot & PROT_WRITE)) mprotect(start, size_t(page), prot);
    return original;
  }
  return nullptr;
}

// Hides every lighthouse device but the headset from the stock driver's device scans.
bool HideOtherDevices(void* library) {
  g_name = reinterpret_cast<GetName>(dlsym(RTLD_DEFAULT, "udev_list_entry_get_name"));
  if (!g_name) return false;
  void* next = HookImport(library, "udev_list_entry_get_next", reinterpret_cast<void*>(&StockNext));
  if (!next) return false;
  g_realNext = reinterpret_cast<GetNext>(next);
  void* first = HookImport(library, "udev_enumerate_get_list_entry", reinterpret_cast<void*>(&StockFirst));
  if (!first) return false;
  g_realFirst = reinterpret_cast<GetFirst>(first);
  return true;
}
}  // namespace
#else
namespace {
// Windows: the stock DLL lists HID devices with SetupDiEnumDeviceInterfaces /
// SetupDiGetDeviceInterfaceDetailA (paths "\\?\hid#vid_28de&pid_2101&...") and opens
// them with CreateFileA. Its enumeration skips every lighthouse device but the headset,
// as on Linux (refusing the open instead crashes the Linux build of it).
using EnumInterfaces = BOOL(WINAPI*)(HDEVINFO, PSP_DEVINFO_DATA, const GUID*, DWORD, PSP_DEVICE_INTERFACE_DATA);
using InterfaceDetailA = BOOL(WINAPI*)(HDEVINFO, PSP_DEVICE_INTERFACE_DATA, PSP_DEVICE_INTERFACE_DETAIL_DATA_A, DWORD,
                                       PDWORD, PSP_DEVINFO_DATA);
EnumInterfaces g_realEnum = nullptr;
InterfaceDetailA g_realDetail = nullptr;

unsigned HexAfter(const std::string& path, const char* key) {
  const auto at = path.find(key);
  return at == std::string::npos ? 0 : unsigned(std::strtoul(path.c_str() + at + std::strlen(key), nullptr, 16));
}

// The device's USB serial ("LHR-0053AF37"), opened without access rights.
std::string SerialOf(const char* path) {
  HANDLE h = CreateFileA(path, 0, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE) return {};
  wchar_t serial[128] = {};
  const BOOL ok = HidD_GetSerialNumberString(h, serial, sizeof serial);
  CloseHandle(h);
  if (!ok) return {};
  char out[128];
  std::snprintf(out, sizeof out, "%ls", serial);
  return Upper(out);
}

bool StockMayUse(const char* devicePath) {
  if (!devicePath) return true;
  const std::string path = Upper(devicePath);
  if (HexAfter(path, "VID_") != 0x28de) return true;
  const unsigned pid = HexAfter(path, "PID_");
  if (pid == 0x2101 || pid == 0x2102) return false;  // dongles: ours only
  if (pid != 0x2300) return true;
  {
    std::lock_guard lock(g_openMutex);
    if (g_allowedHeadsets.count("*")) return true;
  }
  const std::string serial = SerialOf(devicePath);
  std::lock_guard lock(g_openMutex);
  return g_allowedHeadsets.count(serial) > 0;
}

// Whether interface `index` of the set is hidden; reads its path with the real detail call.
bool HiddenInterface(HDEVINFO set, PSP_DEVICE_INTERFACE_DATA data) {
  DWORD size = 0;
  g_realDetail(set, data, nullptr, 0, &size, nullptr);
  if (size == 0 || size > 4096) return false;
  alignas(8) char buffer[4096];
  auto* detail = reinterpret_cast<PSP_DEVICE_INTERFACE_DETAIL_DATA_A>(buffer);
  detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
  if (!g_realDetail(set, data, detail, size, nullptr, nullptr)) return false;
  if (StockMayUse(detail->DevicePath)) return false;
  LogHidden(detail->DevicePath);
  return true;
}

// Index `wanted` among the visible interfaces: the caller counts 0, 1, 2... until it fails.
BOOL WINAPI StockEnumInterfaces(HDEVINFO set, PSP_DEVINFO_DATA info, const GUID* guid, DWORD wanted,
                                PSP_DEVICE_INTERFACE_DATA data) {
  DWORD visible = 0;
  for (DWORD real = 0;; ++real) {
    if (!g_realEnum(set, info, guid, real, data)) return FALSE;  // its error (no more items)
    if (HiddenInterface(set, data)) continue;
    if (visible++ == wanted) return TRUE;
  }
}

// Points the module's import `dll`!`name` (its IAT slot) at `hook`; returns the original.
void* HookImport(HMODULE module, const char* dll, const char* name, void* hook) {
  auto* base = reinterpret_cast<uint8_t*>(module);
  auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
  if (dos->e_magic != IMAGE_DOS_SIGNATURE) return nullptr;
  auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
  const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
  if (!dir.VirtualAddress) return nullptr;
  for (auto* imp = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress); imp->Name; ++imp) {
    if (_stricmp(reinterpret_cast<const char*>(base + imp->Name), dll) != 0) continue;
    auto* names = reinterpret_cast<IMAGE_THUNK_DATA*>(base + (imp->OriginalFirstThunk ? imp->OriginalFirstThunk : imp->FirstThunk));
    auto* slots = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imp->FirstThunk);
    for (; names->u1.AddressOfData; ++names, ++slots) {
      if (IMAGE_SNAP_BY_ORDINAL(names->u1.Ordinal)) continue;
      auto* byName = reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + names->u1.AddressOfData);
      if (std::strcmp(reinterpret_cast<const char*>(byName->Name), name) != 0) continue;
      DWORD old = 0;
      if (!VirtualProtect(&slots->u1.Function, sizeof(void*), PAGE_READWRITE, &old)) return nullptr;
      void* original = reinterpret_cast<void*>(slots->u1.Function);
      slots->u1.Function = reinterpret_cast<ULONG_PTR>(hook);
      VirtualProtect(&slots->u1.Function, sizeof(void*), old, &old);
      return original;
    }
  }
  return nullptr;
}

// The stock DLL carries hidapi and exports it (hid_open_path, hid_get_feature_report...),
// and drivers that rewrite a headset's config as it is read (CustomHeadsetOpenVR for Pimax)
// detour those exports in "driver_lighthouse.dll". Here that module is ours and the
// stock code is "driver_lighthouse.dll.stock", so they hook ours and the stock driver read
// the raw config: no display. Each exported hid_* of the stock DLL is patched to jump to
// ours, so every HID call it makes (internal ones too: the patch is at the function's entry)
// goes through our exports and whatever hooks them. hid_device handles then are ours on
// both sides. All or none: a mix would pass one hidapi's handle to the other.
hid_device_info* StockEnumerate(unsigned short vid, unsigned short pid) {
  hid_device_info* list = hid_enumerate(vid, pid);
  if (std::getenv("LH_STOCK_ALL_DEVICES")) return list;
  hid_device_info* hidden = nullptr;
  for (hid_device_info** at = &list; *at;) {
    hid_device_info* d = *at;
    if (!d->path || StockMayUse(d->path)) {
      at = &d->next;
      continue;
    }
    LogHidden(d->path);
    *at = d->next;  // unlinked from what the stock driver sees, freed here
    d->next = hidden;
    hidden = d;
  }
  hid_free_enumeration(hidden);
  return list;
}

hid_device* StockOpen(unsigned short vid, unsigned short pid, const wchar_t* serial) {
  hid_device_info* list = StockEnumerate(vid, pid);
  std::string path;
  for (auto* d = list; d && path.empty(); d = d->next)
    if (d->path && (!serial || (d->serial_number && std::wcscmp(d->serial_number, serial) == 0))) path = d->path;
  hid_free_enumeration(list);
  return path.empty() ? nullptr : hid_open_path(path.c_str());
}

bool ForwardHidToOurs(HMODULE module) {
  struct Entry { const char* name; void* ours; };
  const Entry entries[] = {
      {"hid_init", reinterpret_cast<void*>(&hid_init)},
      {"hid_exit", reinterpret_cast<void*>(&hid_exit)},
      {"hid_enumerate", reinterpret_cast<void*>(&StockEnumerate)},
      {"hid_free_enumeration", reinterpret_cast<void*>(&hid_free_enumeration)},
      {"hid_open", reinterpret_cast<void*>(&StockOpen)},
      {"hid_open_path", reinterpret_cast<void*>(&hid_open_path)},
      {"hid_close", reinterpret_cast<void*>(&hid_close)},
      {"hid_read", reinterpret_cast<void*>(&hid_read)},
      {"hid_read_timeout", reinterpret_cast<void*>(&hid_read_timeout)},
      {"hid_write", reinterpret_cast<void*>(&hid_write)},
      {"hid_set_nonblocking", reinterpret_cast<void*>(&hid_set_nonblocking)},
      {"hid_get_feature_report", reinterpret_cast<void*>(&hid_get_feature_report)},
      {"hid_send_feature_report", reinterpret_cast<void*>(&hid_send_feature_report)},
      {"hid_get_manufacturer_string", reinterpret_cast<void*>(&hid_get_manufacturer_string)},
      {"hid_get_product_string", reinterpret_cast<void*>(&hid_get_product_string)},
      {"hid_get_serial_number_string", reinterpret_cast<void*>(&hid_get_serial_number_string)},
      {"hid_get_indexed_string", reinterpret_cast<void*>(&hid_get_indexed_string)},
      {"hid_error", reinterpret_cast<void*>(&hid_error)},
  };
  // mov rax, imm64; jmp rax: 12 bytes, only rax (volatile at a call) touched.
  constexpr size_t kPatch = 12;
  struct Target { uint8_t* at; void* ours; };
  std::vector<Target> targets;
  // Function starts we know of: every unwind-table entry and every export. A leaf function
  // (hid_read, hid_error, hid_set_nonblocking in SteamVR 2.18/2.19) has no unwind entry; its
  // room is the gap to the next start (16-byte slots, int3-padded, there).
  auto* base = reinterpret_cast<uint8_t*>(module);
  auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + reinterpret_cast<IMAGE_DOS_HEADER*>(base)->e_lfanew);
  const auto& exDir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXCEPTION];
  const auto* fns = reinterpret_cast<const RUNTIME_FUNCTION*>(base + exDir.VirtualAddress);
  const size_t fnCount = exDir.VirtualAddress ? exDir.Size / sizeof(RUNTIME_FUNCTION) : 0;
  std::vector<DWORD> starts;
  for (size_t i = 0; i < fnCount; ++i) starts.push_back(fns[i].BeginAddress);
  for (const auto& e : entries)
    if (auto* at = reinterpret_cast<uint8_t*>(reinterpret_cast<void*>(GetProcAddress(module, e.name))))
      starts.push_back(DWORD(at - base));
  for (const auto& e : entries) {
    auto* at = reinterpret_cast<uint8_t*>(reinterpret_cast<void*>(GetProcAddress(module, e.name)));
    if (!at) continue;  // not exported by this version: nothing calls it from outside
    const DWORD rva = DWORD(at - base);
    DWORD room = 0;
    for (size_t i = 0; i < fnCount; ++i)
      if (fns[i].BeginAddress == rva) room = fns[i].EndAddress - fns[i].BeginAddress;
    if (!room) {
      DWORD next = 0xFFFFFFFF;
      for (DWORD s : starts)
        if (s > rva && s < next) next = s;
      room = next == 0xFFFFFFFF ? 0 : next - rva;
    }
    if (room < kPatch) return false;  // the patch would run into the next function
    targets.push_back({at, e.ours});
  }
  if (targets.empty()) return false;
  for (const auto& t : targets) {
    DWORD old = 0;
    if (!VirtualProtect(t.at, kPatch, PAGE_EXECUTE_READWRITE, &old)) return false;  // none called yet: safe to stop
    uint8_t code[kPatch] = {0x48, 0xB8};
    const uint64_t to = reinterpret_cast<uint64_t>(t.ours);
    std::memcpy(code + 2, &to, sizeof to);
    code[10] = 0xFF;
    code[11] = 0xE0;
    std::memcpy(t.at, code, kPatch);
    VirtualProtect(t.at, kPatch, old, &old);
    FlushInstructionCache(GetCurrentProcess(), t.at, kPatch);
  }
  return true;
}

bool HideOtherDevices(HMODULE module) {
  g_realDetail = reinterpret_cast<InterfaceDetailA>(
      reinterpret_cast<void*>(GetProcAddress(GetModuleHandleA("setupapi.dll"), "SetupDiGetDeviceInterfaceDetailA")));
  if (!g_realDetail) return false;
  void* real = HookImport(module, "SETUPAPI.dll", "SetupDiEnumDeviceInterfaces", reinterpret_cast<void*>(&StockEnumInterfaces));
  if (!real) return false;
  g_realEnum = reinterpret_cast<EnumInterfaces>(real);
  return true;
}
}  // namespace
#endif

void StockHmd::AllowHeadset(const std::string& serial) {
  std::lock_guard lock(g_openMutex);
  g_allowedHeadsets.insert(Upper(serial));
}

// Wraps the stock headset so the host's Activate tells us its object id.
class StockHmd::DeviceProxy : public ITrackedDeviceServerDriver {
 public:
  DeviceProxy(StockHmd* owner, std::string serial, ITrackedDeviceServerDriver* inner)
      : owner_(owner), serial_(std::move(serial)), inner_(inner) {}
  // What the host is given for it: GetPose in the host's order (device_abi.hpp).
  ITrackedDeviceServerDriver* ForHost() { return forHost_.Host(); }
  EVRInitError Activate(uint32_t id) override {
    const EVRInitError r = inner_->Activate(id);
    if (r == VRInitError_None) {
      alive_ = true;
      published_.SetActive(true);  // nothing from an earlier activation
      std::lock_guard lock(owner_->mutex_);
      owner_->activated_.push_back({serial_, id, &alive_, &published_});
      // SteamVR sends "leave standby" when it comes up and the stock driver powers the
      // headset's panels then; other hosts (Monado) never do, and the panels stay dark.
      // Harmless under SteamVR (it is out of standby already).
      {
        VREvent_t wake{};
        wake.eventType = VREvent_LeaveStandbyMode;
        wake.trackedDeviceIndex = id;
        owner_->events_.push_back(wake);
        wake.eventType = VREvent_TrackedDeviceUserInteractionStarted;
        owner_->events_.push_back(wake);
      }
    }
    return r;
  }
  void Deactivate() override {
    alive_ = false;  // ours stops publishing its pose first
    published_.SetActive(false);  // and its last pose isn't this device's any more
    inner_->Deactivate();
  }
  // StockHmd::Cleanup, before the stock provider's: GetPose no longer touches the stock device.
  void Close() { published_.Close(); }
  void EnterStandby() override { inner_->EnterStandby(); }
  void* GetComponent(const char* name) override { return inner_->GetComponent(name); }
  void DebugRequest(const char* request, char* response, uint32_t size) override {
    if (owner_->requestHandler_ && owner_->requestHandler_(request, response, size)) return;
    inner_->DebugRequest(request, response, size);
  }
  // The pose ours published for it, once there is one, with qRotation as the head's rotation
  // (PublishedPose, InHeadFrame). Before that, the stock driver's, which is built with the host's
  // compiler; after cleanup, an invalid pose.
  DriverPose_t GetPose() override {
    return published_.Get([this] { return GetPoseFromOtherDriver(inner_); });
  }

 private:
  StockHmd* owner_;
  std::string serial_;
  ITrackedDeviceServerDriver* inner_;
  std::atomic<bool> alive_{false};
  PublishedPose published_;
  DeviceForHost forHost_{this};
};

class StockHmd::Host : public IVRServerDriverHost {
 public:
  explicit Host(StockHmd* owner) : owner_(owner) {}
  bool TrackedDeviceAdded(const char* serial, ETrackedDeviceClass cls, ITrackedDeviceServerDriver* driver) override {
    if (cls != TrackedDeviceClass_HMD) return true;  // ours already; the stock one stays inactive
    auto* proxy = new DeviceProxy(owner_, serial ? serial : "", driver);
    {
      std::lock_guard lock(owner_->mutex_);
      owner_->devices_.push_back(proxy);
    }
    if (owner_->log_) owner_->log_(std::string("stock driver: head-mounted display ") + (serial ? serial : "?"));
    return owner_->realHost_->TrackedDeviceAdded(serial, cls, proxy->ForHost());
  }
  void TrackedDevicePoseUpdated(uint32_t id, const DriverPose_t& pose, uint32_t size) override {
    {
      std::lock_guard lock(owner_->mutex_);
      if (owner_->owned_.count(id)) return;
    }
    owner_->realHost_->TrackedDevicePoseUpdated(id, pose, size);
  }
  void VsyncEvent(double offset) override { owner_->realHost_->VsyncEvent(offset); }
  void VendorSpecificEvent(uint32_t id, EVREventType type, const VREvent_Data_t& data, double offset) override {
    // Status alerts about the stock driver's own tracking, which isn't used (ours drives
    // the pose): 10002 "Too Many Base Stations" (its TDM tracker in a room of more than
    // two bases, e.g. lhbs multi-base firmware). Other types pass; each is logged once.
    if (int(type) == 10002) return;
    {
      std::lock_guard lock(owner_->mutex_);
      if (owner_->vendorEvents_.insert(int(type)).second && owner_->log_)
        owner_->log_("stock driver: vendor event " + std::to_string(int(type)) + " for device " + std::to_string(id));
    }
    owner_->realHost_->VendorSpecificEvent(id, type, data, offset);
  }
  bool IsExiting() override { return owner_->realHost_->IsExiting(); }
  bool PollNextEvent(VREvent_t* event, uint32_t size) override {
    std::lock_guard lock(owner_->mutex_);
    if (owner_->events_.empty() || size < sizeof(VREvent_t)) return false;
    *event = owner_->events_.front();
    owner_->events_.pop_front();
    return true;
  }
  void GetRawTrackedDevicePoses(float predicted, TrackedDevicePose_t* poses, uint32_t count) override {
    owner_->realHost_->GetRawTrackedDevicePoses(predicted, poses, count);
  }
  void RequestRestart(const char* reason, const char* exe, const char* args, const char* dir) override {
    owner_->realHost_->RequestRestart(reason, exe, args, dir);
  }
  uint32_t GetFrameTimings(Compositor_FrameTiming* timing, uint32_t frames) override {
    return owner_->realHost_->GetFrameTimings(timing, frames);
  }
  void SetDisplayEyeToHead(uint32_t id, const HmdMatrix34_t& left, const HmdMatrix34_t& right) override {
    owner_->realHost_->SetDisplayEyeToHead(id, left, right);
  }
  void SetDisplayProjectionRaw(uint32_t id, const HmdRect2_t& left, const HmdRect2_t& right) override {
    owner_->realHost_->SetDisplayProjectionRaw(id, left, right);
  }
  void SetRecommendedRenderTargetSize(uint32_t id, uint32_t width, uint32_t height) override {
    owner_->realHost_->SetRecommendedRenderTargetSize(id, width, height);
  }

 private:
  StockHmd* owner_;
};

// Settings the host doesn't have come from the stock driver's resources/settings/
// default.vrsettings (SteamVR merges that file itself; other hosts, Monado, don't: without
// it the stock driver has no "usedisambiguation" and never brings its headset up).
class StockHmd::Settings : public IVRSettings {
 public:
  Settings(IVRSettings* real, nlohmann::json defaults) : real_(real), defaults_(std::move(defaults)) {}
  const char* GetSettingsErrorNameFromEnum(EVRSettingsError e) override { return real_->GetSettingsErrorNameFromEnum(e); }
  void SetBool(const char* s, const char* k, bool v, EVRSettingsError* e) override { real_->SetBool(s, k, v, e); }
  void SetInt32(const char* s, const char* k, int32_t v, EVRSettingsError* e) override { real_->SetInt32(s, k, v, e); }
  void SetFloat(const char* s, const char* k, float v, EVRSettingsError* e) override { real_->SetFloat(s, k, v, e); }
  void SetString(const char* s, const char* k, const char* v, EVRSettingsError* e) override { real_->SetString(s, k, v, e); }
  bool GetBool(const char* s, const char* k, EVRSettingsError* e) override {
    EVRSettingsError err = VRSettingsError_None;
    const bool v = real_->GetBool(s, k, &err);
    if (err == VRSettingsError_None) return Ok(e), v;
    const auto* d = Default(s, k);
    if (!d || !(d->is_boolean() || d->is_number())) return Fail(e, err), v;
    return Ok(e), d->is_boolean() ? d->get<bool>() : d->get<double>() != 0;
  }
  int32_t GetInt32(const char* s, const char* k, EVRSettingsError* e) override {
    EVRSettingsError err = VRSettingsError_None;
    const int32_t v = real_->GetInt32(s, k, &err);
    if (err == VRSettingsError_None) return Ok(e), v;
    const auto* d = Default(s, k);
    if (!d || !(d->is_boolean() || d->is_number())) return Fail(e, err), v;
    return Ok(e), d->is_boolean() ? int32_t(d->get<bool>()) : int32_t(d->get<double>());
  }
  float GetFloat(const char* s, const char* k, EVRSettingsError* e) override {
    EVRSettingsError err = VRSettingsError_None;
    const float v = real_->GetFloat(s, k, &err);
    if (err == VRSettingsError_None) return Ok(e), v;
    const auto* d = Default(s, k);
    if (!d || !d->is_number()) return Fail(e, err), v;
    return Ok(e), float(d->get<double>());
  }
  void GetString(const char* s, const char* k, char* v, uint32_t n, EVRSettingsError* e) override {
    EVRSettingsError err = VRSettingsError_None;
    real_->GetString(s, k, v, n, &err);
    if (err == VRSettingsError_None) return Ok(e);
    const auto* d = Default(s, k);
    if (!d || !d->is_string() || !v || n == 0) return Fail(e, err);
    const std::string str = d->get<std::string>();
    std::snprintf(v, n, "%s", str.c_str());
    Ok(e);
  }
  void RemoveSection(const char* s, EVRSettingsError* e) override { real_->RemoveSection(s, e); }
  void RemoveKeyInSection(const char* s, const char* k, EVRSettingsError* e) override { real_->RemoveKeyInSection(s, k, e); }

 private:
  static void Ok(EVRSettingsError* e) { if (e) *e = VRSettingsError_None; }
  static void Fail(EVRSettingsError* e, EVRSettingsError err) { if (e) *e = err; }
  const nlohmann::json* Default(const char* s, const char* k) const {
    if (!s || !k || !defaults_.is_object()) return nullptr;
    const auto sec = defaults_.find(s);
    if (sec == defaults_.end() || !sec->is_object()) return nullptr;
    const auto it = sec->find(k);
    return it == sec->end() ? nullptr : &*it;
  }
  IVRSettings* real_;
  nlohmann::json defaults_;
};

// The stock driver's log, minus repeats: its own tracker runs for the headset too (unused,
// ours drives the pose) and in rooms it can't handle (more than 2 bases on TDM, multi-base
// lhbs firmware) repeats the same lines several times a second. A line repeated within
// a minute is dropped.
class StockHmd::LogFilter : public IVRDriverLog {
 public:
  explicit LogFilter(IVRDriverLog* real) : real_(real) {}
  void Log(const char* text) override {
    if (!text || !real_) return;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard lock(mutex_);
      auto& last = seen_[text];
      const bool repeat = last.time_since_epoch().count() != 0 && now - last < std::chrono::minutes(1);
      if (!repeat) last = now;
      if (seen_.size() > 4096) seen_.clear();
      if (repeat) return;
    }
    real_->Log(text);
  }

 private:
  IVRDriverLog* real_;
  std::mutex mutex_;
  std::map<std::string, std::chrono::steady_clock::time_point> seen_;
};

// The stock driver's own tracker (unused: ours drives the pose) writes its room's universe
// onto the headset each time it acquires, so SteamVR switches to that universe's chaperone
// (play area, standing and seated origins). Ours writes the universe only when its own
// changes, so a stock tracker that lost its light and re-acquired mid-session left SteamVR
// in the stock universe until the end (live: twice in five sessions, the two universes'
// origins 15 cm / 1.2 deg apart seated, 32 cm / 12 deg standing). The universe is ours to
// set: the stock driver's writes of it are dropped (reported as done); all else passes.
class StockHmd::Properties : public IVRProperties {
 public:
  Properties(IVRProperties* real, StockHmd* owner) : real_(real), owner_(owner) {}
  ETrackedPropertyError ReadPropertyBatch(PropertyContainerHandle_t c, PropertyRead_t* batch, uint32_t n) override {
    return real_->ReadPropertyBatch(c, batch, n);
  }
  ETrackedPropertyError WritePropertyBatch(PropertyContainerHandle_t c, PropertyWrite_t* batch, uint32_t n) override {
    std::vector<uint32_t> kept;
    for (uint32_t i = 0; batch && i < n; ++i) {
      if (batch[i].prop != Prop_CurrentUniverseId_Uint64) {
        kept.push_back(i);
        continue;
      }
      batch[i].eSetError = batch[i].eError = TrackedProp_Success;
      if (!logged_.exchange(true) && owner_->log_)
        owner_->log_("stock driver: its universe writes are dropped (ours sets the headset's universe)");
    }
    if (!batch || kept.size() == n) return real_->WritePropertyBatch(c, batch, n);
    if (kept.empty()) return TrackedProp_Success;
    std::vector<PropertyWrite_t> pass;
    pass.reserve(kept.size());
    for (const uint32_t i : kept) pass.push_back(batch[i]);
    const ETrackedPropertyError r = real_->WritePropertyBatch(c, pass.data(), uint32_t(pass.size()));
    for (size_t k = 0; k < kept.size(); ++k) {
      batch[kept[k]].eSetError = pass[k].eSetError;
      batch[kept[k]].eError = pass[k].eError;
    }
    return r;
  }
  const char* GetPropErrorNameFromEnum(ETrackedPropertyError e) override { return real_->GetPropErrorNameFromEnum(e); }
  PropertyContainerHandle_t TrackedDeviceToPropertyContainer(TrackedDeviceIndex_t d) override {
    return real_->TrackedDeviceToPropertyContainer(d);
  }

 private:
  IVRProperties* real_;
  StockHmd* owner_;
  std::atomic<bool> logged_{false};
};

class StockHmd::Context : public IVRDriverContext {
 public:
  Context(IVRDriverContext* real, Host* host, Settings* settings, Properties* properties, StockHmd::LogFilter* log)
      : real_(real), host_(host), settings_(settings), properties_(properties), log_(log) {}
  void* GetGenericInterface(const char* version, EVRInitError* error) override {
    if (version && std::string(version) == IVRServerDriverHost_Version) {
      if (error) *error = VRInitError_None;
      return host_;
    }
    if (properties_ && version && std::string(version) == IVRProperties_Version) {
      if (error) *error = VRInitError_None;
      return properties_;
    }
    if (log_ && version && std::string(version) == IVRDriverLog_Version) {
      if (error) *error = VRInitError_None;
      return log_;
    }
    if (settings_ && version && std::string(version) == IVRSettings_Version) {
      if (error) *error = VRInitError_None;
      return settings_;
    }
    if (version && std::string(version) == kMailboxVersion) {
      // Its own mailboxes pass; the pairing one is ours (pairing_mailbox.hpp).
      if (!mailbox_)
        if (auto* real = static_cast<IVRMailbox*>(real_->GetGenericInterface(version, error))) mailbox_ = new StockMailbox(real);
      if (error) *error = mailbox_ ? VRInitError_None : VRInitError_Init_InterfaceNotFound;
      return mailbox_;
    }
    return real_->GetGenericInterface(version, error);
  }
  DriverHandle_t GetDriverHandle() override { return real_->GetDriverHandle(); }

 private:
  IVRDriverContext* real_;
  Host* host_;
  Settings* settings_;
  Properties* properties_;
  StockHmd::LogFilter* log_;
  StockMailbox* mailbox_ = nullptr;  // created on first request, never freed (as the rest)
};

namespace {
// hidapi's strings are wide; USB names and serials are ASCII ('?' otherwise). Same on
// every backend, unlike printf's %ls.
std::string Narrow(const wchar_t* text) {
  std::string out;
  for (; text && *text; ++text) out += *text < 0x80 ? char(*text) : '?';
  return out;
}
}  // namespace

std::string StockHmd::DefaultPath() {
  if (const char* p = std::getenv("LH_STOCK_DRIVER"); p && *p) return p;
#ifdef _WIN32
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&StockHmd::DefaultPath), &self))
    return {};
  wchar_t name[32768];
  const DWORD n = GetModuleFileNameW(self, name, DWORD(std::size(name)));
  if (n == 0 || n >= std::size(name)) return {};
  const auto path = std::filesystem::path(std::wstring(name, n)).parent_path() / L"driver_lighthouse.dll.stock";
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) return {};
  const auto u8 = path.u8string();
  return std::string(u8.begin(), u8.end());
#else
  Dl_info info{};
  if (!dladdr(reinterpret_cast<void*>(&StockHmd::DefaultPath), &info) || !info.dli_fname) return {};
  const auto path = std::filesystem::path(info.dli_fname).parent_path() / "driver_lighthouse.so.stock";
  std::error_code ec;
  return std::filesystem::exists(path, ec) ? path.string() : std::string();
#endif
}

StockHmd::Presence StockHmd::HeadsetPresent(std::string* which, std::vector<std::string>* maybe) {
  // LH_STOCK_HMD=1: load the stock driver even if no USB device looks like a headset.
  if (const char* force = std::getenv("LH_STOCK_HMD"); force && *force == '1') {
    if (which) *which = "(forced by LH_STOCK_HMD)";
    AllowHeadset("*");  // headset unknown: any wired 0x2300 may be it
    return Presence::Yes;
  }
  hid_device_info* list = hid_enumerate(0x28de, 0);
  Presence result = Presence::No;
  for (auto* d = list; d; d = d->next) {
    const std::wstring product = d->product_string ? d->product_string : L"";
    Presence p = Presence::No;
    if (d->product_id == 0x2000) p = Presence::Yes;
    else if (d->product_id == 0x2300 && product.find(L"Controller") == std::wstring::npos) p = Presence::Maybe;
    if (p == Presence::No) continue;
    char id[16];
    std::snprintf(id, sizeof id, "28de:%04x ", d->product_id);
    if (p > result && which) *which = id + Narrow(product.c_str());
    result = std::max(result, p);
    if (p == Presence::Maybe && maybe && d->serial_number) {
      const std::string serial = Narrow(d->serial_number);
      if (std::find(maybe->begin(), maybe->end(), serial) == maybe->end()) maybe->push_back(serial);
    }
  }
  hid_free_enumeration(list);
  return result;
}

bool StockHmd::Load(const std::string& path, IVRDriverContext* realContext, LogFn log) {
  log_ = std::move(log);
  if (path.empty()) return false;
  using Factory = void* (*)(const char*, int*);
#ifdef _WIN32
  // path is UTF-8. LOAD_WITH_ALTERED_SEARCH_PATH: its own dependencies come from its
  // directory (bin/win64), as when SteamVR loads it. The ".stock" extension is fine with
  // a full path (LoadLibrary only appends ".dll" to names without one).
  const std::filesystem::path file(std::u8string(path.begin(), path.end()));
  HMODULE module = LoadLibraryExW(file.wstring().c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
  if (!module) {
    if (log_) log_("stock driver: cannot load " + path + ": error " + std::to_string(GetLastError()));
    return false;
  }
  library_ = module;
  auto factory = reinterpret_cast<Factory>(reinterpret_cast<void*>(GetProcAddress(module, "HmdDriverFactory")));
  if (!std::getenv("LH_STOCK_ALL_DEVICES")) {
    {
      std::lock_guard lock(g_openMutex);
      g_openLog = log_;
    }
    if (!HideOtherDevices(module) && log_) log_("stock driver: could not limit it to the headset's devices");
  }
  // Its HID through our hidapi exports (see ForwardHidToOurs). LH_STOCK_OWN_HID=1: its own.
  if (!std::getenv("LH_STOCK_OWN_HID")) {
    if (ForwardHidToOurs(module)) {
      if (log_) log_("stock driver: its HID access goes through openhouse's hid_* exports");
    } else if (log_) {
      log_("stock driver: could not route its HID access through openhouse's hid_* exports");
    }
  }
#else
  // RTLD_LOCAL: its symbols (HmdDriverFactory, its OpenVR helpers) stay its own.
  library_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!library_) {
    if (log_) log_(std::string("stock driver: cannot load ") + path + ": " + dlerror());
    return false;
  }
  auto factory = reinterpret_cast<Factory>(dlsym(library_, "HmdDriverFactory"));
  if (!std::getenv("LH_STOCK_ALL_DEVICES")) {
    {
      std::lock_guard lock(g_openMutex);
      g_openLog = log_;
    }
    if (!HideOtherDevices(library_) && log_) log_("stock driver: could not limit it to the headset's devices");
  }
#endif
  int code = 0;
  auto* provider = factory ? static_cast<IServerTrackedDeviceProvider*>(factory(IServerTrackedDeviceProvider_Version, &code))
                           : nullptr;
  if (!provider) {
    if (log_) log_("stock driver: no " + std::string(IServerTrackedDeviceProvider_Version) + " in " + path);
#ifdef _WIN32
    FreeLibrary(static_cast<HMODULE>(library_));
#else
    dlclose(library_);
#endif
    library_ = nullptr;
    return false;
  }
  real_ = realContext;
  realHost_ = static_cast<IVRServerDriverHost*>(real_->GetGenericInterface(IVRServerDriverHost_Version, nullptr));
  host_ = new Host(this);
  if (auto* realSettings = static_cast<IVRSettings*>(real_->GetGenericInterface(IVRSettings_Version, nullptr))) {
    // <driver>/bin/{linux64,win64}/<library> -> <driver>/resources/settings/default.vrsettings
    const auto file = std::filesystem::path(std::u8string(path.begin(), path.end())).parent_path().parent_path().parent_path() /
                      "resources" / "settings" / "default.vrsettings";
    nlohmann::json defaults;
    if (std::ifstream in(file); in) defaults = nlohmann::json::parse(in, nullptr, false, true);
    if (defaults.is_discarded() || !defaults.is_object()) {
      if (log_) {
        const auto u8 = file.u8string();
        log_("stock driver: no settings defaults at " + std::string(u8.begin(), u8.end()));
      }
      defaults = nlohmann::json::object();
    }
    settings_ = new Settings(realSettings, std::move(defaults));
  }
  if (auto* realProperties = static_cast<IVRProperties*>(real_->GetGenericInterface(IVRProperties_Version, nullptr)))
    properties_ = new Properties(realProperties, this);
  if (auto* realLog = static_cast<IVRDriverLog*>(real_->GetGenericInterface(IVRDriverLog_Version, nullptr)))
    logProxy_ = new LogFilter(realLog);
  context_ = new Context(real_, host_, settings_, properties_, logProxy_);
  const EVRInitError r = provider->Init(context_);
  if (r != VRInitError_None) {
    if (log_) log_("stock driver: Init failed (" + std::to_string(int(r)) + ")");
    return false;  // library stays loaded: its provider may hold threads
  }
  provider_ = provider;
  if (log_) log_("stock driver loaded for head-mounted displays: " + path);
  return true;
}

void StockHmd::Cleanup() {
  // First: other drivers (Bigscreen's eye tracking) may call the headset's GetPose from their own
  // threads until the host unloads them, and the stock device may be gone after its Cleanup.
  std::deque<DeviceProxy*> devices;
  {
    std::lock_guard lock(mutex_);
    devices = devices_;
  }
  for (auto* proxy : devices) proxy->Close();
  if (provider_) provider_->Cleanup();
  provider_ = nullptr;
  // The library, proxies and context stay alive: the host may still hold the headset's
  // proxy until it shuts down, and unloading code that may still run is worse than a leak.
}

void StockHmd::PushEvent(const VREvent_t& event) {
  if (!provider_) return;
  // Haptics are ours (its other devices are never activated, but don't risk a double pulse).
  if (event.eventType == VREvent_Input_HapticVibration) return;
  std::lock_guard lock(mutex_);
  if (events_.size() >= 256) events_.pop_front();
  events_.push_back(event);
}

void StockHmd::RunFrame() {
  if (provider_) provider_->RunFrame();
}

void StockHmd::LeaveStandby() {
  if (provider_) provider_->LeaveStandby();
}

std::optional<StockHmd::Activated> StockHmd::TakeActivated() {
  std::lock_guard lock(mutex_);
  if (activated_.empty()) return std::nullopt;
  auto a = activated_.front();
  activated_.pop_front();
  return a;
}

void StockHmd::OwnPose(uint32_t objectId) {
  std::lock_guard lock(mutex_);
  owned_.insert(objectId);
}

}  // namespace lh::openvr
