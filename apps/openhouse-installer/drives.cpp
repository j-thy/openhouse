#include "drives.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#endif

namespace openhouse::installer {
namespace {

constexpr const char* kBaseLabel = "CRP DISABLD";
constexpr size_t kBaseFlash = 131072;

#ifdef _WIN32
std::vector<Drive> AllDrives() {
  std::vector<Drive> out;
  const DWORD mask = GetLogicalDrives();
  for (int i = 2; i < 26; ++i) {  // from C:, skipping floppies
    if (!(mask & (1u << i))) continue;
    wchar_t root[] = {wchar_t(L'A' + i), L':', L'\\', 0};
    const UINT type = GetDriveTypeW(root);
    if (type != DRIVE_REMOVABLE && type != DRIVE_FIXED) continue;
    Drive d;
    d.mount = root;
    d.name = std::string(1, char('A' + i)) + ":";
    out.push_back(d);
  }
  return out;
}
std::string Label(const Drive& d) {
  wchar_t label[MAX_PATH + 1]{};
  if (!GetVolumeInformationW(d.mount.c_str(), label, MAX_PATH, nullptr, nullptr, nullptr, nullptr, 0)) return {};
  return Utf8(fs::path(label));
}
#else
// /proc/mounts escapes spaces as \040.
std::string Unescape(const std::string& s) {
  std::string out;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\\' && i + 3 < s.size()) {
      out += char(std::strtol(s.substr(i + 1, 3).c_str(), nullptr, 8));
      i += 3;
    } else {
      out += s[i];
    }
  }
  return out;
}
std::vector<Drive> MountedDrives() {
  std::vector<Drive> out;
  std::string text;
  if (!ReadFile("/proc/mounts", &text)) return out;
  std::istringstream in(text);
  std::string device, mount, rest;
  while (in >> device >> mount && std::getline(in, rest)) {
    if (device.rfind("/dev/", 0) != 0) continue;
    Drive d;
    d.device = device;
    d.mount = Unescape(mount);
    d.name = device.substr(5);
    out.push_back(d);
  }
  return out;
}
std::string Mount(const std::string& device) {
  std::string where;
  RunProcess({"udisksctl", "mount", "--no-user-interaction", "-b", device}, [&](const std::string& l) {
    // "Mounted /dev/sdb at /run/media/user/LABEL"
    const size_t at = l.find(" at ");
    if (at != std::string::npos) {
      where = l.substr(at + 4);
      while (!where.empty() && (where.back() == '.' || where.back() == ' ')) where.pop_back();
    }
  });
  return where;
}
// Small removable USB disks not mounted yet: a base station or a dongle in its bootloader.
// Only those are mounted, never a user's other disks.
void MountSmallRemovables() {
  if (!HaveProgram("udisksctl")) return;
  std::vector<std::string> mounted;
  for (const auto& d : MountedDrives()) mounted.push_back(d.device);
  std::error_code ec;
  for (const auto& e : fs::directory_iterator("/sys/block", ec)) {
    const std::string name = e.path().filename().string();
    if (name.rfind("sd", 0) != 0) continue;
    std::string removable, size;
    ReadFile(e.path() / "removable", &removable);
    ReadFile(e.path() / "size", &size);
    const long long sectors = std::atoll(size.c_str());
    if (removable.rfind("1", 0) != 0 || sectors <= 0 || sectors > 2 * 1024 * 64) continue;  // up to 64 MiB
    // The disk itself, or its one partition.
    std::string device = "/dev/" + name;
    for (const auto& part : fs::directory_iterator(e.path(), ec))
      if (part.path().filename().string().rfind(name, 0) == 0 && part.path().filename() != name) device = "/dev/" + part.path().filename().string();
    if (std::find(mounted.begin(), mounted.end(), device) != mounted.end()) continue;
    Mount(device);
  }
}
std::string Label(const Drive& d) {
  std::error_code ec;
  for (const auto& e : fs::directory_iterator("/dev/disk/by-label", ec)) {
    const fs::path target = fs::canonical(e.path(), ec);
    if (!ec && target == d.device) {
      // by-label escapes spaces as \x20
      std::string label = e.path().filename().string(), out;
      for (size_t i = 0; i < label.size(); ++i) {
        if (label.compare(i, 2, "\\x") == 0 && i + 3 < label.size()) {
          out += char(std::strtol(label.substr(i + 2, 2).c_str(), nullptr, 16));
          i += 3;
        } else {
          out += label[i];
        }
      }
      return out;
    }
  }
  return {};
}
std::vector<Drive> AllDrives() {
  MountSmallRemovables();
  return MountedDrives();
}
#endif

std::string Field(const std::string& info, const char* key) {
  const size_t at = info.find(key);
  if (at == std::string::npos) return {};
  size_t start = at + std::strlen(key);
  while (start < info.size() && (info[start] == ':' || info[start] == ' ')) ++start;
  const size_t end = info.find_first_of("\r\n", start);
  return info.substr(start, end == std::string::npos ? std::string::npos : end - start);
}
}  // namespace

std::vector<Drive> FindBaseStations() {
  std::vector<Drive> out;
  std::error_code ec;
  for (const auto& d : AllDrives()) {
    if (Label(d) != kBaseLabel) continue;
    const fs::path image = d.mount / "firmware.bin";
    if (fs::exists(image, ec) && fs::file_size(image, ec) == kBaseFlash) out.push_back(d);
  }
  return out;
}

bool ReadBaseStation(const Drive& d, std::string* image) {
  // Straight from the device, not a cached copy.
#ifdef _WIN32
  HANDLE h = CreateFileW((d.mount / "firmware.bin").c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                         OPEN_EXISTING, FILE_FLAG_NO_BUFFERING, nullptr);
  if (h == INVALID_HANDLE_VALUE) return false;
  void* buffer = VirtualAlloc(nullptr, kBaseFlash, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
  DWORD got = 0;
  const bool ok = buffer && ::ReadFile(h, buffer, DWORD(kBaseFlash), &got, nullptr) && got == kBaseFlash;
  if (ok) image->assign(static_cast<const char*>(buffer), kBaseFlash);
  if (buffer) VirtualFree(buffer, 0, MEM_RELEASE);
  CloseHandle(h);
  return ok;
#else
  const int fd = open((d.mount / "firmware.bin").c_str(), O_RDONLY | O_DIRECT);
  if (fd < 0) return ReadFile(d.mount / "firmware.bin", image) && image->size() == kBaseFlash;
  void* buffer = nullptr;
  if (posix_memalign(&buffer, 4096, kBaseFlash) != 0) { close(fd); return false; }
  size_t total = 0;
  while (total < kBaseFlash) {
    const ssize_t n = pread(fd, static_cast<char*>(buffer) + total, kBaseFlash - total, off_t(total));
    if (n <= 0) break;
    total += size_t(n);
  }
  close(fd);
  const bool ok = total == kBaseFlash;
  if (ok) image->assign(static_cast<const char*>(buffer), kBaseFlash);
  free(buffer);
  return ok;
#endif
}

std::string CheckBaseStationImage(const std::string& image) {
  if (image.size() < 1024) return "it's too small to be base station firmware";
  if (image.size() > kBaseFlash) return "it's larger than the base station's 128 KiB flash";
  // The LPC11U37 refuses to boot an image whose first 8 words don't add up to zero.
  uint32_t sum = 0;
  for (int i = 0; i < 8; ++i) {
    uint32_t w;
    std::memcpy(&w, image.data() + 4 * i, 4);
    sum += w;
  }
  if (sum != 0) return "its vector checksum is wrong (it isn't base station firmware)";
  // A code read protection word at 0x2FC would lock out the USB bootloader for good.
  uint32_t crp;
  std::memcpy(&crp, image.data() + 0x2FC, 4);
  if (crp == 0x4E697370 || crp == 0x12345678 || crp == 0x87654321 || crp == 0x43218765)
    return "it would lock the base station's bootloader";
  return {};
}

bool WriteBaseStation(const Drive& d, const std::string& image, std::string* error, const std::function<void(float)>& progress) {
  const size_t padded = (image.size() + 511) / 512 * 512;
  constexpr size_t kChunk = 4096;
#ifdef _WIN32
  HANDLE h = CreateFileW((d.mount / "firmware.bin").c_str(), GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                         FILE_FLAG_WRITE_THROUGH | FILE_FLAG_NO_BUFFERING, nullptr);
  if (h == INVALID_HANDLE_VALUE) { *error = "couldn't open the base station's firmware.bin for writing"; return false; }
  char* buffer = static_cast<char*>(VirtualAlloc(nullptr, padded, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
  std::memset(buffer, 0xFF, padded);
  std::memcpy(buffer, image.data(), image.size());
  bool ok = true;
  for (size_t off = 0; off < padded && ok; off += kChunk) {
    const DWORD n = DWORD(std::min(kChunk, padded - off));
    DWORD wrote = 0;
    ok = WriteFile(h, buffer + off, n, &wrote, nullptr) && wrote == n;
    if (progress) progress(float(off + n) / float(padded));
  }
  ok = ok && FlushFileBuffers(h);
  VirtualFree(buffer, 0, MEM_RELEASE);
  CloseHandle(h);
#else
  int fd = open((d.mount / "firmware.bin").c_str(), O_WRONLY | O_DIRECT);
  if (fd < 0) fd = open((d.mount / "firmware.bin").c_str(), O_WRONLY);  // never O_TRUNC: written in place
  if (fd < 0) { *error = "couldn't open the base station's firmware.bin for writing"; return false; }
  void* raw = nullptr;
  if (posix_memalign(&raw, 4096, padded) != 0) { close(fd); *error = "out of memory"; return false; }
  char* buffer = static_cast<char*>(raw);
  std::memset(buffer, 0xFF, padded);
  std::memcpy(buffer, image.data(), image.size());
  bool ok = true;
  for (size_t off = 0; off < padded && ok; off += kChunk) {
    const size_t n = std::min(kChunk, padded - off);
    ok = pwrite(fd, buffer + off, n, off_t(off)) == ssize_t(n);
    if (progress) progress(float(off + n) / float(padded));
  }
  ok = ok && fsync(fd) == 0;
  free(raw);
  close(fd);
  sync();
#endif
  if (!ok) *error = "writing to the base station failed";
  return ok;
}

std::vector<Uf2Drive> FindUf2Drives() {
  std::vector<Uf2Drive> out;
  for (const auto& d : AllDrives()) {
    std::string info;
    if (!ReadFile(d.mount / "INFO_UF2.TXT", &info)) continue;
    Uf2Drive u;
    static_cast<Drive&>(u) = d;
    u.board = Field(info, "Board-ID");
    u.model = Field(info, "Model");
    const std::string sd = Field(info, "SoftDevice");
    u.softDeviceKnown = !sd.empty();
    std::string lower = sd;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    u.softDevice = u.softDeviceKnown && lower.find("not") == std::string::npos && lower.find("none") == std::string::npos;
    out.push_back(u);
  }
  return out;
}

bool CopyUf2(const Uf2Drive& d, const fs::path& file, std::string* error, const std::function<void(float)>& progress) {
  std::string data;
  if (!ReadFile(file, &data) || data.size() < 512 || data.size() % 512) { *error = "the firmware file is missing or broken"; return false; }
  const fs::path target = d.mount / "FIRMWARE.UF2";
  std::ofstream out(target, std::ios::binary | std::ios::trunc);
  if (!out) { *error = "couldn't write to the dongle's drive"; return false; }
  // In 512-byte blocks, as UF2 is: the bootloader writes each block as it arrives and resets
  // once the last one is in, so the end of the copy may report an error that isn't one.
  for (size_t off = 0; off < data.size(); off += 512 * 16) {
    const size_t n = std::min<size_t>(512 * 16, data.size() - off);
    out.write(data.data() + off, std::streamsize(n));
    out.flush();
    if (progress) progress(float(off + n) / float(data.size()));
  }
  out.close();
#ifndef _WIN32
  sync();
#endif
  return true;
}

void Eject(const Drive& d) {
#ifndef _WIN32
  if (!d.device.empty()) RunProcess({"udisksctl", "unmount", "--no-user-interaction", "-b", d.device}, nullptr);
#else
  (void)d;
#endif
}

}  // namespace openhouse::installer
