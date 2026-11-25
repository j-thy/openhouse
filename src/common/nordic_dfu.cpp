#include "nordic_dfu.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
#else
#include <cerrno>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace lh::dfu {
namespace {

constexpr uint16_t kVid = 0x1915, kPid = 0x521F;

// ---- Intel HEX ----
int Nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// ---- CRC-32 (IEEE, as zlib: the bootloader's object CRC) ----
uint32_t Crc32(const uint8_t* p, size_t n, uint32_t crc = 0) {
  static uint32_t table[256] = {};
  if (!table[1]) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      table[i] = c;
    }
  }
  crc = ~crc;
  while (n--) crc = table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

void Put32(std::vector<uint8_t>& v, uint32_t x) {
  for (int i = 0; i < 4; ++i) v.push_back(uint8_t(x >> (8 * i)));
}
uint32_t Get32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }

// ---- init packet: dfu-cc.proto, unsigned, the way nrfutil builds it ----
void Varint(std::vector<uint8_t>& v, uint32_t n) {
  do {
    uint8_t b = n & 0x7F;
    n >>= 7;
    v.push_back(uint8_t(b | (n ? 0x80 : 0)));
  } while (n);
}
void FieldVarint(std::vector<uint8_t>& v, int field, uint32_t n) { Varint(v, uint32_t(field << 3)); Varint(v, n); }
void FieldBytes(std::vector<uint8_t>& v, int field, const std::vector<uint8_t>& b) {
  Varint(v, uint32_t(field << 3 | 2));
  Varint(v, uint32_t(b.size()));
  v.insert(v.end(), b.begin(), b.end());
}

// SHA-256 (the init packet carries the image's hash; nrfutil stores it byte-reversed)
std::vector<uint8_t> Sha256(const std::vector<uint8_t>& msg) {
  static const uint32_t k[64] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
      0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
      0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
      0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
      0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
      0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
      0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::vector<uint8_t> m = msg;
  const uint64_t bits = uint64_t(msg.size()) * 8;
  m.push_back(0x80);
  while (m.size() % 64 != 56) m.push_back(0);
  for (int i = 7; i >= 0; --i) m.push_back(uint8_t(bits >> (8 * i)));
  auto rotr = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
  for (size_t off = 0; off < m.size(); off += 64) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i)
      w[i] = uint32_t(m[off + 4 * i]) << 24 | uint32_t(m[off + 4 * i + 1]) << 16 | uint32_t(m[off + 4 * i + 2]) << 8 | m[off + 4 * i + 3];
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t t1 = hh + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) + ((e & f) ^ (~e & g)) + k[i] + w[i];
      const uint32_t t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
      hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }
  std::vector<uint8_t> out;
  for (uint32_t x : h)
    for (int i = 3; i >= 0; --i) out.push_back(uint8_t(x >> (8 * i)));
  return out;
}

std::vector<uint8_t> InitPacket(const std::vector<uint8_t>& app) {
  std::vector<uint8_t> digest = Sha256(app);
  std::reverse(digest.begin(), digest.end());
  std::vector<uint8_t> hash, sdReq, validation, init, command, packet;
  FieldVarint(hash, 1, 3);  // SHA256
  FieldBytes(hash, 2, digest);
  Varint(sdReq, 0x00);      // no SoftDevice
  FieldVarint(validation, 1, 1);  // VALIDATE_GENERATED_CRC (nrfutil's default)
  FieldBytes(validation, 2, {});
  FieldVarint(init, 1, 1);  // fw_version
  FieldVarint(init, 2, 52); // hw_version
  FieldBytes(init, 3, sdReq);
  FieldVarint(init, 4, 0);  // APPLICATION
  FieldVarint(init, 5, 0);  // sd_size
  FieldVarint(init, 6, 0);  // bl_size
  FieldVarint(init, 7, uint32_t(app.size()));
  FieldBytes(init, 8, hash);
  FieldVarint(init, 9, 0);  // is_debug
  FieldBytes(init, 10, validation);
  FieldVarint(command, 1, 1);  // INIT
  FieldBytes(command, 2, init);
  FieldBytes(packet, 1, command);  // Packet{command}: unsigned
  return packet;
}

// ---- serial port ----
class Port {
 public:
  ~Port() { Close(); }
  bool Open(const std::string& name, std::string* error) {
#ifdef _WIN32
    const std::string path = "\\\\.\\" + name;
    h_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h_ == INVALID_HANDLE_VALUE) { *error = "couldn't open " + name + " (is nRF Connect using it?)"; return false; }
    DCB dcb{};
    dcb.DCBlength = sizeof dcb;
    GetCommState(h_, &dcb);
    dcb.BaudRate = 115200; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE; dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;  // the bootloader waits for DTR before it listens
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE; dcb.fInX = FALSE;
    SetCommState(h_, &dcb);
    COMMTIMEOUTS t{};
    t.ReadIntervalTimeout = MAXDWORD; t.ReadTotalTimeoutMultiplier = MAXDWORD; t.ReadTotalTimeoutConstant = 50;
    t.WriteTotalTimeoutConstant = 2000;
    SetCommTimeouts(h_, &t);
    PurgeComm(h_, PURGE_RXCLEAR | PURGE_TXCLEAR);
#else
    fd_ = ::open(name.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      *error = "couldn't open " + name + ": " + std::strerror(errno);
      if (errno == EACCES) *error += " (set up device access)";
      return false;
    }
    termios t{};
    tcgetattr(fd_, &t);
    cfmakeraw(&t);
    cfsetspeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 1;  // reads return after 100 ms
    tcsetattr(fd_, TCSANOW, &t);
    tcflush(fd_, TCIOFLUSH);
#endif
    return true;
  }
  void Close() {
#ifdef _WIN32
    if (h_ != INVALID_HANDLE_VALUE) CloseHandle(h_);
    h_ = INVALID_HANDLE_VALUE;
#else
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
#endif
  }
  bool Write(const uint8_t* p, size_t n) {
    while (n) {
#ifdef _WIN32
      DWORD done = 0;
      if (!WriteFile(h_, p, DWORD(n), &done, nullptr) || !done) return false;
#else
      const ssize_t done = ::write(fd_, p, n);
      if (done < 0 && errno == EINTR) continue;
      if (done <= 0) return false;
#endif
      p += done;
      n -= size_t(done);
    }
    return true;
  }
  // Up to n bytes; 0 after a short wait with nothing; -1 on error (unplugged).
  long Read(uint8_t* p, size_t n) {
#ifdef _WIN32
    DWORD got = 0;
    if (!ReadFile(h_, p, DWORD(n), &got, nullptr)) return -1;
    return long(got);
#else
    const ssize_t got = ::read(fd_, p, n);
    if (got < 0) return errno == EINTR || errno == EAGAIN ? 0 : -1;
    return long(got);
#endif
  }

 private:
#ifdef _WIN32
  HANDLE h_ = INVALID_HANDLE_VALUE;
#else
  int fd_ = -1;
#endif
};

// ---- the DFU protocol (SLIP frames) ----
enum : uint8_t {
  kCreate = 0x01, kSetPrn = 0x02, kCrc = 0x03, kExecute = 0x04, kSelect = 0x06, kMtu = 0x07, kWrite = 0x08, kPing = 0x09,
  kResponse = 0x60,
};

std::string ResultText(uint8_t result, uint8_t ext) {
  static const std::map<uint8_t, const char*> results = {
      {0x00, "invalid opcode"}, {0x02, "opcode not supported"}, {0x03, "invalid parameter"}, {0x04, "not enough memory"},
      {0x05, "invalid object"}, {0x07, "unsupported type"}, {0x08, "operation not permitted"}, {0x0A, "operation failed"}};
  static const std::map<uint8_t, const char*> exts = {
      {0x02, "wrong command format"}, {0x03, "unknown command"}, {0x04, "init packet invalid"},
      {0x05, "firmware version too old"}, {0x06, "wrong hardware version"},
      {0x07, "a SoftDevice is installed (this firmware needs a dongle without one)"},
      {0x08, "the bootloader wants a signed image"}, {0x09, "wrong hash type"}, {0x0A, "hash failed"},
      {0x0B, "wrong signature type"}, {0x0C, "image check failed"}, {0x0D, "the image doesn't fit"}};
  if (result == 0x0B) {
    const auto e = exts.find(ext);
    return e != exts.end() ? e->second : "error " + std::to_string(ext);
  }
  const auto r = results.find(result);
  return r != results.end() ? r->second : "error " + std::to_string(result);
}

class Dfu {
 public:
  explicit Dfu(Port& port) : port_(port) {}

  bool Send(const std::vector<uint8_t>& msg) {
    std::vector<uint8_t> out;
    out.reserve(msg.size() * 2 + 1);
    for (uint8_t b : msg) {
      if (b == 0xC0) { out.push_back(0xDB); out.push_back(0xDC); }
      else if (b == 0xDB) { out.push_back(0xDB); out.push_back(0xDD); }
      else out.push_back(b);
    }
    out.push_back(0xC0);
    return port_.Write(out.data(), out.size());
  }

  // The response to op: its payload. False (and *error) on a timeout or a failure result.
  bool Request(const std::vector<uint8_t>& msg, std::vector<uint8_t>* payload, double timeout, std::string* error) {
    if (!Send(msg)) { *error = "the dongle went away"; return false; }
    const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    std::vector<uint8_t> frame;
    bool esc = false;
    while (std::chrono::steady_clock::now() < until) {
      uint8_t buf[256];
      const long n = port_.Read(buf, sizeof buf);
      if (n < 0) { *error = "the dongle went away"; return false; }
      for (long i = 0; i < n; ++i) {
        const uint8_t b = buf[i];
        if (b == 0xC0) {
          if (frame.size() >= 3 && frame[0] == kResponse && frame[1] == msg[0]) {
            if (frame[2] != 0x01) {
              *error = "the bootloader said: " + ResultText(frame[2], frame.size() > 3 ? frame[3] : 0);
              return false;
            }
            if (payload) payload->assign(frame.begin() + 3, frame.end());
            return true;
          }
          frame.clear();
          esc = false;
        } else if (esc) {
          frame.push_back(b == 0xDC ? 0xC0 : b == 0xDD ? 0xDB : b);
          esc = false;
        } else if (b == 0xDB) {
          esc = true;
        } else {
          frame.push_back(b);
        }
      }
    }
    *error = "the bootloader didn't answer";
    return false;
  }

  // Sends one object (init packet: type 1, firmware: type 2) in pieces of at most maxObject bytes.
  bool SendObject(uint8_t type, const std::vector<uint8_t>& data, size_t chunk, const std::function<void(float)>& progress,
                  std::string* error) {
    std::vector<uint8_t> p;
    if (!Request({kSelect, type}, &p, 2.0, error)) return false;
    if (p.size() < 12) { *error = "short select reply"; return false; }
    const uint32_t maxObject = Get32(p.data());
    if (!maxObject) { *error = "the bootloader reported no room"; return false; }
    uint32_t crc = 0;
    for (size_t off = 0; off < data.size(); off += maxObject) {
      const size_t size = std::min<size_t>(maxObject, data.size() - off);
      std::vector<uint8_t> create{kCreate, type};
      Put32(create, uint32_t(size));
      if (!Request(create, nullptr, 10.0, error)) return false;  // may erase flash first
      for (size_t k = 0; k < size; k += chunk) {
        std::vector<uint8_t> w{kWrite};
        const size_t n = std::min(chunk, size - k);
        w.insert(w.end(), data.begin() + long(off + k), data.begin() + long(off + k + n));
        if (!Send(w)) { *error = "the dongle went away"; return false; }
      }
      crc = Crc32(data.data() + off, size, crc);
      if (!Request({kCrc}, &p, 5.0, error)) return false;
      if (p.size() < 8 || Get32(p.data()) != off + size || Get32(p.data() + 4) != crc) {
        *error = "the transfer got corrupted, try again";
        return false;
      }
      if (!Request({kExecute}, nullptr, 10.0, error)) return false;
      if (progress) progress(float(off + size) / float(data.size()));
    }
    return true;
  }

 private:
  Port& port_;
};

}  // namespace

bool LoadHex(const std::string& path, uint32_t minAddress, uint32_t maxAddress, Image* image, std::string* error) {
  std::ifstream in(path);
  if (!in) { *error = "can't read " + path; return false; }
  std::map<uint32_t, uint8_t> bytes;
  uint32_t upper = 0;
  std::string line;
  int lineNo = 0;
  bool ended = false;
  while (std::getline(in, line) && !ended) {
    ++lineNo;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
    if (line.empty()) continue;
    if (line[0] != ':' || line.size() < 11 || line.size() % 2 == 0) { *error = "bad hex line " + std::to_string(lineNo); return false; }
    std::vector<uint8_t> r;
    for (size_t i = 1; i + 1 < line.size(); i += 2) {
      const int hi = Nibble(line[i]), lo = Nibble(line[i + 1]);
      if (hi < 0 || lo < 0) { *error = "bad hex line " + std::to_string(lineNo); return false; }
      r.push_back(uint8_t(hi << 4 | lo));
    }
    uint8_t sum = 0;
    for (uint8_t b : r) sum = uint8_t(sum + b);
    if (sum != 0 || r.size() != size_t(r[0]) + 5) { *error = "bad hex checksum on line " + std::to_string(lineNo); return false; }
    const uint32_t offset = uint32_t(r[1]) << 8 | r[2];
    const uint8_t* d = r.data() + 4;
    switch (r[3]) {
      case 0x00:
        for (int i = 0; i < r[0]; ++i) bytes[upper + offset + uint32_t(i)] = d[i];
        break;
      case 0x01: ended = true; break;
      case 0x02: upper = (uint32_t(d[0]) << 8 | d[1]) << 4; break;
      case 0x04: upper = (uint32_t(d[0]) << 8 | d[1]) << 16; break;
      default: break;  // start addresses
    }
  }
  if (bytes.empty()) { *error = "the hex file is empty"; return false; }
  const uint32_t lo = bytes.begin()->first, hi = bytes.rbegin()->first + 1;
  if (lo < minAddress || hi > maxAddress) {
    char text[128];
    std::snprintf(text, sizeof text, "the hex file covers 0x%X-0x%X, outside 0x%X-0x%X", lo, hi, minAddress, maxAddress);
    *error = text;
    return false;
  }
  image->base = lo;
  image->data.assign((hi - lo + 3) & ~3u, 0xFF);  // whole words: flash is written in words
  for (const auto& [a, b] : bytes) image->data[a - lo] = b;
  return true;
}

std::vector<std::string> FindBootloaderPorts() {
  std::vector<std::string> out;
#ifdef _WIN32
  static const GUID kPorts = {0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};
  HDEVINFO set = SetupDiGetClassDevsW(&kPorts, nullptr, nullptr, DIGCF_PRESENT);
  if (set == INVALID_HANDLE_VALUE) return out;
  char want[32];
  std::snprintf(want, sizeof want, "VID_%04X&PID_%04X", kVid, kPid);
  SP_DEVINFO_DATA info{};
  info.cbSize = sizeof info;
  for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &info); ++i) {
    char ids[1024] = {};
    if (!SetupDiGetDeviceRegistryPropertyA(set, &info, SPDRP_HARDWAREID, nullptr, reinterpret_cast<BYTE*>(ids), sizeof ids - 2, nullptr))
      continue;
    bool match = false;
    for (const char* s = ids; *s; s += std::strlen(s) + 1) {
      std::string id = s;
      for (auto& c : id) c = char(std::toupper(static_cast<unsigned char>(c)));
      if (id.find(want) != std::string::npos) match = true;
    }
    if (!match) continue;
    HKEY key = SetupDiOpenDevRegKey(set, &info, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
    if (key == INVALID_HANDLE_VALUE) continue;
    char name[64] = {};
    DWORD size = sizeof name - 1, type = 0;
    if (RegQueryValueExA(key, "PortName", nullptr, &type, reinterpret_cast<BYTE*>(name), &size) == ERROR_SUCCESS && type == REG_SZ)
      out.push_back(name);
    RegCloseKey(key);
  }
  SetupDiDestroyDeviceInfoList(set);
#else
  auto readHex = [](const std::string& file) {
    std::ifstream f(file);
    unsigned v = 0;
    f >> std::hex >> v;
    return f ? v : 0u;
  };
  if (DIR* dir = opendir("/sys/class/tty")) {
    while (dirent* e = readdir(dir)) {
      const std::string name = e->d_name;
      if (name.rfind("ttyACM", 0) != 0) continue;
      // device -> the USB interface; its parent holds the IDs
      const std::string usb = "/sys/class/tty/" + name + "/device/../";
      if (readHex(usb + "idVendor") == kVid && readHex(usb + "idProduct") == kPid) out.push_back("/dev/" + name);
    }
    closedir(dir);
  }
  std::sort(out.begin(), out.end());
#endif
  return out;
}

bool Flash(const std::string& portName, const Image& image, const std::function<void(float)>& progress, std::string* error) {
  Port port;
  if (!port.Open(portName, error)) return false;
  Dfu dfu(port);
  // The first bytes after the port opens can get lost: ping until it answers.
  bool alive = false;
  for (int i = 0; i < 6 && !alive; ++i) alive = dfu.Request({kPing, uint8_t(i + 1)}, nullptr, 0.5, error);
  if (!alive) { *error = "the bootloader on " + portName + " didn't answer"; return false; }
  error->clear();
  if (!dfu.Request({kSetPrn, 0, 0}, nullptr, 2.0, error)) return false;
  std::vector<uint8_t> p;
  if (!dfu.Request({kMtu}, &p, 2.0, error)) return false;
  const size_t mtu = p.size() >= 2 ? size_t(p[0] | p[1] << 8) : 64;
  const size_t chunk = std::max<size_t>(16, (mtu - 1) / 2 - 1);  // room for SLIP escaping and the opcode
  if (!dfu.SendObject(1, InitPacket(image.data), chunk, nullptr, error)) return false;
  return dfu.SendObject(2, image.data, chunk, progress, error);
}

}  // namespace lh::dfu
