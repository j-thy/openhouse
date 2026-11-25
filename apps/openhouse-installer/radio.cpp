#include "radio.hpp"
#include "common/dongle_bank.hpp"

#include <hidapi/hidapi.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <thread>

namespace openhouse::installer {
namespace {

using Clock = std::chrono::steady_clock;
double Now() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
void Sleep(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

constexpr uint16_t kValve = 0x28de;
constexpr uint16_t kWired2 = 0x2300, kTracker10 = 0x2022, kWand = 0x2012, kWatchman = 0x2101, kWatchmanV2 = 0x2102;
// The Vive wand's LPC bootloader (its application is kWand): where its radio is updated from.
constexpr uint16_t kWandBootloader = 0x2011;
constexpr uint32_t kWandHwid = 0x81010500;
constexpr const char* kWandBootloaderSerial = "(controller in bootloader)";

bool HidPresent(uint16_t vid, uint16_t pid) {
  hid_device_info* list = hid_enumerate(vid, pid);
  const bool any = list != nullptr;
  hid_free_enumeration(list);
  return any;
}
constexpr uint16_t kDoormanVid = 0x6767, kDoormanPid = 0x0727;

std::string Narrow(const wchar_t* w) {
  std::string s;
  if (w) for (; *w; ++w) s += *w < 0x80 ? char(*w) : '?';
  return s;
}

uint32_t Rd32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }

// The device's hardware id: feature report 0x05, bytes 41-44 with the report id at [0]. A
// receiver relays the request over the radio, so early reads can fail: retried, as stock does.
uint32_t HardwareId(hid_device* dev) {
  for (int attempt = 0; attempt < 40; ++attempt) {
    unsigned char b[65] = {0x05};
    const int n = hid_get_feature_report(dev, b, sizeof b);
    if (n >= 45) {
      const uint32_t build = Rd32(b + 1), hwid = Rd32(b + 41);
      return build && hwid ? hwid : 0;
    }
    Sleep(10);
  }
  return 0;
}

// A Watchman dongle's device: feature 0xff [0x83], then a TLV list of [type][u32]: type 0 is the
// device's serial, 4 its VRC version, 5 its radio firmware version (as SteamVR's log prints them).
uint32_t ConnectedSerial(hid_device* dev, uint32_t* radio) {
  *radio = 0;
  unsigned char req[64] = {0xff, 0x83};
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (hid_send_feature_report(dev, req, sizeof req) < 0) { Sleep(5); continue; }
    for (int k = 0; k < 10; ++k) {
      unsigned char buf[64] = {0xff};
      const int n = hid_get_feature_report(dev, buf, sizeof buf);
      if (n >= 3 && buf[1] == 0x83 && buf[2] != 0) {
        uint32_t serial = 0;
        for (int i = 0; i < buf[2] / 5 && 3 + 5 * i + 5 <= n; ++i) {
          if (buf[3 + 5 * i] == 0) serial = Rd32(buf + 4 + 5 * i);
          if (buf[3 + 5 * i] == 5) *radio = Rd32(buf + 4 + 5 * i);
        }
        return serial;
      }
      Sleep(5);
    }
  }
  return 0;
}

std::string LhrSerial(uint32_t v) {
  char s[16];
  std::snprintf(s, sizeof s, "LHR-%08X", v);
  return s;
}

// ---- the Doorman Dongle's HID protocol (ai/doorman/protocol.md) ----
class DoormanDongle {
 public:
  explicit DoormanDongle(const std::string& path) {
    dev0_ = hid_open_path(path.c_str());
    if (!dev0_) return;
    devs_[0] = dev0_;
    // The other interfaces carry the slots' streams (slot s on interface s % 4).
    if (hid_device_info* list = hid_enumerate(kDoormanVid, kDoormanPid)) {
      wchar_t serial[64]{};
      hid_get_serial_number_string(dev0_, serial, 64);
      for (hid_device_info* d = list; d; d = d->next)
        if (d->interface_number > 0 && d->interface_number < kInterfaces && !devs_[d->interface_number] && d->serial_number &&
            std::wcscmp(d->serial_number, serial) == 0)
          devs_[d->interface_number] = hid_open_path(d->path);
      hid_free_enumeration(list);
    }
  }
  ~DoormanDongle() {
    for (hid_device* d : devs_) if (d) hid_close(d);
  }
  bool Ok() const { return dev0_ != nullptr; }

  bool Command(uint8_t cmd, const uint8_t* args, size_t n, uint8_t* reply = nullptr, int timeoutMs = 500) {
    seq_ = uint8_t(0x80 | (seq_ + 1));   // 128..255: the driver uses 1..127 (it sees our replies too)
    const uint8_t seq = seq_;
    uint8_t out[65] = {0, cmd, seq};
    std::memcpy(out + 3, args, std::min<size_t>(n, 62));
    if (hid_write(dev0_, out, sizeof out) < 0) return false;
    const double until = Now() + timeoutMs / 1000.0;
    while (Now() < until && !ScanStopping()) {
      int status = -1;
      if (!Pump(5, [&](const uint8_t* r) {
            if (r[0] != cmd || r[1] != seq) return false;
            if (reply) std::memcpy(reply, r + 2, 62);
            status = r[2];
            return true;
          }))
        return false;
      if (status >= 0) return status == 0;
    }
    return false;
  }
  bool Send(int slot, const uint8_t* frame, size_t n) {
    uint8_t args[62] = {uint8_t(slot), uint8_t(n)};
    std::memcpy(args + 2, frame, std::min<size_t>(n, 60));
    return Command(0x21, args, n + 2);
  }
  // The next frame from `slot` that `want` accepts, or empty after the timeout.
  std::string Await(int slot, const std::function<bool(const std::string&)>& want, double seconds) {
    const double until = Now() + seconds;
    while (Now() < until && !ScanStopping()) {
      for (auto it = frames_.begin(); it != frames_.end(); ++it)
        if (it->first == slot && want(it->second)) {
          std::string f = it->second;
          frames_.erase(it);
          return f;
        }
      frames_.clear();
      if (!Pump(10, nullptr)) return {};
    }
    return {};
  }

 private:
  // Reads everything every interface has queued (each carries some slots' stream: reading only one
  // of its reports per wait let it pile up, and those slots' replies arrived seconds late),
  // then waits up to `waitMs` on interface 0 if nothing was there. A command reply goes to `reply`
  // (true: taken); stream reports are parsed into frames_. False on a read error.
  bool Pump(int waitMs, const std::function<bool(const uint8_t*)>& reply) {
    uint8_t r[64];
    bool any = false;
    for (int round = 0; round < 512; ++round) {
      bool got = false;
      for (hid_device* dev : devs_) {
        if (!dev) continue;
        const int n = hid_read_timeout(dev, r, sizeof r, 0);
        if (n < 0) return false;
        if (n == 0) continue;
        got = any = true;
        if (r[0] >= 0x80 && r[0] <= 0x82) Stream(r, n);
        else if (reply && n >= 3) reply(r);
      }
      if (!got) break;
    }
    if (any) return true;
    const int n = hid_read_timeout(dev0_, r, sizeof r, waitMs);
    if (n < 0) return false;
    if (n > 0) {
      if (r[0] >= 0x80 && r[0] <= 0x82) Stream(r, n);
      else if (reply && n >= 3) reply(r);
    }
    return true;
  }

  void Stream(const uint8_t* r, int n) {
    if (n < 3) return;
    if (r[0] == 0x82) {
      // Packed (cmd 0x37, which the driver turns on for the whole dongle, and it stays on after the
      // driver closes): [len][frame] items back to back across reports. A seq gap loses what was
      // being put together; reading starts again at an item.
      if (n < 5) return;
      const int slot = r[1], len = std::min<int>(r[2], n - 5), seq = r[3], first = r[4];
      std::string& b = packed_[slot];
      int from = 0;
      const auto at = packSeq_.find(slot);
      if (at == packSeq_.end() || at->second != seq) {
        b.clear();
        if (first == 0xFF || first > len) { packSeq_.erase(slot); return; }
        from = first;
      }
      packSeq_[slot] = (seq + 1) & 0xFF;
      b.append(reinterpret_cast<const char*>(r + 5 + from), size_t(len - from));
      size_t i = 0;
      while (i < b.size() && i + 1 + uint8_t(b[i]) <= b.size()) {
        if (!b[i]) { b.clear(); packSeq_.erase(slot); return; }
        frames_.emplace_back(slot, b.substr(i + 1, uint8_t(b[i])));
        i += 1 + uint8_t(b[i]);
      }
      b.erase(0, i);
      return;
    }
    if (r[0] == 0x81) {  // a frame too big for one report, in pieces
      std::string& b = frag_[r[1]];
      b.append(reinterpret_cast<const char*>(r + 3), std::min<int>(r[2], n - 3));
      if (!b.empty() && b.size() >= 1u + uint8_t(b[0])) {
        frames_.emplace_back(r[1], b.substr(1, uint8_t(b[0])));
        frag_.erase(r[1]);
      }
      return;
    }
    if (r[0] != 0x80) return;
    frag_.erase(r[1]);
    const int len = std::min<int>(r[2], n - 3);
    for (int i = 0; i < len && i + 1 + r[3 + i] <= len;) {
      const int m = r[3 + i];
      if (m) frames_.emplace_back(r[1], std::string(reinterpret_cast<const char*>(r + 4 + i), m));
      i += 1 + m;
    }
  }
  static constexpr int kInterfaces = 4;   // interface 0: commands; slot s streams on interface s % 4
  hid_device* dev0_ = nullptr;
  hid_device* devs_[kInterfaces] = {};
  uint8_t seq_ = 0;
  std::vector<std::pair<int, std::string>> frames_;
  std::map<int, std::string> frag_, packed_;
  std::map<int, int> packSeq_;
};

// The air updater (radio-fw main.c air_update): downlink op 0xF0, then 'X' info, 'Y' open,
// 'Z' [offset u32][block], '[' [signature] commit.
std::string AirRequest(DoormanDongle& d, int slot, char sub, const std::string& payload,
                       const std::function<bool(const std::string&)>& want = nullptr) {
  std::string f = std::string("\xF0", 1) + sub + payload;
  for (int attempt = 0; attempt < 4 && !ScanStopping(); ++attempt) {
    if (!d.Send(slot, reinterpret_cast<const uint8_t*>(f.data()), f.size())) { Sleep(20); continue; }
    const std::string r = d.Await(slot, [&](const std::string& fr) {
      return fr.size() >= 3 && uint8_t(fr[0]) == 0xF0 && fr[1] == sub && (!want || want(fr));
    }, 1.5);
    if (!r.empty()) return r;
  }
  return {};
}

// The radio updater's check of what it staged (radio_image.py radio_signature): a 4-word LFSR
// over the image padded with 0xFF to 16 bytes.
std::string Signature(const std::string& img) {
  const size_t n = (img.size() + 15) & ~size_t(15);
  std::string data = img;
  data.resize(n, '\xFF');
  uint32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0;
  for (size_t o = 0; o < n; o += 16) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data.data()) + o;
    const uint32_t w0 = Rd32(p), w1 = Rd32(p + 4), w2 = Rd32(p + 8), w3 = Rd32(p + 12);
    const uint32_t top = s1 << 31;
    const uint32_t s1n = (s1 >> 1) ^ (s2 << 31) ^ w1;
    const uint32_t s2n = w2 ^ ((s2 >> 1) | (s3 << 31));
    const uint32_t t = s0 ^ (w3 >> 31);
    const uint32_t s3n = ((w3 & 0x7FFFFFFFu) ^ (s3 >> 1)) | (((t ^ (s0 >> 2) ^ (s0 >> 27) ^ (s0 >> 29)) & 1u) << 31);
    s0 = ((s0 >> 1) | top) ^ w0;
    s1 = s1n, s2 = s2n, s3 = s3n;
  }
  std::string sig(16, '\0');
  for (int i = 0; i < 4; ++i) {
    const uint32_t v = i == 0 ? s0 : i == 1 ? s1 : i == 2 ? s2 : s3;
    for (int b = 0; b < 4; ++b) sig[size_t(4 * i + b)] = char(v >> (8 * b));
  }
  return sig;
}

bool FlashDoorman(const WirelessDevice& device, const RadioImage& image, const Progress& progress, std::string* error) {
  DoormanDongle d(device.donglePath);
  if (!d.Ok()) { *error = "couldn't open the Doorman Dongle (close SteamVR and doorman-dongle-manager)"; return false; }
  const int slot = device.slot;
  progress(0, "Opening the radio's update area");
  std::string r = AirRequest(d, slot, 'Y', "");
  if (r.empty() || r[2]) { *error = "the device didn't answer (is it on and connected?)"; return false; }
  std::string pad = image.data;
  pad.resize((pad.size() + 3) & ~size_t(3), '\xFF');
  constexpr size_t kBlock = 48;
  for (size_t off = 0; off < pad.size(); off += kBlock) {
    const std::string block = pad.substr(off, kBlock);
    std::string payload(4, '\0');
    for (int b = 0; b < 4; ++b) payload[size_t(b)] = char(off >> (8 * b));
    const uint32_t end = uint32_t(off + block.size());
    r = AirRequest(d, slot, 'Z', payload + block, [&](const std::string& fr) {
      return fr.size() >= 7 && (fr[2] != 0 || Rd32(reinterpret_cast<const uint8_t*>(fr.data()) + 3) == end);
    });
    if (r.empty() || r[2]) { *error = "sending the firmware failed at byte " + std::to_string(off) + " (the device's signal may be weak)"; return false; }
    progress(float(off + block.size()) / float(pad.size()), "Sending the firmware");
  }
  progress(1, "Checking it on the device");
  r = AirRequest(d, slot, '[', Signature(image.data));
  if (r.empty() || r[2]) { *error = "the device's check of the firmware failed; its current radio firmware stays"; return false; }
  progress(1, "Turning the device off");
  const uint8_t off[] = {'P', 0x9f, 4, 'o', 'f', 'f', '!'};
  d.Send(slot, off, sizeof off);
  Sleep(500);
  return true;
}

// Through a Valve/HTC Watchman dongle (index_radio_ota.py): feature 0xff with 0x97 open,
// 0x98 [n][block] write, 0x99 [signature] commit, then 0x94 status.
bool FlashWatchman(const WirelessDevice& device, const RadioImage& image, const Progress& progress, std::string* error) {
  hid_device* dev = hid_open_path(device.donglePath.c_str());
  if (!dev) { *error = "couldn't open the dongle (close SteamVR first)"; return false; }
  auto cmd = [&](uint8_t op, const std::string& data) {
    unsigned char pkt[64] = {0xff, op, uint8_t(data.size())};
    std::memcpy(pkt + 3, data.data(), std::min<size_t>(data.size(), 61));
    // The HTC dongle relays these but doesn't complete the request (the radio sends no
    // reply): a failure here isn't one.
    hid_send_feature_report(dev, pkt, sizeof pkt);
  };
  progress(0, "Opening the radio's update area");
  cmd(0x97, {});
  Sleep(300);
  std::string pad = image.data;
  pad.resize((pad.size() + 3) & ~size_t(3), '\xFF');
  constexpr size_t kBlock = 48;
  for (size_t off = 0; off < pad.size(); off += kBlock) {
    cmd(0x98, pad.substr(off, kBlock));
    Sleep((off + kBlock) % 4096 < kBlock ? 150 : 10);  // the radio erases a 4 KiB page
    progress(float(off + kBlock) / float(pad.size()), "Sending the firmware");
  }
  progress(1, "Checking it on the device");
  cmd(0x99, Signature(image.data));
  bool ok = false;
  for (int i = 0; i < 20 && !ok; ++i) {
    Sleep(200);
    unsigned char st[64] = {0xff};
    if (hid_get_feature_report(dev, st, sizeof st) >= 3 && st[1] == 0x94 && st[2] == 0x02) ok = true;
  }
  if (!ok) {
    hid_close(dev);
    *error = "the device didn't confirm the firmware; its current radio firmware stays";
    return false;
  }
  progress(1, "Turning the device off");
  cmd(0x9f, "off!");
  Sleep(300);
  hid_close(dev);
  return true;
}

}  // namespace

// Index controllers with the retail radio (nRF52, 1562882729): every 0x11..0009 revision, and the
// EV2 dev kits (0x1108/0x1109..06, the same radio). Not EV1.3 (0x1105..), which has an older radio.
static bool IndexControllerRadio(uint32_t hwid) {
  if ((hwid >> 24) != 0x11) return false;
  if ((hwid & 0xff) == 0x09) return true;
  return (hwid & 0xff) == 0x06 && ((hwid >> 16) & 0xff) >= 0x08;
}

// Tundra Trackers: SteamVR lists 0xf0000109; later revisions (0xf1040009, "rev 4.0.9") report other
// ids. Their radio is a variant of Valve's Index controller radio build (same image layout and entry).
static bool TundraTracker(uint32_t hwid) { return (hwid >> 28) == 0xf && (hwid & 0xff) == 0x09; }

RadioModel ModelFor(uint32_t hwid) {
  struct Known { uint32_t hwid; const char* name; RadioFamily family; };
  static const Known kKnown[] = {
      {0x110f0009, "Valve Index Controller", RadioFamily::Newer},
      {0x110b0209, "Valve Index Controller", RadioFamily::Newer},
      {0x84020006, "VIVE Tracker (2018)", RadioFamily::Newer},
      {0x84020109, "VIVE Tracker 3.0", RadioFamily::Newer},
      {0x81020006, "VIVE Pro Controller", RadioFamily::Newer},
      {0xf0000109, "Tundra Tracker", RadioFamily::Newer},
      {0x84010004, "VIVE Tracker 1.0", RadioFamily::Tracker10},
      {0x81010500, "VIVE Controller", RadioFamily::Tracker10},
  };
  for (const auto& k : kKnown)
    if (k.hwid == hwid) return {hwid, k.name, k.family, true};
  // SteamVR lists several Index controller revisions without a radio file of their own: they take
  // the same image.
  if (TundraTracker(hwid)) return {hwid, "Tundra Tracker", RadioFamily::Newer, true};
  if (IndexControllerRadio(hwid))
    return {hwid, (hwid & 0xff) == 0x06 ? "Valve Index Controller EV2" : "Valve Index Controller", RadioFamily::Newer, true};
  // HTC's later trackers (3.0) aren't in SteamVR's manifests under their own id: an HTC
  // tracker (0x84) of the second generation or later has the same nRF52 radio.
  if ((hwid >> 24) == 0x84 && ((hwid >> 16) & 0xff) >= 0x02) return {hwid, "VIVE Tracker", RadioFamily::Newer, true};
  // Headsets and base stations: never flashed here.
  const uint32_t top = hwid >> 24;
  if (top == 0x01 || top == 0x80 || top == 0x85 || top == 0x22 || top == 0x09) return {hwid, "Headset", RadioFamily::Unsupported, true};
  // Never seen: a guess at its radio, for a flash the user accepts at their own risk. HTC's first
  // generation (0x81/0x84 with generation 1) has the nRF51 radio; everything newer the nRF52.
  const uint32_t gen = (hwid >> 16) & 0xff;
  const bool older = (top == 0x81 || top == 0x84) && gen == 0x01;
  return {hwid, "Unknown device", older ? RadioFamily::Tracker10 : RadioFamily::Newer, false};
}

fs::path FirmwareFolder() { return ExeDirectory() / "firmwares"; }

fs::path OurRadioImage(RadioFamily family) {
  if (family == RadioFamily::Newer) return FirmwareFolder() / "radio" / "doorman-radio-wm3.fw";
  if (family == RadioFamily::Tracker10) return FirmwareFolder() / "radio" / "doorman-radio-neo.bin";
  return {};
}

fs::path StockRadioImageExact(const fs::path& steamvr, uint32_t hwid);

fs::path StockRadioImage(const fs::path& steamvr, uint32_t hwid) {
  fs::path f = StockRadioImageExact(steamvr, hwid);
  // An Index controller revision SteamVR has no radio file for: the Index controller's.
  if (f.empty() && IndexControllerRadio(hwid)) f = StockRadioImageExact(steamvr, 0x110f0009);
  // A Tundra revision SteamVR has no entry for: Tundra's own radio file.
  if (f.empty() && TundraTracker(hwid)) f = StockRadioImageExact(steamvr, 0xf0000109);
  // HTC's 3.0 tracker (0x84020109) isn't in SteamVR's manifests, but it runs the Index controller's
  // radio build (1562882729), so it gets that file.
  if (f.empty() && (hwid >> 24) == 0x84 && ((hwid >> 16) & 0xff) >= 0x02) f = StockRadioImageExact(steamvr, 0x110f0009);
  return f;
}

fs::path StockRadioImageExact(const fs::path& steamvr, uint32_t hwid) {
  char key[16];
  std::snprintf(key, sizeof key, "0x%08x", hwid);
  std::error_code ec;
  for (const auto& driver : fs::directory_iterator(steamvr / "drivers", ec)) {
    std::string text;
    if (!ReadFile(driver.path() / "resources" / "firmware" / "manifest.vrfirmware", &text)) continue;
    const nlohmann::json j = nlohmann::json::parse(text, nullptr, false);
    if (!j.is_object()) continue;
    for (const auto& [k, v] : j.items()) {
      std::string lower = k;
      for (auto& c : lower) c = char(std::tolower(static_cast<unsigned char>(c)));
      if (!v.is_object() || lower != key) continue;
      const std::string bin = v.value("RadioBin", "");
      if (bin.empty()) continue;
      // "{indexcontroller}/firmware/..." is that driver's resources; "tools/..." is SteamVR's.
      if (bin[0] == '{') {
        const size_t close = bin.find('}');
        const fs::path f = steamvr / "drivers" / bin.substr(1, close - 1) / "resources" / Path(bin.substr(close + 2));
        return fs::exists(f, ec) ? f : fs::path();
      }
      const fs::path f = steamvr / Path(bin);
      return fs::exists(f, ec) ? f : fs::path();
    }
  }
  return {};
}

bool IsDoormanRadio(uint32_t version) { return (version >> 24) == 0x6A || (version >> 24) == 0x0A; }

uint32_t OurRadioVersion(RadioFamily family) {
  RadioImage image;
  std::string error;
  return LoadRadioImage(OurRadioImage(family), &image, &error) ? image.version : 0;
}

std::vector<StockModel> StockModels(const fs::path& steamvr) {
  std::vector<StockModel> out;
  std::error_code ec;
  for (const auto& driver : fs::directory_iterator(steamvr / "drivers", ec)) {
    std::string text;
    if (!ReadFile(driver.path() / "resources" / "firmware" / "manifest.vrfirmware", &text)) continue;
    const nlohmann::json j = nlohmann::json::parse(text, nullptr, false);
    if (!j.is_object()) continue;
    for (const auto& [k, v] : j.items()) {
      if (!v.is_object() || !v.contains("RadioBin")) continue;
      const uint32_t hwid = uint32_t(std::strtoul(k.c_str(), nullptr, 16));
      const RadioFamily family = ModelFor(hwid).family;
      if (family == RadioFamily::Unsupported) continue;
      StockModel m{hwid, v.value("Name", k), family, StockRadioImage(steamvr, hwid)};
      if (!m.image.empty()) out.push_back(m);
    }
  }
  // Not in SteamVR's manifests: the 3.0 tracker (the Index controller's radio file).
  if (const fs::path f = StockRadioImage(steamvr, 0x84020109); !f.empty()) out.push_back({0x84020109, "VIVE Tracker 3.0", RadioFamily::Newer, f});
  std::sort(out.begin(), out.end(), [](const StockModel& a, const StockModel& b) { return a.name < b.name; });
  return out;
}

bool LoadRadioImage(const fs::path& file, RadioImage* image, std::string* error) {
  std::string m;
  if (!ReadFile(file, &m) || m.size() < 0x40) { *error = "the firmware file is missing (" + Utf8(file.filename()) + ")"; return false; }
  const uint8_t* p = reinterpret_cast<const uint8_t*>(m.data());
  const uint32_t sp = Rd32(p), reset = Rd32(p + 4);
  // nRF52 radios (Valve): app at 0x1000, length at +0x1C, version at +0x24; the nRF51 (HTC
  // Tracker 1.0): app at 0x400, length at +0x10, version at +0x18.
  struct Layout { RadioFamily family; uint32_t base, length, version, ramEnd; };
  for (const Layout& l : {Layout{RadioFamily::Newer, 0x1000, 0x1C, 0x24, 0x20010000},
                          Layout{RadioFamily::Tracker10, 0x400, 0x10, 0x18, 0x20004000}}) {
    const uint32_t n = Rd32(p + l.length);
    if (!(l.base < n && n <= m.size() && n <= 0xF800 && l.base < reset && reset < l.base + n && 0x20000000 < sp && sp <= l.ramEnd)) continue;
    image->data = m.substr(0, n);
    image->family = l.family;
    image->version = Rd32(p + l.version);
    return true;
  }
  *error = Utf8(file.filename()) + " isn't a radio firmware image";
  return false;
}

std::vector<WiredDevice> FindWiredDevices() {
  std::vector<WiredDevice> out;
  hid_device_info* list = hid_enumerate(kValve, 0);
  for (hid_device_info* d = list; d; d = d->next) {
    if (d->product_id == kWandBootloader) {
      // A wand left in its bootloader (an update that didn't finish): it has no serial and no
      // hardware id there, but it can still be flashed, or restored.
      if (std::none_of(out.begin(), out.end(), [](const WiredDevice& o) { return o.pid == kWandBootloader; })) {
        WiredDevice w;
        w.serial = kWandBootloaderSerial;
        w.pid = kWandBootloader;
        w.model = ModelFor(kWandHwid);
        out.push_back(w);
      }
      continue;
    }
    if (d->product_id != kWired2 && d->product_id != kTracker10 && d->product_id != kWand) continue;
    if (d->interface_number > 0) continue;  // one interface per device
    WiredDevice w;
    w.serial = Narrow(d->serial_number);
    w.pid = d->product_id;
    if (std::any_of(out.begin(), out.end(), [&](const WiredDevice& o) { return o.serial == w.serial; })) continue;
    if (hid_device* dev = hid_open_path(d->path)) {
      w.model = ModelFor(HardwareId(dev));
      hid_close(dev);
    }
    if (w.model.hwid == 0) continue;  // couldn't read it (in use?)
    // An unknown device's radio, by how it's plugged in: the nRF51 devices have their own USB ids.
    if (!w.model.known) w.model.family = w.pid == kWired2 ? RadioFamily::Newer : RadioFamily::Tracker10;
    // A headset (Index, Beyond...) is on 28de:2300 too: never listed.
    if (w.model.family == RadioFamily::Unsupported && w.model.known) continue;
    out.push_back(w);
  }
  hid_free_enumeration(list);
  return out;
}

fs::path WatchmanUpdateTool(const fs::path& steamvr) {
#ifdef _WIN32
  return steamvr / "tools" / "lighthouse" / "bin" / "win32" / "lighthouse_watchman_update.exe";
#else
  return steamvr / "tools" / "lighthouse" / "bin" / "linux64" / "lighthouse_watchman_update";
#endif
}

bool FlashWired(const fs::path& steamvr, const WiredDevice& device, const fs::path& image, const Progress& progress, std::string* error) {
  const fs::path tool = WatchmanUpdateTool(steamvr);
  std::error_code ec;
  if (!fs::exists(tool, ec)) { *error = "SteamVR's firmware updater isn't there (" + Utf8(tool) + ")"; return false; }
  std::vector<std::string> args{Utf8(tool)};
  if (device.model.family == RadioFamily::Newer) {
    args.insert(args.end(), {"-s", device.serial, "--target=nrf52", Utf8(image)});
  } else if (device.model.hwid == kWandHwid) {
    // The wand: "-r" radio, "n" its NEO VRC (28de:2012; "v" is 28de:2010, not a wand). The tool
    // restarts it into its LPC bootloader (28de:2011), which relays the radio update. In one
    // go it looks for the bootloader at once, before Windows has it (the first time it even sets
    // up its driver) and fails with "unable to open device"; so the wand is restarted into the
    // bootloader first ("-bn") and waited for, and then the tool finds it there already. Its
    // type (controller or tracker) is in its config, which none of this touches.
    if (!HidPresent(kValve, kWandBootloader)) {
      for (int attempt = 0; attempt < 2 && !HidPresent(kValve, kWandBootloader); ++attempt) {
        progress(0, "Restarting the controller into its bootloader");
        RunProcess({Utf8(tool), "-bn"}, [&](const std::string& line) { progress(0, line); });
        for (int i = 0; i < 100 && !HidPresent(kValve, kWandBootloader); ++i) Sleep(100);
      }
      if (!HidPresent(kValve, kWandBootloader)) {
        *error = "the controller didn't restart into its bootloader; unplug it, plug it back in and try again";
        return false;
      }
      Sleep(1000);  // let Windows finish setting it up
    }
    args.insert(args.end(), {"-rn", Utf8(image)});
  } else if (device.model.family == RadioFamily::Tracker10) {
    // The Tracker 1.0's radio: "-r" radio, "n"+"t" device type 6. The tool opens it by USB id,
    // so only one may be plugged in (the installer checks).
    args.insert(args.end(), {"-rnt", Utf8(image)});
  } else {
    *error = "this device's radio can't be flashed here";
    return false;
  }
  progress(0, "Starting SteamVR's firmware updater");
  std::string last;
  float fraction = 0;
  const int rc = RunProcess(args, [&](const std::string& line) {
    last = line;
    // It prints a percentage as it goes ("... 42%"): shown when it does.
    const size_t pct = line.rfind('%');
    if (pct != std::string::npos && pct > 0) {
      size_t start = pct;
      while (start > 0 && (std::isdigit(static_cast<unsigned char>(line[start - 1])) || line[start - 1] == '.')) --start;
      if (start < pct) fraction = std::clamp(float(std::atof(line.substr(start, pct - start).c_str())) / 100.f, 0.f, 1.f);
    }
    progress(fraction, line);
  });
  if (rc != 0) {
    *error = rc < 0 ? "couldn't start SteamVR's firmware updater" : "SteamVR's firmware updater failed: " + last;
    return false;
  }
  progress(1, "Done");
  return true;
}

std::vector<WirelessDevice> FindWirelessDevices() {
  std::vector<WirelessDevice> out;
  // Valve/HTC Watchman dongles, and the device each is connected to.
  hid_device_info* list = hid_enumerate(kValve, 0);
  for (hid_device_info* d = list; d; d = d->next) {
    if (d->product_id != kWatchman && d->product_id != kWatchmanV2) continue;
    if (d->interface_number > 0) continue;
    hid_device* dev = hid_open_path(d->path);
    if (!dev) continue;
    uint32_t radio = 0;
    const uint32_t serial = ConnectedSerial(dev, &radio);
    if (serial) {
      WirelessDevice w;
      w.route = Route::Watchman;
      w.dongle = Narrow(d->serial_number);
      w.donglePath = d->path;
      w.serial = LhrSerial(serial);
      w.radioVersion = radio;
      // An unknown device's radio, by the stock radio it runs (SteamVR's nRF51 images vs nRF52 ones).
      if (!w.model.known && radio && !IsDoormanRadio(radio)) {
        static const uint32_t kOlder[] = {1466630404u, 1497924533u, 1532585738u};
        w.model.family = std::find(std::begin(kOlder), std::end(kOlder), radio) != std::end(kOlder) ? RadioFamily::Tracker10 : RadioFamily::Newer;
      }
      w.model = ModelFor(HardwareId(dev));
      if (w.model.hwid) out.push_back(w);
    }
    hid_close(dev);
  }
  hid_free_enumeration(list);
  // Doorman Dongles: the devices in their slots that are connected now, running our radio.
  for (const auto& dongle : FindDoormanDongles())
    for (const auto& s : ScanDongleSlots(dongle))
      if (s.answered) out.push_back(s.device);
  return out;
}

std::vector<DongleSlot> ScanDongleSlots(const DoormanDongleInfo& dongle) {
  const int n = dongle.slots;
  std::vector<DongleSlot> out(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) out[size_t(i)].slot = i;
  DoormanDongle d(dongle.path);
  if (!d.Ok()) return out;
  uint8_t r[62];
  for (int slot = 0; slot < n; ++slot) {
    const uint8_t first = uint8_t(slot / 6 * 6);   // 6 slots per reply
    if (slot == first && !d.Command(0x25, &first, 1, r)) { slot = first + 5; continue; }
    const uint8_t* e = r + 1 + (slot - first) * 10;  // [used][active][8-byte id]
    DongleSlot& s = out[size_t(slot)];
    s.paired = e[0] != 0;
    s.on = e[0] && e[1];
    if (!s.on) continue;
    const std::string info = AirRequest(d, slot, 'X', "");
    if (info.size() < 6) continue;
    WirelessDevice& w = s.device;
    s.answered = true;
    w.route = Route::Doorman;
    w.dongle = dongle.serial;
    w.donglePath = dongle.path;
    w.slot = slot;
    w.radioVersion = Rd32(reinterpret_cast<const uint8_t*>(info.data()) + 2);
    const uint32_t family = w.radioVersion >> 24;
    w.model.family = family == 0x6A ? RadioFamily::Newer : family == 0x0A ? RadioFamily::Tracker10 : RadioFamily::Unsupported;
    w.model.name = family == 0x6A ? "Controller / tracker (2.0)" : family == 0x0A ? "VIVE Tracker 1.0 / VIVE Controller" : "Unknown device";
    w.model.known = family == 0x6A || family == 0x0A;
    w.model.hwid = w.radioVersion;
    // The main MCU behind our radio still answers feature 0x05: its hardware id at [41].
    const uint8_t req[2] = {'P', 0x05};
    if (d.Send(slot, req, 2)) {
      const std::string f = d.Await(slot, [](const std::string& x) { return x.size() >= 45 && uint8_t(x[0]) == 'P'; }, 1.5);
      const uint32_t hwid = f.size() >= 45 ? Rd32(reinterpret_cast<const uint8_t*>(f.data()) + 41) : 0;
      const RadioModel m = hwid ? ModelFor(hwid) : RadioModel{};
      if (m.known && m.family == w.model.family) w.model = m;
    }
    char id[24];
    std::snprintf(id, sizeof id, "slot %d", slot + 1);
    w.serial = id;
  }
  return out;
}

void PowerOffDevice(const WirelessDevice& device) {
  DoormanDongle d(device.donglePath);
  if (!d.Ok()) return;
  const uint8_t off[] = {'P', 0x9f, 4, 'o', 'f', 'f', '!'};
  d.Send(device.slot, off, sizeof off);
  Sleep(500);
}

bool FlashWireless(const WirelessDevice& device, const RadioImage& image, const Progress& progress, std::string* error) {
  if (device.model.family != image.family) { *error = "this firmware is for another kind of device"; return false; }
  return device.route == Route::Doorman ? FlashDoorman(device, image, progress, error) : FlashWatchman(device, image, progress, error);
}

std::vector<DoormanDongleInfo> FindDoormanDongles() {
  std::vector<DoormanDongleInfo> out;
  hid_device_info* list = hid_enumerate(kDoormanVid, kDoormanPid);
  for (hid_device_info* d = list; d; d = d->next) {
    if (d->interface_number > 0) continue;
    DoormanDongleInfo o;
    o.serial = Narrow(d->serial_number);
    o.path = d->path;
    DoormanDongle dev(o.path);
    uint8_t r[62];
    if (dev.Ok() && dev.Command(0x01, nullptr, 0, r) && std::memcmp(r + 1, "OHDG", 4) == 0) {
      o.firmware = uint16_t(r[5] | r[6] << 8);
      o.softDeviceBuild = r[7] != 0;
      o.board = r[58] <= 3 ? r[58] : 0;
      o.bank = r[60] <= 2 ? r[60] : 0;
      o.slots = r[61] >= 1 && r[61] <= 16 ? r[61] : 8;
    }
    out.push_back(o);
  }
  hid_free_enumeration(list);
  return out;
}

bool BankUpdate(const DoormanDongleInfo& d, std::vector<uint8_t> image, const std::function<void(float)>& progress,
                std::string* error) {
  namespace bank = lh::dongle_bank;
  DoormanDongle dev(d.path);
  if (!dev.Ok()) { *error = "couldn't open the dongle, close SteamVR and doorman-dongle-manager"; return false; }
  image.resize((image.size() + 3) & ~size_t(3), 0xFF);
  const uint32_t size = uint32_t(image.size()), crc = bank::Crc32(image);
  uint8_t args[62];
  std::memcpy(args, &size, 4); std::memcpy(args + 4, &crc, 4);
  if (!dev.Command(bank::kBegin, args, 8, nullptr, 15000)) { *error = "the dongle didn't start the update"; return false; }  // erases first
  for (uint32_t off = 0; off < size; off += bank::kChunk) {
    const uint8_t n = uint8_t(std::min<uint32_t>(bank::kChunk, size - off));
    std::memcpy(args, &off, 4); args[4] = n; std::memcpy(args + 5, image.data() + off, n);
    if (!dev.Command(bank::kWrite, args, 5u + n, nullptr, 1000)) { *error = "writing the firmware failed"; return false; }
    if (progress) progress(float(off + n) / float(size));
  }
  if (!dev.Command(bank::kCommit, nullptr, 0, nullptr, 2000)) { *error = "the new firmware didn't check out"; return false; }
  return true;
}

bool RebootToBootloader(const DoormanDongleInfo& d) {
  DoormanDongle dev(d.path);
  if (!dev.Ok()) return false;
  dev.Command(0x10, nullptr, 0, nullptr, 200);  // it resets right away: no reply expected
  return true;
}

}  // namespace openhouse::installer
