#include "doorman_dongle.hpp"

#include <hidapi/hidapi.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>

#include "device_config.hpp"
#include "platform/platform.hpp"
#include "usb_transport.hpp"

namespace lh::usb {
namespace {

constexpr uint16_t kVid = 0x6767, kPid = 0x0727;
// Radio path latency on top of the wired 0.9 ms the IMU parser already subtracts
// (sample -> LPC -> SPI -> air, up to one TDMA cycle -> USB). ClockSync follows the minimum.
constexpr double kExtraLatency = 0.0015;
constexpr double kFeatureTimeout = 0.15;

constexpr uint8_t kCmdRejoin = 0x33, kCmdSetName = 0x34, kCmdGetName = 0x35, kCmdPacked = 0x37, kCmdInfo = 0x01;
constexpr size_t kNameMax = 59;   // what the dongle keeps (link.c DML_NAME_MAX)
constexpr uint8_t kCmdDown = 0x21, kCmdStats = 0x23, kCmdPair = 0x24, kCmdSlots = 0x25, kCmdLive = 0x29, kCmdIdent = 0x2A;

uint32_t Rd32(const uint8_t* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }

std::string Narrow(const wchar_t* w) {
  std::string s;
  for (; w && *w; ++w) s += char(*w < 128 ? *w : '?');
  return s;
}

// "device_serial_number" from a device config JSON, "" if absent.
std::string ConfigSerial(const std::string& json) {
  size_t k = json.find("\"device_serial_number\"");
  if (k == std::string::npos) return "";
  k = json.find(':', k);
  if (k == std::string::npos) return "";
  const size_t a = json.find('"', k), b = a == std::string::npos ? a : json.find('"', a + 1);
  if (b == std::string::npos || b - a > 40) return "";
  return json.substr(a + 1, b - a - 1);
}

void AddSensorType(std::string* json, int type) {
  const size_t brace = json->find('{');
  if (brace == std::string::npos || json->find("\"sensor_type\"") != std::string::npos) return;
  json->insert(brace + 1, "\"sensor_type\":" + std::to_string(type) + ",");
}

}  // namespace

std::vector<DoormanDongleFound> FindDoormanDongles(const hid_device_info* list, std::map<std::string, int>* shared) {
  // Every interface carries the dongle's USB serial: that groups them, whatever order hidapi lists them in.
  std::map<std::string, std::array<std::vector<std::string>, kDoormanInterfaces>> bySerial;
  for (const hid_device_info* d = list; d; d = d->next) {
    if (d->vendor_id != kVid || d->product_id != kPid || !d->path) continue;
    if (Narrow(d->product_string).rfind("Doorman Dongle", 0) != 0) continue;   // "Doorman Dongle <board>"
    if (d->interface_number < 0 || d->interface_number >= kDoormanInterfaces) continue;
    bySerial[Narrow(d->serial_number)][size_t(d->interface_number)].push_back(d->path);
  }
  std::vector<DoormanDongleFound> out;
  for (const auto& [usbSerial, found] : bySerial) {
    const std::string serial = "OHDG-" + usbSerial;
    size_t most = 0, fewest = SIZE_MAX;
    for (const auto& paths : found) { most = std::max(most, paths.size()); fewest = std::min(fewest, paths.size()); }
    if (most > 1) {   // several dongles: whose interfaces are whose?
      if (shared) (*shared)[serial] = int(most);
      continue;
    }
    if (fewest == 0) continue;   // not every interface listed yet
    DoormanDongleFound f{serial, {}};
    for (int i = 0; i < kDoormanInterfaces; ++i) f.paths[size_t(i)] = found[size_t(i)].front();
    out.push_back(std::move(f));
  }
  return out;
}

std::vector<DoormanDongleFound> ScanDoormanDongles(std::map<std::string, int>* shared) {
  hid_device_info* list = hid_enumerate(kVid, kPid);
  auto found = FindDoormanDongles(list, shared);
  hid_free_enumeration(list);
  return found;
}

std::optional<DoormanCarriers::Carrier> DoormanCarriers::Take(const DeviceId& device, const std::string& dongle, int slot) {
  if (Wired(device)) return Carrier{"USB", -1};
  const auto [it, fresh] = carriers_.try_emplace(device, Carrier{dongle, slot});
  if (fresh || (it->second.dongle == dongle && it->second.slot == slot)) return std::nullopt;
  return it->second;
}

void DoormanCarriers::Release(const DeviceId& device, const std::string& dongle, int slot) {
  const auto it = carriers_.find(device);
  if (it != carriers_.end() && it->second.dongle == dongle && it->second.slot == slot) carriers_.erase(it);
}

DoormanDongle::DoormanDongle(std::string serial, IInputSink* sink, ReportDecoder* decoder, RawCaptureWriter* capture,
                             std::atomic<uint16_t>* nextEp, std::string steamConfigDir, LogFn log, DoormanCarriers* carriers)
    : serial_(std::move(serial)), sink_(sink), decoder_(decoder), capture_(capture), nextEp_(nextEp),
      steamConfigDir_(std::move(steamConfigDir)), log_(std::move(log)), carriers_(carriers) {
  if (steamConfigDir_.empty()) steamConfigDir_ = platform::SteamConfigDirectory();
}

DoormanDongle::~DoormanDongle() { Close(); }

namespace {
// One dongle open at a time, as when the reader thread opened them: hid_open_path resets and sets
// hidapi's process-wide error string.
std::mutex g_openMutex;

// Every interface of a dongle, or none (a slot on a missing interface would never be heard); with
// writeHandle, a second handle on interface 0 for the writer thread too.
void OpenInterfaces(const std::array<std::string, kDoormanInterfaces>& paths,
                    std::array<hid_device_*, kDoormanInterfaces>* devs, hid_device_** writeDev, bool writeHandle) {
  std::lock_guard lock(g_openMutex);
  for (size_t i = 0; i < paths.size(); ++i) {
    (*devs)[i] = hid_open_path(paths[i].c_str());
    if (!(*devs)[i]) {
      for (auto& d : *devs) if (d) { hid_close(d); d = nullptr; }
      return;
    }
    hid_set_nonblocking((*devs)[i], 1);
  }
  // Blocking: the writer thread waits in hid_write, not the reader. None: Send writes from the
  // reader thread, as before.
  if (writeHandle) *writeDev = hid_open_path(paths[0].c_str());
}
}  // namespace

bool DoormanDongle::Open(const DoormanDongleFound& found, double now) {
  if (dev_ || TakeOpened()) return true;
  if (opener_.joinable() || now < nextOpen_) return false;   // still opening, or tried less than 2 s ago
  nextOpen_ = now + 2.0;
  if (g_doormanInlineIo) {
    OpenInterfaces(found.paths, &openedDevs_, &openedWriteDev_, false);
    return TakeOpened();
  }
  openerDone_ = false;
  opener_ = std::thread([this, paths = found.paths] {
    OpenInterfaces(paths, &openedDevs_, &openedWriteDev_, true);
    openerDone_ = true;
  });
  return false;
}

bool DoormanDongle::TakeOpened() {
  if (opener_.joinable()) {
    if (!openerDone_) return false;
    opener_.join();
  }
  if (!openedDevs_[0]) return false;   // not opened (all or nothing)
  devs_ = std::exchange(openedDevs_, {});
  dev_ = devs_[0];
  writeDev_ = std::exchange(openedWriteDev_, nullptr);
  if (writeDev_) StartWriter();
  {
    std::lock_guard lock(mutex_);
    opened_ = true;
  }
  for (auto& s : slots_) { s.used = s.active = s.noIdent = false; s.identSeq = -1; s.identTries = 0; }
  for (int i = 0; i < kSlots; ++i) { frag_[i].clear(); packSeq_[i] = -1; }
  const uint8_t on = 1;
  Send(kCmdPacked, &on, 1);   // full stream reports; a dongle without it ignores the command and sends whole frames
  slotCount_ = 8;
  Send(kCmdInfo, nullptr, 0);   // how many slots it has
  nextLive_ = 0;
  log_(LogLevel::Info, "Doorman Dongle " + serial_ + " opened");
  return true;
}

bool g_doormanInlineIo = false;

void DoormanDongle::StartWriter() {
  writerStop_ = false;
  writeFailed_ = false;
  writer_ = std::thread([this] {
    for (;;) {
      std::array<uint8_t, 65> buf;
      {
        std::unique_lock lock(writeMutex_);
        writeCv_.wait(lock, [&] { return writerStop_ || !writes_.empty(); });
        if (writerStop_) return;
        buf = writes_.front();
        writes_.pop_front();
      }
      if (hid_write(writeDev_, buf.data(), buf.size()) < 0) { writeFailed_ = true; return; }
    }
  });
}

void DoormanDongle::StopWriter() {
  if (writer_.joinable()) {
    {
      std::lock_guard lock(writeMutex_);
      writerStop_ = true;
    }
    writeCv_.notify_all();
    writer_.join();
  }
  if (writeDev_) hid_close(writeDev_);
  writeDev_ = nullptr;
  std::lock_guard lock(writeMutex_);
  writes_.clear();
}

void DoormanDongle::Drop() {
  if (!dev_) return;
  StopWriter();
  // Each slot is forgotten as when its device turns off: plugged back in, the dongle's devices are
  // identified and attached again. Keeping the serial meant nothing re-attached them (listed in
  // SteamVR, never tracked).
  for (int i = 0; i < kSlots; ++i) {
    Slot& s = slots_[i];
    if (s.attached) Detach(i);
    s.serial.clear();
    s.noIdent = false;
    s.identTries = 0;
    s.identSeq = -1;
    s.sensorType = -1;
    s.ringN = s.ringPos = 0;
    s.waiting = false;   // its wait for another dongle ends with this one
    s.waitConfig.clear();
    frag_[i].clear();
    packSeq_[i] = -1;
  }
  for (auto& d : devs_) if (d) { hid_close(d); d = nullptr; }
  dev_ = nullptr;
  {
    std::lock_guard lock(mutex_);
    opened_ = false;
  }
  log_(LogLevel::Info, "Doorman Dongle " + serial_ + " closed");
}

void DoormanDongle::Close() {
  Drop();
  if (opener_.joinable()) opener_.join();   // still opening: waits for it, and closes what it opened
  for (auto& d : openedDevs_) if (d) { hid_close(d); d = nullptr; }
  if (openedWriteDev_) hid_close(openedWriteDev_);
  openedWriteDev_ = nullptr;
  {
    std::lock_guard lock(mutex_);
    for (auto& w : featureWaits_) w->done = true;
    featureWaits_.clear();
  }
  cv_.notify_all();
  for (auto& s : slots_) if (s.worker.joinable()) s.worker.join();
}

void DoormanDongle::Send(uint8_t cmd, const uint8_t* args, size_t n) {
  if (!dev_) return;
  // Every program on the dongle sees every reply, matched by [cmd][seq]: the driver numbers its
  // commands 1..127, the tools (dongle manager, installer, scripts) 128..255, so a reply to one
  // is never taken for the other's (the manager's slots 6-7 read as the driver's 0-5 dropped them all).
  seq_ = uint8_t(seq_ % 127 + 1);
  std::array<uint8_t, 65> buf{0, cmd, seq_};
  std::memcpy(buf.data() + 3, args, std::min<size_t>(n, 62));
  if (writeDev_) {
    {
      std::lock_guard lock(writeMutex_);
      writes_.push_back(buf);
    }
    writeCv_.notify_one();
  } else if (hid_write(dev_, buf.data(), buf.size()) < 0) {
    Drop();
  }
}

void DoormanDongle::Downlink(int slot, const uint8_t* frame, size_t n) {
  uint8_t a[62] = {uint8_t(slot), uint8_t(n)};
  std::memcpy(a + 2, frame, std::min<size_t>(n, 60));
  Send(kCmdDown, a, 2 + std::min<size_t>(n, 60));
}

void DoormanDongle::Pump(double now) {
  if (!dev_ && !TakeOpened()) return;   // the transport opens it again when it's listed again
  if (writeFailed_) { Drop(); return; }   // unplugged: the writer saw it first
  if (now >= nextLive_) {
    nextLive_ = now + 0.25;
    for (int page = 0; page < 2; ++page) {   // 6 slots per reply
      const uint8_t first = uint8_t(page * 6);
      Send(kCmdLive, &first, 1);
      liveSeq_[page] = seq_;
    }
  }
  if (now >= nextIds_) {               // which device each slot is paired to: a re-pair can swap it with no gap
    nextIds_ = now + 1.0;
    for (int page = 0; page < 2; ++page) {
      const uint8_t first = uint8_t(page * 6);
      Send(kCmdSlots, &first, 1);
      idsSeq_[page] = seq_;
    }
  }
  if (now >= nextStats_) {             // one paired slot's counters every 1/kSlots s: each slot once a second
    nextStats_ = now + 1.0 / kSlots;
    for (int k = 0; k < kSlots; ++k) {
      const int i = (statsSlot_ + k) % kSlots;
      if (!slots_[i].used) continue;
      statsSlot_ = (i + 1) % kSlots;
      const uint8_t a = uint8_t(i);
      Send(kCmdStats, &a, 1);
      statsSeq_ = seq_ | (i << 8);
      break;
    }
    RefreshStatsCache();
  }
  if (now >= nextLinkLog_) {           // each connected slot's link over the last 30 s, for logs sent in by users
    const bool first = nextLinkLog_ == 0;
    nextLinkLog_ = now + 30;
    std::string line;
    for (int i = 0; i < kSlots; ++i) {
      Slot& s = slots_[i];
      if (!s.active || !s.haveStats) { s.logHave = false; continue; }
      if (s.logHave && !first && s.statsT > s.logT) {
        const double dt = s.statsT - s.logT;
        const uint32_t dp = s.polls - s.logPolls, dok = s.respOk - s.logOk;
        char b[160];
        std::snprintf(b, sizeof b, "%s slot %d: loss %.1f%%, %.1f kB/s, drops %.1f/s, rssi %d/%d dBm",
                      line.empty() ? "" : ";", i, dp ? 100.0 * (1.0 - double(std::min(dok, dp)) / dp) : 0.0,
                      (s.bytesUp - s.logBytes) / dt / 1000, (s.drops - s.logDrops) / dt, s.rssiD, s.rssiT);
        line += b;
      }
      s.logHave = true; s.logT = s.statsT;
      s.logPolls = s.polls; s.logOk = s.respOk; s.logBytes = s.bytesUp; s.logDrops = s.drops;
    }
    if (!line.empty()) log_(LogLevel::Info, "Doorman Dongle " + serial_ + " link (30 s):" + line);
  }
  {   // names kept on the dongle: writes first, then every paired slot's every 2 s
    std::vector<std::pair<int, std::string>> writes;
    { std::lock_guard lock(mutex_); writes.swap(nameWrites_); }
    for (const auto& [slot, name] : writes) {
      uint8_t a[2 + kNameMax] = {uint8_t(slot), uint8_t(name.size())};
      std::memcpy(a + 2, name.data(), name.size());
      Send(kCmdSetName, a, 2 + name.size());
      slots_[slot].name = name;   // shown at once; the next read confirms it
      nextNames_ = 0;
    }
    if (now >= nextNames_) {
      nextNames_ = now + 2.0;
      for (int i = 0; i < kSlots; ++i) {
        if (!slots_[i].used) { slots_[i].name.clear(); slots_[i].haveName = false; continue; }
        const uint8_t a = uint8_t(i);
        Send(kCmdGetName, &a, 1);
        slots_[i].nameSeq = seq_;
      }
    }
  }
  std::optional<int> pair;
  int pairSlot = -1;
  std::deque<std::pair<int, std::vector<uint8_t>>> downs;
  struct Request { int slot; uint8_t id, tag; };
  std::vector<Request> requests;
  {
    std::lock_guard lock(mutex_);
    pair.swap(pairMs_);
    pairSlot = pairSlot_;
    // The main MCU's answers carry no id and the radio tags them in order, so an answer to a write
    // must not land while a read waits: writes to a device wait for its read's answer, and a read
    // waits 50 ms after a write (time for the write's answer, if there is one).
    std::array<bool, kSlots> reading{};
    for (auto& w : featureWaits_)
      if (w->sent && !w->done && w->slot >= 0 && w->slot < kSlots) reading[w->slot] = true;
    for (auto it = downlinks_.begin(); it != downlinks_.end();) {
      if (it->first >= 0 && it->first < kSlots && reading[it->first]) { ++it; continue; }
      if (it->first >= 0 && it->first < kSlots) readsAfter_[it->first] = now + 0.05;
      downs.push_back(std::move(*it));
      it = downlinks_.erase(it);
    }
    for (auto& w : featureWaits_)
      if (!w->sent && !(w->slot >= 0 && w->slot < kSlots && now < readsAfter_[w->slot])) {
        w->sent = true;
        w->tag = nextTag_++;
        requests.push_back({w->slot, w->id, w->tag});
      }
  }
  if (pair) {
    // [secs][slot + 1] (0: the first free slot; older dongle firmware ignores it)
    const uint8_t args[2] = {uint8_t(std::clamp((*pair + 999) / 1000, 0, 255)), uint8_t(pairSlot + 1)};
    Send(kCmdPair, args, 2);
  }
  for (auto& [slot, frame] : downs) {
    Downlink(slot, frame.data(), frame.size());
    if (slot >= 0 && slot < kSlots) slots_[slot].hostBusyT = now;
  }
  for (const auto& r : requests) {
    const uint8_t f[3] = {0xC5, r.tag, r.id};   // the radio answers [0xC4][tag][reply]
    Downlink(r.slot, f, 3);
    if (r.slot >= 0 && r.slot < kSlots) slots_[r.slot].hostBusyT = now;
  }
  for (int i = 0; i < kSlots; ++i) {
    Slot& s = slots_[i];
    WatchSilent(i, now);
    if (s.workerDone) {
      s.worker.join();
      s.workerDone = false;
      if (s.serial.empty() && !s.fetchedSerial.empty()) {
        s.serial = s.fetchedSerial;
        s.gyro = s.fetchedGyro;
        s.accel = s.fetchedAccel;
      }
      if (s.sensorType < 0) s.sensorType = s.fetchedSensorType;
      if (s.active && !s.serial.empty() && !s.attached) {
        std::string config = s.fetchedConfig;
        if (config.empty()) config = UsbTransport::LoadSteamConfig(steamConfigDir_, s.serial);
        Attach(i, config);
      }
    }
    if (s.attached && carriers_ && carriers_->Wired(s.serial)) {   // plugged in by USB: the cable carries it
      std::string config = s.fetchedConfig;
      if (config.empty()) config = UsbTransport::LoadSteamConfig(steamConfigDir_, s.serial);
      Detach(i, /*handedOver=*/true);
      Attach(i, config);   // waits (Take says USB) until it's unplugged
    }
    if (s.waiting && s.active && !s.attached) Attach(i, s.waitConfig);   // has the other carrier let go?
    if (s.active && s.serial.empty() && s.noIdent && !s.worker.joinable() && now - s.identAskT > 5.0) {
      s.identAskT = now;
      StartConfigFetch(i);   // retried every 5 s until the device answers
    } else if (s.active && s.serial.empty() && !s.noIdent && !s.worker.joinable() && now - s.identAskT > 0.5) {
      s.identAskT = now;
      if (++s.identTries > 4) { s.noIdent = true; s.identAskT = -1e9; continue; }   // no 'H': identify it from its config
      const uint8_t a = uint8_t(i);
      Send(kCmdIdent, &a, 1);
      s.identSeq = seq_;
    }
  }
  if (!dev_) return;
  auto drain = [&](hid_device_* dev) {
    uint8_t buf[2][64];
    int n[2] = {0, 0}, cur = 0;
    n[cur] = hid_read(dev, buf[cur], 64);
    for (int count = 0; n[cur] > 0 && count < 64; ++count) {
      const int next = cur ^ 1;
      n[next] = hid_read(dev, buf[next], 64);
      OnReport(now, buf[cur], n[cur], n[next] <= 0);
      cur = next;
    }
    return n[cur] >= 0;
  };
  for (auto* d : devs_)
    if (!drain(d)) { Drop(); return; }
}

void DoormanDongle::OnReport(double t, const uint8_t* r, int n, bool last) {
  if (n < 3) return;
  if (r[0] == 0x82 && n >= 5) {
    // Packed (cmd 0x37): the slot's [len][frame] items back to back across reports. A missing report
    // (seq gap) loses what was being put together; reading starts again where an item begins.
    const int slot = r[1], len = std::min<int>(r[2], n - 5), seq = r[3], first = r[4];
    if (slot >= kSlots) return;
    auto& buf = frag_[slot];
    int from = 0;
    if (packSeq_[slot] != seq) {
      buf.clear();
      if (first == 0xFF || first > len) { packSeq_[slot] = -1; return; }
      from = first;
    }
    packSeq_[slot] = (seq + 1) & 0xFF;
    buf.insert(buf.end(), r + 5 + from, r + 5 + len);
    size_t i = 0;
    while (i < buf.size() && i + 1 + buf[i] <= buf.size()) {
      const int fl = buf[i];
      if (!fl) { buf.clear(); packSeq_[slot] = -1; return; }   // never sent: out of step
      OnFrame(slot, t, buf.data() + i + 1, fl, last && i + 1 + fl >= buf.size());
      i += 1 + fl;
    }
    buf.erase(buf.begin(), buf.begin() + long(i));
    return;
  }
  if (r[0] == 0x80 || r[0] == 0x81) {
    const int slot = r[1], len = std::min<int>(r[2], n - 3);
    if (slot >= kSlots) return;
    auto& frag = frag_;   // this dongle's own: another dongle's reports come in between
    const uint8_t* body = r + 3;
    if (r[0] == 0x81) {
      frag[slot].insert(frag[slot].end(), body, body + len);
      // The first buffered byte is the frame's length (protocol.md: a fragment carries the raw
      // bytes of [len][frame]). A fragment that brought none — its start was lost, or a short
      // report — leaves the accumulator empty, and reading its first byte there is past the end.
      if (!frag[slot].empty() && frag[slot].size() >= size_t(1 + frag[slot][0])) {
        OnFrame(slot, t, frag[slot].data() + 1, frag[slot][0], last);
        frag[slot].clear();
      }
      return;
    }
    frag[slot].clear();
    for (int i = 0; i < len && i + 1 + body[i] <= len;) {
      const int fl = body[i];
      if (!fl) break;
      OnFrame(slot, t, body + i + 1, fl, last && i + 1 + fl >= len);
      i += 1 + fl;
    }
    return;
  }
  const uint8_t* d = r + 3;
  const int livePage = r[1] == liveSeq_[0] ? 0 : r[1] == liveSeq_[1] ? 1 : -1;
  if (r[0] == kCmdLive && r[2] == 0 && n >= 63 && livePage >= 0) {
    liveSeq_[livePage] = -1;
    for (int i = livePage * 6; i < kSlots && i < livePage * 6 + 6; ++i) {
      const uint8_t* e = d + (i - livePage * 6) * 10;
      Slot& s = slots_[i];
      s.used = e[0] & 1;
      const bool active = (e[0] & 2) != 0;
      if (s.active && !active) ResetSlot(i);
      s.active = active;
    }
  } else if (r[0] == kCmdSlots && r[2] == 0 && n >= 63 && (r[1] == idsSeq_[0] || r[1] == idsSeq_[1])) {
    const int page = r[1] == idsSeq_[0] ? 0 : 1;
    idsSeq_[page] = -1;
    for (int i = page * 6; i < kSlots && i < page * 6 + 6; ++i) {
      const uint8_t* e = d + (i - page * 6) * 10;
      Slot& s = slots_[i];
      if (!e[0]) { s.havePairId = false; continue; }   // not paired
      std::array<uint8_t, 8> id;
      std::memcpy(id.data(), e + 2, 8);
      if (s.havePairId && id != s.pairId) {
        log_(LogLevel::Info, "Doorman Dongle " + serial_ + " slot " + std::to_string(i) + ": paired to another device" +
                                 (s.serial.empty() ? std::string() : " (was " + s.serial + ")"));
        ResetSlot(i);
      }
      s.pairId = id;
      s.havePairId = true;
    }
  } else if (r[0] == kCmdStats && r[2] == 0 && n >= 63 && statsSeq_ >= 0 && r[1] == (statsSeq_ & 0xff)) {
    Slot& s = slots_[statsSeq_ >> 8];
    statsSeq_ = -1;
    uint32_t v[15];
    std::memcpy(v, d, sizeof v);
    if (s.haveStats && t > s.statsT) {
      const double dt = t - s.statsT;
      const uint32_t dp = v[0] - s.polls, dok = v[1] - s.respOk;
      s.fps = (v[5] - s.framesUp) / dt;
      s.kBps = (v[6] - s.bytesUp) / dt / 1000;
      s.dps = (v[7] - s.drops) / dt;
      s.loss = dp ? 100.0 * (1.0 - double(std::min(dok, dp)) / dp) : -1;
    }
    s.haveStats = true; s.statsT = t;
    std::copy(std::begin(v), std::end(v), s.raw.begin());
    s.polls = v[0]; s.respOk = v[1]; s.framesUp = v[5]; s.bytesUp = v[6]; s.drops = v[7];
    s.rssiD = -std::abs(int32_t(v[10])); s.rssiT = -std::abs(int32_t(v[11]));   // dongle keeps magnitudes
  } else if (r[0] == kCmdGetName) {
    for (auto& s : slots_) {
      if (s.nameSeq != r[1]) continue;
      s.nameSeq = -1;
      s.haveName = r[2] == 0;   // older firmware: not a command it knows
      if (s.haveName) s.name.assign(reinterpret_cast<const char*>(d + 1), std::min<size_t>(d[0], kNameMax));
      break;
    }
  } else if (r[0] == kCmdInfo && n >= 64 && std::memcmp(r + 3, "OHDG", 4) == 0) {
    slotCount_ = r[63] >= 1 && r[63] <= kSlots ? r[63] : 8;   // 0 from firmware before 0x002A: 8 slots
  } else if (r[0] == kCmdIdent && n >= 19) {
    for (int i = 0; i < kSlots; ++i) {
      Slot& s = slots_[i];
      if (s.identSeq != r[1]) continue;
      s.identSeq = -1;
      if (r[2] == 0 && d[0] == 'H') { if (s.active) OnIdent(i, d, 16); }
      else if (r[2] == 0 && d[0] == 0) {   // the device sent no 'H': not our firmware
        s.noIdent = true;
        s.identAskT = -1e9;                 // read its config now: the 5 s is only between retries
      }
      break;
    }
  }
}

void DoormanDongle::OnFrame(int slot, double t, const uint8_t* f, int n, bool last) {
  Slot& s = slots_[slot];
  if (f[0] == 'p' || f[0] == 'q' || f[0] == 's') {
    if (s.silentStage)
      log_(LogLevel::Warn, "Doorman Dongle " + serial_ + " slot " + std::to_string(slot) + ": data back after " +
                                  std::to_string(int(t - s.lastDataT)) + " s");
    s.silentStage = 0;
    s.lastDataT = t;
    s.streamed = true;
  }
  if (f[0] == 0xF0 && n >= 36 && f[1] == 'X') { OnRadioStatus(slot, f, n); return; }
  switch (f[0]) {
    case 0xC4: {
      if (n < 2) return;
      std::lock_guard lock(mutex_);
      for (auto& w : featureWaits_) {
        if (w->slot != slot || !w->sent || w->done || w->tag != f[1]) continue;
        w->reply.assign(1, w->id);
        w->reply.insert(w->reply.end(), f + 2, f + n);
        w->done = true;
        cv_.notify_all();
        break;
      }
      return;
    }
    case 'H':
      OnIdent(slot, f, n);
      return;
    case 'p': {
      if (n != 18) return;
      CheckImuStream(slot, t, f);
      std::memcpy(s.ring[s.ringPos].data(), f + 1, 17);
      if (s.ringN == 0) s.ring[1] = s.ring[2] = s.ring[0];
      s.ringPos = (s.ringPos + 1) % 3;
      s.ringN = std::min(s.ringN + 1, 3);
      if (!s.attached) return;
      uint8_t rep[1 + 3 * 17] = {0x20};
      for (int k = 0; k < 3; ++k) std::memcpy(rep + 1 + 17 * k, s.ring[k].data(), 17);
      capture_->WriteReport(t - kExtraLatency, s.epImu, last, rep, sizeof rep);
      decoder_->OnReport(t - kExtraLatency, s.epImu, last, rep, sizeof rep);
      return;
    }
    case 'q': {
      if (!s.attached || n < 9 || (n - 1) % 8 || n > 57) return;
      uint8_t rep[0x39];
      std::memset(rep, 0xff, sizeof rep);
      rep[0] = 0x21;
      std::memcpy(rep + 1, f + 1, size_t(n - 1));
      capture_->WriteReport(t, s.epOpt, last, rep, sizeof rep);
      decoder_->OnReport(t, s.epOpt, last, rep, sizeof rep);
      return;
    }
    case 's': {   // LH2 stream: wired 0x28 is [0x28][len][stream]
      if (!s.attached || n < 2) return;
      uint8_t rep[64] = {0x28, uint8_t(std::min(n - 1, 62))};
      std::memcpy(rep + 2, f + 1, rep[1]);
      capture_->WriteReport(t, s.epOpt, last, rep, sizeof rep);
      decoder_->OnReport(t, s.epOpt, last, rep, sizeof rep);
      return;
    }
    case 'Q':     // input: the wired buttons report's bytes 8..23 (u32 buttons at 8, trackpad at 0x14)
    case 'n':     // Index input: wired type 0x0700 bytes 8.. (buttons, trigger, axes, 0xa1 record at 20)
    case 'V': {   // other controller reports: the wired report from byte 2
      if (!s.attached) return;
      uint8_t rep[64] = {0x01};
      if (f[0] == 'Q') {
        if (n < 17) return;
        rep[2] = 1; rep[3] = 0x3C;
        std::memcpy(rep + 8, f + 1, 16);
      } else if (f[0] == 'n') {
        if (n < 2) return;
        rep[1] = 0x00; rep[2] = 0x07;
        std::memcpy(rep + 8, f + 1, size_t(std::min(n - 1, 56)));
      } else {
        std::memcpy(rep + 2, f + 1, size_t(std::min(n - 1, 62)));
      }
      capture_->WriteReport(t, s.epBtn, last, rep, sizeof rep);
      decoder_->OnReport(t, s.epBtn, last, rep, sizeof rep);
      return;
    }
    case 'B':
      if (s.attached && n >= 2) sink_->OnBattery(s.serial, {0.0, f[1]});
      return;
    default:
      return;
  }
}

void DoormanDongle::OnIdent(int slot, const uint8_t* f, int n) {
  if (n < 16 || f[1] != 1) return;
  Slot& s = slots_[slot];
  char serial[16];
  std::snprintf(serial, sizeof serial, "LHR-%08X", unsigned(Rd32(f + 2)));
  if (s.serial == serial && (s.attached || s.waiting)) return;   // waiting: on another dongle for now
  if (s.attached) Detach(slot);
  s.waiting = false;
  s.waitConfig.clear();
  s.serial = serial;
  s.gyro = int8_t(f[6]);
  s.accel = int8_t(f[7]);
  const uint32_t hw = Rd32(f + 8);
  s.sensorType = (hw & 0xff) <= 11 && hw ? int(hw & 0xff) : -1;
  std::string config = UsbTransport::LoadSteamConfig(steamConfigDir_, s.serial);
  if (!config.empty()) Attach(slot, config);
  else if (!s.worker.joinable()) StartConfigFetch(slot);
}

void DoormanDongle::StartConfigFetch(int slot) {
  Slot& s = slots_[slot];
  s.fetchedConfig.clear();
  s.workerDone = false;
  s.fetchedSerial.clear();
  const std::string serial = s.serial;
  const std::string who = serial.empty() ? serial_ + " slot " + std::to_string(slot) : serial;
  log_(LogLevel::Info, who + ": reading device config over the radio");
  s.worker = std::thread([this, slot, serial, who] {
    Slot& w = slots_[slot];
    std::string json, error;
    const auto t0 = std::chrono::steady_clock::now();
    auto read = [&](uint8_t* data, size_t size) { return FeatureRead(slot, data, size); };
    // The config first: its read retries until the main MCU answers. Right after a device joins
    // (Index controller just switched on) the main MCU doesn't answer feature reads yet, and one
    // early try for the IMU ranges left them unknown: every IMU sample was then dropped
    // (ImuEmitter::Emit) and the device stayed "not tracked" for good.
    const bool ok = ReadDeviceConfig(read, &json, &error, true);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (serial.empty()) {          // unidentified device: IMU ranges from feature 0x01 (as wired)
      w.fetchedGyro = w.fetchedAccel = -1;
      for (int a = 0; a < 10 && w.fetchedGyro < 0; ++a) {
        if (a) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        uint8_t r[64] = {0x01};
        if (read(r, sizeof r) >= 3 && r[0] == 0x01) { w.fetchedGyro = int8_t(r[1]); w.fetchedAccel = int8_t(r[2]); }
      }
    }
    w.fetchedSensorType = -1;
    if (w.sensorType < 0) {        // sensor type: hardware id low byte in feature 0x05 at [41] (as wired:
      for (int a = 0; a < 5 && w.fetchedSensorType < 0; ++a) {   // without it, OOTX decoding guesses)
        if (a) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        uint8_t r[64] = {0x05};
        if (read(r, sizeof r) >= 45 && r[0] == 0x05) {
          const uint32_t build = Rd32(r + 1), hw = Rd32(r + 41);
          if (build && hw && (hw & 0xff) <= 11) w.fetchedSensorType = int(hw & 0xff);
          break;
        }
      }
      log_(LogLevel::Info, who + ": sensor type " + (w.fetchedSensorType < 0 ? std::string("unavailable") :
                               std::to_string(w.fetchedSensorType)) + " (feature 0x05)");
    }
    if (ok) {
      w.fetchedConfig = json;
      if (serial.empty()) {
        if (w.fetchedGyro < 0 || w.fetchedAccel < 0) {
          // without the ranges its IMU can't be used: don't attach it yet, read again in 5 s
          log_(LogLevel::Warn, who + ": IMU ranges unavailable (feature 0x01): reading again in 5 s");
        } else {
          w.fetchedSerial = ConfigSerial(json);
        }
      }
      log_(LogLevel::Info, who + ": read device config (" + std::to_string(json.size()) + " bytes) in " +
                               std::to_string(int(ms)) + " ms" +
                               (serial.empty() ? ", serial " + (!w.fetchedSerial.empty() ? w.fetchedSerial : w.fetchedGyro < 0 || w.fetchedAccel < 0 ? "not yet" : "missing") +
                                                     ", IMU ranges gyro " + std::to_string(w.fetchedGyro) + " accel " +
                                                     std::to_string(w.fetchedAccel)
                                               : ""));
    } else {
      log_(LogLevel::Warn, who + ": device config unavailable (" + error + ")");
    }
    w.workerDone = true;
  });
}

int DoormanDongle::FeatureRead(int slot, uint8_t* data, size_t size) {
  auto w = std::make_shared<FeatureWait>();
  w->slot = slot;
  w->id = data[0];
  std::unique_lock lock(mutex_);
  if (!opened_) return -1;
  featureWaits_.push_back(w);
  cv_.wait_for(lock, std::chrono::duration<double>(kFeatureTimeout), [&] { return w->done; });
  std::erase(featureWaits_, w);
  if (w->reply.empty()) return -1;
  const size_t n = std::min(size, w->reply.size());
  std::memcpy(data, w->reply.data(), n);
  return int(n);
}

void DoormanDongle::Attach(int slot, const std::string& configIn) {
  Slot& s = slots_[slot];
  if (carriers_) {
    if (const auto other = carriers_->Take(s.serial, serial_, slot)) {
      if (!s.waiting) {  // once per episode: it waits until the other lets go or this slot goes quiet
        if (other->slot < 0)
          log_(LogLevel::Info, s.serial + ": plugged in by USB; Doorman Dongle " + serial_ + " slot " +
                                   std::to_string(slot) + " waits until it's unplugged");
        else
          log_(LogLevel::Info, s.serial + ": also on Doorman Dongle " + serial_ + " slot " + std::to_string(slot) +
                                   "; kept on Doorman Dongle " + other->dongle + " slot " + std::to_string(other->slot));
      }
      s.waiting = true;
      s.waitConfig = configIn;
      return;
    }
  }
  std::string config = configIn;   // before the clear below: configIn may be s.waitConfig
  s.waiting = false;
  s.waitConfig.clear();
  if (s.sensorType >= 0 && !config.empty()) AddSensorType(&config, s.sensorType);
  if (config.empty()) log_(LogLevel::Warn, s.serial + ": no config (plug it in by USB once, or check the radio link)");
  EndpointInfo imu;
  imu.ep = (*nextEp_)++;
  imu.vid = kVid;
  imu.pid = kPid;
  imu.iface = uint8_t(slot);
  imu.kind = EndpointKind::Imu;
  imu.serial = s.serial;
  imu.gyroRange = s.gyro;
  imu.accelRange = s.accel;
  imu.configJson = config;
  EndpointInfo opt = imu;
  opt.ep = (*nextEp_)++;
  opt.kind = EndpointKind::Optical;
  opt.configJson.clear();
  EndpointInfo btn = opt;
  btn.ep = (*nextEp_)++;
  btn.kind = EndpointKind::Buttons;
  s.epImu = imu.ep;
  s.epOpt = opt.ep;
  s.epBtn = btn.ep;
  capture_->WriteEndpoint(imu);
  capture_->WriteEndpoint(opt);
  capture_->WriteEndpoint(btn);
  decoder_->AddEndpoint(imu);
  decoder_->AddEndpoint(opt);
  decoder_->AddEndpoint(btn);
  s.attached = true;
  {
    std::lock_guard lock(mutex_);
    eps_.push_back(imu);
    eps_.push_back(opt);
    eps_.push_back(btn);
  }
  log_(LogLevel::Info, s.serial + ": connected on Doorman Dongle " + serial_ + " slot " + std::to_string(slot));
}

namespace {
std::string SlotCounters(const std::array<uint32_t, 15>& v, const std::array<uint32_t, 15>* since) {
  static const char* kNames[] = {"polls", "resp_ok", "crc_err", "timeouts", "dup", "frames_up", "bytes_up", "drops",
                                 "down_frames", "down_resends", "rssi_dongle", "rssi_device", "device_missed",
                                 "device_crc", "ms_since_resp"};
  std::string out;
  for (int k = 0; k < 15; ++k) {
    const bool delta = since && k != 10 && k != 11 && k != 14;
    out += std::string(k ? " " : "") + kNames[k] + (delta ? " +" : " ") +
           std::to_string(delta ? v[size_t(k)] - (*since)[size_t(k)] : v[size_t(k)]);
  }
  return out;
}
}  // namespace

// A device that streamed and then went quiet while its slot still answers: the stall that needs a
// power cycle. At 3 s the dongle's counters and the radio's own status go to the log; at 6 s the
// counters again (what kept moving), then the dongle makes it rejoin (firmware 0x0024 and newer).
void DoormanDongle::WatchSilent(int i, double now) {
  Slot& s = slots_[i];
  if (!s.active || !s.streamed || !s.haveStats) { s.silentStage = 0; return; }
  // Talking to its main MCU (a firmware update, a config read) can pause its data: left alone.
  if (now - s.hostBusyT < 10) { s.lastDataT = std::max(s.lastDataT, now - 2); return; }
  const double quiet = now - s.lastDataT;
  const std::string who = "Doorman Dongle " + serial_ + " slot " + std::to_string(i) +
                          (s.serial.empty() ? std::string() : " (" + s.serial + ")");
  if (s.silentStage == 0 && quiet > 3) {
    s.silentStage = 1;
    s.silentRaw = s.raw;
    log_(LogLevel::Warn, who + ": connected but no data for 3 s; " + SlotCounters(s.raw, nullptr));
    const uint8_t ask[2] = {0xF0, 'X'};   // the radio's status, if its uplink still works
    Downlink(i, ask, sizeof ask);
  } else if (s.silentStage == 1 && quiet > 6) {
    s.silentStage = 2;
    log_(LogLevel::Warn, who + ": still no data at 6 s, last 3 s: " + SlotCounters(s.raw, &s.silentRaw) +
                                "; making it rejoin");
    const uint8_t a = uint8_t(i);
    Send(kCmdRejoin, &a, 1);
  } else if (s.silentStage == 2 && quiet > 20) {
    s.silentStage = 3;
    log_(LogLevel::Warn, who + ": still connected with no data after the rejoin; " + SlotCounters(s.raw, &s.silentRaw));
  }
}

// The air codec codes each IMU sample against the ones before it (doorman_codec.h), so the dongle's
// decoder must step exactly as the radio's coder. One corrupted answer that passed the radio's
// CRC-16 puts it out of step for good: every sample after it decodes as plausible garbage, each
// timestamp ~31 s on and the accelerometer ramping by a constant step, and the device sits
// connected, streaming, and never tracking until it rejoins (a power cycle did it). The data
// still flows, so WatchSilent never sees it. Its timestamps give it away: a run of samples that go
// back or leap is answered with a rejoin, which resets the codec on both sides.
void DoormanDongle::CheckImuStream(int i, double now, const uint8_t* f) {
  Slot& s = slots_[i];
  uint32_t ticks;
  std::memcpy(&ticks, f + 13, 4);
  constexpr uint32_t kMaxStepTicks = 48000000u / 5;   // 200 ms of the device's 48 MHz clock
  const uint32_t step = ticks - s.lastImuTicks;
  const bool sane = !s.haveImuTicks || (step > 0 && step < kMaxStepTicks);
  s.lastImuTicks = ticks;
  s.haveImuTicks = true;
  if (sane) { s.garbledImu = 0; return; }
  if (++s.garbledImu < 25 || now - s.garbledRejoinT < 10) return;
  s.garbledImu = 0;
  s.garbledRejoinT = now;
  log_(LogLevel::Warn, "Doorman Dongle " + serial_ + " slot " + std::to_string(i) +
                           (s.serial.empty() ? std::string() : " (" + s.serial + ")") +
                           ": IMU stream garbled (the air codec out of step); making it rejoin");
  const uint8_t a = uint8_t(i);
  Send(kCmdRejoin, &a, 1);
}

void DoormanDongle::OnRadioStatus(int slot, const uint8_t* f, int n) {
  uint32_t ver, frames, empty;
  std::memcpy(&ver, f + 2, 4);
  std::memcpy(&frames, f + 11, 4);
  std::memcpy(&empty, f + 15, 4);
  char b[200];
  std::snprintf(b, sizeof b, ": radio 0x%08x, main MCU frames %u (empty %u), safe %d, safe request %d, sleep %d, "
                "replies queued %d B, quick boots %d", ver, frames, empty, f[19] & 1, (f[19] >> 1) & 1, (f[19] >> 2) & 1,
                f[34], f[35]);
  std::string line = "Doorman Dongle " + serial_ + " slot " + std::to_string(slot) + b;
  if (n >= 60) {
    line += ", main MCU ops:";
    for (int k = 0; k < 6; ++k) {
      const uint8_t* o = f + 36 + 4 * k;
      if (!o[3]) continue;
      std::snprintf(b, sizeof b, " %02x %02x %02x x%d", o[0], o[1], o[2], o[3]);
      line += b;
    }
  }
  log_(slots_[size_t(slot)].silentStage ? LogLevel::Warn : LogLevel::Info, line);
}

void DoormanDongle::ResetSlot(int slot) {
  Slot& s = slots_[slot];
  s.haveImuTicks = false;
  s.garbledImu = 0;
  s.streamed = false;
  s.silentStage = 0;
  if (s.attached) Detach(slot);
  s.serial.clear();
  s.noIdent = false;
  s.identTries = 0;
  s.identAskT = 0;
  s.waiting = false;
  s.waitConfig.clear();
}

void DoormanDongle::Detach(int slot, bool handedOver) {
  Slot& s = slots_[slot];
  if (!s.attached) return;
  decoder_->RemoveEndpoint(s.epImu);
  decoder_->RemoveEndpoint(s.epOpt);
  decoder_->RemoveEndpoint(s.epBtn);
  if (carriers_) carriers_->Release(s.serial, serial_, slot);
  if (!handedOver) sink_->OnDeviceDetached(s.serial);   // handed over: the device itself carries on (USB)
  s.attached = false;
  s.ringN = 0;
  {
    std::lock_guard lock(mutex_);
    std::erase_if(eps_, [&](const EndpointInfo& e) { return e.serial == s.serial; });
  }
  log_(LogLevel::Info, s.serial + ": disconnected from Doorman Dongle " + serial_ + " slot " + std::to_string(slot));
}

std::vector<EndpointInfo> DoormanDongle::Endpoints() const {
  std::lock_guard lock(mutex_);
  return eps_;
}

std::vector<DeviceId> DoormanDongle::Connected() const {
  std::lock_guard lock(mutex_);
  std::vector<DeviceId> v;
  for (auto& e : eps_) if (e.kind == EndpointKind::Imu) v.push_back(e.serial);
  return v;
}

void DoormanDongle::DeviceGone(const DeviceId& serial) {
  std::lock_guard lock(mutex_);
  for (auto& st : statsCache_)
    if (st.serial == serial) st.serial.clear();
}

bool DoormanDongle::Command(const DeviceId& serial, const std::array<uint8_t, 64>& report, const char** status, size_t size) {
  int slot = -1;
  {
    std::lock_guard lock(mutex_);
    for (auto& e : eps_) if (e.serial == serial && e.kind == EndpointKind::Imu) slot = e.iface;
    if (slot < 0) return false;
    // Radio 'P' frame, as Valve's dongles tunnel host feature reports to a device's main MCU
    // (HTC's and Valve's main firmwares, and ours, all read it this way):
    //   [0xff][cmd][len][data] -> [P][cmd][len][data]   (command; cmd >= 0x80)
    //   [id][data] (id < 0x80) -> [P][id][data]         (SET_FEATURE; [P][id] alone is a GET)
    std::vector<uint8_t> frame(1, 'P');
    if (report[0] == 0xff) {
      const size_t n = std::min<size_t>(2 + report[2], 59);
      frame.insert(frame.end(), report.begin() + 1, report.begin() + 1 + long(n));
    } else {
      const size_t n = std::min<size_t>(std::max<size_t>(size, 2), 60);
      frame.insert(frame.end(), report.begin(), report.begin() + long(n));
    }
    downlinks_.emplace_back(slot, std::move(frame));
  }
  *status = "written";
  return true;
}

bool DoormanDongle::SetName(int slot, const std::string& nameIn) {
  if (slot < 0 || slot >= slotCount_) return false;
  std::string name = nameIn;
  if (name.size() > kNameMax) {   // cut on a character boundary (UTF-8)
    size_t n = kNameMax;
    while (n > 0 && (uint8_t(name[n]) & 0xC0) == 0x80) --n;
    name.resize(n);
  }
  std::lock_guard lock(mutex_);
  if (!opened_) return false;
  for (auto& st : statsCache_) if (st.slot == slot) st.name = name;   // seen at once
  nameWrites_.emplace_back(slot, std::move(name));
  return true;
}

bool DoormanDongle::Pair(int milliseconds, int slot) {
  std::lock_guard lock(mutex_);
  if (!opened_ || slot >= slotCount_) return false;   // a slot this dongle doesn't have (an 8-slot one: 8, 9)
  pairMs_ = milliseconds;
  pairSlot_ = slot >= 0 ? slot : -1;
  return true;
}

std::vector<DoormanDongle::SlotStat> DoormanDongle::SlotStats() const {
  std::lock_guard lock(mutex_);
  return opened_ ? statsCache_ : std::vector<SlotStat>();
}

void DoormanDongle::RefreshStatsCache() {     // reader thread: slots_ is its own
  std::vector<SlotStat> out;
  int joined = 0;
  for (const auto& s : slots_) joined += s.active;
  for (int i = 0; i < slotCount_; ++i) {
    const Slot& s = slots_[i];
    SlotStat st;
    st.slot = i; st.used = s.used; st.active = s.active; st.serial = s.attached ? s.serial : DeviceId();
    if (s.active) {
      st.kBytesPerSec = s.kBps; st.framesPerSec = s.fps; st.lossPercent = s.loss; st.dropsPerSec = s.dps;
      st.rssiDongle = s.rssiD; st.rssiDevice = s.rssiT;
    }
    st.devicesOnDongle = joined;
    st.name = s.name; st.haveName = s.haveName;
    out.push_back(st);
  }
  std::lock_guard lock(mutex_);
  statsCache_.swap(out);
}

}  // namespace lh::usb
