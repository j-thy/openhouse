#include "dongle.hpp"

#include <cmath>

#include <hidapi/hidapi.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>

#include "common/dongle_bank.hpp"
#include "device_config.hpp"

namespace openhouse::dongle {
namespace {
constexpr uint16_t kVid = 0x6767, kPid = 0x0727;
// Commands (doorman/dongle-fw/src/dongle.h, link.c)
constexpr uint8_t kInfo = 0x01, kBootloader = 0x10, kReset = 0x12, kClearCrash = 0x14, kLink = 0x20, kDown = 0x21,
                  kScan = 0x22, kStats = 0x23, kPair = 0x24, kSlotsCmd = 0x25, kForget = 0x26, kChannels = 0x27,
                  kCodec = 0x28, kLive = 0x29, kIdent = 0x2A, kTiming = 0x2D, kSetName = 0x34, kGetName = 0x35;
constexpr size_t kNameMax = 59;  // what the dongle keeps (link.c DML_NAME_MAX)

hid_device* H(void* p) { return static_cast<hid_device*>(p); }
uint32_t Rd32(const uint8_t* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }

std::string Narrow(const wchar_t* w) {
  std::string s;
  for (; w && *w; ++w) s += *w < 128 ? char(*w) : '?';
  return s;
}

std::string Hex(const uint8_t* p, int n) {
  static const char* d = "0123456789abcdef";
  std::string s;
  for (int i = n - 1; i >= 0; --i) { s += d[p[i] >> 4]; s += d[p[i] & 15]; }   // as the dongle prints it
  return s;
}

void Push(std::deque<float>& h, float v) {
  h.push_back(v);
  while (h.size() > size_t(kHistory)) h.pop_front();
}

// A model name for the UI, from the device's config.
std::string ModelName(const nlohmann::json& j) {
  const std::string model = j.value("model_number", std::string());
  if (!model.empty()) return model;
  const std::string render = j.value("render_model", std::string());
  if (render.find("tracker_vive_1_0") != std::string::npos) return "Vive Tracker 1.0";
  if (render.find("tracker") != std::string::npos) return "Tracker";
  const std::string cls = j.value("device_class", std::string());
  return cls.empty() ? "Lighthouse device" : cls;
}
}  // namespace

double Now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// ---------------------------------------------------------------------------------------------
Dongle::Dongle(std::string path, std::string serial) : path_(std::move(path)), serial_(std::move(serial)) {
  state_.serial = serial_;
  state_.path = path_;
  packSeq_.fill(-1);
  thread_ = std::thread([this] { Run(); });
}

Dongle::~Dongle() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
}

DongleState Dongle::Snapshot() const {
  std::lock_guard lock(mutex_);
  return state_;
}

void Dongle::Queue(std::function<void()> action) {
  std::lock_guard lock(mutex_);
  actions_.push_back(std::move(action));
}

void Dongle::SetNickname(int slot, const std::string& name) {
  std::lock_guard lock(mutex_);
  if (slot >= 0 && slot < kSlots) state_.slots[slot].nickname = name;
}

void Dongle::SetName(int slot, std::string name, Done done) {
  // Cut to what the dongle keeps, on a character boundary (UTF-8).
  if (name.size() > kNameMax) {
    size_t n = kNameMax;
    while (n > 0 && (uint8_t(name[n]) & 0xC0) == 0x80) --n;
    name.resize(n);
  }
  Queue([=, this] {
    uint8_t a[2 + kNameMax] = {uint8_t(slot), uint8_t(name.size())};
    std::memcpy(a + 2, name.data(), name.size());
    Reply r;
    const bool ok = Command(kSetName, a, 2 + name.size(), &r) && r.status == 0;
    if (ok) { std::lock_guard lock(mutex_); state_.slots[slot].nickname = name; }
    if (done) done(ok, ok ? "Name saved on the dongle" : "the dongle didn't take the name");
  });
}

void Dongle::SetKnown(const std::string& pairId, const Identity& id) {
  std::lock_guard lock(mutex_);
  for (auto& [k, v] : known_)
    if (k == pairId) { v = id; return; }
  known_.emplace_back(pairId, id);
}

bool Dongle::Open() {
  devs_[0] = hid_open_path(path_.c_str());
  if (!devs_[0]) return false;
  hid_set_nonblocking(H(devs_[0]), 1);
  // the other interfaces (slot s streams on interface s % 4): the siblings with the same serial
  if (hid_device_info* list = hid_enumerate(kVid, kPid)) {
    for (hid_device_info* d = list; d; d = d->next)
      if (d->interface_number > 0 && d->interface_number < kInterfaces && !devs_[d->interface_number] &&
          Narrow(d->serial_number) == serial_.substr(5)) {
        devs_[d->interface_number] = hid_open_path(d->path);
        if (devs_[d->interface_number]) hid_set_nonblocking(H(devs_[d->interface_number]), 1);
      }
    hid_free_enumeration(list);
  }
  dev0_ = devs_[0];
  return true;
}

void Dongle::Close() {
  for (auto& d : devs_) { if (d) hid_close(H(d)); d = nullptr; }
  dev0_ = nullptr;
}

// One command and its reply; stream reports read meanwhile are handled as they come.
bool Dongle::Command(uint8_t cmd, const uint8_t* args, size_t n, Reply* reply, int timeoutMs) {
  if (!dev0_) return false;
  seq_ = uint8_t(0x80 | (seq_ + 1));   // 128..255: the driver uses 1..127 (it sees our replies too)
  const uint8_t seq = seq_;
  uint8_t out[65] = {0, cmd, seq};
  std::memcpy(out + 3, args, std::min<size_t>(n, 62));
  if (hid_write(H(dev0_), out, sizeof out) < 0) { alive_ = false; return false; }
  const double until = Now() + timeoutMs / 1000.0;
  uint8_t r[64];
  while (Now() < until && !stop_) {
    const int got = hid_read_timeout(H(dev0_), r, sizeof r, 5);
    if (got < 0) { alive_ = false; return false; }
    for (int i = 1; i < kInterfaces; ++i)
      if (devs_[i]) for (int k = 0; k < 16; ++k) {
        uint8_t q[64];
        const int m = hid_read(H(devs_[i]), q, sizeof q);
        if (m <= 0) break;
        OnReport(q, m);
      }
    if (got == 0) continue;
    if (r[0] == cmd && r[1] == seq) {
      if (reply) { reply->status = r[2]; std::memcpy(reply->data.data(), r + 3, 61); }
      return r[2] == 0;
    }
    OnReport(r, got);
  }
  return false;
}

void Dongle::ReadPending(int timeoutMs) {
  uint8_t r[64];
  for (int pass = 0; pass < 200; ++pass) {
    bool any = false;
    for (void* d : devs_) {
      if (!d) continue;
      const int n = pass == 0 && d == dev0_ ? hid_read_timeout(H(d), r, sizeof r, timeoutMs) : hid_read(H(d), r, sizeof r);
      if (n < 0) { alive_ = false; return; }
      if (n > 0) { OnReport(r, n); any = true; }
    }
    if (!any) return;
  }
}

void Dongle::OnReport(const uint8_t* r, int n) {
  if (n >= 5 && r[0] == 0x82 && r[1] < kSlots) {
    // Packed (cmd 0x37, which the driver and tools turn on for the whole dongle): [len][frame] items back to
    // back across reports. A seq gap loses what was being put together; reading starts again at an item.
    const int slot = r[1], len = std::min<int>(r[2], n - 5), seq = r[3], first = r[4];
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
      if (!buf[i]) { buf.clear(); packSeq_[slot] = -1; return; }
      OnFrame(slot, buf.data() + i + 1, buf[i]);
      i += 1 + buf[i];
    }
    buf.erase(buf.begin(), buf.begin() + long(i));
    return;
  }
  if (n < 3 || (r[0] != 0x80 && r[0] != 0x81) || r[1] >= kSlots) return;
  const int slot = r[1], len = std::min<int>(r[2], n - 3);
  const uint8_t* body = r + 3;
  auto& frag = frag_[slot];
  if (r[0] == 0x81) {
    frag.insert(frag.end(), body, body + len);
    if (!frag.empty() && frag.size() >= size_t(frag[0]) + 1) {
      if (frag[0]) OnFrame(slot, frag.data() + 1, frag[0]);
      frag.clear();
    }
    return;
  }
  frag.clear();
  for (int i = 0; i < len && i + 1 + body[i] <= len; i += 1 + body[i])
    if (body[i]) OnFrame(slot, body + i + 1, body[i]);
}

void Dongle::OnFrame(int slot, const uint8_t* f, int n) {
  auto& c = counters_[slot];
  switch (f[0]) {
    case 'p': c.imu++; break;
    case 'q': case 's': c.light++; break;
    case 'Q': case 'n': case 'V': c.input++; break;
    case 'P':
      if (slot == waitSlot_ && !waitDone_) { waitReply_.assign(f + 1, f + n); waitDone_ = true; }
      break;
    default: break;
  }
}

// A feature report read through the radio: [P][id] down, the device answers [P][data].
bool Dongle::FeatureRead(int slot, uint8_t* data, size_t size) {
  const uint8_t args[4] = {uint8_t(slot), 2, 'P', data[0]};
  waitSlot_ = slot;
  waitDone_ = false;
  if (!Command(kDown, args, sizeof args, nullptr)) { waitSlot_ = -1; return false; }
  const double until = Now() + 0.5;
  while (!waitDone_ && Now() < until && alive_ && !stop_) ReadPending(5);
  waitSlot_ = -1;
  if (!waitDone_) return false;
  const size_t n = std::min(size - 1, waitReply_.size());
  std::memcpy(data + 1, waitReply_.data(), n);
  return true;
}

void Dongle::Ident(int slot) {
  // our firmware: its 'H' identity, kept by the dongle
  Reply r;
  const uint8_t a = uint8_t(slot);
  if (Command(kIdent, &a, 1, &r) && r.data[0] == 'H' && r.data[1] == 1) {
    char serial[16];
    std::snprintf(serial, sizeof serial, "LHR-%08X", unsigned(Rd32(r.data.data() + 2)));
    std::lock_guard lock(mutex_);
    auto& s = state_.slots[slot];
    if (s.id.serial != serial) { s.id.serial = serial; s.id.source = "firmware"; }
  }
}

void Dongle::Poll(double now) {
  Reply r;
  if (now >= nextLive_) {
    nextLive_ = now + 0.25;
    for (int first = 0; first < kSlots; first += 6) {   // 6 slots per reply
      const uint8_t arg = uint8_t(first);
      if (!Command(kLive, &arg, 1, &r)) continue;
      std::lock_guard lock(mutex_);
      for (int i = first; i < kSlots && i < first + 6; ++i) {
        const uint8_t* e = r.data.data() + (i - first) * 10;
        auto& s = state_.slots[i];
        s.used = e[0] & 1;
        const bool wasActive = s.active;
        s.active = (e[0] & 2) != 0;
        // Turned on again: it's asked who it is again. Its tries may have run out while it was on before
        // (the driver running, a weak link), and the slot stayed unnamed until the dongle was replugged.
        if (s.active && !wasActive && s.id.serial.empty()) { identTried_[i] = 0; configTried_[i] = 0; configNext_[i] = 0; }
        s.rssiMin = -int(e[4]);  // min/max are magnitudes: the strongest is the smallest
        s.rssiMax = -int(e[3]);
        // Signal strength differs by several dB from channel to channel (the link hops every
        // cycle), so a single reading jumps around: show an average over all channels instead.
        // The dongle's min/max cover every poll since the last read (all channels), the device's
        // value is one reading (from the previous poll).
        if (!s.active) {
          s.rssiDongle = s.rssiDevice = 0;
          rssiAvgD_[i] = rssiAvgT_[i] = 0;
        } else {
          const double d = e[3] && e[4] ? (double(e[3]) + double(e[4])) / 2 : double(e[1]);
          rssiAvgD_[i] = rssiAvgD_[i] ? rssiAvgD_[i] * 0.7 + d * 0.3 : d;
          rssiAvgT_[i] = rssiAvgT_[i] ? rssiAvgT_[i] * 0.8 + double(e[2]) * 0.2 : double(e[2]);
          s.rssiDongle = -int(std::lround(rssiAvgD_[i]));
          s.rssiDevice = -int(std::lround(rssiAvgT_[i]));
        }
        s.msSinceResponse = Rd32(e + 6);
        state_.currentChannel = e[5];
      }
    }
  }
  if (now >= nextSlow_) {
    nextSlow_ = now + 1.0;
    {
      const uint8_t keep = 0xFF;   // read only: leaves the adaptive timing as it is
      if (Command(kTiming, &keep, 1, &r) && r.data.size() >= 42) {
        const uint8_t* d = r.data.data();
        std::lock_guard lock(mutex_);
        state_.haveTiming = true;
        state_.cycleUs = Rd32(d);
        state_.slotUs = Rd32(d + 4);
        state_.answerBytes = d[21];
        const bool sizing = state_.firmware >= 0x0028;
        state_.normalAnswerBytes = sizing ? d[32] : 0;
        state_.saturationPercent = sizing ? d[30] : -1;
        state_.largeAnswers = sizing && d[31];
        for (int i = 0; i < kSlots; ++i) state_.slots[i].saturationPercent = sizing ? d[34 + i] : -1;
      }
    }
    if (Command(kInfo, nullptr, 0, &r) && std::memcmp(r.data.data(), "OHDG", 4) == 0) {
      const uint8_t* d = r.data.data();
      std::lock_guard lock(mutex_);
      state_.firmware = d[4] | d[5] << 8;
      state_.softdeviceVariant = d[6];
      state_.board = d[57] <= 3 ? d[57] : 0;
      state_.bank = d[59];
      state_.slotCount = d[60] >= 1 && d[60] <= kSlots ? d[60] : 8;
      state_.watchdog = d[7];
      state_.resetReason = Rd32(d + 12);
      state_.uptimeMs = Rd32(d + 16);
      state_.crashCount = Rd32(d + 28);
      state_.crashType = Rd32(d + 32);
      state_.crashPc = Rd32(d + 36);
    }
    for (int first = 0; first < kSlots; first += 6) {   // 6 slots per reply
      const uint8_t arg = uint8_t(first);
      if (!Command(kSlotsCmd, &arg, 1, &r)) continue;
      std::lock_guard lock(mutex_);
      for (int i = first; i < kSlots && i < first + 6; ++i) {
        auto& s = state_.slots[i];
        const uint8_t* e = r.data.data() + (i - first) * 10;
        const std::string id = e[0] ? Hex(e + 2, 8) : std::string();
        if (id != s.pairId) { s.pairId = id; s.id = {}; s.nickname.clear(); s.nameRead = false; identTried_[i] = 0; configTried_[i] = 0; configNext_[i] = 0; }
        if (!id.empty() && s.id.serial.empty())
          for (const auto& [k, v] : known_)
            if (k == id) s.id = v;
      }
      const uint8_t p = r.data[60];
      state_.pairing = p & 1;
      state_.pairTarget = state_.pairing && (p >> 1) ? (p >> 1) - 1 : -1;
      if (!state_.pairing) state_.pairUntil = 0;
    }
    if (Command(kChannels, nullptr, 0, &r)) {
      std::lock_guard lock(mutex_);
      state_.channelMask = uint16_t(r.data[0] | r.data[1] << 8);
      for (int k = 0; k < kHops; ++k) { state_.hopChannel[k] = r.data[2 + k]; state_.hopLoss[k] = r.data[18 + k]; }
      state_.joinsHeard = Rd32(r.data.data() + 34);
    }
    const uint8_t keep = 0xFF;
    if (Command(kLink, &keep, 1, &r)) { std::lock_guard lock(mutex_); state_.linkOn = r.data[0]; }
    // names, kept on the dongle (0x0025+): every paired slot's, once a second
    bool names = false;
    { std::lock_guard lock(mutex_); names = state_.firmware >= 0x0025; state_.names = names; }
    for (int i = 0; names && i < kSlots; ++i) {
      { std::lock_guard lock(mutex_); if (state_.slots[i].pairId.empty()) continue; }
      const uint8_t a = uint8_t(i);
      if (!Command(kGetName, &a, 1, &r) || r.status != 0) continue;
      const size_t n = std::min<size_t>(r.data[0], kNameMax);
      std::lock_guard lock(mutex_);
      state_.slots[i].nickname.assign(reinterpret_cast<const char*>(r.data.data() + 1), n);
      state_.slots[i].nameRead = true;
    }
    // identities: 'H' from our firmware, tried a few times per slot as it joins
    for (int i = 0; i < kSlots; ++i) {
      bool want;
      {
        std::lock_guard lock(mutex_);
        const auto& s = state_.slots[i];
        want = s.active && s.id.serial.empty() && identTried_[i] < 3;
      }
      if (want) { identTried_[i]++; Ident(i); }
      // No 'H' (HTC's and Valve's main firmware don't send it): read the device's config for its
      // serial and model, unless the driver is running (it names devices and reads them itself).
      bool read;
      {
        std::lock_guard lock(mutex_);
        const auto& s = state_.slots[i];
        read = autoIdentify_ && s.active && s.id.serial.empty() && identTried_[i] >= 1 && configTried_[i] < 3 &&
               now >= configNext_[i];
      }
      if (read) {
        configTried_[i]++;
        configNext_[i] = now + 10;
        Identify(i, nullptr);
      }
    }
    // totals for the header
    std::lock_guard lock(mutex_);
    state_.joined = state_.paired = 0;
    state_.kBytesPerSec = 0;
    for (const auto& s : state_.slots) {
      state_.paired += s.used;
      state_.joined += s.active;
      if (s.active) state_.kBytesPerSec += s.kBytesPerSec;
    }
  }
  if (now >= nextStats_) {  // every paired slot's counters once a second, all together
    nextStats_ = now + 1.0;
    for (int slot = 0; slot < kSlots; ++slot) {
      bool used;
      { std::lock_guard lock(mutex_); used = state_.slots[slot].used; }
      if (used) PollStats(slot, now);
    }
  }
}

void Dongle::PollStats(int slot, double now) {
  Reply r, c;
  const uint8_t a = uint8_t(slot);
  if (!Command(kStats, &a, 1, &r)) return;
  const bool codec = Command(kCodec, &a, 1, &c);
  uint32_t v[15];
  std::memcpy(v, r.data.data(), sizeof v);
  auto& k = counters_[slot];
  std::lock_guard lock(mutex_);
  auto& s = state_.slots[slot];
  if (k.have && now > k.t) {
    const double dt = now - k.t;
    const auto rate = [&](int i) { return double(v[i] - k.v[i]) / dt; };
    s.pollsPerSec = rate(0);
    const uint32_t polls = v[0] - k.v[0], ok = v[1] - k.v[1];
    s.lossPercent = polls ? 100.0 * (1.0 - double(std::min(ok, polls)) / polls) : -1;
    s.crcPerSec = rate(2);
    s.timeoutsPerSec = rate(3);
    s.dupPerSec = rate(4);
    s.framesPerSec = rate(5);
    s.kBytesPerSec = rate(6) / 1000;
    s.dropsPerSec = rate(7);
    s.resendsPerSec = rate(9);
    if (codec) {
      s.airKBytesPerSec = double(Rd32(c.data.data()) - k.air) / dt / 1000;
      s.deviceDropsPerSec = double(uint16_t(Rd32(c.data.data() + 12) - k.devDrops)) / dt;
      s.deviceReplyDropsPerSec = double(uint16_t(Rd32(c.data.data() + 16) - k.devReplyDrops)) / dt;
    }
    s.imuHz = double(k.imu - k.imu0) / dt;
    s.lightPerSec = double(k.light - k.light0) / dt;
    s.inputPerSec = double(k.input - k.input0) / dt;
    if (!s.active) s.pollsPerSec = s.framesPerSec = s.kBytesPerSec = s.airKBytesPerSec = s.imuHz = s.lightPerSec = 0;
    Push(s.lossHistory, s.active ? float(std::max(0.0, s.lossPercent)) : 0.f);
    Push(s.rssiHistory, s.active ? float(s.rssiDongle) : -100.f);
    Push(s.kbpsHistory, float(s.kBytesPerSec));
  }
  s.deviceMissed = int(v[12]);
  s.deviceCrc = int(v[13]);
  s.frames = v[5];
  s.drops = v[7];
  k.have = true;
  k.t = now;
  std::memcpy(k.v, v, sizeof v);
  if (codec) {
    k.air = Rd32(c.data.data());
    k.devDrops = uint16_t(Rd32(c.data.data() + 12));
    k.devReplyDrops = uint16_t(Rd32(c.data.data() + 16));
  }
  k.imu0 = k.imu; k.light0 = k.light; k.input0 = k.input;
}

void Dongle::Run() {
  if (!Open()) {
    std::lock_guard lock(mutex_);
    state_.error = "can't open it (in use, or no permission: see the udev rule in the README)";
    alive_ = false;
    return;
  }
  { std::lock_guard lock(mutex_); state_.open = true; }
  while (!stop_ && alive_) {
    std::function<void()> action;
    {
      std::lock_guard lock(mutex_);
      if (!actions_.empty()) { action = std::move(actions_.front()); actions_.pop_front(); }
    }
    if (action) action();
    Poll(Now());
    ReadPending(5);
  }
  Close();
  std::lock_guard lock(mutex_);
  state_.open = false;
}

// ---- actions ----

void Dongle::Pair(int seconds, int slot, Done done) {
  Queue([=, this] {
    const uint8_t a[2] = {uint8_t(std::clamp(seconds, 0, 255)), uint8_t(slot >= 0 && slot < kSlots ? slot + 1 : 0)};
    Reply r;
    const bool ok = Command(kPair, a, 2, &r);
    if (ok) {
      std::lock_guard lock(mutex_);
      state_.pairing = seconds > 0;
      state_.pairTarget = seconds > 0 && slot >= 0 && state_.firmware >= 4 ? slot : -1;
      state_.pairUntil = seconds > 0 ? Now() + seconds : 0;
    }
    if (done) {
      if (!ok) done(false, "the dongle didn't answer");
      else if (slot >= 0 && state_.firmware < 4) done(true, "Pairing (this dongle firmware pairs into the first free slot: update it to choose)");
      else done(true, slot >= 0 ? "Pairing into slot " + std::to_string(slot) : "Pairing into the first free slot");
    }
  });
}

void Dongle::StopPairing(Done done) {
  Queue([=, this] {
    const uint8_t a[2] = {0, 0};
    const bool ok = Command(kPair, a, 2, nullptr);
    if (ok) { std::lock_guard lock(mutex_); state_.pairing = false; state_.pairUntil = 0; }
    if (done) done(ok, ok ? "Pairing stopped" : "the dongle didn't answer");
  });
}

void Dongle::Forget(int slot, Done done) {
  Queue([=, this] {
    const uint8_t a = slot < 0 ? 0xFF : uint8_t(slot);
    const bool ok = Command(kForget, &a, 1, nullptr);
    if (done) done(ok, ok ? (slot < 0 ? "Every slot unpaired" : "Slot " + std::to_string(slot) + " unpaired") : "the dongle didn't answer");
  });
}

void Dongle::PowerOff(int slot, Done done) {
  Queue([=, this] {
    const uint8_t a[2 + 7] = {uint8_t(slot), 7, 'P', 0x9f, 4, 'o', 'f', 'f', '!'};
    const bool ok = Command(kDown, a, sizeof a, nullptr);
    if (done) done(ok, ok ? "Power off sent to slot " + std::to_string(slot) : "the dongle didn't answer");
  });
}

void Dongle::Identify(int slot, Done done) {
  Queue([=, this] {
    std::string json, error;
    const auto read = [&](uint8_t* data, size_t size) -> int {
      return FeatureRead(slot, data, size) ? int(size) : -1;
    };
    const bool ok = lh::usb::ReadDeviceConfig(read, &json, &error);
    if (!ok) { if (done) done(false, "no config from the device (" + error + ")"); return; }
    const auto j = nlohmann::json::parse(json, nullptr, false);
    Identity id;
    id.serial = j.is_object() ? j.value("device_serial_number", std::string()) : std::string();
    id.model = j.is_object() ? ModelName(j) : std::string();
    id.source = "config";
    {
      std::lock_guard lock(mutex_);
      auto& s = state_.slots[slot];
      if (!id.serial.empty()) s.id = id;
    }
    if (done) done(!id.serial.empty(), id.serial.empty() ? "the config has no serial" : "Slot " + std::to_string(slot) + ": " + id.serial + " (" + id.model + ")");
  });
}

void Dongle::Scan(Done done) {
  Queue([=, this] {
    std::vector<uint8_t> all;
    bool ok = true;
    for (uint8_t first : {uint8_t(0), uint8_t(60)}) {  // 60 channels a request: 2400..2480 MHz in two
      Reply r;
      if (!Command(kScan, &first, 1, &r, 1500)) { ok = false; break; }
      all.insert(all.end(), r.data.begin(), r.data.begin() + 60);
    }
    if (ok) {
      all.resize(81);
      std::lock_guard lock(mutex_);
      state_.scan = all;
      state_.scanFirst = 0;
      state_.scanTime = Now();
    }
    if (done) done(ok, ok ? "Band scanned" : "the dongle didn't answer the scan");
  });
}

void Dongle::Reset(Done done) {
  Queue([=, this] {
    const bool ok = Command(kReset, nullptr, 0, nullptr);
    if (done) done(ok, ok ? "Dongle restarting" : "the dongle didn't answer");
  });
}

void Dongle::EnterBootloader(Done done) {
  Queue([=, this] {
    const bool ok = Command(kBootloader, nullptr, 0, nullptr);
    if (done) done(ok, ok ? "Dongle in update mode" : "the dongle didn't answer");
  });
}

void Dongle::BankUpdate(std::vector<uint8_t> image, std::function<void(float)> progress, Done done) {
  Queue([=, this, image = std::move(image)]() mutable {
    image.resize((image.size() + 3) & ~size_t(3), 0xFF);
    const uint32_t crc = lh::dongle_bank::Crc32(image);
    uint8_t args[61];
    const uint32_t size = uint32_t(image.size());
    std::memcpy(args, &size, 4); std::memcpy(args + 4, &crc, 4);
    Reply r;
    if (!Command(lh::dongle_bank::kBegin, args, 8, &r, 15000) || r.status) { if (done) done(false, "the dongle didn't start the update"); return; }  // erases first
    for (uint32_t off = 0; off < size; off += lh::dongle_bank::kChunk) {
      const uint8_t n = uint8_t(std::min<uint32_t>(lh::dongle_bank::kChunk, size - off));
      std::memcpy(args, &off, 4); args[4] = n; std::memcpy(args + 5, image.data() + off, n);
      if (!Command(lh::dongle_bank::kWrite, args, 5u + n, &r, 1000) || r.status) { if (done) done(false, "writing failed"); return; }
      if (progress) progress(float(off + n) / float(size));
    }
    if (!Command(lh::dongle_bank::kCommit, nullptr, 0, &r, 2000) || r.status) { if (done) done(false, "the new firmware didn't check out"); return; }
    if (done) done(true, "Done: the dongle restarts with the new firmware");
  });
}

void Dongle::ClearCrash(Done done) {
  Queue([=, this] {
    const bool ok = Command(kClearCrash, nullptr, 0, nullptr);
    if (done) done(ok, ok ? "Crash record cleared" : "the dongle didn't answer");
  });
}

// ---------------------------------------------------------------------------------------------
// A Dongle's destructor waits for its thread, which can sit in a USB call on a device that's going
// away (Windows takes its time there): never on the UI thread.
static std::atomic<int> g_dropping{0};
static void DropLater(std::shared_ptr<Dongle> d) {
  if (!d) return;
  ++g_dropping;
  std::thread([d = std::move(d)]() mutable { d.reset(); --g_dropping; }).detach();
}

DongleManager::DongleManager() {
  hid_init();
  scanner_ = std::thread([this] { Scan(); });
}
DongleManager::~DongleManager() {
  stop_ = true;
  if (scanner_.joinable()) scanner_.join();
  dongles_.clear();
  // Dongles still closing on their own threads: hidapi must outlive them (bounded: we're exiting).
  for (int i = 0; i < 30 && g_dropping > 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (g_dropping == 0) hid_exit();
}

void DongleManager::Scan() {
  while (!stop_) {
    std::vector<Found> found;
    hid_device_info* list = hid_enumerate(kVid, kPid);
    for (hid_device_info* d = list; d; d = d->next)
      if (Narrow(d->product_string).rfind("Doorman Dongle", 0) == 0 && d->interface_number <= 0)
        found.push_back({d->path, "OHDG-" + Narrow(d->serial_number)});
    hid_free_enumeration(list);
    {
      std::lock_guard lock(foundMutex_);
      found_ = std::move(found);
      foundFresh_ = true;
    }
    for (int i = 0; i < 20 && !stop_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void DongleManager::Poll(double) {
  for (auto it = dongles_.begin(); it != dongles_.end();) {
    if (!(*it)->Alive() && (*it)->Snapshot().error.empty()) { DropLater(std::move(*it)); it = dongles_.erase(it); }
    else ++it;
  }
  std::vector<Found> found;
  {
    std::lock_guard lock(foundMutex_);
    if (!foundFresh_) return;
    foundFresh_ = false;
    found = found_;
  }
  for (const auto& f : found) {
    bool have = false;
    for (auto& x : dongles_)
      if (x->Serial() == f.serial) {
        have = true;
        if (!x->Alive()) {  // retry one that failed to open
          DropLater(std::move(x));
          x = std::make_shared<Dongle>(f.path, f.serial);
        }
      }
    if (!have) dongles_.push_back(std::make_shared<Dongle>(f.path, f.serial));
  }
}

}  // namespace openhouse::dongle
