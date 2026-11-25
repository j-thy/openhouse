#include "wireless_decoder.hpp"

#include <cstring>

namespace lh::usb {
namespace {
template <typename T>
T Rd(const uint8_t* p) {
  T v;
  std::memcpy(&v, p, sizeof v);
  return v;
}
}  // namespace

void WirelessPacket::Clear() { *this = WirelessPacket(); }

// ---------------------------------------------------------------- legacy

uint32_t LegacyDecoder::ReconstructTs(uint8_t msb, uint32_t ref, uint32_t ts) {
  // 0x3fd540: extend a 24-bit stamp with msb-1 / msb / msb+1, nearest to ref.
  ts &= 0xffffff;
  const uint32_t a = uint32_t(msb - 1) * 0x1000000u | ts;
  const uint32_t b = uint32_t(msb) << 24 | ts;
  const uint32_t c = uint32_t(msb + 1) * 0x1000000u | ts;
  auto dist = [&](uint32_t x) {
    uint32_t d = ref < x ? x - ref : ref - x;
    if (0x80000000u < d) d = 0u - d;
    return d;
  };
  const uint32_t da = dist(a), db = dist(b), dc = dist(c);
  if (da < db) return da < dc ? a : c;
  return db < dc ? b : c;
}

bool LegacyDecoder::DecodeLightcap(const uint8_t* p, int n, WirelessPacket* out) {
  // 0x3fdb00. Light data is a sequence of sensor bytes read from the front and a
  // varint-encoded list of decreasing timestamps read backwards from the end
  // (the last 3 bytes are the newest 24-bit timestamp).
  out->pulses.clear();
  if (n == 0) return true;
  if (p == nullptr || n < 5) return false;
  struct T { uint32_t t; bool used; };
  std::vector<T> times;
  times.reserve(20);
  int back = n - 4;
  times.push_back({uint32_t(p[n - 1]) << 16 | p[n - 3] | uint32_t(p[n - 2]) << 8, false});
  auto readVarint = [&](uint32_t* v) -> bool {
    uint32_t acc = 0;
    long idx = back - 1;
    for (;;) {
      const uint8_t c = p[idx + 1];
      back = int(idx);
      acc = acc << 7 | (c & 0x7f);
      if (c & 0x80) break;
      idx--;
      if (idx == -2) return false;
    }
    if (back == -1) return false;
    *v = acc;
    return true;
  };
  long k = 1;
  for (;;) {
    const uint8_t b = p[k - 1];
    size_t it = 0;
    while (it < times.size() && times[it].used) it++;
    if (it == times.size()) {
      uint32_t v;
      if (!readVarint(&v)) return false;
      times.push_back({times.back().t - v, false});
      it = times.size() - 1;
    }
    const size_t need = it + 1 + (b & 7);
    while (times.size() <= need) {
      uint32_t v;
      if (!readVarint(&v)) return false;
      times.push_back({times.back().t - v, false});
    }
    T& start = times[it];
    T& end = times[need];
    if (start.used || end.used) return false;
    start.used = true;
    end.used = true;
    if (0xffff < start.t - end.t) return false;
    WirelessPacket::Pulse pl;
    pl.sensor = b >> 3;
    pl.ts = end.t;
    pl.width = uint16_t(start.t - end.t);
    out->pulses.push_front(pl);
    if (back <= int(k) && times.size() <= size_t(k * 2)) break;
    k++;
  }
  return int(k) - 1 == back;
}

bool LegacyDecoder::Decode(uint8_t msb, const uint8_t* p, int n, WirelessPacket* out) const {
  // 0x3fdec0
  out->Clear();
  const uint8_t* end = p + n;
  const uint8_t* q = p + 1;
  if (end == q) return true;
  const uint8_t lsb = p[0];
  const uint8_t b1 = p[1];
  if ((b1 >> 3) == 0x1c) {  // 0xe0..0xe7: status header
    if ((b1 & 1) == 0) {
      q = p + 2;
    } else {
      out->hasBattery = true;
      out->battery = p[2];
      q = p + 3;
    }
    if (b1 & 2) {
      out->versionChange = true;
      out->newVersion = *q;
      q++;
    }
    q += (b1 & 4) != 0;
    if (q == end) return true;
  }
  const uint8_t b = *q;
  const int t = b >> 3;
  const uint8_t* r = q;
  bool imu = false;
  if (t == 0x1e || t == 0x1f) {  // input event (0x1f: followed by IMU)
    if ((b & 7) == 0) {
      out->hasInput8 = true;
      out->input8 = Rd<uint64_t>(q + 1);
      r = q + 9;
    } else {
      r = q + 1;
      if (b & 1) {
        out->hasButtons = true;
        out->buttons = q[1];
        r = q + 2;
      }
      if (b & 4) {
        out->hasTrigger = true;
        out->trigger = *r;
        r++;
      }
      if (b & 2) {
        out->hasTouchpad = true;
        out->touchpad = Rd<uint32_t>(r);
        r += 4;
      }
    }
    imu = t == 0x1f;
  } else if (t == 0x1c) {
    return false;
  } else if (t == 0x1d) {  // 0xe8..0xef: IMU
    r = q + 1;
    imu = true;
  }
  if (imu) {
    out->imuTs = uint32_t(msb) << 24 | uint32_t(lsb) << 16 | uint32_t(*r) << 8;
    std::memcpy(out->imuRaw, r + 1, 12);
    out->hasImu = true;
    r += 0xd;
  }
  if (!DecodeLightcap(r, int(n - (r - p)), out)) return false;
  const uint32_t ref = ((uint32_t(msb) << 24) | (uint32_t(lsb) << 16)) - 0x744800u;
  for (auto& pl : out->pulses) pl.ts = ReconstructTs(msb, ref, pl.ts);
  return true;
}

// ---------------------------------------------------------------- Wv2

bool Wv2Decoder::Decode(uint8_t msb, const uint8_t* p, int n, WirelessPacket* out) {
  // 0x3fe6d0
  out->Clear();
  if (n < 2) return false;
  const uint8_t f = p[1];
  if ((f & 0x80) && n < 15) {
    if (f == 0xe2 && n == 3) {
      out->versionChange = true;
      out->newVersion = p[2];
    }
    return true;
  }
  const bool light = f & 0x10, input = f & 0x20, meta = f & 0x40;
  int sections;
  if (!meta) sections = int(input) + int(light);
  else if (!input) sections = light ? 2 : 1;
  else {
    if (light) return false;
    sections = 2;
  }
  const uint8_t lsb = p[0];
  const uint8_t* q = p + 2;
  if (f & 0x80) {
    q = p + 0xf;
    out->imuTs = uint32_t(p[2]) << 8 | uint32_t(lsb) << 16 | uint32_t(msb) << 24;
    std::memcpy(out->imuRaw, p + 3, 12);
    out->hasImu = true;
  }
  int len = (f & 0xf) + 1;
  bool doInput = input;
  // Lengths come from the packet: none may run past its n bytes (a section announced
  // longer than what was left made the light copy below memcpy a negative length).
  const uint8_t* const end = p + n;
  if (meta) {
    if (sections != 2) {
      len = n - int(q - p);
      if (len <= 0) return false;
    }
    if (q + len > end) return false;
    const uint8_t m = *q;
    // The fields m announces must fit in the section (the r > q check below, made before
    // reading them rather than after).
    if (1 + !!(m & 0x40) + !!(m & 0x80) + !!(m & 0x20) + !!(m & 0x10) > len) return false;
    const uint8_t* r = q + 1;
    if (m & 0x40) {
      out->hasEvent = true;
      out->eventValue = q[1];
      r = q + 2;
      out->eventTs = out->hasImu ? out->imuTs : (uint32_t(msb) << 24 | uint32_t(lsb) << 16);
    }
    if (m & 0x80) {
      out->hasBattery = true;
      out->battery = *r++;
    }
    if (m & 0x20) {
      const uint8_t v = *r++;
      out->opticalSubChanged = opticalSub != v;
      opticalSub = v;
    }
    if (m & 0x10) {
      const uint8_t v = *r++;
      out->controllerSubChanged = controllerSub != v;
      controllerSub = v;
    }
    q += len;
    if (r > q || q > end) return false;
    if (doInput) {
      len = n - int(q - p);
      if (len < 1) return false;
    }
  } else if (doInput && sections != 2) {
    len = n - int(q - p);
    if (len < 1) return false;
  }
  if (doInput) {
    if (len < 0 || len > n - int(q - p)) return false;
    if (controllerSub == 2) {
      if (len > 4) {
        out->hasInputV2 = true;
        out->buttons = q[0];
        out->trigger = q[1];
        out->inputA2 = q[2];
        out->touchpad = uint32_t(uint16_t(int16_t(int8_t(q[3])) << 8)) |
                        uint32_t(uint16_t(int16_t(int8_t(q[4])) << 8)) << 16;
      }
    } else if (controllerSub < 3) {
      if (controllerSub == 0) {
        if (len >= 0x1c) return false;
        uint8_t buf[0x20];
        buf[0] = lsb;
        std::memcpy(buf + 1, q, size_t(len));
        WirelessPacket tmp;
        if (!LegacyDecoder().Decode(msb, buf, len + 1, &tmp)) return false;
        out->hasButtons = tmp.hasButtons;
        out->hasTrigger = tmp.hasTrigger;
        out->hasTouchpad = tmp.hasTouchpad;
        out->hasInput8 = tmp.hasInput8;
        out->buttons = tmp.buttons;
        out->trigger = tmp.trigger;
        out->touchpad = tmp.touchpad;
        out->input8 = tmp.input8;
      } else {
        out->rawInput.assign(q, q + len);
        out->hasRawInput = true;
      }
    } else if (controllerSub == 0xff) {
      return false;
    }
    q += len;
  }
  if (!light) return controllerSub != 0xff && opticalSub != 0xff;
  if (opticalSub == 1) {
    const uint8_t* e = p + n;
    int channel = -1;
    while (q < e) {
      const uint8_t b = *q;
      if ((b & 1) == 0) {
        if (q + 4 > e) break;
        const uint32_t w = Rd<uint32_t>(q);
        if (channel == -1) return false;
        WirelessPacket::Lh2Record rec;
        rec.channel = channel;
        if ((w & 2) == 0) {
          rec.type = 1;
          rec.sensor = -1;
          rec.ts24 = (w >> 2) & 0xffffff;
          rec.bit26 = (w >> 26) & 1;
          rec.bit27 = (w >> 27) & 1;
        } else {
          rec.type = 0;
          rec.sensor = int(w >> 27);
          rec.ts24 = (w >> 3) & 0xffffff;
          rec.halfTick = (w >> 2) & 1;
        }
        out->lh2.push_back(rec);
        q += 4;
      } else {
        if ((b & 0xf) == 1) channel = b >> 4;
        q++;
      }
    }
    if (q != e) return false;
    return controllerSub != 0xff;
  }
  if (opticalSub == 0xff) return false;
  if (opticalSub == 0) {
    const int ll = n - int(q - p);
    if (ll < 0 || ll >= 0x1c) return false;
    uint8_t buf[0x20];
    buf[0] = lsb;
    std::memcpy(buf + 1, q, size_t(ll));
    WirelessPacket tmp;
    if (!LegacyDecoder().Decode(msb, buf, ll + 1, &tmp)) return false;
    out->pulses = std::move(tmp.pulses);
    return controllerSub != 0xff;
  }
  return controllerSub != 0xff;
}

}  // namespace lh::usb
