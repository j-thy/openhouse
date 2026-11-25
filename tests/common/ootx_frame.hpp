#pragma once

#include <cstdint>
#include <vector>

namespace lh::test {
// The lhbs firmware's OOTX encoder (basestation_firmware/src/ootx.c): 17 zeros, a 1, then
// u16 length, payload, pad, crc32, each byte MSB first, a 1 stuffed after 16 bits.
inline std::vector<uint8_t> LhbsFrame(const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> bits(17, 0);
  bits.push_back(1);
  auto bit = [&](int b) {
    bits.push_back(uint8_t(b));
    if (bits.size() > 18 && bits.size() % 17 == 0) bits.push_back(1);
  };
  auto bytes = [&](const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i)
      for (int k = 7; k >= 0; --k) bit((p[i] >> k) & 1);
  };
  const uint8_t len[2] = {uint8_t(payload.size()), 0};
  bytes(len, 2);
  bytes(payload.data(), payload.size());
  if (payload.size() & 1) { const uint8_t z = 0; bytes(&z, 1); }
  uint32_t crc = ~0u;
  for (uint8_t c : payload) {
    crc ^= c;
    for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
  }
  crc = ~crc;
  bytes(reinterpret_cast<const uint8_t*>(&crc), 4);
  return bits;
}
}  // namespace
