// Doorman dongle self-update (doorman/dongle-fw/src/banks.h): a running dongle writes the firmware for
// the bank it isn't running into that bank and restarts into it. Commands 0x40 (begin: size, CRC; erases),
// 0x41 (write: offset, n, bytes), 0x42 (commit: checks the CRC, writes the bank header, restarts).
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace lh::dongle_bank {

constexpr uint8_t kBegin = 0x40, kWrite = 0x41, kCommit = 0x42;
constexpr size_t kChunk = 56;

inline uint32_t Crc32(const std::vector<uint8_t>& data) {
  uint32_t c = 0xFFFFFFFFu;
  for (uint8_t b : data) {
    c ^= b;
    for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

// The packaged image to send (firmwares/dongle/bank/<board>-<a|b>.bin): the one for the bank that
// isn't running. board: cmd 0x01 byte 57 (0 Pro Micro, 1 Holyiot, 2 PCA10059, 3 Kuroshin); sd: the
// Pro Micro build that keeps the SoftDevice area (byte 6); bank: the running one (byte 59: 1 A, 2 B).
inline std::string ImageName(int board, bool sd, int runningBank) {
  const char* name = board == 1 ? "holyiot" : board == 2 ? "pca10059" : board == 3 ? "kuroshin" : sd ? "sd" : "nosd";
  return std::string(name) + (runningBank == 1 ? "-b.bin" : "-a.bin");
}

}  // namespace lh::dongle_bank
