#pragma once
/* Two-bank layout, every board (ai/doorman/dongle-fw.md). A bootloader (UF2 or Nordic DFU) installs only
   a small boot stub with a copy of the firmware as payload; the firmware runs from one of two banks
   above it and updates itself over USB by writing the other bank. The bootloader is only needed for a
   first install or a rescue (and a Kuroshin's can't even be entered from software).

   bank: [header page, 4 KB][image]. The header is written last, after the image's CRC checked out. The
   stub boots the valid bank with the highest sequence number. Each header carries the CRC of the stub
   payload its chain started from (origin): a different payload means the bootloader installed a new
   one, which then wins over the banks.

   Layouts: "std" (MBR only: Nordic DFU boards and the Pro Micro "nosd" build): stub at 0x1000, banks at
   0x10000 / 0x70000. "sd" (Pro Micro with the SoftDevice area kept): stub at 0x26000, banks at 0x30000 /
   0x8E000, ending at the pairing page (0xEC000). */
#include <stdint.h>

#if BANK_LAYOUT_SD
#define BANK_A          0x00030000u
#define BANK_B          0x0008E000u
#define BANK_SIZE       0x0005E000u
#else
#define BANK_A          0x00010000u
#define BANK_B          0x00070000u
#define BANK_SIZE       0x00060000u
#endif
#define BANK_IMAGE_OFS  0x1000u            /* header page, then the image */
#define BANK_IMAGE_MAX  (BANK_SIZE - BANK_IMAGE_OFS)
#define BANK_MAGIC      0x4B42484Fu        /* "OHBK" */

typedef struct { uint32_t magic, seq, size, crc, origin; } bank_header_t;

static inline uint32_t bank_crc32(const uint8_t *p, uint32_t n)
{
  uint32_t c = 0xFFFFFFFFu;
  while (n--) { c ^= *p++; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
  return ~c;
}

static inline const bank_header_t *bank_header(uint32_t base) { return (const bank_header_t *)base; }

/* The bank's image is complete and matches its header. */
static inline int bank_valid(uint32_t base)
{
  const bank_header_t *h = bank_header(base);
  if (h->magic != BANK_MAGIC || !h->size || h->size > BANK_IMAGE_MAX || (h->size & 3)) return 0;
  return bank_crc32((const uint8_t *)(base + BANK_IMAGE_OFS), h->size) == h->crc;
}
