/* Firmware image for bank B, layout "std" (src/banks.h; MBR only: Nordic DFU boards (Holyiot, PCA10059, Kuroshin) and the Pro Micro "nosd" build):
   after the bank's 4 KB header page. Started by the boot stub. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00071000, LENGTH = 0x000D0000 - 0x00071000
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x3E000  /* 0x2003E000+: crash record, see main.c */
}
INCLUDE "nrf_common.ld"
