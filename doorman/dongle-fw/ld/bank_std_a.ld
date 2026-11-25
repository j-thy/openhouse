/* Firmware image for bank A, layout "std" (src/banks.h; MBR only: Nordic DFU boards (Holyiot, PCA10059, Kuroshin) and the Pro Micro "nosd" build):
   after the bank's 4 KB header page. Started by the boot stub. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00011000, LENGTH = 0x00070000 - 0x00011000
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x3E000  /* 0x2003E000+: crash record, see main.c */
}
INCLUDE "nrf_common.ld"
