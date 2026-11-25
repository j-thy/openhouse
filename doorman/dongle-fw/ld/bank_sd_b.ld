/* Firmware image for bank B, layout "sd" (src/banks.h; Pro Micro "sd" build: the SoftDevice area (0x1000-0x25FFF) is left alone):
   after the bank's 4 KB header page. Started by the boot stub. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x0008F000, LENGTH = 0x000EC000 - 0x0008F000
  RAM   (rwx) : ORIGIN = 0x20003400, LENGTH = 0x3E000 - 0x3400  /* below: MBR/SoftDevice RAM (untouched). 0x2003E000+: crash record */
}
INCLUDE "nrf_common.ld"
