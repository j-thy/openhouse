/* Test device on a Pro Micro (nRF52840), Adafruit UF2 bootloader "nosd" build: app from 0x00001000 (DML_FAKE). */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00001000, LENGTH = 0x40000
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x20000
}
ENTRY(Reset_Handler)
SECTIONS
{
  .text : { KEEP(*(.vectors)) *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
  _sidata = LOADADDR(.data);
  .data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
  .bss (NOLOAD) : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
  .noinit (NOLOAD) : { *(.noinit*) } > RAM
  _estack = ORIGIN(RAM) + LENGTH(RAM);
}
