/* Valve nRF52 radio layout: bootloader 0x0-0xFFF, app 0x1000-0xFFFF, staging 0x10000-0x1F7FF, Valve's
   record page 0x1F800 (never linked into). RAM kept within what the stock app used (SP 0x20003240). */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00001000, LENGTH = 0xF000
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x3240
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
  ASSERT(_estack - . >= 1024, "less than 1 KB left for the stack")
}
