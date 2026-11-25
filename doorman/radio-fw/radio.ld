/* HTC radio layout: app 0x400-0xFFFF, staging 0x10000-0x1F7FF (never linked into). RAM kept within
   what HTC's app used (SP 0x20001578), since we don't know this chip's RAM size for sure. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000400, LENGTH = 0xFC00
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x1570
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
