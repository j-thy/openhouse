/* Boot stub, layout "std" (src/stub.c, src/banks.h; MBR only: Nordic DFU boards (Holyiot, PCA10059, Kuroshin) and the Pro Micro "nosd" build): at the app start,
   followed by its payload (bank A's image). Must end below bank A: the linker checks it. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00001000, LENGTH = 0x00010000 - 0x00001000
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x3E000
}
ENTRY(stub_main)
SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) } > FLASH
  .payload : { KEEP(*(.payload)) } > FLASH
  /DISCARD/ : { *(.data*) *(.bss*) *(.ARM.*) }
  __stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
