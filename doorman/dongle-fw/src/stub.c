/* Boot stub (banks.h). Installed by the bootloader at the board's app start, together with a copy of
   the firmware built for bank A (payload). It boots the newest valid bank. It installs the payload
   first when no bank is valid, or when the newest bank's chain didn't start from this payload (the
   bootloader just installed a new one): into the other bank, so the old one stays as a fallback until
   the new one is written. No interrupts, no peripherals besides NVMC. */
#include "nrf.h"
#include "banks.h"

extern const uint8_t payload_start[], payload_end[];

static void nvmc_wait(void) { while (!NRF_NVMC->READY) {} }

static void erase(uint32_t addr, uint32_t len)
{
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een; nvmc_wait();
  for (uint32_t a = addr; a < addr + len; a += 0x1000) { NRF_NVMC->ERASEPAGE = a; nvmc_wait(); }
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren; nvmc_wait();
}

static void write(uint32_t addr, const uint8_t *src, uint32_t len)
{
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen; nvmc_wait();
  for (uint32_t i = 0; i < len; i += 4) {
    uint32_t w = (uint32_t)src[i] | (uint32_t)src[i + 1] << 8 | (uint32_t)src[i + 2] << 16 | (uint32_t)src[i + 3] << 24;
    *(volatile uint32_t *)(addr + i) = w; nvmc_wait();
  }
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren; nvmc_wait();
}

/* Bank A's image only runs from bank A: the payload goes there. If bank A is the one to keep, the
   payload can't be installed without losing it, so then bank A is overwritten anyway (the payload is
   what the bootloader was just asked to install). */
static int install_payload(uint32_t seq, uint32_t origin)
{
  const uint32_t size = (uint32_t)(payload_end - payload_start);
  if (!size || size > BANK_IMAGE_MAX || (size & 3)) return 0;
  erase(BANK_A, BANK_IMAGE_OFS + ((size + 0xFFFu) & ~0xFFFu));
  write(BANK_A + BANK_IMAGE_OFS, payload_start, size);
  const bank_header_t h = { BANK_MAGIC, seq, size, bank_crc32(payload_start, size), origin };
  write(BANK_A, (const uint8_t *)&h, sizeof h);
  return bank_valid(BANK_A);
}

static __attribute__((noreturn)) void jump(uint32_t base)
{
  const uint32_t *vec = (const uint32_t *)(base + BANK_IMAGE_OFS);
  SCB->VTOR = (uint32_t)vec;
  __DSB(); __ISB();
  __set_MSP(vec[0]);
  ((void (*)(void))vec[1])();
  for (;;) {}
}

__attribute__((noreturn)) void stub_main(void)
{
  const uint32_t origin = bank_crc32(payload_start, (uint32_t)(payload_end - payload_start));
  const int a = bank_valid(BANK_A), b = bank_valid(BANK_B);
  uint32_t best = 0;
  if (a) best = BANK_A;
  if (b && (!a || bank_header(BANK_B)->seq > bank_header(BANK_A)->seq)) best = BANK_B;
  if (!best || bank_header(best)->origin != origin) {
    const uint32_t seq = best ? bank_header(best)->seq + 1 : 1;
    if (install_payload(seq, origin)) best = BANK_A;
    else if (best && !bank_valid(best)) best = 0;      /* bank A was the newest and is gone now */
  }
  if (best) jump(best);
  for (;;) { __WFE(); }                                /* nothing to run: the bootloader is the way in */
}

/* The vector table: just the stack and the entry point (no interrupts are enabled here). */
extern uint32_t __stack_top;
__attribute__((section(".isr_vector"), used)) const void *const stub_vectors[2] = { &__stack_top, (void *)stub_main };

/* The payload: bank A's raw image, linked in at build time. */
__asm__(".section .payload,\"a\"\n .balign 4\n .global payload_start\npayload_start:\n"
        " .incbin \"" PAYLOAD_BIN "\"\n .balign 4\n .global payload_end\npayload_end:\n .text\n");
