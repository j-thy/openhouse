/* The synthetic LH2 generator (doorman_lh2gen.h) built for the Index-family radio's Cortex-M4, for gen_emu.py:
   its frames must match the host build's, and nothing between two DLG_YIELDs may run past the answer window. */
#include <stdint.h>
#include <stddef.h>
void *memset(void *d, int c, size_t n) { uint8_t *p = d; while (n--) *p++ = (uint8_t)c; return d; }
void *memcpy(void *d, const void *s, size_t n) { uint8_t *p = d; const uint8_t *q = s; while (n--) *p++ = *q++; return d; }
void *memmove(void *d, const void *s, size_t n) { uint8_t *p = d; const uint8_t *q = s; if (p < q) while (n--) *p++ = *q++; else { p += n; q += n; while (n--) *--p = *--q; } return d; }
__attribute__((noinline)) void yield_hook(void) { __asm volatile(""); }
#define DLG_YIELD() yield_hook()
#include "doorman_lh2gen.h"
dlg_t gen; uint8_t out_buf[4096]; int out_len;
static void emit(const uint8_t *f, int n, void *ctx) { (void)ctx; if (out_len + n + 1 <= (int)sizeof out_buf) { out_buf[out_len++] = (uint8_t)n; memcpy(out_buf + out_len, f, (size_t)n); out_len += n; } }
void init(int bases, int density) { dlg_init(&gen, 1, bases, density, 8); out_len = 0; }
int run(uint32_t now) { out_len = 0; dlg_run(&gen, now, emit, 0); return out_len; }
