/* The radio's batch path (main.c batch_push / batch_close), built for the Index-family radio's Cortex-M4 with
   the radio's flags, for run_emu.py to time on real frames. DMC_YIELD marks where the radio checks for a POLL:
   nothing between two may run past the answer window (~180 us). */
#include <stdint.h>
#include <stddef.h>
void *memset(void *d, int c, size_t n) { uint8_t *p = d; while (n--) *p++ = (uint8_t)c; return d; }
void *memcpy(void *d, const void *s, size_t n) { uint8_t *p = d; const uint8_t *q = s; while (n--) *p++ = *q++; return d; }
int memcmp(const void *a, const void *b, size_t n) { const uint8_t *p = a, *q = b; for (; n; n--, p++, q++) if (*p != *q) return *p - *q; return 0; }
__attribute__((noinline)) void yield_hook(void) { __asm volatile(""); }
#define DMC_YIELD() yield_hook()
#include "doorman_link.h"
#include "doorman_lh2p.h"
dmc_state_t codec; dmp_state_t lh2p; dmb_t bt; uint8_t bat[256], in_buf[80], out_buf[256]; int bat_open, cap, out_len;   /* set in init(): emu.ld leaves .data uninitialised */
void init(int answer) { cap = answer - DML_RESP_HDR - 1; dmc_reset(&codec); codec.check = 1; dmp_reset(&lh2p); bat_open = 0; out_len = 0; }
int close_b(void) { if (!bat_open) return 0; bat_open = 0; int m = dmb_end(&bt, 1); memcpy(out_buf, bat, (size_t)m); return m; }
/* returns a closed batch's length (in out_buf) if this push closed one, else 0 */
int push(int n) {
  int closed = 0;
  if (bat_open && (!dmb_room(&bt) || bt.w.cap * 8 - bt.w.pos < (n * 3 / 4 + 4) * 8)) closed = close_b();
  if (!bat_open) { dmb_begin(&bt, bat, cap); bat_open = 1; }
  if (!dmb_add(&bt, &codec, &lh2p, in_buf, n)) { closed = close_b(); dmb_begin(&bt, bat, cap); bat_open = 1; dmb_add_raw(&bt, in_buf, n); }
  return closed;
}
