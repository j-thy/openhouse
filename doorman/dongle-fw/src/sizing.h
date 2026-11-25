/* Answer size by saturation (cmd 0x2D reports it).
 *
 * The answer limit (POLL max resp) sets how much a device sends per cycle. Each slot's share of the cycle
 * follows the slowest exchange of the last 32 cycles, so a bigger limit costs nothing while devices send
 * less than it: their answers stay short. It only matters when a device has more queued than one answer
 * holds, every cycle; then a bigger answer carries ~15 % more (7-8 devices: 176 -> 250 B), each slot's
 * fixed ~450 us overhead spread over more bytes, at up to ~2.4 ms more cycle.
 *
 * Saturated: a device left frames queued after half its answers over 2 s (the answer's "more" flag; older
 * radios: answers nearly as long as the normal limit for this many devices, so large answers don't hide it). Large answers then, until no
 * device is over a fifth for 10 s. */
#pragma once
#include <stdint.h>
#include "doorman_link.h"

#define SIZING_FULL_SLACK 70              /* older radios (no "more" flag): this short of the limit still counts as
                                             full, answers being filled with whole frames of up to 65 bytes */
#define SIZING_ON_PCT     50
#define SIZING_OFF_PCT    20
#define SIZING_CALM       5               /* rounds (2 s) under SIZING_OFF_PCT before normal answers again: at the
                                             edge of saturation it flipped every few seconds */

typedef struct {
  uint16_t answers[DML_SLOTS], full[DML_SLOTS];
  uint8_t pct[DML_SLOTS];                 /* last round, per slot */
  uint8_t worst, big, calm;
} sizing_t;

static inline int sizing_base(int n)      /* the normal limit for n devices (link_poll's table) */
{
  return n <= 1 ? DML_MAXLEN : n <= 2 ? 160 : n <= 4 ? 96 : n <= 6 ? 120 : 176;   /* 7-10: 176 */
}
static inline int sizing_resp(const sizing_t *z, int n) { return z->big ? DML_MAXLEN : sizing_base(n); }

/* A fresh answer from slot i: len bytes (header included), its flags, n devices in the plan; flagged: its
   radio sets the "more" flag. */
static inline void sizing_answer(sizing_t *z, int i, int len, uint8_t flags, int flagged, int n)
{
  if (i < 0 || i >= DML_SLOTS) return;
  if (z->answers[i] < 0xFFFF) z->answers[i]++;
  const int full = flagged ? (flags & DML_F_MORE) != 0 : len >= sizing_base(n) - SIZING_FULL_SLACK;
  if (full && z->full[i] < 0xFFFF) z->full[i]++;
}
/* Every 2 s. */
static inline void sizing_eval(sizing_t *z)
{
  z->worst = 0;
  for (int i = 0; i < DML_SLOTS; i++) {
    z->pct[i] = z->answers[i] >= 50 ? (uint8_t)(100u * z->full[i] / z->answers[i]) : 0;
    if (z->pct[i] > z->worst) z->worst = z->pct[i];
    z->answers[i] = z->full[i] = 0;
  }
  if (z->worst >= SIZING_ON_PCT) { z->big = 1; z->calm = 0; }
  else if (z->worst < SIZING_OFF_PCT) { if (z->calm < 255) z->calm++; if (z->calm >= SIZING_CALM) z->big = 0; }
  else z->calm = 0;
}
