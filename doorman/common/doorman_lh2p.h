/* LH2 predictive air codec ('s' frames -> 0xC6). Lossless: the dongle rebuilds the frame byte for byte.
 *
 * The device's FPGA already demodulates the beams: its stream has one beam word per base per rotation
 * (the sync time and the OOTX bits) and one sensor word per hit. Every beam passes the device once a
 * rotation (~20 ms), so each hit lands close to where the same sensor's hit on the same beam landed the
 * last two rotations, counted from its rotation's sync, and each sync close to the last plus the period.
 * The coder sends only how far each lands from that:
 *   beam word    bits 26-31 [6], EG(zz(ts - predicted sync))
 *   sensor word  EG(zz(k)): the hit is entry pointer+k of the last rotation's hits in predicted order (k = n:
 *                not there: [sensor 5] EG(zz(time - previous word's time))), then EG(zz(time - prediction))
 *                in half ticks; the half-tick bit is part of the time. The prediction is the same entry's
 *                last two hit times (linear), or its last plus the measured period: the sync's own timing
 *                jitter stays out of it.
 *   channel and other bytes as doorman_codec.h's 's' walk.
 * State per base (4 kept, as many as a device reports; a new one replaces the least recently synced): the last two syncs, the last two rotations'
 * hit offsets per (sensor, beam), and the last rotation's hits in time order. Both ends change it the same
 * way for every 's' frame, coded or raw (dmp_observe). Reset with the codec, on join.
 * Uses doorman_codec.h's bit writer/reader and Exp-Golomb coder. */
#pragma once
#include "doorman_codec.h"
#ifndef DMP_COUNT
#define DMP_COUNT(kind, bits) do { } while (0)   /* tests: where the bits go */
#endif

enum { DMC_LH2P = 0xC6 };
enum { DMP_SLOTS = 4, DMP_E = 64 };      /* entries: sensor * 2 + beam; a device reports 4 LH2 bases at most */
enum { DMP_K = 0, DMP_RES = 1, DMP_ESC = 2, DMP_SYNC = 3, DMP_NCTX = 4 };

typedef struct {
  int8_t ch;                 /* -1 free */
  uint8_t nsync, n1;         /* syncs seen (0..2); entries in order1 */
  uint32_t s1, s2;           /* last two syncs, ticks (24-bit) */
  uint32_t used;             /* recency stamp */
  int32_t cur[DMP_E], o1[DMP_E];   /* hit times this rotation and the last, half ticks (25-bit); DMP_NONE = none */
  int32_t pred1[DMP_E];      /* each entry's predicted time this rotation; while rotating, the one before last's times */
  uint8_t order1[DMP_E];     /* last rotation's entries by predicted time */
  uint8_t pos1[DMP_E];       /* each entry's place in order1 (0xFF: none) */
  uint8_t ptr;               /* next expected entry in order1 */
  int32_t last_res[2];       /* this rotation's last residual per beam: the device's motion moves every hit alike */
} dmp_base_t;

typedef struct {
  dmp_base_t b[DMP_SLOTS];
  uint32_t prev_t;           /* last word's time, half ticks (25-bit) */
  uint32_t stamp;
  int16_t m[DMP_NCTX];
} dmp_state_t;

#define DMP_NONE ((int32_t)0x80000000)

static inline void dmp_reset(dmp_state_t *p)
{
  memset(p, 0, sizeof *p);
  for (int i = 0; i < DMP_SLOTS; i++) {
    p->b[i].ch = -1;
    for (int e = 0; e < DMP_E; e++) { p->b[i].cur[e] = p->b[i].o1[e] = p->b[i].pred1[e] = DMP_NONE; p->b[i].pos1[e] = 0xFF; }
  }
  for (int i = 0; i < DMP_NCTX; i++) p->m[i] = 4 << 4;
}

static inline int32_t dmp_wrap25(uint32_t d) { d &= 0x1FFFFFF; return (d & 0x1000000) ? (int32_t)d - 0x2000000 : (int32_t)d; }

/* Exp-Golomb over this coder's own contexts (doorman_codec.h's take a dmc_state_t). */
static inline void dmp_eg(dmc_w_t *w, dmp_state_t *p, int c, uint32_t u)
{
  dmc_state_t t; t.m[0] = p->m[c];
  dmc_eg(w, &t, 0, u);
  p->m[c] = t.m[0];
}
static inline uint32_t dmp_ug(dmc_r_t *r, dmp_state_t *p, int c)
{
  dmc_state_t t; t.m[0] = p->m[c];
  uint32_t u = dmc_ug(r, &t, 0);
  p->m[c] = t.m[0];
  return u;
}
static inline void dmp_learn(dmp_state_t *p, int c, uint32_t u)
{
  dmc_state_t t; t.m[0] = p->m[c];
  dmc_learn(&t, 0, u);
  p->m[c] = t.m[0];
}
/* code/learn one value: w writes, r reads (returns it), neither just learns */
static inline uint32_t dmp_val(dmc_w_t *w, dmc_r_t *r, dmp_state_t *p, int c, uint32_t u)
{
  if (r) return dmp_ug(r, p, c);
  if (w) dmp_eg(w, p, c, u); else dmp_learn(p, c, u);
  return u;
}
static inline uint32_t dmp_bits(dmc_w_t *w, dmc_r_t *r, uint32_t v, int n)
{
  if (r) return dmc_get(r, n);
  if (w) dmc_put(w, v, n);
  return v;
}

static inline dmp_base_t *dmp_find(dmp_state_t *p, int ch)
{
  for (int i = 0; i < DMP_SLOTS; i++) if (p->b[i].ch == ch) return &p->b[i];
  return 0;
}
static inline dmp_base_t *dmp_take(dmp_state_t *p, int ch)
{
  dmp_base_t *b = dmp_find(p, ch);
  if (b) return b;
  int old = 0;
  for (int i = 1; i < DMP_SLOTS; i++)
    if (p->b[i].ch < 0 || (p->b[old].ch >= 0 && (int32_t)(p->b[i].used - p->b[old].used) < 0)) old = i;
  b = &p->b[old];
  b->nsync = b->n1 = b->ptr = 0;
  b->s1 = b->s2 = 0;
  b->last_res[0] = b->last_res[1] = 0;
  b->ch = (int8_t)ch;
  for (int e = 0; e < DMP_E; e++) {   /* field by field, yielding: a memset of the slot ran past the answer window */
    b->cur[e] = b->o1[e] = b->pred1[e] = DMP_NONE; b->pos1[e] = 0xFF;
    if ((e & 15) == 15) DMC_YIELD();
  }
  return b;
}
static inline uint32_t dmp_period2(const dmp_base_t *b)   /* rotation, half ticks */
{
  const uint32_t per = (b->s1 - b->s2) & 0xFFFFFF;
  return 2u * (b->nsync >= 2 && per > 900000u && per < 1000000u ? per : 959000u);
}
static inline int32_t dmp_pred(const dmp_base_t *b, int e, int32_t t2)   /* entry e this rotation, from its last two */
{
  const uint32_t t1 = (uint32_t)b->o1[e];
  if (t2 != DMP_NONE) {
    const int32_t step = dmp_wrap25(t1 - (uint32_t)t2);
    if (step > 1700000 && step < 2100000) return (int32_t)((t1 + (uint32_t)step) & 0x1FFFFFF);
  }
  return (int32_t)((t1 + dmp_period2(b)) & 0x1FFFFFF);
}
/* A new rotation (its sync, a beam word): the one that ended becomes the last, sorted by offset. */
static inline void dmp_rotate(dmp_base_t *b)
{
  /* each prediction once, sorted by offset from this rotation's sync; yields as it goes (no interrupts
     on the radio: nothing may run past the answer's ~180 us window) */
  const uint32_t base = (b->s1 << 1) & 0x1FFFFFF;
  b->n1 = 0;
  for (int e = 0; e < DMP_E; e++) {
    const int32_t t2 = b->o1[e];             /* the one before last, once this rotation ends */
    b->o1[e] = b->cur[e];
    b->cur[e] = DMP_NONE;
    b->pos1[e] = 0xFF;
    if (b->o1[e] == DMP_NONE) { b->pred1[e] = DMP_NONE; continue; }
    b->pred1[e] = dmp_pred(b, e, t2);
    const int32_t v = dmp_wrap25((uint32_t)b->pred1[e] - base);
    int i = b->n1++;
    while (i > 0 && dmp_wrap25((uint32_t)b->pred1[b->order1[i - 1]] - base) > v) { b->order1[i] = b->order1[i - 1]; i--; }
    b->order1[i] = (uint8_t)e;
    if ((e & 7) == 7) DMC_YIELD();
  }
  for (int i = 0; i < b->n1; i++) b->pos1[b->order1[i]] = (uint8_t)i;
  b->ptr = 0;
  b->last_res[0] = b->last_res[1] = 0;
}

static inline void dmp_beam(dmp_state_t *p, int ch, uint8_t *wd, dmc_w_t *w, dmc_r_t *r)
{
  uint32_t word = r ? 0 : dmc_u32(wd);
  const uint32_t hi = dmp_bits(w, r, word >> 26, 6);
  dmp_base_t *b = ch >= 0 ? dmp_take(p, ch) : 0;
  uint32_t ts;
  const uint32_t prev = p->prev_t >> 1;
  uint32_t pred = prev;
  if (b && b->nsync >= 1) {
    uint32_t per = b->nsync >= 2 ? ((b->s1 - b->s2) & 0xFFFFFF) : 0;
    if (b->nsync < 2 || per < 900000u || per > 1000000u) per = 0;   /* LH2 periods: 0.94-0.96 M ticks */
    pred = b->s1 + (per ? per : 959000u);
  }
  if (r) ts = (pred + (uint32_t)dmc_unzz(dmp_ug(r, p, DMP_SYNC))) & 0xFFFFFF;
  else {
    ts = (word >> 2) & 0xFFFFFF;
    dmp_val(w, 0, p, DMP_SYNC, dmc_zz(dmc_s24(ts - pred)));
  }
  if (r) dmc_w32(wd, ts << 2 | hi << 26);
  p->prev_t = ts << 1;
  if (!b) return;
  b->s2 = b->s1; b->s1 = ts;
  if (b->nsync < 2) b->nsync++;
  b->used = ++p->stamp;
  dmp_rotate(b);
}

static inline void dmp_sensor(dmp_state_t *p, int ch, uint8_t *wd, dmc_w_t *w, dmc_r_t *r)
{
  uint32_t word = r ? 0 : dmc_u32(wd);
  dmp_base_t *b = ch >= 0 ? dmp_find(p, ch) : 0;
  if (b && b->nsync == 0) b = 0;
  const int n = b ? b->n1 : 0, ptr = b ? b->ptr : 0;
  uint32_t t = 0;                 /* half ticks */
  int sensor = 0;
  if (!r) { t = ((word >> 3) & 0xFFFFFF) << 1 | ((word >> 2) & 1); sensor = (int)(word >> 27); }
  const uint32_t sync = b ? b->s1 << 1 : 0;
  /* the entry it is: this sensor's entry predicted nearest in time (within ~5 ms) */
  int idx = -1;
  if (!r && b) {   /* a sensor has two entries at most, one per beam */
    int32_t best = 480000;
    for (int e = sensor * 2; e < sensor * 2 + 2; e++) {
      if (b->pos1[e] == 0xFF) continue;
      int32_t d = dmp_wrap25(t - (uint32_t)b->pred1[e]);
      if (d < 0) d = -d;
      if (d < best) { best = d; idx = b->pos1[e]; }
    }
  }
  /* coded as a step from the pointer; n - ptr + 1 (past every entry) = not there */
  const int esc = n - ptr + 1;
  int k = idx >= 0 ? idx - ptr : esc;
  if (b) k = dmc_unzz(dmp_val(w, r, p, DMP_K, dmc_zz(k)));
  int e = -1;
  if (b && k != esc && ptr + k >= 0 && ptr + k < n) {
    e = b->order1[ptr + k];
    sensor = e >> 1;
    const uint32_t pr0 = (uint32_t)b->pred1[e], pr = pr0 + (uint32_t)b->last_res[e & 1];
    if (r) t = (pr + (uint32_t)dmc_unzz(dmp_ug(r, p, DMP_RES))) & 0x1FFFFFF;
    else dmp_val(w, 0, p, DMP_RES, dmc_zz(dmp_wrap25(t - pr)));
    const int32_t res = dmp_wrap25(t - pr0);
    if (res > -20000 && res < 20000) b->last_res[e & 1] = res;   /* not a stray one */
    b->ptr = (uint8_t)(ptr + k + 1);
    if (w) DMP_COUNT(k == 0 ? 2 : 3, 0);
  } else {
    if (w) DMP_COUNT(b ? 4 : 5, 0);
    sensor = (int)dmp_bits(w, r, (uint32_t)sensor, 5);
    if (r) t = (p->prev_t + (uint32_t)dmc_unzz(dmp_ug(r, p, DMP_ESC))) & 0x1FFFFFF;
    else dmp_val(w, 0, p, DMP_ESC, dmc_zz(dmp_wrap25(t - p->prev_t)));
    if (b) {   /* which beam: the first half turn is beam 0 */
      const int32_t off = dmp_wrap25(t - sync);
      e = sensor * 2 + (off >= 959000 ? 1 : 0);
    }
  }
  if (r) dmc_w32(wd, 2u | (t & 1) << 2 | (t >> 1) << 3 | (uint32_t)sensor << 27);
  p->prev_t = t;
  if (b && e >= 0) b->cur[e] = (int32_t)t;
}

/* One 's' frame: w encodes (from f), r decodes (into f), neither observes. Returns the frame length. */
static inline int dmp_walk(dmp_state_t *p, uint8_t *f, int n, dmc_w_t *w, dmc_r_t *r)
{
  int len = r ? (int)dmc_get(r, 6) + 1 : n - 1;
  if (w) dmc_put(w, (uint32_t)(len - 1), 6);
  if (r) f[0] = 's';
  uint8_t *b = f + 1;
  int ch = -1;
  for (int i = 0; i < len; ) {
    int kind;   /* 0 word, 2 channel, 3 raw byte */
    if (r) kind = dmc_get(r, 1) ? (dmc_get(r, 1) ? 3 : 2) : 0;
    else kind = !(b[i] & 1) && len - i >= 4 ? 0 : (b[i] & 0xF) == 1 ? 2 : 3;
    if (kind == 0 && r && len - i < 4) { r->bad = 1; return 0; }
    if (w) { if (kind == 0) dmc_put(w, 0, 1); else dmc_put(w, kind == 2 ? 2 : 3, 2); }
    if (kind == 2) {
      if (r) b[i] = (uint8_t)(dmc_get(r, 4) << 4 | 1); else if (w) dmc_put(w, b[i] >> 4, 4);
      ch = b[i] >> 4;
      i++;
    } else if (kind == 3) {
      if (r) b[i] = (uint8_t)dmc_get(r, 8); else if (w) dmc_put(w, b[i], 8);
      i++;
    } else {
      const int sensor = r ? (int)dmc_get(r, 1) : (int)((b[i] >> 1) & 1);
      if (w) dmc_put(w, (uint32_t)sensor, 1);
      const int at = w ? w->pos : 0;
      if (sensor) dmp_sensor(p, ch, b + i, w, r); else dmp_beam(p, ch, b + i, w, r);
      if (w) DMP_COUNT(sensor ? 1 : 0, w->pos - at + 1);
      (void)at;
      i += 4;
    }
    DMC_YIELD();
    if (r && r->bad) return 0;
  }
  return 1 + len;
}

static inline void dmp_observe(dmp_state_t *p, const uint8_t *f, int n)
{
  if (dmc_is_lh2(f, n)) dmp_walk(p, (uint8_t *)f, n, 0, 0);
}
/* An 's' frame into o (room for n bytes): the coded length, or 0 = send raw (the state changed either way). */
static inline int dmp_encode(dmp_state_t *p, const uint8_t *f, int n, uint8_t *o, int check)
{
  if (!dmc_is_lh2(f, n)) return 0;
  dmc_w_t w = { o + 1, n - 2, 0, 0 };
  if (w.cap < 1) { dmp_observe(p, f, n); return 0; }
  o[0] = DMC_LH2P;
  dmp_walk(p, (uint8_t *)f, n, &w, 0);
  if (check) dmc_put(&w, dmc_crc8(f, n), 8);
  if (w.over) return 0;
  return 1 + (w.pos + 7) / 8;
}
static inline int dmp_decode(dmp_state_t *p, const uint8_t *i, int n, uint8_t *f, int check)
{
  dmc_r_t r = { i + 1, n - 1, 0, 0 };
  const int m = dmp_walk(p, f, 0, 0, &r);
  if (check && !r.bad && m && (uint8_t)dmc_get(&r, 8) != dmc_crc8(f, m)) return 0;
  if (r.bad || (r.pos + 7) / 8 != r.len) return 0;
  return m;
}

/* ---- batches (0xC7): every frame queued for one answer, coded together ----
 * [count-1:3] then per frame [type:2] and its walk: 0 'p' IMU (doorman_codec.h, full history), 1 'q' LH1
 * records (doorman_codec.h), 2 's' LH2 (above), 3 raw [length-1:6][bytes]; with check, a CRC-8 over the
 * original frames, in order. One tag, length and CRC for the lot: frames queued between two polls cost no
 * more delay than before, and each frame's own overhead (~3 of an IMU sample's ~9 bytes) is gone. A raw
 * frame is observed by both coders on both ends, so their state follows every frame either way. */
enum { DMC_BATCH = 0xC7, DMB_MAX = 8, DMB_MAXF = 80 };   /* frames up to 80 B (Index sends 65-byte 'V') */
/* Wide batches (JOIN DML_JF_BATCH32): the count in 5 bits, up to 32 frames. Eight capped a batch at a cycle's frames
   once cycles got long (ten devices: ~13 ms, a Tundra's ~10 frames), and an answer carries one batch, so a busy
   device fell behind with room left in its answers. */
enum { DMC_BATCH_W = 0xC8, DMB_MAX_W = 32 };

typedef struct {
  dmc_w_t w;
  uint8_t *o;
  int count, max;
  uint8_t crc, bits;
} dmb_t;

static inline void dmb_begin_w(dmb_t *b, uint8_t *o, int cap, int wide)   /* o: room for cap bytes */
{
  b->o = o; o[0] = wide ? DMC_BATCH_W : DMC_BATCH;
  b->bits = wide ? 5 : 3; b->max = wide ? DMB_MAX_W : DMB_MAX;
  b->w.b = o + 1; b->w.cap = cap - 2; b->w.pos = 0; b->w.over = 0;   /* the last byte kept for the CRC */
  dmc_put(&b->w, 0, b->bits);              /* the count, filled in at the end */
  b->count = 0; b->crc = 0;
}
static inline void dmb_begin(dmb_t *b, uint8_t *o, int cap) { dmb_begin_w(b, o, cap, 0); }
/* Whether another frame may go in (count, and a little room). */
static inline int dmb_room(const dmb_t *b) { return b->count < b->max && b->w.cap * 8 - b->w.pos >= 24; }
/* Adds a frame; 0 = it didn't fit: the batch is as it was, but both coders' state has taken the frame (it
   moves the same coded or observed). Close the batch, begin the next and add it with dmb_add_raw. */
static inline int dmb_add(dmb_t *b, dmc_state_t *s, dmp_state_t *p, const uint8_t *f, int n)
{
  uint8_t *g = (uint8_t *)f;               /* encoding only reads */
  const int at = b->w.pos;
  if (f[0] == 'p' && n == 18 && s->nimu == 2) {
    dmc_put(&b->w, 0, 2); dmc_imu_walk(s, g, &b->w, 0); dmc_imu_step(s, f);
  } else if (dmc_is_lh1(f, n)) {
    dmc_put(&b->w, 1, 2); dmc_lh1_walk(s, g, n, &b->w, 0);
  } else if (dmc_is_lh2(f, n)) {
    dmc_put(&b->w, 2, 2); dmp_walk(p, g, n, &b->w, 0);
  } else {
    dmc_put(&b->w, 3, 2); dmc_put(&b->w, (uint32_t)(n - 1), 7);
    for (int i = 0; i < n; i++) dmc_put(&b->w, f[i], 8);
    dmc_observe(s, f, n);
    dmp_observe(p, f, n);
  }
  if (b->w.over) {                         /* back to before it */
    b->w.over = 0; b->w.pos = at;
    if (at & 7) b->w.b[at >> 3] &= (uint8_t)(0xFF << (8 - (at & 7)));
    return 0;
  }
  b->crc = dmc_crc8_add(b->crc, f, n);
  b->count++;
  return 1;
}
/* A frame whose state change both coders have already taken, raw (the dongle observes it: the same change). */
static inline void dmb_add_raw(dmb_t *b, const uint8_t *f, int n)
{
  dmc_put(&b->w, 3, 2); dmc_put(&b->w, (uint32_t)(n - 1), 7);
  for (int i = 0; i < n; i++) dmc_put(&b->w, f[i], 8);
  b->crc = dmc_crc8_add(b->crc, f, n);
  b->count++;
}
/* The batch's length (0: nothing in it). */
static inline int dmb_end(dmb_t *b, int check)
{
  if (!b->count) return 0;
  b->o[1] = (uint8_t)(b->o[1] | (uint8_t)((b->count - 1) << (8 - b->bits)));
  b->w.cap++;
  if (check) dmc_put(&b->w, b->crc, 8);
  return b->w.over ? -1 : 1 + (b->w.pos + 7) / 8;
}

typedef struct {
  dmc_r_t r;
  int left;
  uint8_t crc;
} dmb_r_t;

static inline int dmb_open(dmb_r_t *d, const uint8_t *i, int n)   /* 0: not a batch */
{
  if (n < 2 || (i[0] != DMC_BATCH && i[0] != DMC_BATCH_W)) return 0;
  d->r.b = i + 1; d->r.len = n - 1; d->r.pos = 0; d->r.bad = 0;
  d->left = (int)dmc_get(&d->r, i[0] == DMC_BATCH_W ? 5 : 3) + 1;
  d->crc = 0;
  return 1;
}
/* The next frame into f (room for DMB_MAXF bytes): its length; 0 = none left; -1 = malformed, or (with check,
   after the last) not what was coded. Either way the two ends are out of step: rejoin. */
static inline int dmb_next(dmb_r_t *d, dmc_state_t *s, dmp_state_t *p, uint8_t *f, int check)
{
  if (d->left <= 0) return 0;
  const int type = (int)dmc_get(&d->r, 2);
  int m = 0;
  if (type == 0) {
    if (s->nimu < 2) return -1;
    f[0] = 'p'; dmc_imu_walk(s, f, 0, &d->r);
    if (d->r.bad) return -1;
    dmc_imu_step(s, f); m = 18;
  } else if (type == 1) m = dmc_lh1_walk(s, f, 0, 0, &d->r);
  else if (type == 2) m = dmp_walk(p, f, 0, 0, &d->r);
  else {
    m = (int)dmc_get(&d->r, 7) + 1;
    if (m > DMB_MAXF) return -1;
    for (int k = 0; k < m; k++) f[k] = (uint8_t)dmc_get(&d->r, 8);
    if (!d->r.bad) { dmc_observe(s, f, m); dmp_observe(p, f, m); }
  }
  if (d->r.bad || m <= 0) return -1;
  d->crc = dmc_crc8_add(d->crc, f, m);
  if (--d->left == 0) {
    if (check && (uint8_t)dmc_get(&d->r, 8) != d->crc) return -1;
    if (d->r.bad || (d->r.pos + 7) / 8 != d->r.len) return -1;
  }
  return m;
}
