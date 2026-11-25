/* Air codec: lossless, bit-packed, one frame at a time (no batching, no added latency).
 * The radio codes, the dongle decodes.
 *
 *  'p' IMU (acc[3], gyro[3] s16, t u32, seq u8)  -> 0xC1: [seq is next:1] 6 x EG(zz(x - (2 x1 - x2)))
 *                                                   EG(zz(dt - previous dt)) [seq:8 if not next]
 *  'q' LH1 records (8 B each, 1..7)               -> 0xC2: [count-1:3] per record:
 *        b0: 0 + 5 bits (< 32) | 1 + 8 bits;  b1: 0 | 1 + 8 bits;  EG(A);  EG(zz(t - previous t))
 *        (sync records, b0 = 0xFF, keep their own layout: t @2, A @6)
 *  's' LH2 stream (0x28's bytes, up to 63)        -> 0xC3: [n:6] then tokens until n bytes:
 *        0  word (even first byte, 4 bytes left): [bit1] EG(zz(ts24 - previous ts24)) then
 *           sensor word: [half:1][value:5]   beam word: bits 26..31 [6]
 *        10 channel byte (low nibble 1): high nibble [4]
 *        11 any other byte [8]
 *
 * EG = Exp-Golomb with k adapted per field from a running mean of the values' bit lengths. Values of
 * any size are coded exactly; small ones in a few bits, rare big ones (a new sweep) in a few more.
 *
 * With `check` set (agreed at join, doorman_link.h DML_JF_CHECK), a coded frame ends with a CRC-8 of
 * the original frame. Every frame is coded against the state the ones before it left, so one coded frame
 * that reached the dongle wrong (corrupted in the air, the radio's CRC-16 passing anyway: about one in
 * 65536 corrupted packets) left the two ends out of step for good: every IMU sample after it decoded as
 * plausible garbage, timestamps a constant ~31 s apart and readings ramping by a constant step, and the
 * device sat connected and never tracked until it rejoined. The CRC-8 catches it at that frame.
 *
 * Both ends keep the same state and change it the same way for every original frame, whether it went
 * over the air coded or raw (dmc_encode changes it even when it returns 0 = send raw; the receiver of a
 * raw frame calls dmc_observe). Reset both on (re)join. */
#pragma once
#include <stdint.h>
#include <string.h>
/* Called between coded fields: the radio checks for a POLL's address there (it has no interrupts,
   and a coded frame took ~400 us on the Tracker 1.0's nRF51 before the bit writer was sped up). */
#ifndef DMC_YIELD
#define DMC_YIELD() do { } while (0)
#endif

enum { DMC_IMU = 0xC1, DMC_LH1 = 0xC2, DMC_LH2 = 0xC3 };
enum { DMC_AX = 0, DMC_IMU_T = 6, DMC_W = 7, DMC_LT = 8, DMC_L2 = 9, DMC_NCTX = 10 };

typedef struct {
  int16_t x1[6], x2[6];   /* last two IMU samples */
  uint32_t imu_t; int32_t imu_dt;
  uint8_t seq, nimu;      /* nimu: IMU samples seen (2 = full history) */
  uint32_t light_t;       /* last LH1 record time */
  uint32_t lh2_t;         /* last LH2 word ts24 */
  int16_t m[DMC_NCTX];   /* running mean of bit lengths, x16 */
  uint8_t check;          /* coded frames carry a CRC-8 of the original (set after dmc_reset, at join) */
} dmc_state_t;

static inline void dmc_reset(dmc_state_t *s)
{
  memset(s, 0, sizeof *s);
  for (int i = 0; i < DMC_NCTX; i++) s->m[i] = 4 << 4;
}

/* ---- bits ---- */
typedef struct { uint8_t *b; int cap, pos, over; } dmc_w_t;
typedef struct { const uint8_t *b; int len, pos, bad; } dmc_r_t;
static inline void dmc_put(dmc_w_t *w, uint32_t v, int n)   /* n <= 32 bits, MSB first, a byte at a time */
{
  while (n > 0) {
    int byte = w->pos >> 3, off = w->pos & 7, room = 8 - off, take = n < room ? n : room;
    if (byte >= w->cap) { w->over = 1; return; }
    if (!off) w->b[byte] = 0;
    w->b[byte] |= (uint8_t)(((v >> (n - take)) & ((1u << take) - 1)) << (room - take));
    w->pos += take; n -= take;
  }
}
static inline uint32_t dmc_get(dmc_r_t *r, int n)
{
  uint32_t v = 0;
  for (int i = 0; i < n; i++) {
    if ((r->pos >> 3) >= r->len) { r->bad = 1; return 0; }
    v = v << 1 | ((r->b[r->pos >> 3] >> (7 - (r->pos & 7))) & 1);
    r->pos++;
  }
  return v;
}
static inline int dmc_bits32(uint32_t v) { return v ? 32 - __builtin_clz(v) : 0; }
static inline int dmc_bits(uint64_t v) { uint32_t h = (uint32_t)(v >> 32); return h ? 32 + dmc_bits32(h) : dmc_bits32((uint32_t)v); }
static inline uint32_t dmc_zz(int32_t v) { return ((uint32_t)v << 1) ^ (uint32_t)(v >> 31); }
static inline int32_t dmc_unzz(uint32_t v) { return (int32_t)(v >> 1) ^ -(int32_t)(v & 1); }
static inline int dmc_k(const dmc_state_t *s, int c) { int k = (s->m[c] >> 4) - 2; return k < 0 ? 0 : k > 28 ? 28 : k; }
static inline void dmc_learn(dmc_state_t *s, int c, uint32_t u)
{
  s->m[c] = (int16_t)(s->m[c] + (((dmc_bits32(u) << 4) - s->m[c]) >> 3));
}
static inline void dmc_eg(dmc_w_t *w, dmc_state_t *s, int c, uint32_t u)
{
  DMC_YIELD();
  int k = dmc_k(s, c);
  const uint32_t q = u >> k;
  if (q != 0xFFFFFFFFu) {                                   /* x = q + 1 fits 32 bits: no 64-bit maths */
    const uint32_t x = q + 1;
    const int L = dmc_bits32(x);
    dmc_put(w, 0, L - 1);
    dmc_put(w, x, L);
  } else {                                                  /* x = 2^32: 32 zeros, then 1 and 32 zeros */
    dmc_put(w, 0, 32);
    dmc_put(w, 1, 1);
    dmc_put(w, 0, 32);
  }
  dmc_put(w, u & ((1u << k) - 1), k);
  dmc_learn(s, c, u);
}
static inline uint32_t dmc_ug(dmc_r_t *r, dmc_state_t *s, int c)
{
  int k = dmc_k(s, c), z = 0;
  while (!r->bad && dmc_get(r, 1) == 0) if (++z > 33) { r->bad = 1; return 0; }
  uint64_t x = 1;
  for (int i = 0; i < z; i++) x = x << 1 | dmc_get(r, 1);
  uint32_t u = (uint32_t)((x - 1) << k) | dmc_get(r, k);
  dmc_learn(s, c, u);
  return u;
}
/* CRC-8, poly 0x07, from a table: bit by bit it took ~50 instructions a byte, which for a full frame ran
   past the radio's answer window (~180 us between DMC_YIELDs) on the Tracker 1.0's 16 MHz nRF51. */
static const uint8_t dmc_crc8_table[256] = {
  0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
  0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
  0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
  0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
  0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
  0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
  0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
  0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
  0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
  0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
  0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
  0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
  0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
  0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
  0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
  0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};
static inline uint8_t dmc_crc8_add(uint8_t c, const uint8_t *p, int n)
{
  for (int i = 0; i < n; i++) {
    c = dmc_crc8_table[c ^ p[i]];
    if ((i & 15) == 15) DMC_YIELD();
  }
  return c;
}
static inline uint8_t dmc_crc8(const uint8_t *p, int n) { return dmc_crc8_add(0, p, n); }
static inline uint16_t dmc_u16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static inline uint32_t dmc_u32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static inline void dmc_w16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void dmc_w32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static inline int32_t dmc_s24(uint32_t d) { d &= 0xFFFFFF; return (d & 0x800000) ? (int32_t)d - 0x1000000 : (int32_t)d; }

static inline int dmc_is_lh1(const uint8_t *f, int n) { return f[0] == 'q' && n > 1 && (n - 1) % 8 == 0 && n <= 57; }
static inline int dmc_is_lh2(const uint8_t *f, int n) { return f[0] == 's' && n >= 2 && n <= 64; }

/* ---- one walk per frame type: w != 0 encodes, r != 0 decodes into f, neither = observe ---- */
static inline void dmc_imu_step(dmc_state_t *s, const uint8_t *f)   /* history after sample f */
{
  uint32_t t = dmc_u32(f + 13);
  int32_t dt = (int32_t)(t - s->imu_t);
  for (int k = 0; k < 6; k++) { s->x2[k] = s->x1[k]; s->x1[k] = (int16_t)dmc_u16(f + 1 + 2 * k); }
  s->imu_dt = s->nimu ? dt : 0; s->imu_t = t; s->seq = f[17];
  if (s->nimu < 2) s->nimu++;
}
static inline void dmc_imu_walk(dmc_state_t *s, uint8_t *f, dmc_w_t *w, dmc_r_t *r)
{
  /* only called with nimu == 2 */
  uint8_t next = (uint8_t)(s->seq + 1);
  int seq_next = w ? f[17] == next : r ? (int)dmc_get(r, 1) : f[17] == next;
  if (w) dmc_put(w, (uint32_t)seq_next, 1);
  for (int k = 0; k < 6; k++) {
    int16_t pred = (int16_t)(2 * s->x1[k] - s->x2[k]);
    if (r) { int16_t res = (int16_t)dmc_unzz(dmc_ug(r, s, DMC_AX + k)); dmc_w16(f + 1 + 2 * k, (uint16_t)(pred + res)); }
    else {
      uint32_t u = dmc_zz((int16_t)((int16_t)dmc_u16(f + 1 + 2 * k) - pred));
      if (w) dmc_eg(w, s, DMC_AX + k, u); else dmc_learn(s, DMC_AX + k, u);
    }
  }
  if (r) {
    int32_t dd = dmc_unzz(dmc_ug(r, s, DMC_IMU_T));
    dmc_w32(f + 13, s->imu_t + (uint32_t)(s->imu_dt + dd));
    f[17] = seq_next ? next : (uint8_t)dmc_get(r, 8);
  } else {
    uint32_t u = dmc_zz((int32_t)(dmc_u32(f + 13) - s->imu_t) - s->imu_dt);
    if (w) dmc_eg(w, s, DMC_IMU_T, u); else dmc_learn(s, DMC_IMU_T, u);
    if (w && !seq_next) dmc_put(w, f[17], 8);
  }
}
static inline int dmc_lh1_walk(dmc_state_t *s, uint8_t *f, int n, dmc_w_t *w, dmc_r_t *r)
{
  int count = r ? (int)dmc_get(r, 3) + 1 : (n - 1) / 8;
  if (w) dmc_put(w, (uint32_t)(count - 1), 3);
  if (r) f[0] = 'q';
  for (int i = 0; i < count; i++) {
    uint8_t *e = f + 1 + 8 * i;
    if (r) {
      e[0] = dmc_get(r, 1) ? (uint8_t)dmc_get(r, 8) : (uint8_t)dmc_get(r, 5);
      e[1] = dmc_get(r, 1) ? (uint8_t)dmc_get(r, 8) : 0;
    } else if (w) {
      if (e[0] < 32) { dmc_put(w, 0, 1); dmc_put(w, e[0], 5); } else { dmc_put(w, 1, 1); dmc_put(w, e[0], 8); }
      if (e[1]) { dmc_put(w, 1, 1); dmc_put(w, e[1], 8); } else dmc_put(w, 0, 1);
    }
    int sync = e[0] == 0xFF;
    uint8_t *pa = e + (sync ? 6 : 2), *pt = e + (sync ? 2 : 4);
    if (r) {
      dmc_w16(pa, (uint16_t)dmc_ug(r, s, DMC_W));
      dmc_w32(pt, s->light_t + (uint32_t)dmc_unzz(dmc_ug(r, s, DMC_LT)));
    } else {
      uint32_t a = dmc_u16(pa), d = dmc_zz((int32_t)(dmc_u32(pt) - s->light_t));
      if (w) { dmc_eg(w, s, DMC_W, a); dmc_eg(w, s, DMC_LT, d); }
      else { dmc_learn(s, DMC_W, a); dmc_learn(s, DMC_LT, d); }
    }
    s->light_t = dmc_u32(pt);
    if (r && r->bad) return 0;
  }
  return 1 + 8 * count;
}
static inline int dmc_lh2_walk(dmc_state_t *s, uint8_t *f, int n, dmc_w_t *w, dmc_r_t *r)
{
  int len = r ? (int)dmc_get(r, 6) + 1 : n - 1;   /* stream bytes after the 's' */
  if (w) dmc_put(w, (uint32_t)(len - 1), 6);
  if (r) f[0] = 's';
  uint8_t *b = f + 1;
  for (int i = 0; i < len; ) {
    int kind;   /* 0 word, 2 channel, 3 raw byte */
    if (r) kind = dmc_get(r, 1) ? (dmc_get(r, 1) ? 3 : 2) : 0;
    else kind = !(b[i] & 1) && len - i >= 4 ? 0 : (b[i] & 0xF) == 1 ? 2 : 3;
    if (kind == 0 && r && len - i < 4) { r->bad = 1; return 0; }
    if (w) { if (kind == 0) dmc_put(w, 0, 1); else dmc_put(w, kind == 2 ? 2 : 3, 2); }
    if (kind == 2) {
      if (r) b[i] = (uint8_t)(dmc_get(r, 4) << 4 | 1); else if (w) dmc_put(w, b[i] >> 4, 4);
      i++;
    } else if (kind == 3) {
      if (r) b[i] = (uint8_t)dmc_get(r, 8); else if (w) dmc_put(w, b[i], 8);
      i++;
    } else {
      uint32_t wd = r ? 0 : dmc_u32(b + i);
      int sensor = r ? (int)dmc_get(r, 1) : (int)((wd >> 1) & 1);
      if (w) dmc_put(w, (uint32_t)sensor, 1);
      uint32_t ts;
      if (r) ts = (s->lh2_t + (uint32_t)dmc_unzz(dmc_ug(r, s, DMC_L2))) & 0xFFFFFF;
      else {
        ts = sensor ? (wd >> 3) & 0xFFFFFF : (wd >> 2) & 0xFFFFFF;
        uint32_t u = dmc_zz(dmc_s24(ts - s->lh2_t));
        if (w) dmc_eg(w, s, DMC_L2, u); else dmc_learn(s, DMC_L2, u);
      }
      if (sensor) {
        if (r) wd = 2u | dmc_get(r, 1) << 2 | ts << 3 | dmc_get(r, 5) << 27;
        else if (w) { dmc_put(w, (wd >> 2) & 1, 1); dmc_put(w, wd >> 27, 5); }
      } else {
        if (r) wd = ts << 2 | dmc_get(r, 6) << 26;
        else if (w) dmc_put(w, wd >> 26, 6);
      }
      if (r) dmc_w32(b + i, wd);
      s->lh2_t = ts;
      i += 4;
    }
    if (r && r->bad) return 0;
  }
  return 1 + len;
}

/* The receiver's side of a frame that went over the air raw. */
static inline void dmc_observe(dmc_state_t *s, const uint8_t *f, int n)
{
  uint8_t *g = (uint8_t *)f;   /* walks only read when neither writing nor reading bits */
  if (f[0] == 'p' && n == 18) { if (s->nimu == 2) dmc_imu_walk(s, g, 0, 0); dmc_imu_step(s, f); }
  else if (dmc_is_lh1(f, n)) dmc_lh1_walk(s, g, n, 0, 0);
  else if (dmc_is_lh2(f, n)) dmc_lh2_walk(s, g, n, 0, 0);
}

/* Codes f (n bytes) into o (room for n bytes); returns the coded length, or 0 = send f raw.
 * The state changes either way, as dmc_observe would change it. */
static inline int dmc_encode(dmc_state_t *s, const uint8_t *f, int n, uint8_t *o)
{
  uint8_t *g = (uint8_t *)f;   /* encoding only reads f */
  dmc_w_t w = { o + 1, n - 2, 0, 0 };   /* must come out shorter than the frame */
  if (w.cap < 1) { dmc_observe(s, f, n); return 0; }
  if (f[0] == 'p' && n == 18) {
    if (s->nimu < 2) { dmc_imu_step(s, f); return 0; }
    o[0] = DMC_IMU; dmc_imu_walk(s, g, &w, 0); dmc_imu_step(s, f);
  } else if (dmc_is_lh1(f, n)) { o[0] = DMC_LH1; dmc_lh1_walk(s, g, n, &w, 0); }
  else if (dmc_is_lh2(f, n)) { o[0] = DMC_LH2; dmc_lh2_walk(s, g, n, &w, 0); }
  else return 0;
  if (s->check) dmc_put(&w, dmc_crc8(f, n), 8);
  if (w.over) return 0;
  return 1 + (w.pos + 7) / 8;
}

/* Decodes a 0xC1/0xC2/0xC3 frame into the original (f: room for 64 bytes); returns its length, 0 = malformed
   or (with check) not the frame that was coded. Either way the two ends are out of step: rejoin. */
static inline int dmc_decode(dmc_state_t *s, const uint8_t *i, int n, uint8_t *f)
{
  dmc_r_t r = { i + 1, n - 1, 0, 0 };
  int m = 0;
  if (i[0] == DMC_IMU) {
    if (s->nimu < 2) return 0;
    f[0] = 'p'; dmc_imu_walk(s, f, 0, &r);
    if (r.bad) return 0;
    dmc_imu_step(s, f); m = 18;
  } else if (i[0] == DMC_LH1) m = dmc_lh1_walk(s, f, 0, 0, &r);
  else if (i[0] == DMC_LH2) m = dmc_lh2_walk(s, f, 0, 0, &r);
  if (s->check && !r.bad && m && (uint8_t)dmc_get(&r, 8) != dmc_crc8(f, m)) return 0;
  if (r.bad || (r.pos + 7) / 8 != r.len) return 0;   /* every byte used, only the last one padded */
  return m;
}
