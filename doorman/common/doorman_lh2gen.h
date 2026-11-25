/* Synthetic LH2 device stream for link load tests (no LH2 hardware needed): the frames a Vive Tracker 3.0's
 * main MCU would hand its radio in a room with several Base Station 2.0s, made on the radio itself.
 *
 *   's' frames  every ~4 ms (as the captures' 150-260 a second), the 0x28 stream as the FPGA reports it: a beam word per base per rotation (its
 *               sync, OOTX bit) and a sensor word per hit, a channel byte before each base's words
 *   'p' frames  IMU at 250 Hz
 *
 * Model: each base rotates at its channel's real period; each beam passes the device once a rotation and
 * hits the sensors facing that base, at offsets set by where the sensor sits, moved by a slow sway of the
 * device (so the set of sensors seen and their timing drift, as on a body) plus timing noise. Tuned against
 * two LH2 captures (doorman/test/lh2gen_check.c): hits, beam words, frame sizes and how well the air codec
 * compresses them. `density` scales the sensors seen, to load the link past real rates.
 * Integer arithmetic only (radio firmware has no FPU code); per-sensor tables are set up once. DLG_YIELD is
 * called often enough for the radio's answer window. */
#pragma once
#include <stdint.h>
#include <string.h>

#ifndef DLG_YIELD
#define DLG_YIELD() do { } while (0)   /* the radio: check for a POLL (nothing may run past its answer window) */
#endif
#ifndef DLG_EV
#define DLG_EV 128   /* events pending: a rotation of every base (density 3 needs ~100; set higher where RAM allows) */
#endif
enum { DLG_MAXB = 4, DLG_MAXS = 24, DLG_FRAME = 59 };

typedef struct {
  /* per base, per sensor, per beam: offset in the rotation (ticks), sway amplitude (ticks), phase */
  int32_t off[DLG_MAXB][DLG_MAXS][2];
  int16_t amp[DLG_MAXB][DLG_MAXS][2];
  int16_t vis[DLG_MAXB][DLG_MAXS];          /* seen while vis + sway term > 0 */
  uint16_t ph[DLG_MAXB][DLG_MAXS];
  uint32_t period[DLG_MAXB];                 /* ticks (48 MHz) */
  uint8_t channel[DLG_MAXB];
  uint32_t next_sync[DLG_MAXB];
  int16_t base_vis[DLG_MAXB];               /* the base in view while this + its own sway > 0 */
  uint16_t base_ph[DLG_MAXB];
  uint32_t ootx[DLG_MAXB];
  int nb, ns, noise;
  int imu;                                   /* make 'p' frames too (dlg_init: on); off on a device with its own IMU */
  uint32_t rng;
  /* pending events for the frame being filled, time-ordered per base */
  uint32_t ev_t[DLG_EV]; uint8_t ev_b[DLG_EV], ev_s[DLG_EV], ev_beam[DLG_EV];
  int ev0, nev;                              /* pending: [ev0, nev); compacted when the end is reached */
  uint32_t frame_t, imu_t;
  uint8_t imu_seq;
  uint32_t imu_ticks;
} dlg_t;

static const uint32_t dlg_periods[16] = { 959000, 957000, 953000, 949000, 947000, 943000, 941000, 939000,
                                          937000, 929000, 927000, 923000, 921000, 919000, 911000, 909000 };

static inline uint32_t dlg_rand(dlg_t *g) { g->rng = g->rng * 1664525u + 1013904223u; return g->rng >> 8; }
/* sine, 1024 steps a turn, amplitude 32767: a quarter in flash (computing it on the radio took ~200 us unyielded) */
static const int16_t dlg_sin_q[257] = {
  0, 201, 402, 603, 804, 1005, 1206, 1407, 1608, 1809, 2009, 2210, 2410, 2611, 2811, 3012,
  3212, 3412, 3612, 3811, 4011, 4210, 4410, 4609, 4808, 5007, 5205, 5404, 5602, 5800, 5998, 6195,
  6393, 6590, 6786, 6983, 7179, 7375, 7571, 7767, 7962, 8157, 8351, 8545, 8739, 8933, 9126, 9319,
  9512, 9704, 9896, 10087, 10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353,
  12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828, 14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269,
  15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673, 16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037,
  18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357, 19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631,
  20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856, 22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027,
  23170, 23311, 23452, 23592, 23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201,
  25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674, 26790, 26905, 27019, 27133,
  27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001, 28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803,
  28898, 28992, 29085, 29177, 29268, 29358, 29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195,
  30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297,
  31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736, 31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098,
  32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382, 32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589,
  32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717, 32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766,
  32767
};
static inline int32_t dlg_sin(uint32_t a)
{
  a &= 1023;
  if (a < 256) return dlg_sin_q[a];
  if (a < 512) return dlg_sin_q[512 - a];
  if (a < 768) return -dlg_sin_q[a - 512];
  return -dlg_sin_q[1024 - a];
}

/* bases: 1-4; density: sensors seen per pass ~ 3 x density (real devices: ~2-4) */
static inline void dlg_init(dlg_t *g, uint32_t seed, int bases, int density, int noise_ticks)
{
  memset(g, 0, sizeof *g);
  g->rng = seed * 2654435761u + 1;
  g->nb = bases < 1 ? 1 : bases > DLG_MAXB ? DLG_MAXB : bases;
  g->ns = DLG_MAXS;
  g->noise = noise_ticks;
  static const uint8_t ch[DLG_MAXB] = { 0, 1, 2, 4 };
  for (int b = 0; b < g->nb; b++) {
    g->channel[b] = ch[b];
    g->period[b] = dlg_periods[ch[b]];
    g->next_sync[b] = 2000 + dlg_rand(g) % g->period[b];
    g->ootx[b] = dlg_rand(g);
    g->base_vis[b] = (int16_t)(4000 + dlg_rand(g) % 12000);   /* most bases in view most of the time */
    g->base_ph[b] = (uint16_t)(dlg_rand(g) & 1023);
    /* where the device sits as this base sees it: the sweep reaches it at this fraction of the turn */
    const uint32_t centre = g->period[b] / 8 + dlg_rand(g) % (g->period[b] / 4);
    for (int s = 0; s < g->ns; s++) {
      /* a sensor a few cm from the device's centre: ~0.3 % of a turn, each beam differently (tilted planes) */
      const int32_t x = (int32_t)(dlg_rand(g) % 6000) - 3000, y = (int32_t)(dlg_rand(g) % 6000) - 3000;
      g->off[b][s][0] = (int32_t)centre + x + y / 2;
      g->off[b][s][1] = (int32_t)(centre + g->period[b] / 2) + x - y / 2;
      g->amp[b][s][0] = (int16_t)(400 + dlg_rand(g) % 1600);    /* sway: up to ~40 us */
      g->amp[b][s][1] = (int16_t)(400 + dlg_rand(g) % 1600);
      g->ph[b][s] = (uint16_t)(dlg_rand(g) & 1023);
      /* facing this base: a share of the sensors, more with density */
      g->vis[b][s] = (int16_t)((int32_t)(dlg_rand(g) % 32768) - 32768 + density * 4600);
      DLG_YIELD();
    }
  }
  g->frame_t = 0; g->imu_t = 0; g->imu = 1;
}

static inline void dlg_compact(dlg_t *g)
{
  const int n = g->nev - g->ev0;
  for (int i = 0; i < n; i++) {
    g->ev_t[i] = g->ev_t[g->ev0 + i]; g->ev_b[i] = g->ev_b[g->ev0 + i]; g->ev_s[i] = g->ev_s[g->ev0 + i]; g->ev_beam[i] = g->ev_beam[g->ev0 + i];
    if ((i & 15) == 15) DLG_YIELD();
  }
  g->ev0 = 0; g->nev = n;
}
static inline void dlg_push(dlg_t *g, uint32_t t, int b, int s, int beam)
{
  if (g->nev >= DLG_EV && g->ev0) dlg_compact(g);
  if (g->nev >= DLG_EV) return;
  int i = g->nev++;
  while (i > g->ev0 && (int32_t)(g->ev_t[i - 1] - t) > 0) { g->ev_t[i] = g->ev_t[i - 1]; g->ev_b[i] = g->ev_b[i - 1]; g->ev_s[i] = g->ev_s[i - 1]; g->ev_beam[i] = g->ev_beam[i - 1]; i--; }
  g->ev_t[i] = t; g->ev_b[i] = (uint8_t)b; g->ev_s[i] = (uint8_t)s; g->ev_beam[i] = (uint8_t)beam;
}

/* A base's rotation starting at sync: its beam word and both beams' hits. */
static inline void dlg_rotation(dlg_t *g, int b, uint32_t sync)
{
  const uint32_t slow = sync / 2400000u;                   /* sway phase: ~0.4 turns a second */
  if (g->base_vis[b] + dlg_sin(slow * 5 + g->base_ph[b]) <= 0) return;   /* the body between: nothing from it */
  dlg_push(g, sync + (dlg_rand(g) % 9), b, 0xFF, 0);       /* the beam word, a few ticks of decode jitter */
  for (int s = 0; s < g->ns; s++) {
    const int32_t sw = dlg_sin(slow * 9 + g->ph[b][s]);
    const int32_t v = g->vis[b][s] + sw / 2;
    if (v <= 0) continue;                                  /* facing away now */
    for (int k = 0; k < 2; k++) {
      if (v < 8000 && dlg_rand(g) % 8 < 4) continue;       /* near the edge: seen some passes only */
      DLG_YIELD();
      int32_t o = g->off[b][s][k] + (int32_t)(((int64_t)g->amp[b][s][k] * sw) >> 15);
      if (g->noise) o += (int32_t)(dlg_rand(g) % (uint32_t)(2 * g->noise + 1)) - g->noise;
      dlg_push(g, sync + (uint32_t)o, b, s, k);
    }
  }
}

typedef void (*dlg_emit_t)(const uint8_t *f, int n, void *ctx);

/* Everything up to device time now (ticks): frames to emit, in order. Call often (every ms or so). */
static inline void dlg_run(dlg_t *g, uint32_t now, dlg_emit_t emit, void *ctx)
{
  /* rotations that have started */
  for (int b = 0; b < g->nb; b++)
    while ((int32_t)(g->next_sync[b] - now) <= 0) {   /* its hits come out as the frames reach them */
      dlg_rotation(g, b, g->next_sync[b]);
      g->next_sync[b] += g->period[b];
    }
  /* IMU every 4 ms */
  while ((int32_t)(now - g->imu_t) >= 192000) {
    g->imu_t += 192000;
    if (!g->imu) continue;
    uint8_t f[18];
    f[0] = 'p';
    const int32_t a = dlg_sin(g->imu_t / 960000u * 7), c = dlg_sin(g->imu_t / 480000u * 5);
    const int16_t v[6] = { (int16_t)(a / 64 + (int32_t)(dlg_rand(g) % 33) - 16), (int16_t)(c / 64 + (int32_t)(dlg_rand(g) % 33) - 16),
                           (int16_t)(4096 + (int32_t)(dlg_rand(g) % 33) - 16), (int16_t)(c / 32 + (int32_t)(dlg_rand(g) % 23) - 11),
                           (int16_t)(a / 32 + (int32_t)(dlg_rand(g) % 23) - 11), (int16_t)((int32_t)(dlg_rand(g) % 23) - 11) };
    for (int k = 0; k < 6; k++) { f[1 + 2 * k] = (uint8_t)v[k]; f[2 + 2 * k] = (uint8_t)((uint16_t)v[k] >> 8); }
    const uint32_t ts = g->imu_t;
    f[13] = (uint8_t)ts; f[14] = (uint8_t)(ts >> 8); f[15] = (uint8_t)(ts >> 16); f[16] = (uint8_t)(ts >> 24);
    f[17] = g->imu_seq++;
    emit(f, 18, ctx);
  }
  /* 's' frames: what happened by each ~4 ms boundary */
  while ((int32_t)(now - g->frame_t) >= 192000) {
    g->frame_t += 192000;
    uint8_t f[64]; int n = 1, ch = -1;
    f[0] = 's';
    int used = g->ev0;
    while (used < g->nev && (int32_t)(g->ev_t[used] - g->frame_t) <= 0) {
      const int b = g->ev_b[used];
      const int need = (ch != g->channel[b] ? 1 : 0) + 4;
      if (n + need > DLG_FRAME) { emit(f, n, ctx); n = 1; ch = -1; continue; }
      if (ch != g->channel[b]) { f[n++] = (uint8_t)(g->channel[b] << 4 | 1); ch = g->channel[b]; }
      uint32_t w;
      const uint32_t t = g->ev_t[used];
      if (g->ev_s[used] == 0xFF) {                         /* beam word: ts24 << 2, OOTX bit 26 */
        const uint32_t bit = g->ootx[b] & 1; g->ootx[b] = (g->ootx[b] >> 1) | (bit << 31);
        w = (t & 0xFFFFFF) << 2 | bit << 26;
      } else {                                             /* sensor word: half-tick bit, ts24 << 3, sensor */
        w = 2u | (dlg_rand(g) & 1) << 2 | (t & 0xFFFFFF) << 3 | (uint32_t)g->ev_s[used] << 27;
      }
      f[n++] = (uint8_t)w; f[n++] = (uint8_t)(w >> 8); f[n++] = (uint8_t)(w >> 16); f[n++] = (uint8_t)(w >> 24);
      used++;
    }
    if (n > 1) emit(f, n, ctx);
    DLG_YIELD();
    g->ev0 = used;
    if (g->ev0 == g->nev) g->ev0 = g->nev = 0;
  }
}
