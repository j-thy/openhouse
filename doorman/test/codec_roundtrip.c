/* Air codec round trip on real frames: every frame a capture's devices sent, coded the way the radio
 * does and decoded the way the dongle does, compared byte for byte, with the bytes on the air.
 *
 *   codec_roundtrip FRAMES [POLL_MS [ANSWER_BYTES]]
 *
 * FRAMES: a dump from a replay (LH_AIR=base LH_AIR_DUMP=FRAMES lhtool session CAPTURE, see run.sh):
 * records [device u8][length u8][time u32, 0.1 ms][frame]. Batches (doorman_lh2p.h) take every frame a
 * device queued during one poll period, up to ANSWER_BYTES - 7 (an answer, its header and length byte);
 * defaults: 10 ms and 176 B (8 devices). Today's codec (doorman_codec.h, one frame at a time, with the
 * CRC-8) is the reference for the sizes. Exit status 1 on any mismatch. */
#include <stdio.h>
#include <stdlib.h>
#include "doorman_link.h"
#include "doorman_lh2p.h"

typedef struct {
  dmc_state_t today, enc, dec;     /* today's codec; the batch path's coder and decoder */
  dmp_state_t penc, pdec;
  dmb_t b;
  uint8_t buf[256], orig[2048];
  int on, oi[DMB_MAX_W], olen[DMB_MAX_W], open, init;
  uint32_t t0;
  long bytes_today, bytes_imu, bytes_light, bytes_batched, batches, frames, spills;
} dev_state;

static dev_state dv[256];
static long mismatches;

static void close_batch(dev_state *d)
{
  if (!d->open) return;
  d->open = 0;
  const int m = dmb_end(&d->b, 1);
  if (m <= 0) { mismatches++; return; }
  dmb_r_t r;
  dmb_open(&r, d->buf, m);
  for (int i = 0; i < d->b.count; i++) {
    uint8_t f[80];
    const int g = dmb_next(&r, &d->dec, &d->pdec, f, 1);
    if (g != d->olen[i] || memcmp(f, d->orig + d->oi[i], (size_t)g)) {
      if (!mismatches) fprintf(stderr, "first mismatch: batch %ld frame %d (%d bytes back, %d sent)\n", d->batches, i, g, d->olen[i]);
      mismatches++;
      break;
    }
  }
  d->bytes_batched += m + 1;
  d->batches++;
  d->on = 0;
}

static int wide;   /* "wide" as the 4th argument: 32-frame batches (DMC_BATCH_W) */
static void begin(dev_state *d, int cap, uint32_t t) { dmb_begin_w(&d->b, d->buf, cap, wide); d->open = 1; d->t0 = t; d->on = 0; }

static void keep(dev_state *d, const uint8_t *f, int n)
{
  const int i = d->b.count;
  d->oi[i] = d->on; d->olen[i] = n;
  memcpy(d->orig + d->on, f, (size_t)n);
  d->on += n;
}

static uint8_t crc_bitwise(const uint8_t *p, int n)
{
  uint8_t c = 0;
  for (int i = 0; i < n; i++) { c ^= p[i]; for (int b = 0; b < 8; b++) c = (uint8_t)(c & 0x80 ? (c << 1) ^ 0x07 : c << 1); }
  return c;
}

int main(int argc, char **argv)
{
  if (argc < 2) { fprintf(stderr, "usage: codec_roundtrip FRAMES [POLL_MS [ANSWER_BYTES [wide]]]\n"); return 2; }
  FILE *f = fopen(argv[1], "rb");
  if (!f) { perror(argv[1]); return 2; }
  const double poll = argc > 2 ? atof(argv[2]) : 10.0;
  wide = argc > 4 && argv[4][0] == 'w';
  const int cap = (argc > 3 ? atoi(argv[3]) : 176) - DML_RESP_HDR - 1;

  /* the table CRC-8 the frame check uses gives the bitwise one's result */
  srand(3);
  for (int t = 0; t < 20000; t++) {
    uint8_t b[300]; const int n = rand() % 300;
    for (int i = 0; i < n; i++) b[i] = (uint8_t)rand();
    if (crc_bitwise(b, n) != dmc_crc8(b, n)) { fprintf(stderr, "table CRC-8 differs from the bitwise one\n"); return 1; }
  }

  int c;
  while ((c = fgetc(f)) != EOF) {
    const int n = fgetc(f);
    uint32_t t;
    uint8_t fr[256], o[80];
    if (n < 1 || fread(&t, 4, 1, f) != 1 || fread(fr, 1, (size_t)n, f) != (size_t)n) break;
    dev_state *d = &dv[c];
    if (!d->init) {
      dmc_reset(&d->today); dmc_reset(&d->enc); dmc_reset(&d->dec);
      d->today.check = 1;
      dmp_reset(&d->penc); dmp_reset(&d->pdec);
      d->init = 1;
    }
    const int m = dmc_encode(&d->today, fr, n, o);
    const long cost = (m ? m : n) + 1;
    d->bytes_today += cost;
    if (fr[0] == 'p') d->bytes_imu += cost; else d->bytes_light += cost;
    d->frames++;
    /* the radio's batch_push: closed by the answer (a poll) or when this frame probably won't fit */
    if (d->open && ((t - d->t0) / 10.0 >= poll || !dmb_room(&d->b) || d->b.w.cap * 8 - d->b.w.pos < (n * 3 / 4 + 4) * 8))
      close_batch(d);
    if (!d->open) begin(d, cap, t);
    keep(d, fr, n);
    if (!dmb_add(&d->b, &d->enc, &d->penc, fr, n)) {       /* didn't fit: the next batch, raw */
      close_batch(d);
      d->spills++;
      begin(d, cap, t);
      keep(d, fr, n);
      dmb_add_raw(&d->b, fr, n);
    }
  }
  long today = 0, imu = 0, light = 0, batched = 0, batches = 0, frames = 0, spills = 0, devices = 0;
  for (int i = 0; i < 256; i++) {
    if (!dv[i].init) continue;
    close_batch(&dv[i]);
    today += dv[i].bytes_today; imu += dv[i].bytes_imu; light += dv[i].bytes_light;
    batched += dv[i].bytes_batched; batches += dv[i].batches; frames += dv[i].frames; spills += dv[i].spills; devices++;
  }
  if (!frames) { fprintf(stderr, "no frames in %s\n", argv[1]); return 1; }
  printf("%ld devices, %ld frames; poll %.1f ms, answers %d B\n", devices, frames, poll, cap + DML_RESP_HDR + 1);
  printf("today's codec %ld B (IMU %ld, light %ld) -> batches %ld B (%+.1f %%), %.2f frames a batch, %.2f %% spilled raw\n",
         today, imu, light, batched, 100.0 * batched / today - 100, (double)frames / batches, 100.0 * spills / frames);
  printf("%s: %ld mismatches\n", mismatches ? "FAILED" : "every frame rebuilt exactly", mismatches);
  return mismatches ? 1 : 0;
}
