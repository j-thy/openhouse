/* The Doorman cycle under load: fixed answer sizes against sizing.h's adaptive ones. A model, not the firmware:
 * per device per cycle it sends what is queued (whole frames) up to the answer limit, loses a share of answers
 * (resent next cycle), and drops data when the radio's queue (UQ_CAP) is full; each slot lasts the slowest
 * exchange of the last 32 cycles plus a guard, as link_poll times it. The saturation logic is sizing.h itself.
 *   cycle_sim                                     (8 devices, 5 % loss, 120 s per row) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sizing.h"
#define UQ_CAP 384
#define GUARD 120
#define FRAME 30          /* the device queues data in frames of about this size (a batch ~ several) */
static double rnd(void) { return rand() / (RAND_MAX + 1.0); }
typedef struct { double delivered, dropped, delay_sum, delay_n, cycle_sum; long cycles, switches, big_cycles; } R;
static R run(int n, double kBps, double loss, int adaptive, double secs, unsigned seed) {
  srand(seed); R r; memset(&r, 0, sizeof r);
  sizing_t z; memset(&z, 0, sizeof z);
  double q[DML_SLOTS] = {0}, t = 0, next_eval = 2.0; int ex_hist[32] = {0}, eh = 0, was_big = 0;
  double acc[DML_SLOTS] = {0};
  while (t < secs) {
    const int maxr = adaptive ? sizing_resp(&z, n) : sizing_base(n);
    int worst_ex = 0; for (int k = 0; k < 32; k++) if (ex_hist[k] > worst_ex) worst_ex = ex_hist[k];
    const int bound = 450 + 4 * maxr;
    const int slot = worst_ex && worst_ex + GUARD < bound ? worst_ex + GUARD : bound;
    const double period = (n * slot + 700) * 1e-6;
    int cyc_worst = 0;
    for (int i = 0; i < n; i++) {
      /* data arrives over the cycle, in frames; ±30 % burstiness */
      acc[i] += kBps * 1000 * period * (0.7 + 0.6 * rnd());
      while (acc[i] >= FRAME) { acc[i] -= FRAME; if (q[i] + FRAME + 1 >= UQ_CAP) r.dropped += FRAME; else q[i] += FRAME + 1; }
      const int room = maxr - DML_RESP_HDR;
      int send = (int)(q[i] < room ? q[i] : room);
      send -= send % (FRAME + 1);                  /* whole frames */
      const int len = DML_RESP_HDR + send, ex = 450 + 4 * len;
      if (ex > cyc_worst) cyc_worst = ex;
      if (rnd() < loss) continue;                  /* lost: resent next cycle */
      const uint8_t flags = q[i] - send >= FRAME ? DML_F_MORE : 0;   /* still queued after it: the radio's flag */
      sizing_answer(&z, i, len, flags, 1, n);
      /* the frames sent waited, on average, the queue ahead of them at this device's rate */
      r.delay_sum += (q[i] / (kBps * 1000.0)) * send; r.delay_n += send;
      q[i] -= send; r.delivered += send;
    }
    ex_hist[eh++ & 31] = cyc_worst;
    t += period; r.cycle_sum += period; r.cycles++;
    if (adaptive && t >= next_eval) { next_eval += 2.0; sizing_eval(&z); if (z.big != was_big) { r.switches++; was_big = z.big; } }
    if (z.big) r.big_cycles++;
  }
  return r;
}
int main(void) {
  const int n = 8; const double loss = 0.05;
  printf("8 devices, 5%% radio loss, 120 s each\n");
  printf("%-34s %8s %8s %9s %9s %8s %8s\n", "load per device", "mode", "carried", "dropped", "queue ms", "cycle ms", "large");
  double loads[] = {5.5, 10, 13, 14, 16, 18, 20, 24};
  const char *why[] = {"our captures, 4 bases", "", "", "heavy LH2 stream, 3 bases", "", "heavy LH2 stream, 4 bases", "", ""};
  for (int k = 0; k < 8; k++)
    for (int a = 0; a < 2; a++) {
      R r = run(n, loads[k], loss, a, 120, 7 + k);
      const double offered = loads[k] * 1000 * n * 120;
      printf("%5.1f kB/s %-23s %8s %7.1f%% %8.1f%% %9.2f %8.2f %7.0f%%%s\n", loads[k], a ? "" : why[k], a ? "adaptive" : "fixed",
             100 * r.delivered / offered, 100 * r.dropped / offered, 1e3 * r.delay_sum / r.delay_n, 1e3 * r.cycle_sum / r.cycles,
             100.0 * r.big_cycles / r.cycles, a && r.switches > 2 ? "  (switched)" : "");
    }
  return 0;
}
