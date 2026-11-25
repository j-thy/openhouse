/* run_emu.py's reference: the same batch path built for the host. Writes each closed batch as [len u16][bytes].
     batch_ref FRAMES OUT DEVICE LIMIT ANSWER_BYTES POLL_MS */
#include <stdio.h>
#include <stdlib.h>
#include "doorman_link.h"
#include "doorman_lh2p.h"
int main(int argc, char **argv) {
  FILE *f = fopen(argv[1], "rb"), *o = fopen(argv[2], "wb"); int dev = atoi(argv[3]), limit = atoi(argv[4]), cap = atoi(argv[5]) - DML_RESP_HDR - 1; double poll = atof(argv[6]);
  dmc_state_t codec; dmp_state_t lh2p; dmb_t bt; uint8_t bat[256]; int open = 0, c, nf = 0; uint32_t t0 = 0; int have = 0;
  dmc_reset(&codec); codec.check = 1; dmp_reset(&lh2p);
#define CLOSE() do { if (open) { open = 0; int m = dmb_end(&bt, 1); fputc(m & 255, o); fputc(m >> 8, o); fwrite(bat, 1, (size_t)m, o); } } while (0)
  while ((c = fgetc(f)) != EOF && nf < limit) {
    int n = fgetc(f); uint32_t t; if (fread(&t, 4, 1, f) != 1) break; uint8_t fr[256]; if (fread(fr, 1, (size_t)n, f) != (size_t)n) break;
    if (c != dev) continue;
    if (!have) { t0 = t; have = 1; }
    if ((t - t0) / 10.0 >= poll) { CLOSE(); t0 = t; }
    if (open && (!dmb_room(&bt) || bt.w.cap * 8 - bt.w.pos < (n * 3 / 4 + 4) * 8)) CLOSE();
    if (!open) { dmb_begin(&bt, bat, cap); open = 1; }
    if (!dmb_add(&bt, &codec, &lh2p, fr, n)) { CLOSE(); dmb_begin(&bt, bat, cap); open = 1; dmb_add_raw(&bt, fr, n); }
    nf++;
  }
  fclose(o); return 0;
}
