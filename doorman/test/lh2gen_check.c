/* The synthetic LH2 stream (doorman_lh2gen.h) as a frame dump, for codec_roundtrip and a comparison with real
 * captures' statistics.
 *   lh2gen_check OUT DEVICES SECONDS [BASES [DENSITY [NOISE]]] */
#include <stdio.h>
#include <stdlib.h>
#include "doorman_lh2gen.h"
static FILE *out; static int cur; static uint32_t now_t;
static void emit(const uint8_t *f, int n, void *ctx)
{
  (void)ctx;
  const uint32_t t = now_t / 4800;                         /* 0.1 ms, as the dump's clock */
  fputc(cur, out); fputc(n, out); fwrite(&t, 4, 1, out); fwrite(f, 1, (size_t)n, out);
}
int main(int argc, char **argv)
{
  if (argc < 4) { fprintf(stderr, "usage: lh2gen_check OUT DEVICES SECONDS [BASES [DENSITY [NOISE]]]\n"); return 2; }
  out = fopen(argv[1], "wb");
  const int devs = atoi(argv[2]), bases = argc > 4 ? atoi(argv[4]) : 4, density = argc > 5 ? atoi(argv[5]) : 3, noise = argc > 6 ? atoi(argv[6]) : 3;
  const double secs = atof(argv[3]);
  static dlg_t g[16];
  for (int d = 0; d < devs; d++) dlg_init(&g[d], (uint32_t)(d + 1), bases, density, noise);
  for (uint32_t ms = 1; ms <= (uint32_t)(secs * 1000); ms++)
    for (cur = 0; cur < devs; cur++) { now_t = ms * 48000u; dlg_run(&g[cur], now_t, emit, 0); }
  fclose(out);
  return 0;
}
