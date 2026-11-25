/* gen_emu.py's reference: the generator on the host, its output per 1 ms call as [len u16][frames]. */
#include <stdio.h>
#include <stdlib.h>
#include "doorman_lh2gen.h"
static uint8_t buf[4096]; static int len;
static void emit(const uint8_t *f, int n, void *ctx) { (void)ctx; buf[len++] = (uint8_t)n; memcpy(buf + len, f, (size_t)n); len += n; }
int main(int argc, char **argv)
{
  FILE *o = fopen(argv[1], "wb"); const int bases = atoi(argv[2]), density = atoi(argv[3]), calls = atoi(argv[4]);
  static dlg_t g; dlg_init(&g, 1, bases, density, 8);
  for (int ms = 1; ms <= calls; ms++) { len = 0; dlg_run(&g, (uint32_t)ms * 48000u, emit, 0); fputc(len & 255, o); fputc(len >> 8, o); fwrite(buf, 1, (size_t)len, o); }
  fclose(o); return 0;
}
