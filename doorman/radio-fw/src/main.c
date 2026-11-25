/* openhouse radio firmware. Default: the Vive Tracker 1.0's nRF51822 (replaces HTC's radio app at 0x400).
 * DML_IDX: Valve's nRF52 radio module (Index controller and relatives), same updater protocol.
 *
 * SAFETY: this image is installed by HTC's 1 KB bootloader and can only be replaced through its own
 * updater (frames Y/Z/[/!/0x20 below, same protocol and replies as HTC's app). So:
 *   - no interrupts at all (everything is polled): nothing depends on how the bootloader forwards IRQs;
 *   - the SPI slave and the updater are serviced first in every loop iteration;
 *   - the radio link starts 1 s after boot and is skipped entirely in safe mode (3 quick resets in a
 *     row, or the LPC asks for it with frame 'O' 'S'), so a link bug can't lock the updater out;
 *   - a watchdog (8 s) resets any hang; it is fed right before every deliberate reset so the
 *     bootloader always gets the full period to install an image.
 * SPI slave setup copied from HTC's app (FUN_00002de4): SPIS1, SCK P0.0, MISO P0.1, MOSI P0.2,
 * CSN P0.3, mode 0, MSB first, DEF 0xFE ("busy"), ORC 0x00, END->ACQUIRE short. */
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "doorman_link.h"
static void addr_check(void);
#ifdef DML_FAKE
#pragma GCC diagnostic ignored "-Wunused-function"         /* the test device has no main MCU: its SPI side goes unused */
#endif
#define DMC_YIELD() addr_check()                           /* the codec checks for POLLs between fields */
#include "doorman_codec.h"
#ifdef DML_IDX
#include "doorman_lh2p.h"                                  /* batches: the nRF52 radios have the RAM for it */
#define DLG_YIELD() addr_check()
#include "doorman_lh2gen.h"                                /* test load: a synthetic LH2 device (air op 'G') */
#define DML_JF_MINE (DML_JF_CHECK | DML_JF_BATCH | DML_JF_BATCH32)
#else
#define DML_JF_MINE DML_JF_CHECK
#endif

#define STAGE_BASE   0x10000u
#define STAGE_MAX    0xF800u           /* 0x10000-0x1F7FF (Index family: 0x1F800 = Valve's record page) */
#ifdef DML_IDX
/* Valve nRF52 radio (Index controller, Vive Tracker 2018, Vive Pro controller, Tundra tracker), app at
 * 0x1000. Header in the Cortex-M4's reserved vector slots: +0x1C length, +0x20 marker, +0x24 version.
 * SPIS1: MOSI P0.13, MISO P0.12, SCK P0.14, CSN P0.11. P0.20 = "reply waiting" line to the main MCU. */
#define HDR_LEN      0x1Cu
#define HDR_MARK     0x20u
#define ATTN_PIN     20
#else
#define HDR_LEN      0x10u
#define HDR_MARK     0x14u
#endif
#ifdef DML_IDX
#define S_TXPTR   NRF_SPIS1->TXD.PTR
#define S_MAXTX   NRF_SPIS1->TXD.MAXCNT
#define S_AMTTX   NRF_SPIS1->TXD.AMOUNT
#define S_RXPTR   NRF_SPIS1->RXD.PTR
#define S_MAXRX   NRF_SPIS1->RXD.MAXCNT
#define S_AMTRX   NRF_SPIS1->RXD.AMOUNT
#else
#define S_TXPTR   NRF_SPIS1->TXDPTR
#define S_MAXTX   NRF_SPIS1->MAXTX
#define S_AMTTX   NRF_SPIS1->AMOUNTTX
#define S_RXPTR   NRF_SPIS1->RXDPTR
#define S_MAXRX   NRF_SPIS1->MAXRX
#define S_AMTRX   NRF_SPIS1->AMOUNTRX
#endif
#define MARK_INSTALL 0x01051974u
#define MARK_BOOT    0xF001BA11u

/* ---------------- time (TIMER0 free-running at 1 MHz, polled) ---------------- */
static void timer_init(void)
{
  NRF_TIMER0->MODE = 0; NRF_TIMER0->BITMODE = 3; NRF_TIMER0->PRESCALER = 4;   /* 16 MHz / 16 */
  NRF_TIMER0->TASKS_CLEAR = 1; NRF_TIMER0->TASKS_START = 1;
}
static uint32_t now_us(void) { NRF_TIMER0->TASKS_CAPTURE[0] = 1; return NRF_TIMER0->CC[0]; }

/* ---------------- watchdog / boot counter ---------------- */
typedef struct { uint32_t magic, quick_boots, sum; } boot_rec_t;
static boot_rec_t boot_rec __attribute__((section(".noinit")));
#define BOOT_MAGIC 0x4F48524Fu
static int safe_mode;

static void wdt_feed(void) { NRF_WDT->RR[0] = 0x6E524635u; }
static void wdt_start(void)
{
  if (NRF_WDT->RUNSTATUS) { wdt_feed(); return; }
  NRF_WDT->CONFIG = 1;                 /* keep running while the CPU sleeps, pause on debug halt */
  NRF_WDT->CRV = 8 * 32768;
  NRF_WDT->RREN = 1;
  NRF_WDT->TASKS_START = 1;
}
static void reset_now(void)
{
  wdt_feed();
  NVIC_SystemReset();
}
static void boot_count(void)
{
  if (boot_rec.magic != BOOT_MAGIC || boot_rec.sum != (boot_rec.magic ^ boot_rec.quick_boots)) boot_rec.quick_boots = 0;
  if (NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk) boot_rec.quick_boots = 0;
#ifdef DML_IDX
  /* Index family: safe mode keeps the link off, and the link is the only way to update this radio, so
     only crashes count (watchdog, lockup). Requested resets (the main MCU's \RESET, the bootloader's
     install restart, our own) are normal: an update alone makes two in a row. */
  if (!(NRF_POWER->RESETREAS & (POWER_RESETREAS_DOG_Msk | POWER_RESETREAS_LOCKUP_Msk))) boot_rec.quick_boots = 0;
#endif
  boot_rec.quick_boots++;
  boot_rec.magic = BOOT_MAGIC; boot_rec.sum = boot_rec.magic ^ boot_rec.quick_boots;
  NRF_POWER->RESETREAS = 0xFFFFFFFF;
  if (boot_rec.quick_boots >= 3) safe_mode = 1;
}
static void boot_stable(void) { boot_rec.quick_boots = 0; boot_rec.sum = boot_rec.magic ^ 0; }

#ifdef DML_DIAG   /* dev only: make TARGET=idx DIAG=1 */
/* survives resets (not power-off): what the main MCU sent around the last boots, read with air 'X' */
static struct { uint32_t magic; uint16_t boots, ny, nz, nc; uint8_t cst, rr, nop, scrub; uint16_t zbad, ack_q, ack_out, ybad; uint32_t zmax_us; } dg __attribute__((section(".noinit")));

/* SPI transfer trace, armed by the first SPI 'Y': [dt 100us u16][nrx][ntx][rx0][rx1][tx0][tx1][tx2][st] */
#define TR_N 96
static struct { uint32_t magic, n, t_last; uint8_t e[TR_N][10]; } tr __attribute__((section(".noinit")));
#define DG(x) do { x; } while (0)
#else
#define DG(x) do { } while (0)
#endif

/* ---------------- flash (staging bank only) ---------------- */
static void nvmc_wait(void) { while (!NRF_NVMC->READY) {} }
static void flash_erase_page(uint32_t a)
{
  if (a < STAGE_BASE || a >= STAGE_BASE + STAGE_MAX) return;   /* never outside the staging bank */
  NRF_NVMC->CONFIG = 2; nvmc_wait();
  NRF_NVMC->ERASEPAGE = a; nvmc_wait();
  NRF_NVMC->CONFIG = 0; nvmc_wait();
}
static void flash_write(uint32_t a, const uint32_t *w, uint32_t n)
{
  if (a < STAGE_BASE || a + 4 * n > STAGE_BASE + STAGE_MAX) return;
  NRF_NVMC->CONFIG = 1; nvmc_wait();
  for (uint32_t i = 0; i < n; i++) { ((volatile uint32_t *)a)[i] = w[i]; nvmc_wait(); }
  NRF_NVMC->CONFIG = 0; nvmc_wait();
}

/* ---------------- updater: HTC's protocol, byte for byte ---------------- */
static uint8_t stg_open;
static uint8_t stg_air;                /* staging opened over the radio link: keep the link up */
static uint32_t stg_off;

static int page_blank(uint32_t a)
{
  uint32_t page = NRF_FICR->CODEPAGESIZE;
  for (uint32_t i = 0; i < page; i += 4) if (*(volatile uint32_t *)(a + i) != 0xFFFFFFFFu) return 0;
  return 1;
}
#ifdef DML_IDX
/* A 4 KB nRF52 page erase stalls the CPU for up to ~90 ms, and Valve's main firmware gives each staging
 * step ~120 ms (reply line + read included): erasing on the first block of a page made every USB
 * radio update fail at block 0. So staging is cleared ahead of time, a page per idle pass from 10 s
 * after boot, and blocks only write. A committed image that isn't installed yet is left alone. */
#define APP_BASE 0x1000u
static uint32_t scrub_page = STAGE_BASE;
static int staged_is_pending(void)
{
  uint32_t mark = *(volatile uint32_t *)(STAGE_BASE + HDR_MARK);
  if (mark != MARK_INSTALL && mark != MARK_BOOT) return 0;
  uint32_t len = *(volatile uint32_t *)(STAGE_BASE + HDR_LEN);
  if (len == 0 || len > STAGE_MAX) return 0;
  for (uint32_t i = 0; i < len; i += 4)                    /* same as what runs = already installed */
    if (i != HDR_MARK && *(volatile uint32_t *)(STAGE_BASE + i) != *(volatile uint32_t *)(APP_BASE + i)) return 1;
  return 0;
}
static void staging_scrub(void)
{
  if (stg_open || scrub_page >= STAGE_BASE + STAGE_MAX) return;
  if (scrub_page == STAGE_BASE && staged_is_pending()) { scrub_page = STAGE_BASE + STAGE_MAX; DG(dg.scrub = 0xEE); return; }
  if (!page_blank(scrub_page)) flash_erase_page(scrub_page);
  scrub_page += NRF_FICR->CODEPAGESIZE;
}
#endif
static int stage_write(const uint8_t *d, uint32_t n)       /* HTC FUN_000021c8 */
{
  uint32_t page = NRF_FICR->CODEPAGESIZE;
  if (!stg_open || stg_off > STAGE_MAX || n > 64 || (n & 3) || (stg_off & 3)) { stg_off += n; return 0; }
  if (stg_off + n > STAGE_MAX) return 0;
  uint32_t in_page = stg_off & (page - 1);
  if (in_page == 0) { if (!page_blank(STAGE_BASE + stg_off)) flash_erase_page(STAGE_BASE + stg_off); }
  else if (in_page + n > page && !page_blank(STAGE_BASE + stg_off - in_page + page))
    flash_erase_page(STAGE_BASE + stg_off - in_page + page);
  uint32_t w[16];
  memcpy(w, d, n);
  flash_write(STAGE_BASE + stg_off, w, n / 4);
  stg_off += n;
  return 1;
}
static void signature(uint32_t s[4])                       /* HTC FUN_000020cc */
{
  uint32_t len = *(volatile uint32_t *)(STAGE_BASE + HDR_LEN);
  uint32_t end = (len + STAGE_BASE + 15) & ~15u;
  uint32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0;
  if (len > STAGE_MAX) end = STAGE_BASE;
  for (const volatile uint32_t *p = (const uint32_t *)STAGE_BASE; (uint32_t)p < end; p += 4) {
    uint32_t top = s1 << 31;
    uint32_t n1 = (s1 >> 1) ^ (s2 << 31) ^ p[1];
    uint32_t n2 = p[2] ^ ((s2 >> 1) | (s3 << 31));
    uint32_t t = s0 ^ (p[3] >> 31);
    uint32_t n3 = ((p[3] & 0x7FFFFFFFu) ^ (s3 >> 1)) | (((t ^ (s0 >> 2) ^ (s0 >> 27) ^ (s0 >> 29)) & 1u) << 31);
    s0 = ((s0 >> 1) | top) ^ p[0];
    s1 = n1; s2 = n2; s3 = n3;
  }
  s[0] = s0; s[1] = s1; s[2] = s2; s[3] = s3;
}
static int stage_commit(const uint8_t *sig, uint32_t marker)
{
  if (!stg_open) return 0;
  stg_open = 0;
  uint32_t s[4];
  signature(s);
  if (memcmp(s, sig, 16) != 0) return 0;
  flash_write(STAGE_BASE + HDR_MARK, &marker, 1);
  return 1;
}

/* ---------------- SPI slave link to the LPC ---------------- */
/* Every transaction clocks one LPC frame in and our pending reply out: [len incl. itself][bytes].
 * 0x00 = nothing pending; while we hold the semaphore the master reads DEF = 0xFE and retries. */
static uint8_t spi_rx[80];
static uint8_t spi_tx[80];
#define RQ_BYTES 1024                  /* replies to the LPC (events + downlink frames), [len][frame] */
static uint8_t rq[RQ_BYTES];
static uint16_t rq_head, rq_tail;      /* byte ring */
static uint8_t tx_loaded, tx_used;     /* reply in spi_tx: bytes to clock / ring bytes (0 = none) */

static uint8_t lead0;                  /* Index: 'l' 1 = every reply starts with a 0x00 byte */
static uint16_t rq_used(void) { return (uint16_t)((rq_tail - rq_head) & (RQ_BYTES - 1)); }
static int reply_push(const uint8_t *f, uint8_t n)          /* f = op + payload */
{
  if (n == 0 || n > 63 || rq_used() + n + 1 >= RQ_BYTES) return 0;
  rq[rq_tail] = (uint8_t)(n | (lead0 ? 0x80 : 0));         /* bit 7: send a 0x00 first (Index 'l' mode) */
  rq_tail = (rq_tail + 1) & (RQ_BYTES - 1);
  for (int i = 0; i < n; i++) { rq[rq_tail] = f[i]; rq_tail = (rq_tail + 1) & (RQ_BYTES - 1); }
  return 1;
}
static void reply2(uint8_t op, uint8_t v) { uint8_t b[2] = { op, v }; reply_push(b, 2); }

static void spis_init(void)
{
#ifdef DML_IDX
  /* stock boot levels (FUN_00004fe4/4fb4/4fa4), then the SPI pins and the reply line */
  for (int i = 0; i <= 2; i++) { NRF_P0->PIN_CNF[i] = 3; NRF_P0->OUTSET = 1u << i; }
  for (int i = 4; i <= 6; i++) { NRF_P0->PIN_CNF[i] = 3; NRF_P0->OUTCLR = 1u << i; }
  NRF_P0->PIN_CNF[3] = 0;
  NRF_P0->PIN_CNF[ATTN_PIN] = 3; NRF_P0->OUTCLR = 1u << ATTN_PIN;
  NRF_P0->PIN_CNF[11] = 0; NRF_P0->PIN_CNF[12] = 0; NRF_P0->PIN_CNF[13] = 0; NRF_P0->PIN_CNF[14] = 0;
  NRF_SPIS1->PSEL.CSN = 11; NRF_SPIS1->PSEL.SCK = 14; NRF_SPIS1->PSEL.MISO = 13; NRF_SPIS1->PSEL.MOSI = 12;   /* Valve 0x9420: 0x50C (MISO) = 13, 0x510 (MOSI) = 12 */
#else
  NRF_GPIO->PIN_CNF[1] = 0; NRF_GPIO->PIN_CNF[3] = 0; NRF_GPIO->PIN_CNF[2] = 0; NRF_GPIO->PIN_CNF[0] = 0;
  NRF_SPIS1->PSELCSN = 3; NRF_SPIS1->PSELSCK = 0; NRF_SPIS1->PSELMOSI = 2; NRF_SPIS1->PSELMISO = 1;
#endif
  S_MAXRX = 0; S_MAXTX = 0;
  NRF_SPIS1->CONFIG = 0;               /* MSB first, CPHA 0, CPOL 0 */
  NRF_SPIS1->DEF = 0xFE;
  NRF_SPIS1->ORC = 0x00;
  NRF_SPIS1->EVENTS_END = 0; NRF_SPIS1->EVENTS_ACQUIRED = 0;
  NRF_SPIS1->SHORTS = SPIS_SHORTS_END_ACQUIRE_Msk;
  NRF_SPIS1->INTENCLR = 0xFFFFFFFF;
  NRF_SPIS1->ENABLE = 2;
  NRF_SPIS1->TASKS_ACQUIRE = 1;        /* first ACQUIRED: load buffers */
}

static void load_reply_and_release(void)
{
  tx_loaded = 0; tx_used = 0;
  if (rq_used()) {
    uint8_t h = rq[rq_head], n = h & 0x7F, o = 0;
    if (h & 0x80) spi_tx[o++] = 0;
    spi_tx[o] = (uint8_t)(n + 1);
    for (int i = 0; i < n; i++) spi_tx[o + 1 + i] = rq[(rq_head + 1 + i) & (RQ_BYTES - 1)];
    tx_loaded = (uint8_t)(o + n + 1);                       /* bytes the master must clock */
    tx_used = (uint8_t)(n + 1);                             /* ring bytes this reply takes */
  } else {
    spi_tx[0] = 0;
  }
  S_TXPTR = (uint32_t)spi_tx; S_MAXTX = tx_loaded ? tx_loaded : 1;
  S_RXPTR = (uint32_t)spi_rx; S_MAXRX = sizeof spi_rx;
  NRF_SPIS1->TASKS_RELEASE = 1;
#ifdef DML_IDX
  if (tx_loaded) NRF_P0->OUTSET = 1u << ATTN_PIN; else NRF_P0->OUTCLR = 1u << ATTN_PIN;
#endif
}

static void link_from_lpc(const uint8_t *f, uint8_t n);   /* below */
static void addr_check(void);
static int link_safe_request, link_sleep;
/* survives resets (not power-off): what the main MCU sent around the last boots, read with air 'X' */
#ifdef DML_DIAG
static void dg_boot(void)
{
  if (dg.magic != 0x44474E31u) { memset(&dg, 0, sizeof dg); dg.magic = 0x44474E31u; tr.magic = 0; }
  dg.boots++; dg.rr = (uint8_t)NRF_POWER->RESETREAS; dg.nop = 0;
}
#endif
static uint32_t lpc_frames, lpc_bad;

/* Staging replies: the HTC LPC wants the command echoed ('Y' 0, 'Z' 0, '[' 0); Valve's main firmware
   (Index, LWU radio update FUN_004213b8) counts '!' (0x21) + status replies, one per step, and gives
   up ("LWU: radio err") when one doesn't come within 120 ms. */
/* Staging replies echo the command ('Y' 0, 'Z' st, '[' st), like the HTC LPC wants and like Valve's
   own nRF52 radio does ('Y' reply = const 59 00 @0x9406). Valve's main fw counts 0x59/0x5a/0x5b/0x21
   as acks (FUN_004158a4 -> 0x4159ee) and gives each step ~120 ms (FUN_004213b8). */

#ifdef DML_IDX
/* GPIO service, like Valve's radio: some of the device's buttons (on the Index, the ones it
   pairs with) are wired to the nRF, not the main MCU. The main MCU sets those pins up with 0x83
   requests ([0x83][op][4 B][pin][arg][u16 debounce ms]) and asks to be told about edges; each
   edge comes back as 84 14 00 00 00 00 [pin] [1 low / 2 high]. Without it the main MCU never
   sees those buttons, so its pairing combination never fires. */
#define GW_MAX 6
static struct { uint8_t used, pin, mode, last; uint16_t deb_ms; uint8_t pend; uint32_t since; } gw[GW_MAX];
static int gpio_ours(uint8_t pin) { return pin > 31 || (pin >= 11 && pin <= 14) || pin == ATTN_PIN; }
static void gpio_edge(uint8_t pin, uint8_t level)
{
  uint8_t ev[8] = { 0x84, 0x14, 0, 0, 0, 0, pin, (uint8_t)(level ? 2 : 1) };
  reply_push(ev, 8);
}
static void gpio_watch_poll(void)
{
  for (int i = 0; i < GW_MAX; i++) {
    if (!gw[i].used) continue;
    uint8_t lv = (uint8_t)((NRF_P0->IN >> gw[i].pin) & 1);
    if (lv == gw[i].last) { gw[i].pend = 0; continue; }
    if (!gw[i].pend || lv != (gw[i].pend & 1)) { gw[i].pend = (uint8_t)(2 | lv); gw[i].since = now_us(); }
    if (now_us() - gw[i].since < (uint32_t)gw[i].deb_ms * 1000u) continue;
    gw[i].last = lv; gw[i].pend = 0;
    if (gw[i].mode == 3 || (gw[i].mode == 1 && lv) || (gw[i].mode == 2 && !lv)) gpio_edge(gw[i].pin, lv);
  }
}
/* Analog service, in the same 0x83 requests (Valve's sub-ops 1e/1f/21/28): the Index's thumbstick axes
   and trackpad force sensor are wired to the nRF's analog inputs, and the main MCU has the radio
   sample them. Without it the stick stays centred and force reads nothing, on USB too.
     1e: [res 0-3][oversample 0-8]                     -> 80 st
     1f:                                               -> 84 20 00 00 00 00 [res][oversample]
     21: channel: [ain+][ain-] (0-7 AINn, 8 VDD, 9 none) [res+][res-][gain][ref][tacq]
         [excite pin][active level][settle us u32][use excite]           -> 80 st (3: no free channel)
     28: [u32 mask, bit k = ain+ k]: sample those once, in channel order -> 80 st, then
         84 29 00 00 00 00 [n] n x [ain+][ain-][s16]
   Before each sample the excite pin is driven to its level for the settle time, then released. */
#define ADC_CH 4
static struct { uint8_t used, p, n, pin, level, excite, want; uint32_t config, settle_us; } adc[ADC_CH];
static uint8_t adc_cur = 0xFF, adc_state, adc_rep[7 + 4 * ADC_CH];   /* state 0 idle, 1 settling, 2 converting */
static uint32_t adc_t;
static volatile int16_t adc_sample;
static const uint8_t adc_pin[8] = { 2, 3, 4, 5, 28, 29, 30, 31 };
static uint8_t adc_psel(uint8_t code) { return code < 9 ? (uint8_t)(code + 1) : 0; }   /* 1-8 AIN0-7, 9 VDD */
static void adc_excite(int i, int on)
{
  if (!adc[i].excite || !adc[i].pin || adc[i].pin > 31) return;
  if ((adc[i].level != 0) == (on != 0)) NRF_P0->OUTSET = 1u << adc[i].pin; else NRF_P0->OUTCLR = 1u << adc[i].pin;
}
static void adc_next(void)
{
  int i = adc_cur == 0xFF ? 0 : adc_cur + 1;
  while (i < ADC_CH && !(adc[i].used && adc[i].want)) i++;
  if (i >= ADC_CH) {                                       /* all done: one report */
    adc_cur = 0xFF; adc_state = 0;
    NRF_SAADC->ENABLE = 0;
    adc_rep[0] = 0x84; adc_rep[1] = 0x29; memset(adc_rep + 2, 0, 4);
    reply_push(adc_rep, (uint8_t)(7 + 4 * adc_rep[6]));
    adc_rep[6] = 0;
    return;
  }
  adc_cur = (uint8_t)i; adc[i].want = 0;
  NRF_SAADC->ENABLE = 1;
  NRF_SAADC->CH[0].PSELP = adc_psel(adc[i].p);
  NRF_SAADC->CH[0].PSELN = adc_psel(adc[i].n);
  NRF_SAADC->CH[0].CONFIG = adc[i].config;
  NRF_SAADC->RESULT.PTR = (uint32_t)&adc_sample; NRF_SAADC->RESULT.MAXCNT = 1;
  NRF_SAADC->EVENTS_STARTED = 0; NRF_SAADC->EVENTS_END = 0;
  NRF_SAADC->TASKS_START = 1;
  for (uint32_t t = now_us(); !NRF_SAADC->EVENTS_STARTED && now_us() - t < 100;) {}
  if (adc[i].excite && adc[i].settle_us >= 16) { adc_excite(i, 1); adc_t = now_us(); adc_state = 1; }   /* like stock: < 1 RTC tick, none */
  else { NRF_SAADC->TASKS_SAMPLE = 1; adc_state = 2; }
}
static void adc_poll(void)
{
  if (adc_state == 1 && now_us() - adc_t >= adc[adc_cur].settle_us) { NRF_SAADC->TASKS_SAMPLE = 1; adc_state = 2; }
  if (adc_state != 2 || !NRF_SAADC->EVENTS_END) return;
  NRF_SAADC->EVENTS_END = 0;
  adc_excite(adc_cur, 0);
  uint8_t *e = adc_rep + 7 + 4 * adc_rep[6]++;
  e[0] = adc[adc_cur].p; e[1] = adc[adc_cur].n; e[2] = (uint8_t)adc_sample; e[3] = (uint8_t)((uint16_t)adc_sample >> 8);
  adc_next();
}
static void adc_stop(void)
{
  if (adc_state) { adc_excite(adc_cur, 0); NRF_SAADC->TASKS_STOP = 1; NRF_SAADC->ENABLE = 0; }
  adc_state = 0; adc_cur = 0xFF; adc_rep[6] = 0;
  for (int i = 0; i < ADC_CH; i++) adc[i].want = 0;
}
static void adc_request(const uint8_t *f, uint8_t n)
{
  switch (f[1]) {
  case 0x1e: {
    if (n < 8) { reply2(0x80, 2); return; }
    adc_stop();
    NRF_SAADC->RESOLUTION = f[6] <= 3 ? f[6] : 0;
    NRF_SAADC->OVERSAMPLE = f[7] <= 8 ? f[7] : 0;
    reply2(0x80, 0); return;
  }
  case 0x1f: {
    uint8_t b[8] = { 0x84, 0x20, 0, 0, 0, 0, (uint8_t)(NRF_SAADC->RESOLUTION & 7), (uint8_t)(NRF_SAADC->OVERSAMPLE & 0xF) };
    reply_push(b, 8); return;
  }
  case 0x21: {
    if (n < 20 || (f[6] > 8 && f[7] > 8)) { reply2(0x80, 2); return; }
    uint8_t p = f[6] > 8 ? 9 : f[6], m = f[7] > 8 ? 9 : f[7];
    int k = -1;
    for (int i = 0; i < ADC_CH && k < 0; i++) if (adc[i].used && adc[i].p == p && adc[i].n == m) k = i;
    for (int i = 0; i < ADC_CH && k < 0; i++) if (!adc[i].used) k = i;
    if (k < 0) { reply2(0x80, 3); return; }
    if (adc_state) adc_stop();
    uint32_t c = (uint32_t)(f[8] & 3) | (uint32_t)(f[9] & 3) << 4 | (uint32_t)(f[10] <= 7 ? f[10] : 5) << 8 |
                 (uint32_t)(f[11] == 1) << 12 | (uint32_t)(f[12] <= 5 ? f[12] : 0) << 16 |
                 (uint32_t)(p != 9 && m != 9) << 20 | (uint32_t)((NRF_SAADC->OVERSAMPLE & 0xF) != 0) << 24;
    adc[k].used = 1; adc[k].p = p; adc[k].n = m; adc[k].config = c;
    adc[k].pin = f[13]; adc[k].level = f[14]; adc[k].excite = f[19];
    adc[k].settle_us = (uint32_t)f[15] | (uint32_t)f[16] << 8 | (uint32_t)f[17] << 16 | (uint32_t)f[18] << 24;
    if (p < 8) NRF_P0->PIN_CNF[adc_pin[p]] = 0;            /* as stock: a plain input (boot drives some) */
    if (m < 8) NRF_P0->PIN_CNF[adc_pin[m]] = 0;
    reply2(0x80, 0); return;
  }
  case 0x28: {
    reply2(0x80, n < 10 ? 2 : 0);
    if (n < 10) return;
    if (adc_state) {                                       /* a new request cuts the last one short */
      adc_stop();
      uint8_t b[39] = { 0x84, 0x29 }; reply_push(b, sizeof b);
    }
    uint32_t mask = (uint32_t)f[6] | (uint32_t)f[7] << 8 | (uint32_t)f[8] << 16 | (uint32_t)f[9] << 24;
    for (int i = 0; i < ADC_CH; i++) adc[i].want = adc[i].used && adc[i].p < 9 && (mask >> adc[i].p & 1);
    adc_cur = 0xFF; adc_rep[6] = 0;
    adc_next();
    return;
  }
  }
}

static void gpio_request(const uint8_t *f, uint8_t n)
{
  if (n >= 2 && (f[1] == 0x1e || f[1] == 0x1f || f[1] == 0x21 || f[1] == 0x28)) { adc_request(f, n); return; }
  if (n < 6) { reply2(0x80, 0x04); return; }
  const uint8_t op = f[1], pin = f[6], arg = n > 7 ? f[7] : 0;
  uint8_t st = 2;
  if (gpio_ours(pin)) { reply2(0x80, 2); return; }
  volatile uint32_t *cnf = &NRF_P0->PIN_CNF[pin];
  switch (op) {
  case 0x00:                                               /* direction */
    if (n < 8) break;
    st = 0;
    if (arg == 0) *cnf |= 2; else if (arg == 1) *cnf = 0x303; else if (arg == 2) *cnf = 0; else st = 2;
    break;
  case 0x01: {                                             /* get direction */
    uint8_t v = (*cnf & 1) ? 1 : ((*cnf & 2) ? 0 : 2);
    uint8_t b[8] = { 0x84, 2, 0, 0, 0, 0, pin, v }; reply_push(b, 8); return;
  }
  case 0x03:                                               /* pull: 1 up, 2 down */
    if (n < 8) break;
    *cnf = (*cnf & ~0xCu) | ((uint32_t)(arg == 1 ? 3 : arg == 2 ? 1 : 0) << 2); st = 0; break;
  case 0x04: {
    uint32_t pl = (*cnf & 0xF) >> 2;
    uint8_t b[8] = { 0x84, 5, 0, 0, 0, 0, pin, (uint8_t)(pl == 1 ? 2 : pl == 3 ? 1 : 0) }; reply_push(b, 8); return;
  }
  case 0x06:                                               /* drive strength, same code */
    if (n < 8 || arg > 7) break;
    *cnf = (*cnf & ~0x700u) | ((uint32_t)arg << 8); st = 0; break;
  case 0x07: {
    uint8_t b[8] = { 0x84, 8, 0, 0, 0, 0, pin, (uint8_t)((*cnf >> 8) & 7) }; reply_push(b, 8); return;
  }
  case 0x09:                                               /* output level */
    if (n < 8 || pin == 0) break;
    if (arg == 0) { NRF_P0->OUTCLR = 1u << pin; st = 0; } else if (arg == 1) { NRF_P0->OUTSET = 1u << pin; st = 0; }
    break;
  case 0x0a: {                                             /* read level */
    uint8_t b[8] = { 0x84, 0x0b, 0, 0, 0, 0, pin, (uint8_t)((NRF_P0->IN >> pin) & 1) }; reply_push(b, 8); return;
  }
  case 0x0c: {                                             /* watch: arg 1 falling, 2 rising, 3 both, 0 off */
    if (n < 10) break;
    int k = -1;
    for (int i = 0; i < GW_MAX; i++) if (gw[i].used && gw[i].pin == pin) k = i;
    if (k >= 0) gw[k].used = 0;
    if (arg == 0) { st = 0; break; }
    if ((*cnf & 3) != 0) break;                            /* must be a connected input */
    if (k < 0) for (int i = 0; i < GW_MAX; i++) if (!gw[i].used) { k = i; break; }
    if (k < 0) break;
    gw[k].pin = pin; gw[k].mode = arg == 1 ? 2 : arg == 2 ? 1 : 3;   /* mode: 1 reports high, 2 low */
    gw[k].deb_ms = (uint16_t)(f[8] | f[9] << 8); gw[k].last = 0xFF; gw[k].pend = 0; gw[k].used = 1;
    st = 0;
    reply2(0x80, st);
    gpio_watch_poll();                                     /* like stock: report the level right away */
    return;
  }
  case 0x0d: {
    int k = -1;
    for (int i = 0; i < GW_MAX; i++) if (gw[i].used && gw[i].pin == pin) k = i;
    uint8_t m = k < 0 ? 0 : gw[k].mode == 1 ? 2 : gw[k].mode == 2 ? 1 : 3;
    uint16_t d = k < 0 ? 0xFFFF : gw[k].deb_ms;
    uint8_t b[10] = { 0x84, 0x0e, 0, 0, 0, 0, pin, m, (uint8_t)d, (uint8_t)(d >> 8) }; reply_push(b, 10); return;
  }
  default: break;
  }
  reply2(0x80, st);
}
#endif

/* The last frames from the main MCU besides the IMU/light stream and filler: [op][b1][b2][count]
   (a repeat of the newest op is counted). Air 'X' reports them: what a main firmware sends that this
   radio may not know, such as a host command relayed to the radio. */
#define OPS_N 6
static uint8_t ops_log[OPS_N][4], ops_head;
static void ops_note(const uint8_t *f, uint32_t n)
{
  if (f[0] == 'p' || f[0] == 'q' || f[0] == 0x00 || f[0] == 0xFF) return;
  uint8_t *last = ops_log[(ops_head + OPS_N - 1) % OPS_N];
  if (last[3] && last[0] == f[0]) { if (last[3] < 255) last[3]++; return; }
  uint8_t *e = ops_log[ops_head]; ops_head = (uint8_t)((ops_head + 1) % OPS_N);
  e[0] = f[0]; e[1] = n > 1 ? f[1] : 0; e[2] = n > 2 ? f[2] : 0; e[3] = 1;
}

static void handle_lpc_frame(const uint8_t *f, uint32_t n)
{
  uint8_t op = f[0];
  ops_note(f, n);
  DG(if (op == 'Y') dg.ny++; else if (op == 'Z') dg.nz++);
  switch (op) {
  case 0x00: case 0xFF: return;                            /* filler: only clocks our reply out */
  case 'X': {
    uint8_t b[5] = { 'X' }; uint32_t v = DML_VERSION; memcpy(b + 1, &v, 4); reply_push(b, 5); return;
  }
  case 'c': {
    uint8_t b[9] = { 'c' }; memcpy(b + 1, (const void *)NRF_FICR->DEVICEID, 8); reply_push(b, 9); return;
  }
  case 'Y':                                                /* drop pending replies: during staging only
                                                              2-byte updater replies are ever pending, so
                                                              they never stretch a Z frame with fill */
    if (tx_loaded) { rq_head = (rq_head + tx_used) & (RQ_BYTES - 1); tx_loaded = 0; }
    rq_head = rq_tail;
    stg_open = 1; stg_air = 0; stg_off = 0; reply2('Y', 0); return;
  case 'Z': reply2('Z', stage_write(f + 1, n - 1) ? 0 : 1); return;
  case '[': reply2('[', (n == 17 && stage_commit(f + 1, MARK_INSTALL)) ? 0 : 1); return;
  case '!': reply2('!', (n == 17 && stage_commit(f + 1, MARK_BOOT)) ? 0 : 1); return;
  case 0x20: {                                             /* HTC: reboot into the bootloader to install */
    if (stg_open) { reply2(0x20, 1); return; }
    if (*(volatile uint32_t *)(STAGE_BASE + HDR_LEN) != 0xFFFFFFFFu &&
        *(volatile uint32_t *)(STAGE_BASE + HDR_MARK) == MARK_BOOT) {
      NRF_POWER->GPREGRET = 0x71;
      reset_now();
    }
    reply2(0x20, 2); return;
  }
  case '\\':
    if (n >= 6 && memcmp(f, "\\RESET", 6) == 0) reset_now();
    return;
  case 'O':                                                /* ours: 'O' 'S' = stay in safe mode */
    if (n >= 2 && f[1] == 'S') link_safe_request = 1;
    reply2('O', (uint8_t)(safe_mode | link_safe_request << 1));
    return;
#ifdef DML_IDX
  case 'l': reply2('l', n >= 2 ? f[1] : 0); lead0 = n >= 2 && f[1] != 0; return;   /* reply, then switch */
  case 0x81: reply2(0x82, 0x01); return;                   /* stock constant (0x9434) */
  case 0x83: gpio_request(f, (uint8_t)n); return;
  case 0xA0: case 'o': case 'W': case '^': case 'a': case 'b': case 'e': case 'k': case '_': case '`':
  case '"': case 0x90: case 'f': case 'd': case 'm':
    return;                                                /* Valve link key / pairing store / test mode */
#endif
  default:
    link_from_lpc(f, (uint8_t)n);                          /* p/q/s/r/Q/n/V/P data, S/R/T link control */
    return;
  }
}

static uint8_t dbg_rx[12], dbg_nrx, dbg_ntx;   /* last SPI frame from the main MCU (air 'X' diagnostics) */
static void spis_poll(void)
{
  if (!NRF_SPIS1->EVENTS_ACQUIRED) return;
  NRF_SPIS1->EVENTS_ACQUIRED = 0;
  if (NRF_SPIS1->EVENTS_END) {
    NRF_SPIS1->EVENTS_END = 0;
    uint32_t nrx = S_AMTRX, ntx = S_AMTTX;
#ifdef DML_IDX
    NRF_P0->OUTCLR = 1u << ATTN_PIN;
#endif
    if (tx_loaded && ntx >= tx_loaded) {                   /* the master clocked our whole reply */
      rq_head = (rq_head + tx_used) & (RQ_BYTES - 1);
      tx_loaded = 0;
    }
    addr_check();
#ifdef DML_DIAG
    /* dev: log every unusual transfer (not the 0x70/0x71/0x83 data stream, or one where we sent more
       than a byte), kept across resets until full: what the main MCU does around power-off */
    if (tr.magic != 0x54524332u) { tr.magic = 0x54524332u; tr.n = 0; tr.t_last = now_us(); }
    const uint8_t c = nrx > 0 ? spi_rx[0] : 0xff;
    const int unusual = (c != 0x70 && c != 0x71 && c != 0x83 && c != 0x6e && c != 0xff) || ntx > 1;
    if (unusual && tr.n < TR_N) {
      uint32_t t = now_us(), dt = (t - tr.t_last) / 100; tr.t_last = t;
      uint8_t *e = tr.e[tr.n++];
      e[0] = (uint8_t)dt; e[1] = (uint8_t)(dt > 0xFFFF ? 0xFF : dt >> 8);
      e[2] = (uint8_t)nrx; e[3] = (uint8_t)ntx; e[4] = spi_rx[0]; e[5] = spi_rx[1];
      e[6] = spi_tx[0]; e[7] = spi_tx[1]; e[8] = spi_tx[2]; e[9] = (uint8_t)NRF_SPIS1->STATUS;
      NRF_SPIS1->STATUS = 3;
    }
#endif
    if (nrx > 0) { memcpy(dbg_rx, spi_rx, 12); dbg_nrx = (uint8_t)nrx; dbg_ntx = (uint8_t)ntx; lpc_frames++; handle_lpc_frame(spi_rx, nrx); } else lpc_bad++;
    addr_check();
  }
  load_reply_and_release();
  addr_check();
}

/* A reply queued while the bus is idle: take the semaphore and load it now, so the reply line (ATTN)
   rises at once, like stock (the tail of its queue function, 0x868c: ACQUIRE when SEMSTAT is free,
   load, RELEASE). Without it a reply waited for the main MCU's next transfer. That is soon while the
   main MCU streams, but a 3.0 tracker in pairing mode sends nothing and waits for the reply line: its
   0x52 sat unread, so its pairing never completed (LED blinking blue until its pairing timed out). */
static void spis_kick(void)
{
#ifdef DML_IDX                     /* only the nRF52 side has the reply line; the Tracker 1.0 is unchanged */
  /* SEMSTAT 0: free. Taken by a transfer, or by us (acquire pending), it isn't: no second kick. */
  if (tx_loaded || !rq_used() || NRF_SPIS1->SEMSTAT != 0) return;
  NRF_SPIS1->TASKS_ACQUIRE = 1;   /* spis_poll() takes the ACQUIRED (no END): loads the reply, releases */
#endif
}

/* ---------------- radio link v1 (tracker side, see common/doorman_link.h) ---------------- */
#define UQ_BYTES 1024                  /* uplink frames from the LPC, [len][frame] */
#define UQ_CAP 384                     /* at most this much queued: ~25 ms at 15 kB/s (uplink_push) */
#ifndef LIGHT_MAX_AGE_US
#define LIGHT_MAX_AGE_US 25000u        /* batches: light waits behind at most this much, or 3 cycles (batch_push) */
#endif
#ifndef UQ_ANSWERS
#define UQ_ANSWERS 4                   /* batches: queued up to this many answers' worth (batch_push) */
#endif
#ifdef DML_MULTI
/* Test device answering for several slots (DML_MULTI): everything a device keeps for its link, once per slot.
   The code below reads it through D, the slot being served; the radio and the hop schedule are shared (they
   are the dongle's, the same for every slot). */
#define DML_NINST 4
#define BQ_N 32
#define PQ_BYTES 256                           /* as below */
#define TQ 8
typedef struct {
  uint8_t m_uq[UQ_BYTES]; uint16_t m_uq_head, m_uq_tail; uint32_t m_uq_drops;
  dmc_state_t m_codec; dmp_state_t m_lh2p; uint8_t m_batching, m_bat_open, m_bat[DML_MAXLEN]; dmb_t m_bt;
  uint32_t m_bat_t0, m_bq_t[BQ_N]; uint8_t m_bq_head, m_bq_n; uint32_t m_age_drops;
  uint8_t m_pq[PQ_BYTES]; uint16_t m_pq_head, m_pq_tail; uint32_t m_pq_drops;
  uint8_t m_tq_tag[TQ], m_tq_head, m_tq_n; uint32_t m_tq_t[TQ];
  uint8_t m_pb[3][2 + DML_MAXLEN], m_ob[3], m_on, m_send_i, m_next_seq, m_armed, m_sent_max, m_jreq[2 + DML_JREQ_LEN], m_last_dseq;
  uint8_t m_link_up, m_joined, m_my_slot; uint32_t m_net_base, m_last_poll_us;
  uint8_t m_poll_rssi, m_poll_missed, m_poll_crc_err, m_poll_ok, m_poll_ack, m_max_resp, m_poll_sack;
  uint8_t m_pair_mode; uint32_t m_pair_since, m_drops_at;
  uint8_t m_load_kbps; uint32_t m_load_at, m_load_budget;
  dlg_t m_gen; uint8_t m_gen_bases, m_gen_density; uint32_t m_gen_t0;
  uint32_t lp_t; uint8_t lp_c, lp_have;     /* its last poll: when, which cycle (the hop rule) */
  uint16_t dg_uq_max, dg_open_max, dg_lim; uint32_t dg_builds, dg_early;   /* diagnostics (USB 0x08) */
} inst_t;
static inst_t inst[DML_NINST];
static inst_t *D = &inst[0];
static uint8_t ninst = 1;                  /* slots served (USB 0x07) */
static uint8_t multi_hop, isr_inst;        /* radio set to the joined slots' addresses; the one that matched */
static volatile uint8_t rx_live;           /* armed by arm_rx: a DISABLED after it is an exchange, not radio_stop's */
static uint32_t loop_max_us, mev_lost;
static uint32_t hop_n, hop_busy_tx, hop_busy_heard, hop_cyc[4];   /* test-device diagnostics (USB 0x01) */
typedef struct { uint8_t kind, a, b, c; uint32_t t; } dlog_t;      /* kind 1 poll [cycle, ch, k], 2 hop [j, from, to] */
static dlog_t dlog[16]; static uint8_t dlog_n;
static void dlog_add(uint8_t kind, uint8_t a, uint8_t b, uint8_t c) { dlog_t *e = &dlog[dlog_n++ & 15]; e->kind = kind; e->a = a; e->b = b; e->c = c; e->t = now_us(); }
static void multi_join_pick(void)          /* D = the next slot (round robin) that isn't joined, if any */
{
  static uint8_t rr;
  for (int i = 1; i <= ninst; i++) {
    const uint8_t k = (uint8_t)((rr + i) % ninst);
    if (!inst[k].m_joined) { D = &inst[k]; rr = k; return; }
  }
}
static void multi_after_join(void);
static void multi_restart(void);
#define uq (D->m_uq)
#define uq_head (D->m_uq_head)
#define uq_tail (D->m_uq_tail)
#define uq_drops (D->m_uq_drops)
#define codec (D->m_codec)
#define lh2p (D->m_lh2p)
#define batching (D->m_batching)
#define bat_open (D->m_bat_open)
#define bat_t0 (D->m_bat_t0)
#define bq_t (D->m_bq_t)
#define bq_head (D->m_bq_head)
#define bq_n (D->m_bq_n)
#define age_drops (D->m_age_drops)
#define bat (D->m_bat)
#define bt (D->m_bt)
#define pq (D->m_pq)
#define pq_head (D->m_pq_head)
#define pq_tail (D->m_pq_tail)
#define pq_drops (D->m_pq_drops)
#define tq_tag (D->m_tq_tag)
#define tq_head (D->m_tq_head)
#define tq_n (D->m_tq_n)
#define tq_t (D->m_tq_t)
#define pb (D->m_pb)
#define ob (D->m_ob)
#define on (D->m_on)
#define send_i (D->m_send_i)
#define next_seq (D->m_next_seq)
#define armed (D->m_armed)
#define sent_max (D->m_sent_max)
#define jreq (D->m_jreq)
#define last_dseq (D->m_last_dseq)
#define link_up (D->m_link_up)
#define joined (D->m_joined)
#define my_slot (D->m_my_slot)
#define net_base (D->m_net_base)
#define last_poll_us (D->m_last_poll_us)
#define poll_rssi (D->m_poll_rssi)
#define poll_missed (D->m_poll_missed)
#define poll_crc_err (D->m_poll_crc_err)
#define poll_ok (D->m_poll_ok)
#define poll_ack (D->m_poll_ack)
#define poll_sack (D->m_poll_sack)
#define max_resp (D->m_max_resp)
#define pair_mode (D->m_pair_mode)
#define pair_since (D->m_pair_since)
#define drops_at (D->m_drops_at)
#define load_kbps (D->m_load_kbps)
#define load_at (D->m_load_at)
#define load_budget (D->m_load_budget)
#define gen (D->m_gen)
#define gen_bases (D->m_gen_bases)
#define gen_density (D->m_gen_density)
#define gen_t0 (D->m_gen_t0)
#endif
#ifndef DML_MULTI
static uint8_t uq[UQ_BYTES];
static uint16_t uq_head, uq_tail;
static uint32_t uq_drops;
static dmc_state_t codec;              /* air codec state, reset on every join */
#endif
#ifdef DML_IDX
/* Batches (JOIN DML_JF_BATCH): the frames queued for one answer, coded together into the open batch as
   they come; the answer being built closes it. Sized to the answer the dongle last announced. */
#ifndef DML_MULTI
static dmp_state_t lh2p;
static uint8_t batching, bat_open;
static uint8_t bat[DML_MAXLEN];
static dmb_t bt;
#define BQ_N 32
static uint32_t bat_t0, bq_t[BQ_N];   /* when each queued batch's first frame came (bat_t0: the open one's) */
static uint8_t bq_head, bq_n;
static uint32_t age_drops;
#endif
#endif
static uint16_t uq_used(void) { return (uint16_t)((uq_tail - uq_head) & (UQ_BYTES - 1)); }
/* Replies to the host ('P' feature replies, 'H' identity) queue apart and go out first. In the shared
   queue they waited behind ~1 KB of IMU/light when the link was busy (six devices on one dongle), or
   were dropped when it was full: a config read over the radio is a stream of 0x11 replies, and one
   late or lost reply broke the whole read. They don't touch the air codec (only 'p'/'q' do), so
   sending them ahead keeps both ends in step. */
#define PQ_BYTES 256
#ifndef DML_MULTI
static uint8_t pq[PQ_BYTES];
static uint16_t pq_head, pq_tail;
static uint32_t pq_drops;
#endif
static uint16_t pq_used(void) { return (uint16_t)((pq_tail - pq_head) & (PQ_BYTES - 1)); }
/* Tagged feature reads waiting for the main MCU's answer (it answers in order). An entry older than
   500 ms is forgotten: a request it never answered must not shift every later tag. */
#define TQ 8
#ifndef DML_MULTI
static uint8_t tq_tag[TQ], tq_head, tq_n;
static uint32_t tq_t[TQ];
#endif

static uint8_t pkt_rx[2 + DML_MAXLEN];
#ifndef DML_MULTI
static uint8_t pb[3][2 + DML_MAXLEN];  /* RESP buffers: [len][payload] */
static uint8_t ob[3], on;              /* outstanding (sent, unacked) buffers, oldest first */
static uint8_t send_i;                 /* next ob[] entry to (re)send */
static uint8_t next_seq;
static uint8_t armed;                  /* buffer the next auto-TX sends */
static uint8_t sent_max;               /* seq of the last RESP that has actually gone out */
static uint8_t jreq[2 + DML_JREQ_LEN];
static uint8_t last_dseq = 1;
#endif

#ifndef DML_MULTI
static uint8_t link_up, link_enabled, joined, my_slot;
static uint32_t net_base;
#else
static uint8_t link_enabled;
#endif
static uint16_t mask = 0xFFFF;
static uint8_t cur_ch;
#ifndef DML_MULTI
static uint32_t last_poll_us;
static uint8_t poll_rssi, poll_missed, poll_crc_err, last_cycle, have_cycle;
static uint8_t got_addr, heard, poll_ok, poll_ack, answer_join;   /* got_addr: answer pointer set */
static uint8_t poll_sack;                                 /* the dongle holds our answer after the one it acks */
#else
static uint8_t last_cycle, have_cycle;
static uint8_t got_addr, heard, answer_join;
#endif
/* At ADDRESS, point the radio at the prepared answer before the hardware turnaround
   (END -> DISABLED -> TXEN, ~200 us later) starts the TX. No interrupts (HTC's bootloader owns the
   vector table), so addr_check() is also called from inside the main loop's longer steps. */
static uint32_t ans_ptr, isr_t;
static uint8_t isr_heard;
/* Link timing (air 'T'): how late ADDRESS was seen (the hardware captures its time through PPI
   channel 26 into TIMER0 CC[1]), answers lost to that, and the codec's cost. Running totals. */
static struct { uint32_t addr_n, late100, late200, no_answer, enc_n, enc_us; uint16_t lat_max, enc_max; } lt;   /* drops: batch / plain path, last one's queue, open batch, frame */
#ifdef DML_MULTI
static void addr_check_now(void);
static void addr_check(void)
{
  if (multi_hop) return;                                     /* RADIO_IRQHandler takes ADDRESS */
  addr_check_now();
}
static void addr_check_now(void)
#else
static void addr_check(void)
#endif
{
  if (!NRF_RADIO->EVENTS_ADDRESS) return;
  NRF_RADIO->EVENTS_ADDRESS = 0;
#ifdef DML_MULTI
  if (multi_hop) {                                           /* whose address it was: that slot answers */
    isr_inst = (uint8_t)NRF_RADIO->RXMATCH;
    if (ans_ptr && isr_inst < ninst) { inst_t *x = &inst[isr_inst]; ans_ptr = (uint32_t)x->m_pb[x->m_armed]; NRF_RADIO->TXADDRESS = isr_inst; }
  }
#endif
  if (ans_ptr) NRF_RADIO->PACKETPTR = ans_ptr;
  isr_t = NRF_TIMER0->CC[1];                                 /* the ADDRESS time itself (PPI) */
  isr_heard = 1;
  uint32_t lat = now_us() - isr_t;
  lt.addr_n++;
  if (lat > 100) lt.late100++;
  if (lat > 200) lt.late200++;
  if (lat > lt.lat_max) lt.lat_max = (uint16_t)(lat > 0xFFFF ? 0xFFFF : lat);
}
static uint32_t period_us = 2000, t_addr, t_last_poll;    /* cycle period (from POLLs), poll timing */
#ifndef DML_MULTI
static uint8_t max_resp = DML_MAXLEN;
#endif
static uint32_t rng = 0x12345678u;
enum { R_RX, R_TX } rstate;

static void uplink_push(const uint8_t *f, uint8_t n);
static void go_unjoined(void);
/* Pairing, like stock: the main MCU starts it ('T', from its pairing buttons or the host's pair
   command) and ends it ('R', or its own timeout). While pairing, the radio leaves its dongle, says
   so in its join requests, and only joins a dongle that has pairing open, so the dongle it was on
   can't take it back; a dongle only takes a device it doesn't know when it asks to pair. */
#ifndef DML_MULTI
static uint8_t pair_mode;
static uint32_t pair_since;
#endif
static void pair_start(void)
{
#ifdef DML_MULTI
  inst_t *const me = D;                                    /* go_unjoined moves D to the slot the radio asks for next */
#endif
  pair_mode = 1; pair_since = now_us();
  if (link_enabled && (joined || link_up)) go_unjoined();  /* reports 0x53: the LED shows pairing */
  else if (link_up) { link_up = 0; uint8_t ev[1] = { 0x53 }; reply_push(ev, 1); }
#ifdef DML_MULTI
  me->m_jreq[10] = DML_JF_PAIR | DML_JF_MINE;
#else
  jreq[10] = DML_JF_PAIR | DML_JF_MINE;
#endif
}
static void pair_end(void) { pair_mode = 0; jreq[10] = DML_JF_MINE; }
#ifdef DML_IDX
/* Holding the system button turns the device off. Valve's main firmware leaves that to the
   radio side: it only reports the button (bit 0x2000 of the 0x6e input frame's first word), and
   stock turns it off after ~4 s, with or without a host. We hand the main MCU the same power-off
   command a host sends ('P' 9f 04 "off!"), once per hold. */
static uint32_t sys_since;
static uint8_t sys_held, sys_off_sent;
/* Only System on its own counts, pairing or not: System + B is the pairing gesture (held from
   power-on), and a combination never turns the device off. Other buttons in the first word:
   trigger click (bit 0), A (2), B (12), stick click (18). */
#define SYS_OTHER_BUTTONS ((1u << 0) | (1u << 2) | (1u << 12) | (1u << 18))
static void sys_button(const uint8_t *f, uint8_t n)
{
  if (n < 5) return;
  uint32_t b; memcpy(&b, f + 1, 4);
  if (!(b & 0x2000u)) { sys_held = 0; sys_off_sent = 0; return; }
  if (b & SYS_OTHER_BUTTONS) { sys_held = 0; return; }   /* a combination: start over when it's alone again */
  if (!sys_held) { sys_held = 1; sys_since = now_us(); }
}
static void sys_button_poll(void)
{
  if (!sys_held || sys_off_sent || now_us() - sys_since < 4000000u) return;
  static const uint8_t off[7] = { 'P', 0x9f, 4, 'o', 'f', 'f', '!' };
  if (reply_push(off, sizeof off)) sys_off_sent = 1;
}
#endif
static void link_from_lpc(const uint8_t *f, uint8_t n)
{
#ifdef DML_IDX
  if (f[0] == 0x6e) sys_button(f, n);
  if (f[0] == 'S') {                                       /* main MCU sleeps: radio off until 'R'/'T' */
    /* Valve's radio answers every 'S' with 0x53 (link down), connected or not, and the main
       firmware waits for it before it powers the device off: without it, holding the system
       button never turned the controller off. */
    link_sleep = 1; link_up = 0;
    uint8_t ev[1] = { 0x53 }; reply_push(ev, 1);
    return;
  }
  if (f[0] == 'T') { link_sleep = 0; pair_start(); return; }
  if (f[0] == 'R') { link_sleep = 0; pair_end(); return; }
#else
  if (f[0] == 'T') { pair_start(); return; }               /* HTC link control: T pair, R connect */
  /* 'R' doesn't end pairing: a Vive wand's main firmware sends it all the time ('R' 0 unlinked, 'R' 0xFF
     linked), right after the 'T' of a host's "pair" too, which ended every pairing at once. Pairing ends
     when it joins a dongle that has pairing open, or after 35 s. */
  if (f[0] == 'R') return;
  if (f[0] == 'S') return;
#endif
  if (!link_up) return;                                    /* like HTC: data only while connected */
  uplink_push(f, n);
}
#ifdef DML_IDX
static void batch_close(void)
{
  if (!bat_open) return;
  bat_open = 0;
  const int m = dmb_end(&bt, codec.check);
  if (m <= 0) return;                                      /* (the CRC's byte is kept: can't happen) */
  if (bq_n < BQ_N) { bq_t[(bq_head + bq_n) % BQ_N] = bat_t0; bq_n++; }
  uq[uq_tail] = (uint8_t)m; uq_tail = (uq_tail + 1) & (UQ_BYTES - 1);
  for (int i = 0; i < m; i++) { uq[uq_tail] = bat[i]; uq_tail = (uq_tail + 1) & (UQ_BYTES - 1); }
}
static void batch_open(void)
{
  int cap = max_resp - DML_RESP_HDR - 1;                   /* one answer: its header and this one's length byte */
  if (cap < DMB_MAXF + 8) cap = DMB_MAXF + 8;              /* the longest raw frame always fits */
  dmb_begin_w(&bt, bat, cap, batching == 2);               /* 2: wide batches (JOIN DML_JF_BATCH32) */
  bat_open = 1;
}
static void batch_push(const uint8_t *f, uint8_t n)
{
  /* dropped before the coders see it, as without batches; the open batch counts as queued */
  const int open = bat_open ? 2 + (bt.w.pos + 7) / 8 + 1 : 0;
  /* Room for UQ_ANSWERS answers' worth: a burst of light (several bases' sweeps in one cycle), or a few answers
     lost in a row, drains over the next cycles instead of being dropped. One answer carries one batch, so two (what
     this was) dropped frames at 10 devices on 4 LH2 bases with no loss at all. Costs latency only while it drains. */
  int lim = UQ_ANSWERS * max_resp + 8;
  if (lim < UQ_CAP) lim = UQ_CAP;
  if (lim > UQ_BYTES - 80) lim = UQ_BYTES - 80;
#ifdef DML_MULTI
  if (uq_used() > D->dg_uq_max) D->dg_uq_max = uq_used();
  if (open > D->dg_open_max) D->dg_open_max = (uint16_t)open;
  D->dg_lim = (uint16_t)lim;
#endif
  /* Over the limit, light goes first: an IMU sample may use the whole queue. When a device can't send everything
     (a burst on a full link, answers lost in a row), it loses some hits, not the IMU that carries its tracking
     through. (Replies have their own queue.) */
  if (f[0] == 'p' && lim < UQ_BYTES - 80) lim = UQ_BYTES - 80;
  if (n > DMB_MAXF || uq_used() + open + n + 4 >= lim) { uq_drops++; return; }
  /* Light is dropped once what waits ahead of it is older than LIGHT_MAX_AGE: with answers being lost, a
     backlog otherwise made every frame behind it (IMU too) that much later at the PC, and a hit that old does
     little for tracking. A good link never gets there, so it loses nothing. The IMU still always goes. */
  const uint32_t tn = now_us();
  if (f[0] != 'p') {
    const uint32_t oldest = bq_n ? bq_t[bq_head] : bat_open ? bat_t0 : tn;
    /* three cycles: a clean link's head batch is up to a cycle or two old when it goes (it waits for its
       slot, then for an answer with room), so less dropped light with nothing lost */
    uint32_t max_age = 3 * period_us;
    if (max_age < LIGHT_MAX_AGE_US) max_age = LIGHT_MAX_AGE_US;
    if (tn - oldest > max_age || bq_n >= BQ_N) { age_drops++; uq_drops++; return; }
  }
  addr_check();
  uint32_t te = now_us();
  /* closed early when this frame probably won't fit: coded frames rarely take more than 3/4 of raw */
  if (bat_open && (!dmb_room(&bt) || bt.w.cap * 8 - bt.w.pos < (n * 3 / 4 + 4) * 8)) {
#ifdef DML_MULTI
    D->dg_early++;
#endif
    batch_close();
  }
  if (!bat_open) { batch_open(); bat_t0 = tn; }
  if (!dmb_add(&bt, &codec, &lh2p, f, n)) {                /* didn't fit after all: the next batch, raw */
    batch_close(); batch_open(); bat_t0 = tn;
    dmb_add_raw(&bt, f, n);
  }
  te = now_us() - te; lt.enc_n++; lt.enc_us += te; if (te > lt.enc_max) lt.enc_max = (uint16_t)(te > 0xFFFF ? 0xFFFF : te);
  addr_check();
}
#endif
static void uplink_push(const uint8_t *f, uint8_t n)
{
  static uint8_t tagged[80];                               /* answers are up to 64 B, 65 tagged */
  if (n && f[0] == 'P' && n < sizeof tagged) {             /* answer to a tagged read: give it its tag */
    while (tq_n && now_us() - tq_t[tq_head] > 500000u) { tq_head = (uint8_t)((tq_head + 1) % TQ); tq_n--; }
    if (tq_n) {
      tagged[0] = DML_TAGGED_REPLY; tagged[1] = tq_tag[tq_head];
      memcpy(tagged + 2, f + 1, n - 1u);
      tq_head = (uint8_t)((tq_head + 1) % TQ); tq_n--;
      f = tagged; n = (uint8_t)(n + 1);
    }
  }
  if (n && (f[0] == 'P' || f[0] == 'H' || f[0] == 'D' || f[0] == DML_TAGGED_REPLY)) {
    if (pq_used() + n + 1 >= PQ_BYTES) { pq_drops++; return; }
    pq[pq_tail] = n; pq_tail = (pq_tail + 1) & (PQ_BYTES - 1);
    for (int i = 0; i < n; i++) { pq[pq_tail] = f[i]; pq_tail = (pq_tail + 1) & (PQ_BYTES - 1); }
    return;
  }
  /* Held to UQ_CAP, not the whole queue: a full 1 KB took ~66 ms to drain at the ~15 kB/s one slot
     carries, so while the link was behind every IMU sample reached the PC that late. Dropped here, before
     the codec sees it, so both codec ends stay in step. */
#ifdef DML_IDX
  if (batching) { batch_push(f, n); return; }
#endif
  if (uq_used() + n + 1 >= UQ_CAP) { uq_drops++; return; }
  static uint8_t enc[80];
  addr_check();
  uint32_t te = now_us();
  int m = dmc_encode(&codec, f, n, enc);
  te = now_us() - te; lt.enc_n++; lt.enc_us += te; if (te > lt.enc_max) lt.enc_max = (uint16_t)(te > 0xFFFF ? 0xFFFF : te);
  addr_check();
  if (m) { f = enc; n = (uint8_t)m; }
  uq[uq_tail] = n; uq_tail = (uq_tail + 1) & (UQ_BYTES - 1);
  for (int i = 0; i < n; i++) { uq[uq_tail] = f[i]; uq_tail = (uq_tail + 1) & (UQ_BYTES - 1); }
}

/* Once a second while connected: what this radio dropped, ['D'][queue drops u16][reply drops u16]
   (running totals, saturating). The dongle keeps the latest per slot (its cmd 0x28): without it, frames
   lost here couldn't be told from frames lost in the dongle. */
#ifndef DML_MULTI
static uint32_t drops_at;
#endif
static void drops_report(void)
{
  if (!link_up || now_us() - drops_at < 1000000u) return;
  drops_at = now_us();
  uint16_t u = uq_drops > 0xFFFF ? 0xFFFF : (uint16_t)uq_drops, q = pq_drops > 0xFFFF ? 0xFFFF : (uint16_t)pq_drops;
  uint8_t f[5] = { 'D', (uint8_t)u, (uint8_t)(u >> 8), (uint8_t)q, (uint8_t)(q >> 8) };
  uplink_push(f, sizeof f);
}

/* Test load (air op 'L' [kB/s]): filler 'Z' frames sent up as if the main MCU sent more, so a few
   devices load the radio cycle like many. 0 = off (the default; not kept across a reboot). The budget
   is kept in thousandths of a byte, so a loop pass shorter than 1000/kB/s us still counts. */
#ifndef DML_MULTI
static uint8_t load_kbps;
static uint32_t load_at, load_budget;
#endif
#ifdef DML_IDX
/* Test load (air op 'G' [bases 0-4][density]): the frames an LH2 tracker's main MCU would send in a room with
   that many Base Station 2.0s (doorman_lh2gen.h; density 2 ~ the captures, higher loads the link past them),
   light only: the device's own IMU stays (a test device makes IMU too). For link tests without LH2 hardware:
   run them without the driver. 0 bases = off; not kept across a reboot. */
#ifndef DML_MULTI
static dlg_t gen;
static uint8_t gen_bases, gen_density;
static uint32_t gen_t0;
#endif
static void gen_emit(const uint8_t *f, int n, void *ctx) { (void)ctx; uplink_push(f, (uint8_t)n); }
static void gen_start(uint8_t bases, uint8_t density)
{
  gen_bases = bases > DLG_MAXB ? DLG_MAXB : bases; gen_density = density;
  if (gen_bases) {
    dlg_init(&gen, NRF_FICR->DEVICEID[0], gen_bases, gen_density, 8); gen_t0 = now_us();
#ifndef DML_FAKE
    gen.imu = 0;                                           /* the device's own IMU: a second one would be no tracker's */
#endif
  }
}
static void gen_tick(void)
{
  if (!link_up || !gen_bases) return;
  dlg_run(&gen, (now_us() - gen_t0) * 48u, gen_emit, 0);   /* the device's 48 MHz clock */
}
#endif
#ifdef DML_FAKE
extern const uint32_t vectors[];
#include "fake/usb.c"                                      /* USB commands, DFU, loss injection */
#endif
static void load_tick(void)
{
  const uint32_t now = now_us(), dt = now - load_at;
  load_at = now;
  if (!link_up || !load_kbps || dt > 100000u) return;
  load_budget += (uint32_t)load_kbps * dt;                      /* kB/s x us = 1/1000 B */
  static uint8_t z[40] = { 'Z' };
  while (load_budget >= 1000u * sizeof z) { load_budget -= 1000u * sizeof z; z[1]++; uplink_push(z, sizeof z); }
}

static void set_addr(uint32_t base, uint8_t prefix)
{
#ifdef DML_MULTI
  if (prefix != DML_JOIN_PREFIX) {                           /* every joined slot: logical address k = slot k here */
    uint32_t p0 = 0, p1 = 0, rx = 0;
    for (int k = 0; k < ninst; k++) if (inst[k].m_joined) {
      const uint32_t b = (uint32_t)(DML_SLOT_PREFIX + inst[k].m_my_slot);
      if (k < 4) p0 |= b << (8 * k); else p1 |= b << (8 * (k - 4));
      rx |= 1u << k;
    }
    NRF_RADIO->BASE0 = base; NRF_RADIO->BASE1 = base; NRF_RADIO->PREFIX0 = p0; NRF_RADIO->PREFIX1 = p1;
    NRF_RADIO->TXADDRESS = 0; NRF_RADIO->RXADDRESSES = rx;
    multi_hop = 1;
    NRF_RADIO->INTENSET = RADIO_INTENSET_ADDRESS_Msk | RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;
    NVIC_SetPriority(RADIO_IRQn, 0); NVIC_ClearPendingIRQ(RADIO_IRQn); NVIC_EnableIRQ(RADIO_IRQn);
    return;
  }
  multi_hop = 0;
  NVIC_DisableIRQ(RADIO_IRQn); NRF_RADIO->INTENCLR = 0xFFFFFFFFu; NVIC_ClearPendingIRQ(RADIO_IRQn);
#endif
  NRF_RADIO->BASE0 = base; NRF_RADIO->PREFIX0 = prefix;
  NRF_RADIO->TXADDRESS = 0; NRF_RADIO->RXADDRESSES = 1;
}
static void radio_setup(void)
{
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0; NRF_CLOCK->TASKS_HFCLKSTART = 1;
  uint32_t t = now_us();
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED && now_us() - t < 5000) {}
  NRF_RADIO->POWER = 1;
  NRF_PPI->CHENSET = 1u << 26;                               /* pre-set: RADIO ADDRESS -> TIMER0 CAPTURE[1] */
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit;
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;
  NRF_RADIO->PCNF0 = 8 << RADIO_PCNF0_LFLEN_Pos;
  NRF_RADIO->PCNF1 = (DML_MAXLEN << RADIO_PCNF1_MAXLEN_Pos) | (4 << RADIO_PCNF1_BALEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos);
  NRF_RADIO->CRCCNF = 2; NRF_RADIO->CRCINIT = 0xFFFF; NRF_RADIO->CRCPOLY = 0x11021;
  NRF_RADIO->INTENCLR = 0xFFFFFFFF;
#ifdef DML_MULTI
  for (int k = 0; k < DML_NINST; k++) {                      /* slot 0 is the board's own id (its pairing stays) */
    memcpy(inst[k].m_jreq + 2, (const void *)NRF_FICR->DEVICEID, 8);
    inst[k].m_jreq[2 + 7] ^= (uint8_t)(k * 0x5D);
  }
#else
  memcpy(jreq + 2, (const void *)NRF_FICR->DEVICEID, 8);
#endif
  rng ^= NRF_FICR->DEVICEID[0];
}
static void radio_stop(void)
{
#ifdef DML_MULTI
  __disable_irq();
  rx_live = 0;
#endif
  ans_ptr = 0;
  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->EVENTS_DISABLED = 0; NRF_RADIO->TASKS_DISABLE = 1;
  uint32_t t = now_us(); while (!NRF_RADIO->EVENTS_DISABLED && now_us() - t < 200) {}
  NRF_RADIO->EVENTS_DISABLED = 0;
#ifdef DML_MULTI
  NRF_RADIO->EVENTS_ADDRESS = 0;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  __enable_irq();
#endif
}
/* listen; if auto_tx, the radio answers the next packet on its own with pb[armed] / jreq */
static void arm_rx(int auto_tx)
{
#ifdef DML_FAKE
  if (auto_tx && joined && (fk_drop = fk_decide()) != 0) auto_tx = 0;   /* this exchange is lost (injected) */
#endif
  NRF_RADIO->FREQUENCY = cur_ch;
  NRF_RADIO->PACKETPTR = (uint32_t)pkt_rx;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |
                      RADIO_SHORTS_ADDRESS_RSSISTART_Msk | (auto_tx ? RADIO_SHORTS_DISABLED_TXEN_Msk : 0);
  NRF_RADIO->EVENTS_DISABLED = 0; NRF_RADIO->EVENTS_ADDRESS = 0; NRF_RADIO->EVENTS_READY = 0;
  got_addr = 0; heard = 0; isr_heard = 0;
  ans_ptr = auto_tx ? (joined ? (uint32_t)pb[armed] : (uint32_t)jreq) : 0;
  NRF_RADIO->TASKS_RXEN = 1;
  rstate = R_RX;
#ifdef DML_MULTI
  rx_live = 1;
#endif
}

static void go_unjoined(void)
{
  if (link_up) { uint8_t ev[1] = { 0x53 }; reply_push(ev, 1); }
  link_up = 0; joined = 0; on = 0; send_i = 0; uq_head = uq_tail; pq_head = pq_tail;
#ifdef DML_IDX
  bq_n = 0;
#endif
#ifdef DML_IDX
  bat_open = 0;                                            /* the queue is gone, and the open batch with it */
#endif
#ifdef DML_MULTI
  jreq[0] = DML_JREQ_LEN; jreq[1] = DML_JREQ; jreq[10] = (uint8_t)((pair_mode ? DML_JF_PAIR : 0) | DML_JF_MINE);
  multi_join_pick();                                       /* the radio asks for the unjoined slots in turn */
#endif
  radio_stop();
  set_addr(DML_JOIN_BASE, DML_JOIN_PREFIX);
  cur_ch = DML_JOIN_CHANNEL;
  jreq[0] = DML_JREQ_LEN; jreq[1] = DML_JREQ; jreq[10] = (uint8_t)((pair_mode ? DML_JF_PAIR : 0) | DML_JF_MINE);
  answer_join = 0;
  arm_rx(0);
}
static void go_joined(uint32_t base, uint8_t slot, uint8_t check, uint8_t batch)
{
  radio_stop();
  joined = 1; my_slot = slot; net_base = base;
  dmc_reset(&codec); codec.check = check;
#ifdef DML_IDX
  dmp_reset(&lh2p); batching = batch; bat_open = 0;
#else
  (void)batch;
#endif
  uq_head = uq_tail; pq_head = pq_tail; tq_n = 0;
#ifdef DML_IDX
  bq_n = 0;
#endif
  on = 0; send_i = 0; next_seq = 0; last_dseq = 1; have_cycle = 0;
  sent_max = 7;                    /* nothing sent yet: the dongle acks 7 until it has our seq 0 */
  mask = 0xFFFF; cur_ch = dml_hop_ch[0]; period_us = 2000; max_resp = DML_MAXLEN;
  last_poll_us = now_us();
  set_addr(base, (uint8_t)(DML_SLOT_PREFIX + slot));
#ifdef DML_IDX
  if (!link_up) { link_up = 1; uint8_t ev[1] = { 0x52 }; reply_push(ev, 1); }
#else
  if (!link_up) { link_up = 1; uint8_t ev[6] = { 0x52, slot, 0, 0, 0, 0 }; reply_push(ev, 6); }
#endif
}

static uint8_t build_new(void)                             /* fill a free buffer from the uplink queue */
{
#ifdef DML_IDX
  /* What came since the last answer goes in this one, unless a closed batch is still waiting: then that one goes
     and the open batch keeps filling, up to a whole answer. Closing it every time sent one batch an answer (a
     batch can't be split), so after a burst the queue stayed a batch behind for good and the next burst
     overflowed it, with half-empty answers going out. */
  if (batching && !uq_used()) batch_close();
#endif
#ifdef DML_MULTI
  D->dg_builds++;
#endif
  uint8_t b = 0;
  while ((on > 0 && ob[0] == b) || (on > 1 && ob[1] == b)) b++;
  uint8_t *p = pb[b] + 1;
  int len = DML_RESP_HDR;
  while (pq_used()) {                                      /* replies first */
    uint8_t n = pq[pq_head];
    if (len + 1 + n > max_resp) break;
    p[len++] = n;
    for (int i = 0; i < n; i++) p[len++] = pq[(pq_head + 1 + i) & (PQ_BYTES - 1)];
    pq_head = (pq_head + 1 + n) & (PQ_BYTES - 1);
  }
  while (uq_used()) {
    uint8_t n = uq[uq_head];
    /* A batch closed for a bigger answer than the dongle now asks for (another device joined): sent alone,
       over the limit. The radio takes up to DML_MAXLEN; the limit only times the slots, so this delays the
       rest of one cycle a little. A rejoin instead dropped the device for a second. */
    const int fits = len + 1 + n <= max_resp || (len == DML_RESP_HDR && len + 1 + n <= DML_MAXLEN);
    if (!fits) break;
    p[len++] = n;
    for (int i = 0; i < n; i++) p[len++] = uq[(uq_head + 1 + i) & (UQ_BYTES - 1)];
    uq_head = (uq_head + 1 + n) & (UQ_BYTES - 1);
#ifdef DML_IDX
    if (batching && bq_n) { bq_head = (uint8_t)((bq_head + 1) % BQ_N); bq_n--; }
#endif
  }
  pb[b][0] = (uint8_t)len;
  p[0] = DML_RESP; p[1] = my_slot; p[2] = (uint8_t)((next_seq & DML_F_SEQ_MASK) | (uq_used() ? DML_F_MORE : 0));
  next_seq = (next_seq + 1) & DML_F_SEQ_MASK;
  ob[on++] = b;
  return b;
}
static void arm_prepare(void)                              /* pick what the next auto-TX sends */
{
  uint8_t b;
  if (send_i < on) b = ob[send_i];
  else if (on < 2) { b = build_new(); send_i = (uint8_t)(on - 1); }
  else { send_i = 0; b = ob[0]; }
  uint8_t *p = pb[b] + 1;
  p[2] = (uint8_t)((p[2] & (DML_F_SEQ_MASK | DML_F_MORE)) | (last_dseq ? DML_F_DSEQ : 0));
  p[3] = poll_rssi; p[4] = poll_missed; p[5] = poll_crc_err;
  armed = b;
}
static void arm_next(void) { arm_prepare(); arm_rx(1); }   /* ... then listen */
static void exchange_done(void)                            /* our RESP pb[armed] just went out */
{
  if (poll_ok) {                                           /* drop everything the poll acknowledged */
    while (on && ((poll_ack - (pb[ob[0]][3] & DML_F_SEQ_MASK)) & 7) < 4) {
      ob[0] = ob[1]; ob[1] = ob[2]; on--;
    }
    /* the dongle kept the one after the lost one (DML_F_SACK): only the lost one goes again */
    if (poll_sack && on >= 2 && (pb[ob[1]][3] & DML_F_SEQ_MASK) == ((poll_ack + 2) & DML_F_SEQ_MASK)) {
      ob[1] = ob[2]; on--;
    }
  }
  int pos = -1;
  for (int i = 0; i < on; i++) if (ob[i] == armed) pos = i;
  if (poll_ok && on && ob[0] != armed) send_i = 0;         /* an older one is still unacked: lost, go back */
  else send_i = (uint8_t)(pos + 1);
  sent_max = pb[armed][3] & DML_F_SEQ_MASK;                /* what this exchange actually sent */
}

/* Recovery updater over the radio link (works without the main MCU): a downlink frame starting
 * with DML_AIR_UPD is handled here and never reaches the main MCU. Same staging bank, signature and
 * install marker as the SPI updater, so any radio image (ours or Valve's) can be put back.
 *   F0 'X'          -> F0 'X' ver[4] open off[4]
 *   F0 'Y'          -> F0 'Y' 0          open staging (link stays up)
 *   F0 'Z' off[4] d -> F0 'Z' st off[4]  write d at off (must equal the next offset; resends are acked)
 *   F0 '[' sig[16]  -> F0 '[' st         check + commit (installs at the next reset)
 *   F0 'R'          -> F0 'R' 0          reset ~300 ms later */
static uint32_t air_reset_at;
static void air_reply(const uint8_t *b, uint8_t n) { uplink_push(b, n); }
static void air_update(const uint8_t *f, uint8_t n)
{
  uint8_t r[12] = { DML_AIR_UPD, n > 1 ? f[1] : 0 };
  if (n < 2) return;
  switch (f[1]) {
  case 'X': {
    uint8_t x[60] = { DML_AIR_UPD, 'X' }; uint32_t v = DML_VERSION;
    memcpy(x + 2, &v, 4); x[6] = stg_open; memcpy(x + 7, &stg_off, 4);
    memcpy(x + 11, &lpc_frames, 4); memcpy(x + 15, &lpc_bad, 4);
    x[19] = (uint8_t)(safe_mode | link_safe_request << 1 | link_sleep << 2 | lead0 << 3);
    x[20] = dbg_nrx; x[21] = dbg_ntx; memcpy(x + 22, dbg_rx, 12);
    x[34] = (uint8_t)rq_used(); x[35] = (uint8_t)boot_rec.quick_boots;
#ifdef DML_DIAG
    memcpy(x + 36, &dg.boots, 8); x[44] = dg.cst; x[45] = dg.rr; x[46] = dg.scrub; memcpy(x + 47, &dg.zbad, 12);
    air_reply(x, 59); return;
#else
    for (int i = 0; i < OPS_N; i++) memcpy(x + 36 + 4 * i, ops_log[(ops_head + i) % OPS_N], 4);   /* oldest first */
    air_reply(x, 36 + 4 * OPS_N); return;
#endif
  }
  case 'L': {                                              /* test load: [kB/s], 0 off */
    load_kbps = n > 2 ? f[2] : 0; load_budget = 0;
    uint8_t x[3] = { DML_AIR_UPD, 'L', load_kbps };
    air_reply(x, 3); return;
  }
#ifdef DML_IDX
  case 'G': {                                              /* synthetic LH2 load: [bases 0-4][density] */
    gen_start(n > 2 ? f[2] : 0, n > 3 ? f[3] : 2);
    uint8_t x[4] = { DML_AIR_UPD, 'G', gen_bases, gen_density };
    air_reply(x, 4); return;
  }
#endif
  case 'T': {                                              /* link timing (lt) */
    uint8_t x[40] = { DML_AIR_UPD, 'T' };
    memcpy(x + 2, &lt, sizeof lt);
    air_reply(x, (uint8_t)(2 + sizeof lt)); return;
  }
#ifdef DML_DIAG
  case 'D': {                                              /* trace dump, 5 entries per request */
    uint8_t x[60] = { DML_AIR_UPD, 'D' }; uint8_t i0 = n > 2 ? f[2] : 0;
    x[2] = i0; x[3] = (uint8_t)(tr.magic == 0x54524332u ? tr.n : 0);
    for (int k = 0; k < 5 && i0 + k < TR_N; k++) memcpy(x + 4 + 10 * k, tr.e[i0 + k], 10);
    air_reply(x, 54); return;
  }
#endif
#ifdef DML_FAKE
  case 'K': {                                              /* injected loss: [%][mean burst][share losing the answer, %] */
    fk_set_loss(n > 2 ? f[2] : 0, n > 3 ? f[3] : 1, n > 4 ? f[4] : 100);
    uint8_t x[5] = { DML_AIR_UPD, 'K', fk_loss, fk_burst, fk_answer_pct };
    air_reply(x, 5); return;
  }
  case 'U': fk_after = 1; r[2] = 0; air_reply(r, 3); return;   /* UF2 bootloader */
  case 'Y': r[2] = 1; air_reply(r, 3); return;             /* a test device: its flash holds its own image there */
#else
  case 'Y': stg_open = 1; stg_air = 1; stg_off = 0; r[2] = 0; air_reply(r, 3); return;
#endif
  case 'Z': {
    uint32_t off; if (n < 6) return;
    memcpy(&off, f + 2, 4);
    if (!stg_open || !stg_air) r[2] = 2;
    else if (off + (n - 6) == stg_off) r[2] = 0;                     /* resend of the last block */
    else if (off != stg_off) r[2] = 3;
    else r[2] = stage_write(f + 6, n - 6) ? 0 : 1;
    memcpy(r + 3, &stg_off, 4); air_reply(r, 7); return;
  }
  case '[': r[2] = (n == 18 && stg_air && stage_commit(f + 2, MARK_INSTALL)) ? 0 : 1; stg_air = 0; air_reply(r, 3); return;
  case 'R': r[2] = 0; air_reply(r, 3); air_reset_at = now_us() + 300000u; if (!air_reset_at) air_reset_at = 1; return;
  }
}

__attribute__((noipa)) static void handle_poll(const uint8_t *p, int len)   /* noinline: radio_emu calls it */
{
  uint8_t cyc = p[3];
  if (have_cycle) poll_missed += (uint8_t)(cyc - last_cycle - 1);
  last_cycle = cyc; have_cycle = 1;
  poll_rssi = (uint8_t)NRF_RADIO->RSSISAMPLE;
  poll_ack = p[2] & DML_F_SEQ_MASK;
  poll_sack = (p[2] & DML_F_SACK) ? 1 : 0;
  /* The ack names the last RESP the dongle accepted, so it can only name a seq that has already
     gone out. An ack ahead of that had its bits corrupted while the CRC still passed; believing
     it drops buffers the dongle is still waiting for, and the exchange that would have delivered
     them is never sent again. Issue #130: the uplink then stalls until the device rejoins. */
  if ((uint8_t)((sent_max - poll_ack) & 7) > 3) poll_ok = 0;
  mask = (uint16_t)(p[4] | p[5] << 8);
  if (p[6]) period_us = p[6] * 50u;
  max_resp = p[7] >= DML_RESP_HDR + 2 && p[7] <= DML_MAXLEN ? p[7] : DML_MAXLEN;
  last_poll_us = now_us();
  t_last_poll = t_addr;
  if ((p[2] & DML_F_DOWN) && ((p[2] & DML_F_DSEQ) ? 1 : 0) != last_dseq) {
    last_dseq = (p[2] & DML_F_DSEQ) ? 1 : 0;              /* new downlink frames: hand them to the LPC */
    for (int i = DML_POLL_HDR; i + 1 <= len; ) {
      uint8_t n = p[i];
      if (n == 0 || i + 1 + n > len) break;
      if (p[i + 1] == DML_AIR_UPD) air_update(p + i + 1, n);
      else if (p[i + 1] == DML_TAGGED_GET && n == 3) {    /* ask the main MCU as ever, remember the tag */
        if (tq_n == TQ) { tq_head = (uint8_t)((tq_head + 1) % TQ); tq_n--; }
        uint8_t k = (uint8_t)((tq_head + tq_n) % TQ);
        tq_tag[k] = p[i + 2]; tq_t[k] = now_us(); tq_n++;
        uint8_t get[2] = { 'P', p[i + 3] };
        reply_push(get, 2);
      }
      else reply_push(p + i + 1, n);
      i += 1 + n;
    }
  }
#ifndef DML_MULTI
  cur_ch = dml_channel((uint8_t)(cyc + 1), mask);          /* where the next cycle will be */
#endif                                                     /* (several slots: others may still be polled this cycle) */
}

#ifdef DML_MULTI
static void multi_poll(void);
#endif
static void radio_poll(void)
{
  if (!link_enabled) return;
#ifdef DML_MULTI
  if (multi_hop) { multi_poll(); return; }
#endif
  addr_check();
  if (isr_heard && rstate == R_RX) {
    isr_heard = 0;
    t_addr = isr_t;
    heard = 1;
    got_addr = ans_ptr != 0;
  }
  if (!NRF_RADIO->EVENTS_DISABLED) {
    uint32_t t = now_us();
#ifdef DML_MULTI
    for (int k = 0; k < ninst; k++)
      if (inst[k].m_joined && t - inst[k].m_last_poll_us > 1000000) { D = &inst[k]; go_unjoined(); return; }
    if (multi_hop) for (int k = 0; k < ninst; k++) if (inst[k].m_joined) { D = &inst[k]; break; }   /* hop on any joined slot's view */
#endif
    if (joined && t - last_poll_us > 1000000) go_unjoined();          /* dongle gone */
    else if (joined && have_cycle && rstate == R_RX && !heard) {
      /* missed polls: keep hopping on our own clock. The poll for cycle last+j is due at
         t_last_poll + j*P; be on its channel from half a period before until half after. */
      uint32_t j = (t - t_last_poll + period_us / 2) / period_us;
      if (j >= 1) {
        uint8_t ch = dml_channel((uint8_t)(last_cycle + j), mask);
        if (ch != cur_ch) { radio_stop(); cur_ch = ch; arm_next(); }
      }
    }
    return;
  }
  NRF_RADIO->EVENTS_DISABLED = 0;
  if (rstate == R_RX) {
    const uint8_t *p = pkt_rx + 1;
    int len = pkt_rx[0];
    int crc = NRF_RADIO->CRCSTATUS;
#ifdef DML_MULTI
    if (multi_hop && isr_inst < ninst) D = &inst[isr_inst];
#endif
    if (joined) {
      /* TXEN already fired through the shortcut: stop it from firing again after the TX */
      NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
#ifdef DML_FAKE
      if (fk_drop) {                                       /* injected: the answer never went out, or the poll wasn't heard */
        const int answer_lost = fk_drop == 1; fk_drop = 0;
        if (answer_lost && heard) {                        /* taken as sent: go-back-N resends it, as for a real loss */
          poll_ok = crc && len >= DML_POLL_HDR && p[0] == DML_POLL && p[1] == my_slot;
          if (poll_ok) { handle_poll(p, len); exchange_done(); }
        }
        arm_next(); return;
      }
#endif
      if (!got_addr) { lt.no_answer++; radio_stop(); arm_next(); return; } /* never answer with the wrong buffer */
      poll_ok = crc && len >= DML_POLL_HDR && p[0] == DML_POLL && p[1] == my_slot;
      if (!crc) poll_crc_err++;
      if (poll_ok) handle_poll(p, len);
      rstate = R_TX;
      return;
    }
    /* unjoined: JOIN beacons */
    if (answer_join) {
      NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
      rstate = R_TX;                                       /* our JREQ is going out */
    }
    if (crc && len >= DML_JOIN_LEN && p[0] == DML_JOIN) {
      uint32_t base; memcpy(&base, p + 1, 4);
#ifdef DML_MULTI
      for (int k = 0; k < ninst; k++) if (!inst[k].m_joined && memcmp(p + 6, inst[k].m_jreq + 2, 8) == 0) { D = &inst[k]; break; }
#endif
      if (memcmp(p + 6, jreq + 2, 8) == 0 && p[14] < DML_SLOTS && (!pair_mode || (p[5] & DML_JF_PAIR))) {
        pair_end();                                         /* paired (or back on its dongle) */
        if (rstate == R_TX) { while (!NRF_RADIO->EVENTS_DISABLED) {} NRF_RADIO->EVENTS_DISABLED = 0; }
        go_joined(base, p[14], (p[5] & DML_JF_CHECK) ? 1 : 0,
                  (p[5] & DML_JF_BATCH) ? ((p[5] & DML_JF_BATCH32) ? 2 : 1) : 0);   /* an old dongle sets neither */
        mask = (uint16_t)(p[15] | p[16] << 8);                 /* follow the hops from the start */
        if (p[18]) period_us = p[18] * 50u;
        last_cycle = p[17]; have_cycle = 1;
        t_last_poll = t_addr - (uint32_t)(p[19] | p[20] << 8);
        cur_ch = dml_channel((uint8_t)(last_cycle + (now_us() - t_last_poll + period_us / 2) / period_us), mask);
        arm_next();
#ifdef DML_MULTI
        multi_after_join();
#endif
        return;
      }
      rng = rng * 1664525u + 1013904223u;
      answer_join = (rng >> 24) & 1;                       /* answer half the beacons: two new trackers
                                                              won't collide forever */
    }
#ifdef DML_MULTI
    if (rstate == R_RX) multi_join_pick();                 /* the next beacon asks for the next unjoined slot */
#endif
    if (rstate == R_RX) arm_rx(answer_join);
    return;
  }
  /* R_TX finished */
  if (joined) { exchange_done(); arm_next(); }
  else arm_rx(answer_join);
}

#ifdef DML_FAKE
/* A Pro Micro (nRF52840) as a test device for link load tests: this radio firmware with no main MCU behind
   it. It pairs like a device (pairing mode for 35 s on USB command 0x06, tools/testdev.py pair), sends the synthetic
   LH2 load from the start (4 bases, density 4: a Tundra Tracker's stream), and shows on its LED: blinking while
   connected, on while pairing. USB (fake/usb.c): info, DFU, settings, loss injection. */
#define FAKE_LED 15                                        /* P0.15, active high */
#ifdef DML_MULTI
/* A slot joined: if another still isn't, back to the join channel for it (the joined ones keep their state and
   wait; all of them join within the 1 s a device waits for polls); otherwise listen on every slot's address. */
/* Turnaround in the radio interrupt: one radio answers for several slots, and the dongle can poll two of them
   back to back, the second ~150 us after the first answer ends (a device of its own has a whole cycle). The
   interrupt only points ADDRESS at the polled slot's prepared answer, keeps the poll, and listens again at once;
   the link work for it (the poll's acks and downlink, exchange_done, the next answer) runs in the main loop as on
   a device, with a cycle to do it in. */
enum { MEV_ANSWERED = 1, MEV_UNANSWERED, MEV_SENT };
typedef struct { uint8_t kind, k, crc, drop; uint32_t t; uint8_t pkt[2 + DML_MAXLEN]; } mev_t;
#define MEV_N 8
static mev_t mev[MEV_N];
static volatile uint8_t mev_head, mev_tail;
static uint8_t tx_inst;
static mev_t *mev_push(uint8_t kind, uint8_t k)
{
  const uint8_t nt = (uint8_t)((mev_tail + 1) % MEV_N);
  if (nt == mev_head) { mev_lost++; return 0; }
  mev_t *e = &mev[mev_tail]; e->kind = kind; e->k = k;
  return e;
}
void RADIO_IRQHandler(void)
{
  /* Ramp-up time changed only once a ramp-up is over (changing it during one broke the transmit): while an answer
     goes out the next ramp-up (back to receive) is the fast one; while receiving, the normal one, so an answer
     turns around as a device's. */
  if (NRF_RADIO->EVENTS_READY) {
    NRF_RADIO->EVENTS_READY = 0;
    if (rstate == R_TX) NRF_RADIO->MODECNF0 = (NRF_RADIO->MODECNF0 & ~RADIO_MODECNF0_RU_Msk) | (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
    else NRF_RADIO->MODECNF0 &= ~RADIO_MODECNF0_RU_Msk;
  }
  if (NRF_RADIO->EVENTS_ADDRESS) addr_check_now();
  if (!NRF_RADIO->EVENTS_DISABLED) return;
  NRF_RADIO->EVENTS_DISABLED = 0;
  if (!rx_live) return;                                      /* radio_stop's */
  if (rstate == R_RX) {
    const uint8_t k = isr_inst;
    const int answered = ans_ptr != 0 && isr_heard;          /* the auto-TX is going out now */
    mev_t *e = isr_heard && k < ninst ? mev_push(answered ? MEV_ANSWERED : MEV_UNANSWERED, k) : 0;
    if (e) {
      e->crc = (uint8_t)NRF_RADIO->CRCSTATUS; e->t = isr_t; e->drop = fk_drop;
      const int n = pkt_rx[0] <= DML_MAXLEN ? pkt_rx[0] : DML_MAXLEN;
      memcpy(e->pkt, pkt_rx, (size_t)n + 1);
      mev_tail = (uint8_t)((mev_tail + 1) % MEV_N);
    }
    fk_drop = 0;
    if (answered) {
      /* The answer's turnaround already started at the device's normal ramp-up (a real device's timing). Back to
         receive the moment it ends, by the hardware and with the 40 us ramp-up: the next poll can be for another of
         our slots ~150 us later, and the normal 140 us plus our interrupt missed it. */
      NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
      tx_inst = k; rstate = R_TX;
      return;
    }
  } else {                                                   /* an answer went out; receive is ramping up already */
    mev_t *e = mev_push(MEV_SENT, tx_inst);
    if (e) mev_tail = (uint8_t)((mev_tail + 1) % MEV_N);
    int auto_tx = 1;
    if (joined && (fk_drop = fk_decide()) != 0) auto_tx = 0;
    NRF_RADIO->PACKETPTR = (uint32_t)pkt_rx;                 /* before START, 40 us from now */
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |
                        RADIO_SHORTS_ADDRESS_RSSISTART_Msk | (auto_tx ? RADIO_SHORTS_DISABLED_TXEN_Msk : 0);
    NRF_RADIO->EVENTS_ADDRESS = 0;
    got_addr = 0; heard = 0; isr_heard = 0;
    ans_ptr = auto_tx ? (joined ? (uint32_t)pb[armed] : (uint32_t)jreq) : 0;
    rstate = R_RX;
    return;
  }
  arm_rx(1);                                                 /* every slot's next poll */
}
static void multi_poll(void)
{
  while (mev_head != mev_tail) {
    mev_t *e = &mev[mev_head];
    D = &inst[e->k];
    if (e->kind == MEV_SENT) { exchange_done(); arm_prepare(); }
    else if (e->kind == MEV_ANSWERED || e->drop == 1) {      /* the poll was heard (an injected answer loss too) */
      const uint8_t *p = e->pkt + 1; const int len = e->pkt[0];
      t_addr = e->t;
      poll_ok = e->crc && len >= DML_POLL_HDR && p[0] == DML_POLL && p[1] == my_slot;
      if (!e->crc) poll_crc_err++;
      if (poll_ok) { handle_poll(p, len); dlog_add(1, p[3], cur_ch, e->k); D->lp_t = e->t; D->lp_c = p[3]; D->lp_have = 1; }
      if (e->kind == MEV_UNANSWERED) { if (poll_ok) exchange_done(); arm_prepare(); }   /* taken as sent and lost */
    }
    mev_head = (uint8_t)((mev_head + 1) % MEV_N);
  }
  const uint32_t t = now_us();
  for (int k = 0; k < ninst; k++)
    if (inst[k].m_joined && t - inst[k].m_last_poll_us > 1000000) { D = &inst[k]; go_unjoined(); return; }
  /* missed polls: keep hopping on the dongle's clock, as a device does */
  {
    const uint32_t j = (t - t_last_poll + period_us / 2) / period_us;
    if (have_cycle && j >= 1 && dml_channel((uint8_t)(last_cycle + j), mask) != cur_ch) {
      if (rstate != R_RX) hop_busy_tx++; else if (isr_heard) hop_busy_heard++;
    }
  }
  if (have_cycle && rstate == R_RX && !isr_heard) {
    /* the channel of the cycle due now (half a cycle after the last poll), against where the radio is tuned:
       a board's slots must lie within half a cycle of each other (next to each other in the dongle's order) */
    const uint32_t j = (t - t_last_poll + period_us / 2) / period_us;
    {
      const uint8_t ch = dml_channel((uint8_t)(last_cycle + j), mask);
      if (ch != NRF_RADIO->FREQUENCY) {
        __disable_irq();
        const int idle = rstate == R_RX && !isr_heard;
        __enable_irq();
        if (idle) { dlog_add(2, (uint8_t)j, cur_ch, ch); radio_stop(); cur_ch = ch; arm_rx(1); hop_n++; }
      }
    }
  }
}
static void multi_after_join(void)
{
  inst_t *const joined_now = D;
  for (int k = 0; k < ninst; k++)
    if (!inst[k].m_joined) {
      D = &inst[k];
      radio_stop(); set_addr(DML_JOIN_BASE, DML_JOIN_PREFIX); cur_ch = DML_JOIN_CHANNEL;
      answer_join = 0; arm_rx(0);
      return;
    }
  set_addr(joined_now->m_net_base, (uint8_t)(DML_SLOT_PREFIX + joined_now->m_my_slot));   /* all of them */
  for (int k = 0; k < ninst; k++) { D = &inst[k]; arm_prepare(); }
  D = joined_now; arm_rx(1);
}
/* Every slot leaves and joins again (the number of slots changed). */
static void multi_restart(void)
{
  for (int k = 0; k < DML_NINST; k++) {
    inst_t *x = &inst[k];
    x->m_joined = 0; x->m_link_up = 0; x->lp_have = 0; x->m_on = 0; x->m_send_i = 0; x->m_uq_head = x->m_uq_tail; x->m_pq_head = x->m_pq_tail; x->m_bat_open = 0; x->m_bq_n = 0;
    x->m_jreq[0] = DML_JREQ_LEN; x->m_jreq[1] = DML_JREQ; x->m_jreq[10] = (uint8_t)((x->m_pair_mode ? DML_JF_PAIR : 0) | DML_JF_MINE);
  }
  D = &inst[0];
  if (link_enabled) go_unjoined();
}
#endif
static void fake_init(void)
{
  SCB->VTOR = (uint32_t)vectors;                           /* our table, whatever the bootloader left */
  NRF_P0->PIN_CNF[FAKE_LED] = 1;
  fk_rng ^= NRF_FICR->DEVICEID[0];
  fk_usb_init();
#ifdef DML_MULTI
  for (int k = 0; k < DML_NINST; k++) {
    D = &inst[k];
    last_dseq = 1; max_resp = DML_MAXLEN;
    jreq[0] = DML_JREQ_LEN; jreq[1] = DML_JREQ; jreq[10] = DML_JF_MINE;
    gen_start(4, 4);
  }
  D = &inst[0];
#else
  gen_start(4, 4);                                         /* pairing only when asked (USB 0x06): a reflash keeps it on its dongle */
#endif
}
static void fake_poll(void)
{
  fk_usb_poll();
#ifdef DML_MULTI
  int up = 0, pairing = 0;
  for (int k = 0; k < ninst; k++) { up |= inst[k].m_link_up; pairing |= inst[k].m_pair_mode; }
#else
  const int up = link_up, pairing = pair_mode;
#endif
  const int led = pairing ? 1 : up ? (int)((now_us() >> 18) & 1) : 0;
  if (led) NRF_P0->OUTSET = 1u << FAKE_LED; else NRF_P0->OUTCLR = 1u << FAKE_LED;
}
#endif

/* ---------------- main ---------------- */
int main(void)
{
#ifdef DML_IDX
  NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Msk;        /* off after reset: every fetch waited on flash (~3x slower codec) */
#endif
  DG(dg_boot());
  boot_count();
  wdt_start();
  timer_init();
#ifdef DML_FAKE
  fake_init();
#else
  spis_init();
#endif
  uint32_t t0 = now_us();
  for (;;) {
#ifdef DML_MULTI
    { static uint32_t at; const uint32_t t = now_us(); if (at && t - at > loop_max_us) loop_max_us = t - at; at = t; }
#endif
    wdt_feed();
#ifdef DML_FAKE
    fake_poll();
#else
    spis_poll();                                           /* updater lives here: always first */
    spis_kick();                                           /* a reply queued on an idle bus: raise ATTN now */
#endif
    if (!link_enabled && !safe_mode && !link_safe_request && !link_sleep && !(stg_open && !stg_air) && now_us() - t0 > 1000000) {
      radio_setup(); link_enabled = 1; go_unjoined();
    }
    if (link_enabled && ((stg_open && !stg_air) || link_safe_request || link_sleep)) { /* updating / safe / asleep */
      radio_stop(); link_enabled = 0; joined = 0;
      if (link_up) { link_up = 0; uint8_t ev[1] = { 0x53 }; reply_push(ev, 1); }
    }
    radio_poll();
#if defined(DML_IDX) && !defined(DML_FAKE)
    sys_button_poll();
    gpio_watch_poll();
    adc_poll();
#endif
#ifdef DML_MULTI
    {
      inst_t *const keep = D;                              /* the slot an exchange may be in the middle of */
      for (int k = 0; k < ninst; k++) {
        D = &inst[k];
        if (pair_mode && now_us() - pair_since > 35000000u) pair_end();
        drops_report(); load_tick(); gen_tick();
      }
      D = keep;
    }
#else
    if (pair_mode && now_us() - pair_since > 35000000u) pair_end();   /* the main MCU gives up at 30 s */
    drops_report();
    load_tick();
#ifdef DML_IDX
    gen_tick();
#endif
#endif
    if (air_reset_at && (int32_t)(now_us() - air_reset_at) > 0) reset_now();
    if (boot_rec.quick_boots && now_us() - t0 > 5000000) boot_stable();
#if defined(DML_IDX) && !defined(DML_FAKE)
    if (now_us() - t0 > 10000000) staging_scrub();
#endif
  }
}
