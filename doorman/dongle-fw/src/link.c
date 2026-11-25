/* openhouse link v1, dongle side (see common/doorman_link.h): slots, joining/pairing, hopping, go-back-N.
 * Everything is polled from the main loop (no radio interrupts). */
#pragma GCC optimize("O2")   /* the air codec is decoded here for every device: speed over size (~4700 frames/s at 10) */
#include <string.h>
#include "nrf.h"
#include "doorman_link.h"
#include "doorman_codec.h"
#include "doorman_lh2p.h"
#include "sizing.h"
#include "link.h"
#include "dongle.h"

extern volatile uint32_t ms_ticks;
static uint32_t us_now(void) { NRF_TIMER1->TASKS_CAPTURE[0] = 1; return NRF_TIMER1->CC[0]; }

/* ---------------- paired trackers (persisted in the last app flash page) ---------------- */
#if BOARD_NORDIC_DFU
#define PAIR_PAGE   0xDA000u           /* storage page; Nordic's open bootloader owns 0xE0000+ */
#else
#define PAIR_PAGE   0xEC000u
#endif
#define PAIR_MAGIC  0x3150484Fu        /* "OHP1": DML_SLOTS slots */
#define PAIR_MAGIC8 0x5250484Fu        /* "OHPR": the 8-slot page of fw 0x0029 and earlier, moved over at boot */
/* Names (cmd 0x34/0x35), the user's own for each paired device, kept here so they go wherever the
   dongle goes. A page from older firmware ends after id[] (erased: names_magic is 0xFFFFFFFF), and
   reads as no names. UTF-8, NUL-padded; at most DML_NAME_MAX bytes (what one command carries). */
#define NAME_MAGIC  0x4D4E484Fu        /* "OHNM" */
#define DML_NAME_MAX 59
typedef struct { uint32_t magic, used; uint8_t id[DML_SLOTS][8]; uint32_t names_magic; char name[DML_SLOTS][64]; } pair_store_t;
static pair_store_t pairs;

static void nvmc_wait(void) { while (!NRF_NVMC->READY) {} }
typedef struct { uint32_t magic, used; uint8_t id[8][8]; uint32_t names_magic; char name[8][64]; } pair_store8_t;
static void pairs_save(void);
static void pairs_load(void)
{
  const pair_store8_t *old = (const pair_store8_t *)PAIR_PAGE;
  if (old->magic == PAIR_MAGIC8) {                          /* 8 slots: same pairings and names, two slots more */
    memset(&pairs, 0, sizeof pairs);
    pairs.magic = PAIR_MAGIC; pairs.used = old->used & 0xFF;
    memcpy(pairs.id, old->id, sizeof old->id);
    pairs.names_magic = NAME_MAGIC;
    if (old->names_magic == NAME_MAGIC) memcpy(pairs.name, old->name, sizeof old->name);
    pairs_save();
    return;
  }
  memcpy(&pairs, (const void *)PAIR_PAGE, sizeof pairs);
  if (pairs.magic != PAIR_MAGIC) { memset(&pairs, 0, sizeof pairs); pairs.magic = PAIR_MAGIC; }
  if (pairs.names_magic != NAME_MAGIC) { memset(pairs.name, 0, sizeof pairs.name); pairs.names_magic = NAME_MAGIC; }
}
static void pairs_save(void)
{
  extern uint32_t __etext;                                  /* never erase our own code */
  if ((uint32_t)&__etext > PAIR_PAGE) return;
  NRF_NVMC->CONFIG = 2; nvmc_wait();
  NRF_NVMC->ERASEPAGE = PAIR_PAGE; nvmc_wait();
  NRF_NVMC->CONFIG = 1; nvmc_wait();
  const uint32_t *w = (const uint32_t *)&pairs;
  for (unsigned i = 0; i < sizeof pairs / 4; i++) { ((volatile uint32_t *)PAIR_PAGE)[i] = w[i]; nvmc_wait(); }
  NRF_NVMC->CONFIG = 0; nvmc_wait();
}

/* ---------------- per-slot state ---------------- */
#define UPQ 1024
#define UPQ_REPLIES 256                 /* kept for replies: data may only fill UPQ - this */
typedef struct {
  uint8_t active;                       /* answered recently */
  uint8_t last_seq;                     /* last in-order RESP seq (the ack we send) */
  uint8_t dseq, down_len, down_buf[200];
  uint32_t last_resp_ms;
  uint8_t upq[UPQ]; uint16_t up_head, up_tail;
  /* stats */
  uint32_t polls, resp_ok, crc_err, timeouts, dup, frames_up, bytes_up, drops, down_frames, down_resends;
  uint32_t rssi_d16, rssi_t16, t_missed, t_crc;
  uint8_t last_missed, last_crc, have_tel;
  uint8_t dup_run;                      /* consecutive out-of-order answers (a stalled uplink) */
  uint8_t check;                        /* its radio adds the codec CRC-8 (JREQ DML_JF_CHECK) */
  uint8_t batch;                        /* its radio sends batches (JREQ DML_JF_BATCH) */
  uint16_t batch_mismatch;              /* answers that didn't match the agreed batches: made to rejoin */
  /* a batch being decoded, a frame a step; its frames held until its CRC-8 passes, so a corrupted one
     never reaches the host */
  dmp_state_t lh2p;
  dmb_r_t br;
  uint8_t in_batch, bn, bql[DMB_MAX_W];
  /* Selective ack: an answer that came while the one before it was lost, kept until that one arrives (both then go
     on in order) and announced in the POLL (DML_F_SACK), so the radio resends only the lost one. Go-back alone cost
     two cycles and a wasted answer per loss. */
  uint8_t held, held_seq, held_n, held_buf[DML_MAXLEN];
  uint32_t held_used, held_set;
  uint8_t bq[DMB_MAX_W][DMB_MAXF];          /* a batch's frames, held until its CRC-8 passes */
  uint32_t resyncs;                     /* codec out of step: made to rejoin */
  uint32_t skip_until;                  /* ms_ticks until which this slot is not polled */
  dmc_state_t codec; uint32_t air_bytes, bad_frames;
  uint16_t dev_drops, dev_reply_drops;   /* the radio's own drop totals (its 'D' frames) */
  uint8_t rssi_last_d, rssi_last_t, rssi_min_d, rssi_max_d;
  uint8_t ident[16];                    /* last 'H' frame, ident[0] = 0: none yet */
  uint16_t frag_left;                   /* bytes of a big frame still to send as 0x81 fragments */
  uint16_t pk_rest;                     /* packed reports: bytes of an item that started in an earlier report */
  uint8_t pk_seq;
  uint16_t upq_hw;                      /* most bytes waiting for USB since the last 0x28 read */
  uint8_t pend[DML_MAXLEN], pend_n, pend_k;   /* the last answer's frames, decoded from pend_k on */
  /* Ghost-follow: the hop state the device took from its last two answered POLLs (when the POLL went out, cycle,
     announced mask, period as the device keeps it; [0] valid when g_have), and the polls it missed since */
  uint32_t g_t[2]; uint16_t g_mask[2], g_per[2], g_miss; uint8_t g_cyc[2], g_have;
  uint32_t load_bytes, load_budget;     /* test load (0x2B): filler queued, and owed */   /* raw -dBm; min/max reset on read (0x29) */
} slot_t;
static slot_t sl[DML_SLOTS];
static uint8_t link_on = 1;
static uint32_t net_base;
static uint8_t cycle;
static uint16_t mask = 0xFFFF, next_mask = 0xFFFF;
static uint8_t cur_ch;

/* per hop channel loss statistics (for adaptive hopping) */
static struct { uint16_t polls, lost; uint32_t off_until_ms; uint8_t loss_pct, strikes; } chs[DML_NHOP];
static uint32_t ch_eval_ms;

/* pairing */
static uint32_t pairing_until_ms;
static uint8_t pair_target = 0xFF;   /* slot the next pairing goes to (0xFF: first free) */
/* Pairing open for a device that isn't in pairing mode itself: one with no working buttons, or whose
   main firmware never tells the radio to pair (a Vive wand's "pair" over USB). Only into a chosen slot,
   only while pairing is open, and only a device this dongle doesn't know: ends with the first one. */
static uint8_t pair_any;
static uint8_t assign_id[8], assign_slot = 0xFF, assign_left;
/* The device paired last: its slot and until when its join beacons still say "pairing open". Pairing
   closes here the moment the device is stored, but the device stays in pairing mode until it joins, and
   a device in pairing mode only takes a beacon with pairing open: without this it ignored its own slot
   assignment until its pairing timed out (~35 s), shown here as paired with 100% loss. Ends when it
   answers a poll: a pairing started after that needs pairing open again. */
static uint8_t just_paired_slot = 0xFF;
static uint32_t just_paired_until_ms;
static uint32_t last_join_ms, joins_heard;

static uint8_t pkt_tx[2 + DML_MAXLEN], pkt_rx[2 + DML_MAXLEN];
static uint32_t period_us = 2000, slot_us = 1450;
#define PERIOD_MAX_US (255u * 50u)              /* what POLL p[6] / JOIN p[18] can say */
static sizing_t sizing;                    /* answer size by saturation (sizing.h) */
static int plan_n;                         /* devices in this cycle's plan */
static uint32_t sizing_ms;
static uint8_t max_resp = DML_MAXLEN;
static void (*idle_hook)(void);
/* Where USB is serviced inside a radio exchange (cmd 0x2C, for A/B tests): bit 0 while the POLL goes
   out, bit 1 while listening for the answer, bit 2 between decoded frames. Measured with six devices:
   0 -> ~500 reports/s per interface, 2 or 3 -> 1021/s (the USB limit). Bit 0 is off: while the packet
   goes out, the shortcuts turn the radio around to receive on their own, and USB work there can outlast
   the whole answer, which is then missed. Join requests got lost that way (Tracker 1.0s couldn't pair).
   Bit 2 delays the next POLL and loss went to ~18%, so it's off too. */
static uint8_t usb_in_radio = 2;
/* Exchange timing (POLL out, answer in): the 90th percentile of each 32 cycles sizes the slots; a longer
   exchange just starts the next POLL a little late (the slowest one alone kept every slot at the bound). */
#define EX_BUCKETS 50                         /* 50 us each, up to 2.5 ms */
static uint16_t ex_hist[EX_BUCKETS]; static uint32_t ex_hn;
#define SLOT_GUARD_US 120u
static uint8_t adaptive = 1, repoll = 0;   /* repoll: tried, ~8% of re-polls were answered (a miss repeats on the same channel) */
static uint32_t repolls, repolls_ok;
/* Ghost-follow (cmd 0x32, on by default). A device that missed the one POLL announcing a new mask (or period) keeps
   hopping on what it heard last (radio main.c: cycle = last + round(dt / P), dml_channel() with the old mask), and
   hears its dongle again only where the old and new channels happen to coincide: 0.1-1 s, or it rejoins. While a
   slot misses polls, its POLL goes where the device listens by that same rule, worked out from what the dongle sent
   in the device's last answered POLL (misses 1, 4, 7, ..), in the one before (2, 5, ..: a POLL received with a bad
   CRC is still answered but not used), and on the cycle's channel (3, 6, ..). Only within 1 s of the last answered
   POLL (the device rejoins after that). Where the device's rule gives the cycle's channel, nothing changes.
   Ghost polls stay out of the per-channel loss statistics. */
static uint8_t ghost = 1;
static uint32_t ghost_polls, ghost_ok;
static uint32_t ex_budget, ex_cycles, ex_sum, ex_n, ex_max;
void link_set_idle_hook(void (*f)(void)) { idle_hook = f; }
static int decode_some(void);
static uint8_t defer_decode = 1;           /* cmd 0x2E: 0 decodes each answer at once (A/B) */
static void wait_until(uint32_t t)
{
  while ((int32_t)(us_now() - t) < 0) {
    if (idle_hook) idle_hook();
    if ((int32_t)(us_now() - t) < -150) decode_some();     /* a frame takes up to ~100 us */
  }
}

static uint16_t upq_used(const slot_t *s) { return (uint16_t)((s->up_tail - s->up_head) & (UPQ - 1)); }
/* One frame into a slot's USB queue; 0 if dropped. The queue's last UPQ_REPLIES bytes are for replies
   (feature/config answers, identity): with the queue filling, they were dropped like IMU and light, and
   every config read over a busy link failed and was retried, which made it busier. A light or IMU frame
   can be lost; a reply can't. */
static int upq_push(slot_t *s, const uint8_t *f, int gl)
{
  const int reply = f[0] == 'P' || f[0] == 'H' || f[0] == DML_TAGGED_REPLY || f[0] == DML_AIR_UPD;   /* + radio update answers */
  if (upq_used(s) + gl + 1 >= (reply ? UPQ : UPQ - UPQ_REPLIES)) { s->drops++; return 0; }
  s->upq[s->up_tail] = (uint8_t)gl; s->up_tail = (s->up_tail + 1) & (UPQ - 1);
  for (int i = 0; i < gl; i++) { s->upq[s->up_tail] = f[i]; s->up_tail = (s->up_tail + 1) & (UPQ - 1); }
  if (upq_used(s) > s->upq_hw) s->upq_hw = upq_used(s);
  return 1;
}
/* Decodes one frame of a slot's kept answer (in order: the air codec's state follows every frame);
   0 when there's nothing left. */
static uint32_t dec_sum, dec_n, dec_max;
/* A decoded frame on its way to the host. */
static void deliver(slot_t *s, const uint8_t *f, int gl)
{
  if (f[0] == 'H' && gl <= 16) { memset(s->ident, 0, 16); memcpy(s->ident, f, gl); }
  if (f[0] == 'D' && gl >= 5) { s->dev_drops = (uint16_t)(f[1] | f[2] << 8); s->dev_reply_drops = (uint16_t)(f[3] | f[4] << 8); }
  else { s->frames_up++; s->bytes_up += (uint32_t)gl; upq_push(s, f, gl); }
}
/* Not what the radio coded: the coders are out of step for good (every frame after it decodes as
   garbage). Left alone past the radio's 1 s "dongle gone", it rejoins and both ends reset. */
static void codec_lost(slot_t *s)
{
  s->bad_frames++; s->resyncs++; s->pend_k = s->pend_n; s->in_batch = 0;
  s->skip_until = ms_ticks + 1200;
}
static int decode_step(slot_t *s)
{
  if (s->in_batch) {
    const uint32_t td = us_now();
    const int g = dmb_next(&s->br, &s->codec, &s->lh2p, s->bq[s->bn], s->check);
    if (g <= 0) { codec_lost(s); return 0; }
    s->bql[s->bn++] = (uint8_t)g;
    if (s->br.left == 0) {                                  /* whole, CRC passed: hand it on */
      for (int i = 0; i < s->bn; i++) deliver(s, s->bq[i], s->bql[i]);
      s->in_batch = 0;
    }
    const uint32_t dd = us_now() - td;
    dec_sum += dd; dec_n++; if (dd > dec_max) dec_max = dd;
    return 1;
  }
  if (s->pend_k >= s->pend_n) return 0;
  const uint32_t td = us_now();
  const uint8_t *r = s->pend; const int n = s->pend_n, k = s->pend_k;
  uint8_t fl = r[k];
  if (fl == 0 || k + 1 + fl > n) { s->bad_frames++; s->pend_k = s->pend_n; return 0; }
  const uint8_t *f = r + k + 1;
  s->pend_k = (uint8_t)(k + 1 + fl);
  /* Batches are agreed at join and both ends keep to it: with them on, only replies come outside a batch.
     Anything else means the radio and we disagree (it kept what it agreed with a dongle before ours): we
     skip it past its 1 s timeout and it rejoins, negotiating again. Without this it ran without batches,
     at 3x the airtime, until it happened to rejoin. */
  const int reply = f[0] == 'P' || f[0] == 'H' || f[0] == 'D' || f[0] == DML_TAGGED_REPLY;
  const int is_batch = f[0] == DMC_BATCH || f[0] == DMC_BATCH_W;
  if (!reply && is_batch != (s->batch != 0)) { s->batch_mismatch++; codec_lost(s); return 0; }
  if (is_batch && s->batch) {                     /* its frames come out a step at a time */
    if (!dmb_open(&s->br, f, fl)) { codec_lost(s); return 0; }
    s->in_batch = 1; s->bn = 0;
    return 1;
  }
  uint8_t g[DMB_MAXF]; int gl = fl;
  if (f[0] >= DMC_IMU && f[0] <= DMC_LH2) {
    gl = dmc_decode(&s->codec, f, fl, g); f = g;
    if (!gl) { codec_lost(s); return 0; }
  }
  else { dmc_observe(&s->codec, f, fl); if (s->batch) dmp_observe(&s->lh2p, f, fl); }
  deliver(s, f, gl);
  const uint32_t dd = us_now() - td;
  dec_sum += dd; dec_n++; if (dd > dec_max) dec_max = dd;
  return 1;
}
/* One frame of whichever slot has a kept answer; 0 if none. */
static int decode_some(void)
{
  for (int i = 0; i < DML_SLOTS; i++) if (decode_step(&sl[i])) return 1;
  return 0;
}

/* ---------------- front-end amplifier (Holyiot 21017: SKY66112, CTX P0.24 / CRX P0.22) ----------------
   Both pins are GPIOTE task outputs. Software raises CTX before TXEN and CRX before RXEN; the TX->RX
   turnaround (DISABLED_RXEN shortcut) is too fast for the CPU, so PPI swaps CTX->CRX on that DISABLED. */
#if BOARD_HOLYIOT
#define FEM_CTX_PIN 24
#define FEM_CRX_PIN 22
#define FEM_GT_CTX 6
#define FEM_GT_CRX 7
#define FEM_PPI 0
static void fem_init(void)
{
  NRF_GPIOTE->CONFIG[FEM_GT_CTX] = 3 | (FEM_CTX_PIN << 8);   /* task mode, starts low */
  NRF_GPIOTE->CONFIG[FEM_GT_CRX] = 3 | (FEM_CRX_PIN << 8);
  NRF_PPI->CH[FEM_PPI].EEP = (uint32_t)&NRF_RADIO->EVENTS_DISABLED;
  NRF_PPI->CH[FEM_PPI].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[FEM_GT_CTX];
  NRF_PPI->FORK[FEM_PPI].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[FEM_GT_CRX];
  NRF_PPI->CHENCLR = 1u << FEM_PPI;
}
static void fem_tx(void)  { NRF_GPIOTE->TASKS_CLR[FEM_GT_CRX] = 1; NRF_GPIOTE->TASKS_SET[FEM_GT_CTX] = 1; }
static void fem_rx(void)  { NRF_GPIOTE->TASKS_CLR[FEM_GT_CTX] = 1; NRF_GPIOTE->TASKS_SET[FEM_GT_CRX] = 1; }
static void fem_off(void) { NRF_PPI->CHENCLR = 1u << FEM_PPI;
                            NRF_GPIOTE->TASKS_CLR[FEM_GT_CTX] = 1; NRF_GPIOTE->TASKS_CLR[FEM_GT_CRX] = 1; }
static void fem_turnaround_arm(void)    { NRF_PPI->CHENSET = 1u << FEM_PPI; }
static void fem_turnaround_disarm(void) { NRF_PPI->CHENCLR = 1u << FEM_PPI; }
/* SKY66112 adds ~22 dB: -4 dBm at the chip is ~+18 dBm out, under the 20 dBm EIRP limit */
#define RADIO_TXPOWER RADIO_TXPOWER_TXPOWER_Neg4dBm
#else
static void fem_init(void) {}
static void fem_tx(void) {}
static void fem_rx(void) {}
static void fem_off(void) {}
static void fem_turnaround_arm(void) {}
static void fem_turnaround_disarm(void) {}
#define RADIO_TXPOWER RADIO_TXPOWER_TXPOWER_Pos8dBm
#endif

/* ---------------- radio primitives ---------------- */
static void radio_off(void)
{
  NRF_RADIO->SHORTS = 0;
  fem_turnaround_disarm();
  NRF_RADIO->EVENTS_DISABLED = 0; NRF_RADIO->TASKS_DISABLE = 1;
  uint32_t t = us_now(); while (!NRF_RADIO->EVENTS_DISABLED && us_now() - t < 200) {}
  NRF_RADIO->EVENTS_DISABLED = 0;
  fem_off();
}
/* send pkt_tx, then listen for one answer into pkt_rx. Returns answer length (payload) or -1/-2. */
static int txrx(uint8_t ch, uint32_t base, uint8_t prefix, uint32_t wait_us)
{
  NRF_RADIO->FREQUENCY = ch;
  NRF_RADIO->BASE0 = base; NRF_RADIO->PREFIX0 = prefix;
  NRF_RADIO->TXADDRESS = 0; NRF_RADIO->RXADDRESSES = 1;
  NRF_RADIO->PACKETPTR = (uint32_t)pkt_tx;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
  NRF_RADIO->EVENTS_DISABLED = 0; NRF_RADIO->EVENTS_ADDRESS = 0;
  fem_tx(); fem_turnaround_arm();                           /* PPI flips to RX on the turnaround */
  NRF_RADIO->TASKS_TXEN = 1;
  uint32_t t0 = us_now();
  while (!NRF_RADIO->EVENTS_ADDRESS) if (us_now() - t0 > 1500) { radio_off(); return -2; }
  NRF_RADIO->EVENTS_ADDRESS = 0;
  NRF_RADIO->PACKETPTR = (uint32_t)pkt_rx;                  /* latched at the RX START */
  /* From here the radio runs on its own (shortcuts): USB is serviced while it sends and listens. The
     exchange took up to ~2.5 ms per slot with USB untouched, and with six devices the interfaces got
     one report out per 2 ms instead of per 1 ms (measured: ~470/s with the link on, 1021/s off). */
  while (!NRF_RADIO->EVENTS_DISABLED) {
    if (us_now() - t0 > 2500) { radio_off(); return -2; }
    if (idle_hook && (usb_in_radio & 1)) idle_hook();
  }
  NRF_RADIO->EVENTS_DISABLED = 0;                           /* TX done, RXEN fired by the shortcut */
  fem_turnaround_disarm();                                  /* the RX end must not re-trigger it */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
  uint32_t t1 = us_now();
  for (;;) {
    if (NRF_RADIO->EVENTS_DISABLED) break;
    uint32_t dt = us_now() - t1;
    if (NRF_RADIO->EVENTS_ADDRESS ? dt > 1600 : dt > wait_us) { radio_off(); return -1; }
    if (usb_in_radio & 2) { if (idle_hook) idle_hook(); decode_some(); }   /* the radio receives on its own */
  }
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->SHORTS = 0;
  fem_off();
  if (!NRF_RADIO->CRCSTATUS) return -3;
  return pkt_rx[0];
}

/* ---------------- one poll ---------------- */
/* Consecutive out-of-order answers after which a slot is left alone so its radio rejoins. At the
   poll rate (a few hundred a second per slot) this is a few hundred ms of a link that is receiving
   answers but cannot advance, not a loss burst. */
#define DUP_RUN_LIMIT 64
static int slot_skipping(const slot_t *s) { return s->skip_until && (int32_t)(ms_ticks - s->skip_until) < 0; }
/* Its radio gives up after 1 s without a POLL and is usually back 1.0-1.2 s after the skip starts, sometimes
   later (1.3 s seen); slot_reset ends the skip then. Polled again before that, the slot goes inactive and the
   host drops the device. */
static void slot_skip(slot_t *s) { s->skip_until = ms_ticks + 1500; if (!s->skip_until) s->skip_until = 1; }

static int poll_slot(uint8_t i, uint8_t ch, uint32_t t_tx)   /* 1: it answered; ch: cur_ch, or a ghost-follow channel */
{
  slot_t *s = &sl[i];
  while (decode_step(s)) {}                                 /* its last answer first: the ack is for it */
  uint8_t *p = pkt_tx + 1;
  int len = DML_POLL_HDR;
  p[0] = DML_POLL; p[1] = i;
  p[2] = (uint8_t)((s->last_seq & DML_F_SEQ_MASK) | (s->dseq ? DML_F_DSEQ : 0) | (s->held ? DML_F_SACK : 0));
  p[3] = cycle; p[4] = (uint8_t)next_mask; p[5] = (uint8_t)(next_mask >> 8);
  p[6] = (uint8_t)(period_us >= PERIOD_MAX_US ? 255 : period_us / 50); p[7] = max_resp;
  if (s->down_len) { p[2] |= DML_F_DOWN; memcpy(p + len, s->down_buf, s->down_len); len += s->down_len; }
  pkt_tx[0] = (uint8_t)len;
  s->polls++;
  int hop = -1;
  if (ch == cur_ch) { for (int k = 0; k < DML_NHOP; k++) if (dml_hop_ch[k] == cur_ch) hop = k; }
  else ghost_polls++;                                       /* ghost polls stay out of the channel statistics */
  if (hop >= 0) chs[hop].polls++;
  int n = txrx(ch, net_base, (uint8_t)(DML_SLOT_PREFIX + i), 400);
  const uint8_t *r = pkt_rx + 1;
  if (n < DML_RESP_HDR || r[0] != DML_RESP || r[1] != i) {
    if (n == -3) s->crc_err++; else s->timeouts++;
    if (hop >= 0) chs[hop].lost++;
    if (s->active && ms_ticks - s->last_resp_ms > 1000) s->active = 0;
    if (s->g_miss < 0xFFFF) s->g_miss++;
    return 0;
  }
  s->resp_ok++;
  if (ch != cur_ch) ghost_ok++;
  s->g_t[1] = s->g_t[0]; s->g_cyc[1] = s->g_cyc[0]; s->g_mask[1] = s->g_mask[0];
  s->g_per[1] = s->g_have ? s->g_per[0] : 0;               /* 0: no earlier record */
  s->g_t[0] = t_tx; s->g_cyc[0] = cycle; s->g_mask[0] = next_mask;
  s->g_per[0] = (uint16_t)(period_us / 50 * 50); s->g_have = 1;   /* what the device keeps of p[6] */
  s->g_miss = 0;
  s->active = 1; s->last_resp_ms = ms_ticks;
  if (i == just_paired_slot) just_paired_slot = 0xFF;     /* it joined: its pairing is over, the next one needs pairing open */
  uint8_t rd = (uint8_t)NRF_RADIO->RSSISAMPLE;
  s->rssi_last_d = rd; s->rssi_last_t = r[3];
  if (!s->rssi_min_d || rd < s->rssi_min_d) s->rssi_min_d = rd;   /* strongest (smallest -dBm) */
  if (rd > s->rssi_max_d) s->rssi_max_d = rd;                      /* weakest */
  s->rssi_d16 = s->rssi_d16 ? s->rssi_d16 - s->rssi_d16 / 16 + rd : (uint32_t)rd * 16;
  s->rssi_t16 = s->rssi_t16 ? s->rssi_t16 - s->rssi_t16 / 16 + r[3] : (uint32_t)r[3] * 16;
  if (s->have_tel) { s->t_missed += (uint8_t)(r[4] - s->last_missed); s->t_crc += (uint8_t)(r[5] - s->last_crc); }
  s->last_missed = r[4]; s->last_crc = r[5]; s->have_tel = 1;
  uint8_t dack = (r[2] & DML_F_DSEQ) ? 1 : 0;
  if (s->down_len) { if (dack == s->dseq) { s->down_len = 0; s->dseq ^= 1; } else s->down_resends++; }
  uint8_t seq = r[2] & DML_F_SEQ_MASK;
  if (seq != ((s->last_seq + 1) & DML_F_SEQ_MASK)) {                          /* duplicate / out of order */
    if (!s->held && seq == ((s->last_seq + 2) & DML_F_SEQ_MASK) && n - DML_RESP_HDR <= DML_MAXLEN) {   /* the one after a lost one */
      s->held = 1; s->held_seq = seq; s->held_n = (uint8_t)(n - DML_RESP_HDR); s->held_set++;
      memcpy(s->held_buf, r + DML_RESP_HDR, s->held_n);
      s->dup_run = 0;
      return 1;
    }
    s->dup++;
    /* Go-back gives one or two of these in a row. A long run means the radio is repeating a seq we
       already accepted and cannot hear which one we want - a POLL whose ack bits were corrupted but
       whose CRC passed, and whose answer was lost (issue #130). Nothing we send can break that, so
       leave the slot alone for longer than the radio's 1 s "dongle gone" timeout: it rejoins, which
       resets the sequence on both sides. DUP_RUN_LIMIT is well above any loss burst: a lost answer
       is a timeout, not a dup. */
    if (++s->dup_run >= DUP_RUN_LIMIT) { s->dup_run = 0; slot_skip(s); }
    return 1;
  }
  s->dup_run = 0;
  s->last_seq = seq;
  s->air_bytes += (uint32_t)(n - DML_RESP_HDR);
  sizing_answer(&sizing, i, n, r[2], s->check, plan_n);   /* radios with the CRC-8 set the "more" flag too */
  /* Kept and decoded while the radio is busy with the next exchange (decode_some): decoding here took
     ~320 us of each ~870 us exchange (up to 720) with the radio idle, every slot waiting on it. */
  memcpy(s->pend, r + DML_RESP_HDR, (size_t)(n - DML_RESP_HDR));
  s->pend_n = (uint8_t)(n - DML_RESP_HDR); s->pend_k = 0;
  if (s->held) {
    if (s->held_seq == ((seq + 1) & DML_F_SEQ_MASK)) {      /* the lost one came: it, then the kept one, in order */
      while (decode_step(s)) {}
      s->last_seq = s->held_seq; s->air_bytes += s->held_n; s->held_used++;
      memcpy(s->pend, s->held_buf, s->held_n);
      s->pend_n = s->held_n; s->pend_k = 0;
    }
    s->held = 0;                                           /* (otherwise stale: the radio went back further) */
  }
  if (!defer_decode) while (decode_step(s)) {}
  return 1;
}

/* ---------------- test load (0x2B) ---------------- */
/* Filler 'Z' frames queued for USB as if devices sent them, at load_kbps kB/s on each slot in load_mask
   (empty slots too): the dongle's queues, USB and the host's reader carry them like data (the driver
   skips them), so a busy eight-device dongle can be tried with a few real devices. Off by default. */
static uint8_t load_mask, load_kbps;
static uint32_t load_at;
static void load_tick(void)
{
  const uint32_t now = ms_ticks, dt = now - load_at;
  load_at = now;
  if (!load_mask || !load_kbps || dt > 100) return;
  static uint8_t z[40] = { 'Z' };
  for (int i = 0; i < DML_SLOTS; i++) {
    if (!((load_mask >> i) & 1)) continue;
    slot_t *s = &sl[i];
    s->load_budget += (uint32_t)load_kbps * dt;                 /* kB/s x ms = bytes */
    while (s->load_budget >= sizeof z) {
      s->load_budget -= sizeof z;
      z[1]++;
      if (upq_push(s, z, sizeof z)) s->load_bytes += sizeof z;
    }
  }
}

/* ---------------- joining ---------------- */
static void slot_reset(uint8_t i)
{
  slot_t *s = &sl[i];
  s->last_seq = 7; s->dseq = 0; s->down_len = 0; s->have_tel = 0;
  s->dup_run = 0; s->skip_until = 0;
  dmc_reset(&s->codec);
  s->codec.check = s->check;
  dmp_reset(&s->lh2p);
  s->in_batch = 0;
  memset(s->ident, 0, sizeof s->ident);
  s->up_head = s->up_tail; s->frag_left = 0; s->pk_rest = 0;
  s->pend_n = s->pend_k = 0;                               /* the old device's answer isn't for the new codec */
  s->held = 0;
  s->g_have = 0; s->g_miss = 0; s->g_per[1] = 0;           /* joined afresh from a beacon: no old hop state */
  s->active = 1; s->last_resp_ms = ms_ticks;               /* poll it right away */
}
static uint32_t t_c;                                       /* current cycle start */
/* The slots polled this cycle: paired and connected. A paired device that's off costs no airtime;
   when it comes back it rejoins through the join beacons (its slot_reset puts it back in). */
static uint32_t plan;
static void join_beacon(void)
{
  uint8_t *p = pkt_tx + 1;
  p[0] = DML_JOIN; memcpy(p + 1, &net_base, 4);
  p[5] = ms_ticks < pairing_until_ms ||
         (assign_left && assign_slot == just_paired_slot && (int32_t)(just_paired_until_ms - ms_ticks) > 0) ? 1 : 0;
  if (assign_left) {
    memcpy(p + 6, assign_id, 8); p[14] = assign_slot; assign_left--;
    if (sl[assign_slot].check) p[5] |= DML_JF_CHECK;        /* only for a radio that asked: an old one reads p[5] as "pairing open" */
    if (sl[assign_slot].batch) p[5] |= DML_JF_BATCH;
    if (sl[assign_slot].batch == 2) p[5] |= DML_JF_BATCH32;
  }
  else { memset(p + 6, 0, 8); p[14] = 0xFF; }
  p[15] = (uint8_t)next_mask; p[16] = (uint8_t)(next_mask >> 8); p[17] = cycle; p[18] = (uint8_t)(period_us >= PERIOD_MAX_US ? 255 : period_us / 50);
  uint32_t since = 0;
  if (p[14] < DML_SLOTS) {                                  /* the assigned slot's place in this cycle */
    int k = 0;
    const uint32_t in = plan | 1u << p[14];               /* it's in the next cycle's plan */
    for (int i = 0; i < p[14]; i++) k += (in >> i) & 1;
    since = us_now() - (t_c + (uint32_t)k * slot_us);
    if (since > 0xFFFF) since = 0xFFFF;
  }
  p[19] = (uint8_t)since; p[20] = (uint8_t)(since >> 8);
  pkt_tx[0] = DML_JOIN_LEN;
  int n = txrx(DML_JOIN_CHANNEL, DML_JOIN_BASE, DML_JOIN_PREFIX, 400);
  const uint8_t *r = pkt_rx + 1;
  if (n < DML_JREQ_LEN || r[0] != DML_JREQ) return;
  joins_heard++;
  const uint8_t *id = r + 1;
  int pairing = r[9] & DML_JF_PAIR;                         /* the device is in pairing mode */
  int slot = -1;
  for (int i = 0; i < DML_SLOTS; i++) if ((pairs.used >> i) & 1 && memcmp(pairs.id[i], id, 8) == 0) slot = i;
  if (!pairing && pair_any && slot < 0 && pair_target < DML_SLOTS && ms_ticks < pairing_until_ms) {
    pairing = 1; pair_any = 0;
  }
  /* Like stock, a new pairing takes both sides: pairing open here, and the device in pairing mode
     (its pairing buttons, or a host's pair command). Known devices just reconnect. */
  if (pairing && ms_ticks < pairing_until_ms) {
    /* paired here: a new device, one already paired here (re-paired), or one moving to the chosen slot */
    int to = slot;
    if (pair_target < DML_SLOTS) to = pair_target;          /* the chosen slot: replaces what was there */
    else if (to < 0) for (int i = 0; i < DML_SLOTS && to < 0; i++) if (!((pairs.used >> i) & 1)) to = i;
    if (to < 0) return;                                     /* no free slot */
    if (to != slot) {
      if (slot >= 0) pairs.used &= ~(1u << slot);
      slot = to;
      memcpy(pairs.id[slot], id, 8); pairs.used |= 1u << slot;
      memset(pairs.name[slot], 0, sizeof pairs.name[slot]);   /* another device: not its name */
      pairs_save();
    }
    pairing_until_ms = 0;
    just_paired_slot = (uint8_t)slot; just_paired_until_ms = ms_ticks + 35000u;
  } else if (pairing && !(slot >= 0 && slot == just_paired_slot && (int32_t)(just_paired_until_ms - ms_ticks) > 0)) {
    /* in pairing mode but not pairing here: it only joins a beacon with pairing open, so assigning it
       would show it connected at 100% loss until its pairing times out. Left alone until then. */
    return;
  }
  if (slot < 0) return;
  memcpy(assign_id, id, 8); assign_slot = (uint8_t)slot; assign_left = 5;
  sl[slot].check = (r[9] & DML_JF_CHECK) ? 1 : 0;
  sl[slot].batch = (r[9] & DML_JF_BATCH) ? ((r[9] & DML_JF_BATCH32) ? 2 : 1) : 0;   /* 2: wide batches */
  slot_reset((uint8_t)slot);
}

/* ---------------- adaptive hopping ---------------- */
/* Every 2 s: drop the WORST channel over the threshold (repeatedly, down to CH_MIN_ON), bring back at
 * most one channel whose time off is over, and keep repeat offenders off longer (30 s doubling to 5 min;
 * a channel that stays clean for a while is forgiven). Dropping worst-first matters when most of the band
 * is busy (Wi-Fi): dropping in frequency order kept the worst ones once the minimum was reached. */
#define CH_MIN_ON   8
#define CH_BAD_PCT  8
#define CH_EVERYWHERE_PCT 20
/* A channel is bad when it loses clearly more than the channels around it. Loss a device has on
   every channel (weak signal, body in the way) is not the channel's fault: judged on absolute loss,
   one weak device struck clean channels until only a few were left to hop on. */
static uint8_t channels_typical_loss(uint16_t m)
{
  uint8_t v[DML_NHOP]; int n = 0;
  for (int k = 0; k < DML_NHOP; k++)
    if ((m >> k) & 1 && chs[k].polls >= 20) {
      uint8_t x = chs[k].loss_pct; int i = n++;
      while (i > 0 && v[i - 1] > x) { v[i] = v[i - 1]; i--; }
      v[i] = x;
    }
  return n ? v[n / 2] : 0;
}
static void channels_eval(void)
{
  if (ms_ticks - ch_eval_ms < 2000) return;
  ch_eval_ms = ms_ticks;
  uint16_t m = mask;
  int on = 0;
  for (int k = 0; k < DML_NHOP; k++) {
    chs[k].loss_pct = chs[k].polls ? (uint8_t)(100u * chs[k].lost / chs[k].polls) : 0;
    if ((m >> k) & 1) {
      on++;
      if (chs[k].polls >= 20 && chs[k].loss_pct <= 2 + channels_typical_loss(m) && chs[k].strikes && ms_ticks - chs[k].off_until_ms > 120000)
        chs[k].strikes--;                                    /* clean for 2 min since it came back */
    }
  }
  const unsigned typical = channels_typical_loss(m), bad = CH_BAD_PCT + typical;
  /* Loss this high on the typical channel is the link being full (or every channel bad alike), not some
     channels: striking them only narrowed the hopping down to the floor (8 of 16 with 8 busy devices). */
  const int everywhere = typical >= CH_EVERYWHERE_PCT;
  for (;;) {                                                 /* drop the worst bad channel, again and again */
    int worst = -1;
    for (int k = 0; k < DML_NHOP; k++)
      if ((m >> k) & 1 && chs[k].polls >= 20 && chs[k].loss_pct > bad &&
          (worst < 0 || chs[k].loss_pct > chs[worst].loss_pct)) worst = k;
    if (worst < 0 || everywhere || on <= CH_MIN_ON) break;
    m &= (uint16_t)~(1u << worst); on--;
    uint32_t off = 30000u << (chs[worst].strikes < 4 ? chs[worst].strikes : 4);
    if (off > 300000u) off = 300000u;
    chs[worst].off_until_ms = ms_ticks + off;
    if (chs[worst].strikes < 8) chs[worst].strikes++;
  }
  int back = -1;                                             /* one expired channel back per round */
  for (int k = 0; k < DML_NHOP; k++)
    if (!((m >> k) & 1) && (int32_t)(ms_ticks - chs[k].off_until_ms) >= 0 &&
        (back < 0 || (int32_t)(chs[k].off_until_ms - chs[back].off_until_ms) < 0)) back = k;
  if (back >= 0 || on == 0) { if (back < 0) back = 0; m |= (uint16_t)(1u << back); chs[back].off_until_ms = ms_ticks; }
  for (int k = 0; k < DML_NHOP; k++) { chs[k].polls = 0; chs[k].lost = 0; }
  next_mask = m;
}

/* ---------------- main-loop entry points ---------------- */
void link_init(void)
{
  NRF_TIMER1->MODE = 0; NRF_TIMER1->BITMODE = 3; NRF_TIMER1->PRESCALER = 4;   /* 1 MHz */
  NRF_TIMER1->TASKS_CLEAR = 1; NRF_TIMER1->TASKS_START = 1;
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0; NRF_CLOCK->TASKS_HFCLKSTART = 1;
  uint32_t t = us_now(); while (!NRF_CLOCK->EVENTS_HFCLKSTARTED && us_now() - t < 5000) {}
  NRF_RADIO->POWER = 1;
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit;
  NRF_RADIO->MODECNF0 = RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;   /* 40 us ramp-up */
  NRF_RADIO->TXPOWER = RADIO_TXPOWER;
  NRF_RADIO->PCNF0 = (8 << RADIO_PCNF0_LFLEN_Pos) | (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);
  NRF_RADIO->PCNF1 = (DML_MAXLEN << RADIO_PCNF1_MAXLEN_Pos) | (4 << RADIO_PCNF1_BALEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos);
  NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two; NRF_RADIO->CRCINIT = 0xFFFF; NRF_RADIO->CRCPOLY = 0x11021;
  NRF_RADIO->INTENCLR = 0xFFFFFFFF;
  uint32_t h = NRF_FICR->DEVICEID[0] * 2654435761u ^ NRF_FICR->DEVICEID[1];
  net_base = (h & 0x7F7F7F7Fu) | 0x01010101u;               /* per-dongle network address */
  fem_init();
  pairs_load();
  for (int i = 0; i < DML_SLOTS; i++) { sl[i].last_seq = 7; sl[i].dseq = 0; }
}

void link_poll(void)
{
  load_tick();
  if (!link_on) { radio_off(); return; }
  /* TDMA plan, rebuilt each cycle from the connected slots: the k-th polled at t_c + k*slot_us, then the
     join window. Radios don't need their place in it: they listen for their own address and take the
     period from each POLL, so the plan can change from one cycle to the next. */
  plan = 0;
  int n = 0;
  for (int i = 0; i < DML_SLOTS; i++) {
    if (sl[i].skip_until && !slot_skipping(&sl[i])) sl[i].skip_until = 0;   /* over (and no wrap later) */
    /* A skipping slot keeps its place, with no POLL in it: the cycle's length and every other
       slot's time stay as they were (and so does where they fall against another dongle's). */
    if (((pairs.used >> i) & 1) && sl[i].active) { plan |= 1u << i; n++; }
  }
  /* bigger responses amortise the ~450 us per-slot overhead: 6 x 120 B = 6.3 ms cycle, ~18 kB/s per device
     (72 B gave 12.9 kB/s, too little headroom for resends; queues overflowed). 7-8 devices: 176 B, 9.9 ms
     and ~17.7 kB/s each at 8, over the ~13 kB/s a device sends at its worst (LH1, two bases) */
  plan_n = n;
  if (ms_ticks - sizing_ms >= 2000) { sizing_ms = ms_ticks; sizing_eval(&sizing); }
  max_resp = (uint8_t)sizing_resp(&sizing, n);   /* sizing.h: the size for this many devices, or large answers while saturated */
  /* The cycle goes out as one byte of 50 us (POLL p[6], JOIN p[18]): 12.75 ms at most. Nine or ten devices with
     large answers made it longer, the byte wrapped, and every device hopped on a wrong clock and missed polls.
     So with many devices the answers stay short enough for it: 10 -> 188 B, 9 -> 222 B, 8 or fewer unchanged. */
  if (n) {
    const int cap = (int)(((PERIOD_MAX_US - 700u) / (uint32_t)n - 450u) / 4u);
    if (max_resp > cap) max_resp = (uint8_t)cap;
  }
  /* Each slot's share of the cycle: what the slowest exchange of the last 32 cycles took plus a guard,
     not the worst case (450 us + 4 us per byte of a full answer, kept as the bound). Answers are
     rarely full, so the cycle is shorter and every device is polled more often. Off: cmd 0x2D 0. */
  const uint32_t worst = 450u + 4u * max_resp;
  slot_us = adaptive && ex_budget ? (ex_budget + SLOT_GUARD_US < worst ? ex_budget + SLOT_GUARD_US : worst) : worst;
  period_us = n * slot_us + 700u;
  t_c += period_us;
  if ((int32_t)(us_now() - t_c) > (int32_t)period_us || (int32_t)(t_c - us_now()) > (int32_t)period_us)
    t_c = us_now();                                         /* fell behind / first run: restart the plan */
  wait_until(t_c);
  cycle++;
  mask = next_mask;
  cur_ch = dml_channel(cycle, mask);
  channels_eval();                                          /* sets next_mask (announced in this cycle) */
  int k = 0;
  uint32_t missed = 0;
  for (uint8_t i = 0; i < DML_SLOTS; i++) {
    if (!((plan >> i) & 1)) continue;
    wait_until(t_c + (uint32_t)k * slot_us);
    k++;
    if (slot_skipping(&sl[i])) continue;                    /* its place, left empty */
    const uint32_t t0 = us_now();
    uint8_t ch = cur_ch;
    const slot_t *g = &sl[i];
    if (ghost && g->g_have && g->g_miss % 3 != 0 && t0 - g->g_t[0] < 1000000u) {
      const int e = g->g_miss % 3 == 2 && g->g_per[1] ? 1 : 0;   /* misses 1, 4, ..: last record; 2, 5, ..: the one before */
      uint32_t j = (t0 - g->g_t[e] + g->g_per[e] / 2u) / g->g_per[e];   /* the device's own rule */
      if (j < 1) j = 1;                                    /* it went to the next cycle's channel on that POLL */
      ch = dml_channel((uint8_t)(g->g_cyc[e] + j), g->g_mask[e]);
    }
    if (!poll_slot(i, ch, t0)) missed |= 1u << i;
    const uint32_t dur = us_now() - t0;
    ex_hist[dur / 50 < EX_BUCKETS ? dur / 50 : EX_BUCKETS - 1]++; ex_hn++;
    ex_sum += dur; ex_n++;
    if (dur > ex_max) ex_max = dur;
  }
  if (++ex_cycles >= 32) {                                  /* 90th percentile of the last 32 cycles */
    uint32_t c = 0, b = 0;
    while (b < EX_BUCKETS - 1 && (c += ex_hist[b]) * 10 < ex_hn * 9) b++;
    ex_budget = (b + 1) * 50u;
    memset(ex_hist, 0, sizeof ex_hist); ex_hn = 0; ex_cycles = 0;
  }
  const int join = ms_ticks - last_join_ms >= 20;
  /* A slot that missed its POLL is polled again in this cycle's spare time (cycles without a join
     window, while a whole slot still fits): its data waited a full cycle otherwise, and a weak link
     losing a few in a row piled those up. Off by default: with six devices only 106 of 1357 re-polls
     were answered (whatever lost the POLL was still there on the same channel). On: cmd 0x2F 1. */
  if (repoll && !join)
    for (uint8_t i = 0; i < DML_SLOTS && missed; i++) {
      if (!((missed >> i) & 1)) continue;
      missed &= ~(1u << i);
      const uint32_t tr = us_now();
      if ((int32_t)(t_c + period_us - (tr + slot_us)) < 0) break;
      repolls++;
      if (poll_slot(i, cur_ch, tr)) repolls_ok++;
    }
  if (join) {
    wait_until(t_c + (uint32_t)k * slot_us);
    last_join_ms = ms_ticks; join_beacon();
  }
}

/* 0x80 [slot][n][whole [len][frame] items]; a frame too big for one report goes as consecutive
   0x81 [slot][n][raw bytes of [len][frame]] fragments */
/* Packed stream (cmd 0x37, the host asks for it): [0x82][slot][n][seq][first][n bytes], the slot's
   [len][frame] items back to back across reports, so every report goes out full: whole frames left LH2's
   ~35-byte frames one to a report, and two busy devices filled an interface's 1000 reports a second. seq
   counts the slot's reports (a gap: the host starts again), first is where the first item starting in this
   report begins (0xFF: none). */
static uint8_t usb_packed;
static int stream_pop_packed(uint8_t *rep, slot_t *s, uint8_t si)
{
  const int used = upq_used(s);
  if (!used) return 0;
  const int n = used < 59 ? used : 59;
  int first = s->pk_rest < n ? s->pk_rest : 0xFF;
  if (first != 0xFF) {                                     /* where the next report's first item begins */
    int pos = first;
    while (pos < n) pos += 1 + s->upq[(s->up_head + pos) & (UPQ - 1)];
    s->pk_rest = (uint16_t)(pos - n);
  } else s->pk_rest = (uint16_t)(s->pk_rest - n);
  for (int i = 0; i < n; i++) rep[5 + i] = s->upq[(s->up_head + i) & (UPQ - 1)];
  s->up_head = (s->up_head + n) & (UPQ - 1);
  rep[0] = 0x82; rep[1] = si; rep[2] = (uint8_t)n; rep[3] = s->pk_seq++; rep[4] = (uint8_t)first;
  return 1;
}
int link_stream_pop(uint8_t *rep, int itf)
{
  static uint8_t rrs[DONGLE_NIF];                /* round-robin position per interface */
  uint8_t rr = rrs[itf];
  for (int k = 0; k < DML_SLOTS; k++) {
    uint8_t si = (uint8_t)((rr + k) % DML_SLOTS);
    if (si % DONGLE_NIF != itf) continue;         /* each slot stays on one interface: fragments in order */
    slot_t *s = &sl[si];
    if (usb_packed) {
      if (!stream_pop_packed(rep, s, si)) continue;
      rrs[itf] = (uint8_t)(si + 1);
      return 1;
    }
    int n = 0;
    if (!s->frag_left) {
      while (upq_used(s)) {
        uint8_t fl = s->upq[s->up_head];
        if (n + 1 + fl > 61) break;
        for (int i = 0; i <= fl; i++) rep[3 + n + i] = s->upq[(s->up_head + i) & (UPQ - 1)];
        s->up_head = (s->up_head + 1 + fl) & (UPQ - 1);
        n += 1 + fl;
      }
      if (!n && upq_used(s)) s->frag_left = (uint16_t)(1 + s->upq[s->up_head]);
    }
    if (s->frag_left) {
      n = s->frag_left > 61 ? 61 : s->frag_left;
      for (int i = 0; i < n; i++) rep[3 + i] = s->upq[(s->up_head + i) & (UPQ - 1)];
      s->up_head = (s->up_head + n) & (UPQ - 1);
      s->frag_left = (uint16_t)(s->frag_left - n);
      rep[0] = 0x81; rep[1] = si; rep[2] = (uint8_t)n;
      rrs[itf] = (uint8_t)(si + 1);
      return 1;
    }
    if (!n) continue;
    rep[0] = 0x80; rep[1] = si; rep[2] = (uint8_t)n;
    rrs[itf] = (uint8_t)(si + 1);
    return 1;
  }
  return 0;
}

static void put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

int link_cmd(const uint8_t *in, uint8_t *d, uint8_t *st)    /* 0x20..0x2F, 0x32..0x35; d = 61 bytes */
{
  uint8_t i = in[2] < DML_SLOTS ? in[2] : 0;
  slot_t *s = &sl[i];
  switch (in[0]) {
  case 0x20: if (in[2] != 0xFF) link_on = in[2]; d[0] = link_on; return 1;   /* radio_off happens in link_poll */
  case 0x21: {                                              /* down: [slot][len][frame] */
    uint8_t n = in[3];
    if (n == 0 || n > 63 || s->down_len + 1u + n > sizeof s->down_buf) { *st = 2; return 1; }
    s->down_buf[s->down_len++] = n; memcpy(s->down_buf + s->down_len, in + 4, n); s->down_len += n;
    s->down_frames++;
    return 1;
  }
  case 0x23: {                                              /* per-slot stats */
    uint32_t v[15] = { s->polls, s->resp_ok, s->crc_err, s->timeouts, s->dup, s->frames_up, s->bytes_up,
                       s->drops, s->down_frames, s->down_resends, s->rssi_d16 / 16, s->rssi_t16 / 16,
                       s->t_missed, s->t_crc, ms_ticks - s->last_resp_ms };
    memcpy(d, v, sizeof v);
    return 1;
  }
  case 0x29: {                                              /* live: per slot 10 B, slots [first..first+5] */
    const int first = in[2] < DML_SLOTS ? in[2] : 0;
    for (int k = first; k < DML_SLOTS && k < first + 6; k++) {
      uint8_t *e = d + (k - first) * 10; slot_t *q = &sl[k];
      e[0] = (uint8_t)(((pairs.used >> k) & 1) | (q->active << 1));
      e[1] = q->rssi_last_d; e[2] = q->rssi_last_t; e[3] = q->rssi_min_d; e[4] = q->rssi_max_d;
      e[5] = cur_ch;
      put32(e + 6, ms_ticks - q->last_resp_ms);
      q->rssi_min_d = 0; q->rssi_max_d = 0;
    }
    return 1;
  }
  case 0x2A: memcpy(d, s->ident, 16); return 1;
  case 0x2C: if (in[2] != 0xFF) usb_in_radio = in[2]; d[0] = usb_in_radio; return 1;   /* USB inside radio exchanges */
  case 0x2F:                                                /* re-poll missed slots: [0/1, 0xFF keeps] */
    if (in[2] != 0xFF) repoll = in[2];
    d[0] = repoll; put32(d + 1, repolls); put32(d + 5, repolls_ok); return 1;
  case 0x32:                                                /* ghost-follow: [0/1, 0xFF keeps] */
    if (in[2] > 1 && in[2] != 0xFF) { *st = 2; return 1; }
    if (in[2] != 0xFF) ghost = in[2];
    d[0] = ghost; put32(d + 2, ghost_polls); put32(d + 6, ghost_ok); return 1;
  case 0x33:                                                /* make a slot's radio rejoin: [slot] */
    /* Not polled past the radio's 1 s "dongle gone" timeout, as after a long dup run. The
       host asks for it when a connected device has sent nothing for seconds. */
    if (in[2] >= DML_SLOTS || !((pairs.used >> in[2]) & 1)) { *st = 2; return 1; }
    slot_skip(s);
    d[0] = in[2]; return 1;
  case 0x2E: if (in[2] != 0xFF) defer_decode = in[2]; d[0] = defer_decode; return 1;   /* deferred decode */
  case 0x2D:                                                /* timing: [adaptive 0/1, 0xFF keeps] */
    if (in[2] != 0xFF) adaptive = in[2];
    put32(d, period_us); put32(d + 4, slot_us); put32(d + 8, ex_budget);
    put32(d + 12, ex_n ? ex_sum / ex_n : 0); put32(d + 16, ex_max); d[20] = adaptive; d[21] = max_resp;
    put32(d + 22, dec_n ? dec_sum / dec_n : 0); put32(d + 26, dec_max);
    d[30] = sizing.worst; d[31] = sizing.big; d[32] = (uint8_t)sizing_base(plan_n); d[33] = (uint8_t)plan_n;
    for (int k = 0; k < DML_SLOTS; k++) d[34 + k] = sizing.pct[k];   /* saturation: worst, large answers, normal size, devices, per slot */
    ex_sum = ex_n = ex_max = dec_sum = dec_n = dec_max = 0;
    return 1;
  case 0x37:                                                /* packed stream reports on/off (0x82) */
    if (usb_packed != (in[2] ? 1 : 0)) {                    /* both kinds of report start clean */
      usb_packed = in[2] ? 1 : 0;
      for (int k = 0; k < DML_SLOTS; k++) { sl[k].up_head = sl[k].up_tail; sl[k].frag_left = 0; sl[k].pk_rest = 0; }
    }
    d[0] = usb_packed; return 1;
  case 0x2B: load_mask = in[2]; load_kbps = in[3]; d[0] = load_mask; d[1] = load_kbps; return 1;   /* test load */            /* identity ('H' frame) of a slot */
  case 0x28:                                                /* codec, and the radio's own drops */
    put32(d, s->air_bytes); put32(d + 4, s->bytes_up); put32(d + 8, s->bad_frames);
    put32(d + 12, s->dev_drops); put32(d + 16, s->dev_reply_drops);
    d[20] = (uint8_t)s->upq_hw; d[21] = (uint8_t)(s->upq_hw >> 8); s->upq_hw = 0;   /* USB queue high-water, reset on read */
    put32(d + 22, s->load_bytes);                                                    /* test load queued (0x2B) */
    put32(d + 26, s->resyncs); d[30] = s->check; d[31] = s->batch;                   /* codec resyncs, CRC-8 on, batches */
    put32(d + 32, s->held_set); put32(d + 36, s->held_used);                         /* selective ack: answers kept, delivered */
    return 1;
  case 0x24:                                                /* pair SECS [SLOT+1] [ANY] (0/absent: first free slot) */
    pairing_until_ms = in[2] ? ms_ticks + 1000u * in[2] : 0;
    pair_target = in[3] >= 1 && in[3] <= DML_SLOTS ? (uint8_t)(in[3] - 1) : 0xFF;
    pair_any = in[4] == 1 && pair_target < DML_SLOTS;       /* [4] 1: any unknown device, not only pairing ones */
    d[0] = in[2]; d[1] = pair_target; d[2] = pair_any; return 1;
  case 0x25: {                                              /* slots [first..first+5]: used, active, id x8 */
    const int first = in[2] < DML_SLOTS ? in[2] : 0;
    for (int k = first; k < DML_SLOTS && k < first + 6; k++) {
      uint8_t *e = d + (k - first) * 10;
      e[0] = (pairs.used >> k) & 1; e[1] = sl[k].active;
      memcpy(e + 2, pairs.id[k], 8);
    }
    d[60] = ms_ticks < pairing_until_ms ? (uint8_t)(1 | (pair_target < DML_SLOTS ? (pair_target + 1) << 1 : 0)) : 0;
    return 1;
  }
  case 0x26:                                                /* forget slot (0xFF = all) */
    if (in[2] == 0xFF) { pairs.used = 0; memset(pairs.name, 0, sizeof pairs.name); }
    else if (in[2] < DML_SLOTS) { pairs.used &= ~(1u << in[2]); memset(pairs.name[in[2]], 0, sizeof pairs.name[in[2]]); }
    pairs_save();
    return 1;
  case 0x34: {                                              /* set a slot's name: [slot][len][UTF-8] (len 0 clears) */
    const uint8_t n = in[3];
    if (in[2] >= DML_SLOTS || !((pairs.used >> in[2]) & 1) || n > DML_NAME_MAX) { *st = 2; return 1; }
    char name[64] = {0};
    memcpy(name, in + 4, n);
    if (memcmp(name, pairs.name[in[2]], sizeof name)) { memcpy(pairs.name[in[2]], name, sizeof name); pairs_save(); }
    d[0] = in[2]; return 1;
  }
  case 0x35: {                                              /* a slot's name: [slot] -> [len][UTF-8] */
    if (in[2] >= DML_SLOTS) { *st = 2; return 1; }
    const char *name = pairs.name[in[2]];
    uint8_t n = 0;
    while (n < DML_NAME_MAX && name[n]) n++;
    d[0] = n; memcpy(d + 1, name, n); return 1;
  }
  case 0x27:                                                /* channels: mask, loss % per hop channel */
    d[0] = (uint8_t)mask; d[1] = (uint8_t)(mask >> 8);
    for (int k = 0; k < DML_NHOP; k++) { d[2 + k] = dml_hop_ch[k]; d[18 + k] = chs[k].loss_pct; }
    put32(d + 34, joins_heard); put32(d + 38, net_base);
    return 1;
  }
  return 0;
}

void link_rssi_scan(uint8_t first, uint8_t *out, int n)    /* -dBm per channel, 0xFF = no sample */
{
  uint8_t was_on = link_on;
  link_on = 0; radio_off();
  for (int i = 0; i < n; i++) {
    uint8_t ch = (uint8_t)(first + i);
    if (ch > 100) { out[i] = 0xFF; continue; }
    NRF_RADIO->FREQUENCY = ch;
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->EVENTS_READY = 0; fem_rx(); NRF_RADIO->TASKS_RXEN = 1;
    uint32_t t = us_now(); while (!NRF_RADIO->EVENTS_READY && us_now() - t < 2000) {}
    uint8_t best = 0xFF;
    for (int k = 0; k < 8; k++) {
      NRF_RADIO->EVENTS_RSSIEND = 0; NRF_RADIO->TASKS_RSSISTART = 1;
      uint32_t g = 0; while (!NRF_RADIO->EVENTS_RSSIEND && ++g < 2000) {}
      uint8_t v = (uint8_t)NRF_RADIO->RSSISAMPLE;
      if (v < best) best = v;
    }
    out[i] = best;
    radio_off();
  }
  link_on = was_on;
}
