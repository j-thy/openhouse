/* Pro Micro test device (DML_FAKE): USB. Included into main.c (its link state is static there).
 *
 * One vendor HID interface (VID 0x6767, PID 0x0728, product "openhouse test device"), 64-byte reports, no IDs:
 *   OUT [cmd][seq][args]      IN [cmd][seq][status][data]      status 0 = ok
 *   (or SET_REPORT with the command and a feature GET_REPORT for the reply: control endpoint only)
 *   0x01 info     -> version u32, device id 8, link up, joined, slot, pairing, gen bases, gen density,
 *                    loss %, burst, answer-lost share %, uptime ms u32, exchanges u32, lost u32, sd u8
 *   0x02 dfu      -> UF2 bootloader (the board's drive appears; doorman/tools/testdev.py flashes it)
 *   0x03 reset
 *   0x04 gen      [bases 0-4][density]
 *   0x05 loss     [loss % 0-100][mean burst, exchanges][share of losses that lose the answer, %]
 *   0x06 pair     [slot k of this board, 0xFF all] pairing mode for 35 s
 *   0x07 slots    [n 1-4] how many dongle slots this board answers for (all of them join again)
 *   0x08 slot     [k] -> joined, link up, dongle slot, pairing, device id 8 (this board's k-th)
 * Over the air the same settings are 'G' and 'K' (doorman/tools/load.py), and 'U' enters DFU. */
#include "tusb.h"

#define FAKE_VID 0x6767
#define FAKE_PID 0x0728
#define GPREGRET_UF2 0x57                                  /* Adafruit nRF52 bootloader: UF2 drive */

static const tusb_desc_device_t fk_dev = {
  .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE, .bcdUSB = 0x0200,
  .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, .idVendor = FAKE_VID, .idProduct = FAKE_PID,
  .bcdDevice = (uint16_t)DML_VERSION, .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3, .bNumConfigurations = 1,
};
const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&fk_dev; }
static const uint8_t fk_report[] = { TUD_HID_REPORT_DESC_GENERIC_INOUT(64) };
const uint8_t *tud_hid_descriptor_report_cb(uint8_t itf) { (void)itf; return fk_report; }
static const uint8_t fk_cfg[] = {
  TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN, 0, 100),
  TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(fk_report), 0x01, 0x81, 64, 1),
};
const uint8_t *tud_descriptor_configuration_cb(uint8_t i) { (void)i; return fk_cfg; }
const uint16_t *tud_descriptor_string_cb(uint8_t idx, uint16_t langid)
{
  (void)langid;
  static uint16_t b[33];
  char serial[17]; const char *s;
  if (idx == 0) { b[0] = (TUSB_DESC_STRING << 8) | 4; b[1] = 0x0409; return b; }
  if (idx == 1) s = "openhouse";
  else if (idx == 2) s = "openhouse test device";
  else if (idx == 3) {
    static const char hx[] = "0123456789ABCDEF";
    const uint32_t id[2] = { NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0] };
    for (int i = 0; i < 16; i++) serial[i] = hx[(id[i / 8] >> (28 - 4 * (i % 8))) & 15];
    serial[16] = 0; s = serial;
  } else return NULL;
  int n = 0;
  while (s[n] && n < 32) { b[1 + n] = (uint8_t)s[n]; n++; }
  b[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
  return b;
}

/* ---- loss injection: Gilbert-Elliott, per exchange ---- */
static uint8_t fk_loss, fk_burst = 1, fk_answer_pct = 100, fk_bad, fk_drop;   /* fk_drop: 1 answer lost, 2 poll lost */
static uint32_t fk_exchanges, fk_lost, fk_rng = 0x9E3779B9u;
static uint32_t fk_rand(void) { fk_rng ^= fk_rng << 13; fk_rng ^= fk_rng >> 17; fk_rng ^= fk_rng << 5; return fk_rng; }
static void fk_set_loss(uint8_t pct, uint8_t burst, uint8_t answer_pct)
{
  fk_loss = pct > 100 ? 100 : pct; fk_burst = burst ? burst : 1; fk_answer_pct = answer_pct > 100 ? 100 : answer_pct; fk_bad = 0;
}
/* Whether the coming exchange is lost: a bad state that loses everything, entered so the average loss is fk_loss
   and left after fk_burst exchanges on average. */
static uint8_t fk_decide(void)
{
  fk_exchanges++;
  if (!fk_loss) return 0;
  const uint32_t r = fk_rand() % 100000u;
  if (fk_bad) { if (r < 100000u / fk_burst) fk_bad = 0; }
  else {
    const uint32_t L = fk_loss >= 100 ? 99999u : fk_loss * 1000u;          /* per 100000 */
    const uint32_t enter = (uint32_t)((uint64_t)L * 100000u / ((uint64_t)fk_burst * (100000u - L)));
    if (r < enter) fk_bad = 1;
  }
  if (!fk_bad) return 0;
  fk_lost++;
  return (fk_rand() % 100u) < fk_answer_pct ? 1 : 2;
}

/* ---- commands ---- */
static uint8_t fk_reply[64], fk_have_reply, fk_after;      /* fk_after: 1 = DFU, 2 = reset, once the reply is out */
static void fk_dfu(void) { NRF_POWER->GPREGRET = GPREGRET_UF2; NVIC_SystemReset(); }
static void fk_command(const uint8_t *in, uint16_t n)
{
#ifdef DML_MULTI
  int keep_moved = 0;
#endif
  if (n < 2) return;
  memset(fk_reply, 0, sizeof fk_reply);
  fk_reply[0] = in[0]; fk_reply[1] = in[1];
  uint8_t *d = fk_reply + 3;
#ifdef DML_MULTI
  inst_t *const keep = D;                                  /* the slot an exchange may be in the middle of */
  D = &inst[0];
#endif
  switch (in[0]) {
  case 0x01: {
    const uint32_t v = DML_VERSION, up = now_us() / 1000u;
    memcpy(d, &v, 4); memcpy(d + 4, (const void *)NRF_FICR->DEVICEID, 8);
    d[12] = link_up; d[13] = joined; d[14] = my_slot; d[15] = pair_mode; d[16] = gen_bases; d[17] = gen_density;
    d[18] = fk_loss; d[19] = fk_burst; d[20] = fk_answer_pct;
    memcpy(d + 21, &up, 4); memcpy(d + 25, &fk_exchanges, 4); memcpy(d + 29, &fk_lost, 4);
    d[33] = (uint32_t)vectors >= 0x26000u;               /* built for the SoftDevice layout */
#ifdef DML_MULTI
    memcpy(d + 34, &mev_lost, 4); memcpy(d + 38, &loop_max_us, 4); loop_max_us = 0;   /* radio events dropped, longest main-loop pass */
    memcpy(d + 42, &hop_n, 4); memcpy(d + 46, &hop_busy_tx, 4); memcpy(d + 50, &hop_busy_heard, 4);
    d[54] = rstate; d[55] = isr_heard; d[56] = rx_live; d[57] = cur_ch;
#endif
    break;
  }
#ifdef DML_MULTI
  case 0x09: {                                             /* diagnostics: the last 16 polls / hops, oldest first; [half 0/1] */
    const int h = n > 2 && in[2] ? 8 : 0;
    for (int i = 0; i < 8; i++) { const dlog_t *e = &dlog[(dlog_n + h + i) & 15]; memcpy(d + 7 * i, e, 4); memcpy(d + 7 * i + 4, &e->t, 3); }
    d[56] = (uint8_t)(period_us / 50);
    break;
  }
#endif
  case 0x02: fk_after = 1; break;
  case 0x03: fk_after = 2; break;
#ifdef DML_MULTI
  case 0x04:
    for (int k = 0; k < DML_NINST; k++) { D = &inst[k]; gen_start(n > 2 ? in[2] : 0, n > 3 ? in[3] : 4); }
    D = &inst[0]; d[0] = gen_bases; d[1] = gen_density; break;
  case 0x06:
    for (int k = 0; k < ninst; k++) if (n < 3 || in[2] == 0xFF || in[2] == k) { D = &inst[k]; pair_start(); }
    break;
  case 0x07:
    if (n > 2 && in[2] >= 1 && in[2] <= DML_NINST && in[2] != ninst) { ninst = in[2]; multi_restart(); keep_moved = 1; }
    d[0] = ninst; break;
  case 0x08: {
    const uint8_t k = n > 2 && in[2] < DML_NINST ? in[2] : 0;
    const inst_t *x = &inst[k];
    d[0] = x->m_joined; d[1] = x->m_link_up; d[2] = x->m_my_slot; d[3] = x->m_pair_mode; memcpy(d + 4, x->m_jreq + 2, 8); d[12] = k < ninst;
    memcpy(d + 13, &x->m_uq_drops, 4); memcpy(d + 17, &x->dg_uq_max, 2); memcpy(d + 19, &x->dg_open_max, 2); memcpy(d + 21, &x->dg_lim, 2);
    memcpy(d + 23, &x->dg_builds, 4); memcpy(d + 27, &x->dg_early, 4); d[31] = x->m_max_resp; d[32] = x->m_on;
    inst[k].dg_uq_max = 0; inst[k].dg_open_max = 0;
    break;
  }
#else
  case 0x04: gen_start(n > 2 ? in[2] : 0, n > 3 ? in[3] : 4); d[0] = gen_bases; d[1] = gen_density; break;
#endif
  case 0x05: fk_set_loss(n > 2 ? in[2] : 0, n > 3 ? in[3] : 1, n > 4 ? in[4] : 100); d[0] = fk_loss; d[1] = fk_burst; d[2] = fk_answer_pct; break;
#ifndef DML_MULTI
  case 0x06: pair_start(); break;
#endif
  default: fk_reply[2] = 1;
  }
#ifdef DML_MULTI
  if (!keep_moved) D = keep;                               /* a restart picked its own */
#endif
  fk_have_reply = 1;
}
void tud_hid_set_report_cb(uint8_t itf, uint8_t id, hid_report_type_t t, const uint8_t *buf, uint16_t n)
{
  (void)itf; (void)id; (void)t;
  fk_command(buf, n);
}
/* Over the control endpoint alone (some USB ports carry no interrupt transfers): SET_REPORT brings the command
   (above), a feature GET_REPORT returns its reply. */
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t id, hid_report_type_t t, uint8_t *buf, uint16_t n)
{
  (void)itf; (void)id;
  if (t != HID_REPORT_TYPE_FEATURE || !n) return 0;
  const uint16_t k = n < sizeof fk_reply ? n : (uint16_t)sizeof fk_reply;
  memcpy(buf, fk_reply, k);
  return k;
}

/* USB power events (no SoftDevice is enabled). TinyUSB stops HFCLK when VBUS goes: the radio needs it. */
extern void tusb_hal_nrf_power_event(uint32_t event);
void POWER_CLOCK_IRQHandler(void)
{
  if (NRF_POWER->EVENTS_USBDETECTED) { NRF_POWER->EVENTS_USBDETECTED = 0; tusb_hal_nrf_power_event(0); }
  if (NRF_POWER->EVENTS_USBREMOVED)  { NRF_POWER->EVENTS_USBREMOVED = 0;  tusb_hal_nrf_power_event(1); NRF_CLOCK->TASKS_HFCLKSTART = 1; }
  if (NRF_POWER->EVENTS_USBPWRRDY)   { NRF_POWER->EVENTS_USBPWRRDY = 0;   tusb_hal_nrf_power_event(2); }
}
void USBD_IRQHandler(void) { tud_int_handler(0); }
uint32_t board_millis(void) { return now_us() / 1000u; }

static void fk_usb_init(void)
{
  NVIC_SetPriority(USBD_IRQn, 6);                          /* short handlers; the radio needs no interrupts */
  NVIC_SetPriority(POWER_CLOCK_IRQn, 6);
  tud_init(0);
  NRF_POWER->INTENSET = POWER_INTENSET_USBDETECTED_Msk | POWER_INTENSET_USBREMOVED_Msk | POWER_INTENSET_USBPWRRDY_Msk;
  NVIC_EnableIRQ(POWER_CLOCK_IRQn);
  const uint32_t reg = NRF_POWER->USBREGSTATUS;            /* already powered: no event for that */
  if (reg & POWER_USBREGSTATUS_VBUSDETECT_Msk) tusb_hal_nrf_power_event(0);
  if (reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk) tusb_hal_nrf_power_event(2);
}
static void fk_usb_poll(void)
{
  tud_task();
  if (fk_have_reply && tud_hid_ready()) { tud_hid_report(0, fk_reply, 64); fk_have_reply = 0; }
  if (fk_after && !fk_have_reply) {                        /* let the reply leave first */
    static uint32_t at;
    if (!at) at = now_us() | 1;
    else if (now_us() - at > 150000u) { if (fk_after == 1) fk_dfu(); NVIC_SystemReset(); }   /* an air reply needs a poll or two */
  }
}
