// openhouse nRF52840 dongle, v0: USB command channel, self-update via the UF2 bootloader,
// crash safety, and raw 2 Mbit radio test commands. Same code for both flash variants.
#include <string.h>
#include "tusb.h"
#include "dongle.h"
#include "doorman_link.h"
#include "link.h"
#include "banks.h"

#ifndef VARIANT_SD
#error "build with -DVARIANT_SD=0 or 1"
#endif
#ifndef USE_WDT
#define USE_WDT 0   /* off by default: a running WDT also keeps running inside the bootloader */
#endif

extern uint32_t __isr_vector[];
#if BOARD_NORDIC_DFU
#define LED_PIN 6    /* P0.06: green LED, active low (Holyiot 21017 and PCA10059 alike) */
#define LED_ON()  (NRF_P0->OUTCLR = 1u << LED_PIN)
#define LED_OFF() (NRF_P0->OUTSET = 1u << LED_PIN)
#else
#define LED_PIN 15   /* P0.15: Pro Micro / nice!nano LED, active high */
#define LED_ON()  (NRF_P0->OUTSET = 1u << LED_PIN)
#define LED_OFF() (NRF_P0->OUTCLR = 1u << LED_PIN)
#endif

/* ---------- crash record (survives soft resets; outside the linker's RAM region) ---------- */
typedef struct {
    uint32_t magic, type, pc, lr, psr, cfsr, hfsr, bfar, count, sum;
} crash_rec_t;
#define CRASH_MAGIC 0x43525348u   /* "CRSH" */
#define crash ((volatile crash_rec_t *)0x2003E000)
#define QUICK_CRASH_LIMIT 3       /* this many crashes in a row (each < 10 s uptime) -> UF2 bootloader */

static uint32_t crash_sum(void) {
    uint32_t s = 0x5A5A5A5A;
    const volatile uint32_t *p = &crash->magic;
    for (int i = 0; i < 9; i++) s = (s << 5 | s >> 27) ^ p[i];
    return s;
}
static int crash_valid(void) { return crash->magic == CRASH_MAGIC && crash->sum == crash_sum(); }

/* Nordic DFU boards: GPREGRET 0xB1 asks the bootloader for DFU. Some SDK bootloaders are built without
   that (a Kuroshin dongle's: only pin reset or an invalid app enter DFU). There the app can't help: the
   bootloader ACL-locks the app's flash, and the reset pin (P0.18) can't be driven as a GPIO (bus
   fault). Such a dongle enters DFU only by shorting RESET to GND. */
static void enter_bootloader(uint8_t magic) {
    __disable_irq();
    NRF_POWER->GPREGRET = magic;
    NVIC_SystemReset();
}

void crash_entry(uint32_t *frame, uint32_t type) {
    uint32_t n = crash_valid() ? crash->count : 0;
    crash->magic = CRASH_MAGIC; crash->type = type;
    crash->pc = frame ? frame[6] : 0; crash->lr = frame ? frame[5] : 0; crash->psr = frame ? frame[7] : 0;
    crash->cfsr = SCB->CFSR; crash->hfsr = SCB->HFSR; crash->bfar = SCB->BFAR;
    crash->count = n + 1;
    crash->sum = crash_sum();
    /* quick-crash counter lives in GPREGRET2 (retained across soft resets, cleared at power-on) */
    uint32_t q = (NRF_POWER->GPREGRET2 & 0xFF) + 1;
    NRF_POWER->GPREGRET2 = q;
    if (q >= QUICK_CRASH_LIMIT) { NRF_POWER->GPREGRET2 = 0; enter_bootloader(GPREGRET_UF2_DFU); }
    __disable_irq();
    NVIC_SystemReset();
}

#define FAULT_STUB(name, code) \
    __attribute__((naked)) void name(void) { \
        __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n movs r1, %0\n b crash_entry" :: "i"(code)); }
FAULT_STUB(HardFault_Handler, 1)
FAULT_STUB(MemoryManagement_Handler, 2)
FAULT_STUB(BusFault_Handler, 3)
FAULT_STUB(UsageFault_Handler, 4)
void WDT_IRQHandler(void) { crash_entry(0, 5); }   /* runs just before a watchdog reset (~2 clocks of 32k) */

/* ---------- time, and the hang detector ---------- */
/* The main loop and usb_service() stamp hb_ms. SysTick runs at the highest priority, so it interrupts
   any code stuck with interrupts on (a radio wait, an ISR): stale for HANG_MS, it records the stuck
   code's pc/lr as a "hang" crash and resets. The boot after that runs in safe mode (no radio), so a
   dongle that hangs stays reachable over USB: its record can be read and it can still be updated. */
#define HANG_MS 2000u
volatile uint32_t ms_ticks;
static volatile uint32_t hb_ms;
static uint8_t safe_mode;
void systick_c(uint32_t *frame)
{
    ms_ticks++;
    if (ms_ticks - hb_ms > HANG_MS) { hb_ms = ms_ticks; crash_entry(frame, 6); }
}
__attribute__((naked)) void SysTick_Handler(void)
{
    __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n b systick_c");
}
uint32_t board_millis(void) { return ms_ticks; }

/* ---------- USB power events (no SoftDevice is ever enabled, in either variant) ---------- */
extern void tusb_hal_nrf_power_event(uint32_t event);
void POWER_CLOCK_IRQHandler(void) {
    if (NRF_POWER->EVENTS_USBDETECTED) { NRF_POWER->EVENTS_USBDETECTED = 0; tusb_hal_nrf_power_event(0); }
    if (NRF_POWER->EVENTS_USBREMOVED)  { NRF_POWER->EVENTS_USBREMOVED = 0;  tusb_hal_nrf_power_event(1); }
    if (NRF_POWER->EVENTS_USBPWRRDY)   { NRF_POWER->EVENTS_USBPWRRDY = 0;   tusb_hal_nrf_power_event(2); }
}
void USBD_IRQHandler(void) { tud_int_handler(0); }

/* ---------- command handling ---------- */
static uint32_t boot_resetreas, usb_rx, usb_tx;
static uint8_t reply[64], pending_reply, pending_action, pending_scan, scan_first;

static void put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

/* Self-update (banks.h): this image runs in one bank, updates go into the other. */
static uint32_t upd_size, upd_crc;
static uint8_t upd_open;
static uint32_t running_bank(void) { return (uint32_t)__isr_vector - BANK_IMAGE_OFS; }
static uint32_t other_bank(void) { return running_bank() == BANK_A ? BANK_B : BANK_A; }
static void nvmc_ready(void) { while (!NRF_NVMC->READY) {} }
static void nvmc_erase(uint32_t addr, uint32_t len)
{
    for (uint32_t a = addr; a < addr + len; a += 0x1000) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een; nvmc_ready();
        NRF_NVMC->ERASEPAGE = a; nvmc_ready();          /* ~85 ms each: the CPU waits */
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren; nvmc_ready();
        hb_ms = ms_ticks;                               /* not a hang */
    }
}
static void nvmc_write(uint32_t addr, const uint8_t *src, uint32_t n)
{
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen; nvmc_ready();
    for (uint32_t i = 0; i < n; i += 4) {
        uint32_t w; memcpy(&w, src + i, 4);
        *(volatile uint32_t *)(addr + i) = w; nvmc_ready();
    }
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren; nvmc_ready();
}

static void handle(const uint8_t *in, uint16_t len) {
    usb_rx++;
    if (len < 2) return;
    memset(reply, 0, sizeof reply);
    reply[0] = in[0]; reply[1] = in[1];
    uint8_t *d = reply + 3, st = ST_OK;
    switch (in[0]) {
    case CMD_INFO:
        memcpy(d, "OHDG", 4);
        d[4] = DONGLE_FW_VERSION & 0xFF; d[5] = DONGLE_FW_VERSION >> 8;
        d[6] = VARIANT_SD; d[7] = USE_WDT;
        put32(d + 8, (uint32_t)__isr_vector);
        put32(d + 12, boot_resetreas);
        put32(d + 16, ms_ticks);
        put32(d + 20, NRF_FICR->DEVICEID[0]); put32(d + 24, NRF_FICR->DEVICEID[1]);
        if (crash_valid()) {
            put32(d + 28, crash->count); put32(d + 32, crash->type); put32(d + 36, crash->pc);
            put32(d + 40, crash->lr); put32(d + 44, crash->cfsr); put32(d + 48, crash->hfsr);
        }
        d[52] = NRF_POWER->GPREGRET2;
        put32(d + 53, *(volatile uint32_t *)0x3000 == 0x51B1E5DB);   /* SoftDevice magic still in flash? */
        d[57] = BOARD_ID;    /* 0 UF2 (Pro Micro style), 1 Holyiot 21017, 2 PCA10059, 3 Kuroshin (Nordic DFU) */
        d[58] = safe_mode;
        d[59] = running_bank() == BANK_A ? 1 : 2;   /* which bank runs (banks.h) */
        d[60] = DML_SLOTS;   /* slots it has (0 from firmware before 0x002A: 8) */
        break;
    case CMD_STATS: put32(d, usb_rx); put32(d + 4, usb_tx); put32(d + 8, ms_ticks); break;
    case CMD_BANK_BEGIN: {
        uint32_t size, crc; memcpy(&size, in + 2, 4); memcpy(&crc, in + 6, 4);
        if (!size || size > BANK_IMAGE_MAX || (size & 3)) { st = ST_BAD_ARG; break; }
        const uint32_t b = other_bank();
        nvmc_erase(b, BANK_IMAGE_OFS + ((size + 0xFFFu) & ~0xFFFu));   /* header page first: invalid until commit */
        upd_size = size; upd_crc = crc; upd_open = 1;
        put32(d, b);
        break;
    }
    case CMD_BANK_WRITE: {
        uint32_t off; memcpy(&off, in + 2, 4);
        const uint8_t n = in[6];
        if (!upd_open || n > 56 || (n & 3) || (off & 3) || off + n > upd_size || len < 7u + n) { st = ST_BAD_ARG; break; }
        const uint32_t a = other_bank() + BANK_IMAGE_OFS + off;
        nvmc_write(a, in + 7, n);
        if (memcmp((const void *)a, in + 7, n)) st = ST_TIMEOUT;   /* didn't stick */
        break;
    }
    case CMD_BANK_COMMIT: {
        const uint32_t b = other_bank();
        if (!upd_open || bank_crc32((const uint8_t *)(b + BANK_IMAGE_OFS), upd_size) != upd_crc) { st = ST_BAD_ARG; break; }
        const bank_header_t *cur = (const bank_header_t *)running_bank();
        const bank_header_t h = { BANK_MAGIC, cur->seq + 1, upd_size, upd_crc, cur->origin };   /* same stub payload chain */
        nvmc_write(b, (const uint8_t *)&h, sizeof h);
        if (!bank_valid(b)) { st = ST_TIMEOUT; break; }
        upd_open = 0;
        pending_action = CMD_RESET;                 /* the stub boots the newest valid bank */
        put32(d, h.seq);
        break;
    }
    case 0x7D: {                                /* read memory: [addr u32][n <= 56]; flash, UICR, FICR, RAM */
        uint32_t a; memcpy(&a, in + 2, 4);
        uint8_t n = in[6] > 56 ? 56 : in[6];
        const int ok = (a < 0x100000u && a + n <= 0x100000u) || (a >= 0x10000000u && a + n <= 0x10002000u) ||
                       (a >= 0x20000000u && a + n <= 0x20040000u);
        if (!ok) { st = ST_BAD_ARG; break; }
        put32(d, a); d[4] = n; memcpy(d + 5, (const void *)a, n);
        break;
    }
    case 0x7E:                                  /* USB controller state, for boards whose endpoints stall */
        put32(d, usb_rx); put32(d + 4, usb_tx); put32(d + 8, ms_ticks);
        put32(d + 12, NRF_USBD->EPSTATUS); put32(d + 16, NRF_USBD->EPDATASTATUS);
        put32(d + 20, NRF_USBD->EPINEN); put32(d + 24, NRF_USBD->EPOUTEN);
        put32(d + 28, NRF_USBD->USBADDR); put32(d + 32, NRF_USBD->EVENTCAUSE);
        d[36] = (uint8_t)NRF_USBD->SIZE.EPOUT[1]; d[37] = (uint8_t)NRF_USBD->SIZE.EPOUT[2];
        d[38] = (uint8_t)NRF_USBD->SIZE.EPOUT[3]; d[39] = (uint8_t)NRF_USBD->SIZE.EPOUT[4];
        d[40] = tud_mounted(); d[41] = safe_mode;
        put32(d + 42, NRF_FICR->INFO.PART); put32(d + 46, NRF_FICR->INFO.VARIANT);
        d[50] = (uint8_t)*(volatile uint32_t *)0x10000130ul; d[51] = (uint8_t)*(volatile uint32_t *)0x10000134ul;
        put32(d + 52, NRF_CLOCK->HFCLKSTAT);
        break;
    case CMD_ECHO: memcpy(d, in + 2, len - 2 > 61 ? 61 : len - 2); break;
    case CMD_DFU_UF2: case CMD_DFU_SERIAL: case CMD_RESET: case CMD_CRASH_TEST:
        pending_action = in[0]; d[0] = in[2]; break;   /* done after the reply has gone out */
    case CMD_CLEAR_CRASH: crash->magic = 0; NRF_POWER->GPREGRET2 = 0; break;
    case CMD_RADIO_RSSI_SCAN: scan_first = in[2]; pending_scan = 1; return;   /* runs between cycles */
    default: if (!link_cmd(in, d, &st)) st = ST_BAD_CMD;
    }
    reply[2] = st;
    pending_reply = 1;
}

static void do_action(uint8_t a, uint8_t arg) {
    uint32_t t = ms_ticks; while (ms_ticks - t < 20) tud_task();   /* let the reply leave */
    switch (a) {
    case CMD_DFU_UF2:    enter_bootloader(GPREGRET_UF2_DFU); break;
    case CMD_DFU_SERIAL: enter_bootloader(GPREGRET_SERIAL_DFU); break;
    case CMD_RESET:      NVIC_SystemReset(); break;
    case CMD_CRASH_TEST:
        if (arg == 1) { void (*bad)(void) = (void (*)(void))0xFFFFFFF0; bad(); }  /* HardFault */
        if (arg == 2) { __disable_irq(); for (;;) {} }                            /* hang (needs WDT) */
        break;
    }
}

/* A feature GET_REPORT on interface 0 returns the last command's reply. With SET_REPORT (which already
   carries commands) that is a command path over the control endpoint alone: it still works when the
   HID data endpoints don't (seen on a Kuroshin board), so such a dongle can still be read and updated. */
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t id, hid_report_type_t t, uint8_t *buf, uint16_t n) {
    (void)id;
    if (itf != 0 || t != HID_REPORT_TYPE_FEATURE || !n) return 0;
    const uint16_t k = n < sizeof reply ? n : (uint16_t)sizeof reply;
    memcpy(buf, reply, k);
    return k;
}
void tud_hid_set_report_cb(uint8_t itf, uint8_t id, hid_report_type_t t, const uint8_t *buf, uint16_t n) {
    (void)itf; (void)t;
    if (id == 0 && n) handle(buf, n);
}

/* The next report for an interface: on interface 0 a reply first; slot s's stream
   [0x80][slot][n][bytes] on interface s % DONGLE_NIF. */
static void usb_send_next(uint8_t itf)
{
    static uint8_t rep[DONGLE_NIF][64];
    if (!tud_mounted() || !tud_hid_n_ready(itf)) return;
    if (itf == 0 && pending_reply) { tud_hid_n_report(0, 0, reply, 64); usb_tx++; pending_reply = 0; return; }
    if (link_stream_pop(rep[itf], itf)) tud_hid_n_report(itf, 0, rep[itf], 64);
}
/* A report went out: queue the next one right away. Queued only from the main loop, the endpoint sat
   empty when the host came back for it the next millisecond, and every interface topped out at one
   report per 2 ms (~30 kB/s): two busy devices on one filled the dongle's queue and lost data. */
void tud_hid_report_complete_cb(uint8_t itf, uint8_t const *report, uint16_t len)
{
    (void)report; (void)len;
    if (itf < DONGLE_NIF) usb_send_next(itf);
}

static void usb_service(void)              /* also called while the link waits inside a cycle */
{
    hb_ms = ms_ticks;
    tud_task();
    for (uint8_t itf = 0; itf < DONGLE_NIF; itf++) usb_send_next(itf);
}

int main(void) {
    SCB->VTOR = (uint32_t)__isr_vector;     /* our own table, whatever the bootloader/MBR set */
    NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Msk;   /* off after reset: every fetch waited on flash (decoding ~3x slower) */
    __DSB(); __ISB();
#if BOARD_PCA10059
    /* The PCA10059 runs straight off USB (VDDH), and a blank chip sets its GPIO supply (REGOUT0) to
       1.8 V: too low to light the LEDs well. Like Nordic's board code: once, 3.0 V. Only clears bits
       of a UICR word still at its erased value, so no erase, and only when nothing set it yet. */
    if ((NRF_POWER->MAINREGSTATUS & POWER_MAINREGSTATUS_MAINREGSTATUS_Msk) &&
        (NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) == (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen; while (!NRF_NVMC->READY) {}
        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~UICR_REGOUT0_VOUT_Msk) | (UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos);
        while (!NRF_NVMC->READY) {}
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren; while (!NRF_NVMC->READY) {}
        NVIC_SystemReset();   /* takes effect at the next reset */
    }
#endif
    __enable_irq();                         /* don't rely on the bootloader leaving PRIMASK clear */
    NRF_P0->PIN_CNF[LED_PIN] = 1; LED_ON();   /* LED on: main() reached */
    boot_resetreas = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = 0xFFFFFFFF;
    if (boot_resetreas & (POWER_RESETREAS_RESETPIN_Msk) || boot_resetreas == 0) NRF_POWER->GPREGRET2 = 0;
    NRF_POWER->GPREGRET = 0;

    /* Safe mode: the last reset was a hang (crash type 6) right after boot. GPREGRET2 counts quick
       crashes and is cleared at power-on, so a replug leaves safe mode. */
    safe_mode = crash_valid() && crash->type == 6 && (NRF_POWER->GPREGRET2 & 0xFF) != 0;
    hb_ms = 0;
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0);      /* above USB and everything else: the hang detector */
    NRF_P0->PIN_CNF[LED_PIN] = 1;           /* output */
    NVIC_SetPriority(USBD_IRQn, 2);
    NVIC_SetPriority(POWER_CLOCK_IRQn, 2);

#if USE_WDT
    NRF_WDT->CONFIG = WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos;  /* pause while debugging */
    NRF_WDT->CRV = 4 * 32768;
    NRF_WDT->RREN = 1;
    NRF_WDT->INTENSET = 1; NVIC_EnableIRQ(WDT_IRQn);
    NRF_WDT->TASKS_START = 1;
#endif

    link_init();
    link_set_idle_hook(usb_service);

    tud_init(0);                            /* tusb_init() is a no-op without CFG_TUSB_RHPORT0_MODE */
    NRF_POWER->INTENSET = POWER_INTENSET_USBDETECTED_Msk | POWER_INTENSET_USBREMOVED_Msk | POWER_INTENSET_USBPWRRDY_Msk;
    NVIC_EnableIRQ(POWER_CLOCK_IRQn);
    uint32_t reg = NRF_POWER->USBREGSTATUS;  /* USB may already be up: no event for that */
    if (reg & POWER_USBREGSTATUS_VBUSDETECT_Msk) tusb_hal_nrf_power_event(0);
    if (reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk) tusb_hal_nrf_power_event(2);

    for (;;) {
#if USE_WDT
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;
#endif
        hb_ms = ms_ticks;
        tud_task();
        if (ms_ticks > 10000 && NRF_POWER->GPREGRET2 && !safe_mode) NRF_POWER->GPREGRET2 = 0;   /* stable: not a quick crash */
        if (!safe_mode) link_poll();        /* one TDMA cycle; services USB through usb_service() */
        if (pending_scan) { pending_scan = 0; link_rssi_scan(scan_first, reply + 3, 60); reply[2] = ST_OK; pending_reply = 1; }
        usb_service();
        if (pending_action && !pending_reply) { uint8_t a = pending_action; pending_action = 0; do_action(a, reply[3]); }
        uint32_t period = tud_mounted() ? 1000 : 200;
        if (ms_ticks % period < period / 10) LED_ON(); else LED_OFF();
    }
}
