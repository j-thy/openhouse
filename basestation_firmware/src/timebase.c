#include "timebase.h"
#include "timers.h"
#include "rotor.h"
#include "params.h"
#include "board.h"
#include "carrier.h"
#include "sys.h"
#include "multi.h"

static uint32_t period_cur;         /* stock 0x1000066C+0xC */
static volatile uint32_t edges;
static volatile uint32_t last_edge;     /* CT32B1 time of last reference edge */
static volatile uint32_t last_lock_ms;  /* last time emission was permitted */
static volatile bool tdm_flag = true;   /* stock 0x1000067D: 1 = sweep revolution */

/* multi-station round-robin (ai/firmware/MULTISTATION.md) */
static volatile uint32_t rev;           /* revolution counter, see multi_sweeps_in() */
static volatile bool flash_rev = true;  /* this station flashes in this revolution */
static volatile bool emitting;          /* last emission_update() result */
static volatile uint8_t sf_confirm;     /* consecutive superframe confirmations */
volatile uint32_t sf_realign_count;

/* A slave sweeps only with the round confirmed (the master's sweeps where its round
 * expects them, twice running); it flashes as soon as it holds the master's phase. */
static bool multi_sweeps_now(void) { return multi_sweeps_in(rev) && (MC->slot == 0 || sf_confirm >= 2); }

/* ---- optical slave (stock timebase_capture_isr @0xC8B0, correlator @0x5ED0-
 * 0x6180, classifier @0xB514 + table @0x11EDC, phase callback @0xAAB8) ---- */
#define PIN_OPT_IN      PIN(1, 4)       /* CT32B1_CAP0, photodiode receiver */
#define PIN_OPT_SEL     PIN(1, 15)      /* high = optical receiver selected */
enum { CS_DISABLED, CS_SEARCHING, CS_SYNCING, CS_LOCKED, CS_LOST };
static const char *const corr_state_names[] = { "disabled", "searching", "syncing", "locked", "lost" };
static volatile uint32_t rise_t, last_width, pulses_seen, pulses_used;
static struct { uint32_t last; uint32_t n; double sum_ms; float est; } corr;
static float ph_f, ph_prev, ph_integ;  /* timebase phase loop (stock 0x10000280+0x18..) */
static volatile int32_t mr1_extra, last_phase_err;
static volatile uint8_t phase_good;     /* consecutive phase errors < 20 us */

/* diagnostics: last corrections (lhbs) */
typedef struct { uint8_t kind; int32_t e, extra; uint32_t a, b; } tb_trace_t;
static tb_trace_t trace[16];
static volatile uint8_t trace_pos;
volatile uint32_t tb_guard_count;
static void tb_trace(uint8_t kind, int32_t e, int32_t extra, uint32_t a, uint32_t b)
{
    tb_trace_t *x = &trace[trace_pos++ & 15];
    x->kind = kind; x->e = e; x->extra = extra; x->a = a; x->b = b;
}

/* ---- cable sync (stock timebase_capture_isr @0xC8B0, timebase_mr1_isr @0xCC8C).
 * One pulse per period: rises at the master's edge, falls period/2 later,
 * +-CABLE_TDM_WIDTH by the master's skip flag in TDM. ---- */
#define PIN_CABLE_IN    PIN(0, 12)      /* CT32B1_CAP0 (func 3), selected with P1_15 low */
#define PIN_CABLE_OUT   PIN(0, 14)      /* CT32B1_MAT1 (func 3) */
#define CABLE_TOL       10000u          /* stock: rising edges one period +- this apart */
#define CABLE_TDM_WIDTH 100000u         /* stock 0xCEE8 */
#define CABLE_TDM_SPLIT 50000u          /* stock 0xCA74/0xCA78 */
#define CABLE_WIDTH_TOL 20000u          /* lhbs: a width this close to one stock sends */
#define CABLE_LOST_PERIODS 10u          /* stock: periods without a good rise -> lost */
#define CABLE_LISTEN_MS 2000u           /* lhbs: unlocked this long on the cable -> optical */
#define CABLE_RETRY_MS  5000u           /* lhbs: minimum time on optical before a cable try */
#define CABLE_BACKOFF_MAX 12u           /* lhbs: cap on the retry wait, x CABLE_RETRY_MS */
static volatile bool cable_in;          /* CAP0 on the cable (else the photodiode) */
static volatile bool cable_locked;
static volatile bool cable_tdm = true;  /* from the last width, applied at the next rise */
static volatile uint32_t cable_rise, cable_good, cable_edges, cable_lost_count;
static volatile uint32_t cable_stray_rises, cable_stray_falls, cable_caps;
static uint32_t input_since_ms;
static uint8_t cable_levels;            /* while optical: 1 = P0_12 seen low, 2 = high */
static uint8_t cable_misses;            /* cable tries without a lock, since the last lock */

static bool slave_mode(void)
{
    return P.timebase_mode == TB_SLAVE_OPTICAL || P.timebase_mode == TB_SLAVE_CABLE;
}

static bool cable_mode(void)
{
    return P.timebase_mode == TB_SLAVE_CABLE && !multi_active();
}

/* console `set` can change the mode without a reselect: trust the mode */
static bool on_cable(void) { return cable_in && cable_mode(); }

/* stock classifier table @0x11EDC: width (48 MHz ticks) -> pulse type, tol 250 */
static int classify(uint32_t w)
{
    static const uint16_t width[8] = { 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500 };
    static const uint8_t type[8]   = { 2,    4,    3,    5,    6,    8,    7,    9 };
    for (int i = 0; i < 8; i++)
        if (w + 250 >= width[i] && w <= width[i] + 250u) return type[i];
    return 1;
}

static void capture_irq(bool on)
{
    TMR_CT32B1->IR = IR_CR(0);          /* drop a capture latched while masked */
    if (on) TMR_CT32B1->CCR |= CCR_I(0); else TMR_CT32B1->CCR &= ~CCR_I(0);
}

/* stock cc64: mask our own emissions from the receiver (slave + optical.mask) */
void timebase_mask_own(bool masked)
{
    /* Called from CT32B0, which preempts CT32B1: capture_irq's IR clear would drop a
     * latched cable edge, and the cable can't see our light anyway. */
    if (on_cable()) return;
    if (!slave_mode() || !P.timebase_optical_mask) masked = false;
    capture_irq(!masked);
}

/* stock callback @0xAAB8: correlator -> timebase phase/period */
static void corr_callback(uint32_t t, bool nonskip, bool first)
{
    uint32_t offset = multi_active() ? MC->slot * MC->stagger : P.timebase_optical_offset;
    int32_t e = (int32_t)(t - last_edge) - (int32_t)(period_cur >> 2) + (int32_t)offset;
    if (multi_active() && nonskip && !first) {
        /* the master sweeps in revolutions r = 0 mod count: align our counter */
        const uint32_t count = (uint32_t)multi_count();
        multi_master_sweep(rev % count == 0);
        if (rev % count) {
            rev -= rev % count;
            sf_confirm = 0;
            sf_realign_count++;
        } else if (sf_confirm < 3) {
            sf_confirm++;
        }
    }
    last_phase_err = e;
    /* multi slave: sweep only once the stagger is held (flashes 250 us apart) */
    {
        int32_t a = e < 0 ? -e : e;
        if (a > 1920) phase_good = 0;                       /* > 40 us: stop */
        else if (a < 960 && phase_good < 20) phase_good++;  /* < 20 us */
    }
    float k = P.timebase_k;
    float f = (1.0f - k) * (float)e + k * ph_f;
    int32_t deriv = (int32_t)(f - ph_prev);
    ph_prev = f;
    ph_f = f;
    if (!first) {
        float half = (float)(period_cur >> 1);
        ph_integ += f;
        while (ph_integ >= half) ph_integ -= half;
        while (ph_integ <= -half) ph_integ += half;
    } else {
        ph_f = ph_prev = ph_integ = 0;
        mr1_extra = e;                  /* jump straight onto the master */
        deriv = 0;
        tdm_flag = !nonskip;            /* sweep when the master skips */
    }
    period_cur = (uint32_t)corr.est;
    mr1_extra += (int32_t)(P.timebase_p * ph_f + P.timebase_i * ph_integ + P.timebase_d * (float)deriv);
    tb_trace(first ? 'F' : 'C', e, mr1_extra, t, last_edge);
}

/* stock correlator_add @0x5F64. lhbs, multi: `first` = the first sync flash of its
 * group. Searching, stock takes any two consecutive flashes one period apart; with
 * other slaves already flashing 250-750 us after the master, consecutive flashes are
 * never a period apart (purple for minutes, locking only when a flash was missed, and
 * then onto whichever station). The master flashes first in every group: searching
 * looks at first flashes only. Once syncing, the period check rejects the others. */
static void corr_pulse(uint32_t t, bool nonskip, bool first)
{
    if (!P.corr_enable) { P.corr_status = CS_DISABLED; return; }
    if (P.corr_status == CS_DISABLED) P.corr_status = CS_SEARCHING;
    if (multi_active() && !first && (P.corr_status == CS_SEARCHING || P.corr_status == CS_LOST)) return;
    uint32_t per = P.timebase_period, tol = P.corr_tolerance;
    uint32_t dt = t - corr.last;
    if (dt > per + tol || dt < per - tol) {
        if ((P.corr_status == CS_SYNCING || P.corr_status == CS_LOCKED) && corr.est > 0) {
            uint32_t est = (uint32_t)corr.est;
            int32_t r = (int32_t)(dt % est);
            if (r > (int32_t)(est / 2)) r -= (int32_t)est;
            if ((uint32_t)(r < 0 ? -r : r) >= 2 * tol) return;    /* stray pulse */
            P.corr_missed += dt / est;                             /* missed periods */
        }
        corr.last = t;
        return;
    }
    switch (P.corr_status) {
    case CS_SEARCHING:
    case CS_LOST:
        corr.est = (float)dt;
        corr.n = 0; corr.sum_ms = 0;
        P.corr_status = CS_SYNCING;
        break;
    case CS_SYNCING:
        if (corr.n >= P.corr_settle) {
            corr.est = (float)(corr.sum_ms / corr.n * 48000.0);
            corr.n = 0; corr.sum_ms = 0;
            corr_callback(t, nonskip, true);
            P.corr_status = CS_LOCKED;
        }
        break;
    case CS_LOCKED:
        corr.est = (float)((1.0 - P.corr_k) * (double)dt + P.corr_k * (double)corr.est);
        corr_callback(t, nonskip, false);
        break;
    }
    P.corr_period = corr.est;
    corr.n++;
    corr.sum_ms += (double)dt / 48000.0;
    corr.last = t;
    pulses_used++;
}

/* stock correlator_tick @0x5EF0, every timebase edge */
static void corr_tick(uint32_t now)
{
    if (multi_take_relock() && slave_mode()) {      /* locked onto a slave: search again */
        corr.n = 0; corr.sum_ms = 0;
        corr.last = 0;
        P.corr_status = CS_SEARCHING;
        return;
    }
    if ((P.corr_status == CS_SYNCING || P.corr_status == CS_LOCKED) &&
        now - corr.last > P.corr_timeout * P.timebase_period) {
        corr.n = 0; corr.sum_ms = 0;
        corr.last = 0;
        P.corr_status = CS_LOST;
    }
}

/* Multi: whose flash this is. Every station flashes a quarter period after its own
 * timebase edge, slot s * stagger after the master (slaves hold that by the phase
 * loop): so relative to our own edge and slot, slot s' flashes at s' * stagger, at
 * both index events of a revolution. -1: not on the slot grid. */
static int flash_slot(uint32_t t)
{
    const int32_t half = (int32_t)(period_cur >> 1);
    const int32_t stagger = (int32_t)MC->stagger;
    int32_t d = (int32_t)(t - last_edge) - (int32_t)(period_cur >> 2) + MC->slot * stagger;
    d %= half;
    if (d < -half / 4) d += half;
    if (d >= half - half / 4) d -= half;
    if (d < -stagger / 4) return -1;
    const int32_t s = (d + stagger / 2) / stagger;
    const int32_t err = d - s * stagger;
    if (s >= MULTI_SLOTS || err > 2880 || err < -2880) return -1;   /* 60 us */
    return (int)s;
}

static void optical_edge(uint32_t t)
{
    if (gpio_read(PIN_OPT_IN)) { rise_t = t; return; }    /* light on: pulse start */
    uint32_t w = t - rise_t;                              /* falling edge: pulse end */
    last_width = w;
    pulses_seen++;
    int type = classify(w);
    /* multi: first flash of its group (stations flash within 1 ms; groups are a half
     * period apart). Our own flashes are masked from the receiver. */
    static uint32_t last_sync;
    bool first = true;
    if (type >= 2) {
        first = rise_t - last_sync > 57600;          /* 1.2 ms */
        last_sync = rise_t;
    }
    if (type >= 2 && multi_active()) {
        const int s = flash_slot(rise_t);
        if (s >= 0 && s != MC->slot) multi_seen(s, type >= 6);
        /* a slave decodes the master's frame: data bit of every master flash (ALL) */
        if (s == 0 && MC->slot != 0 && P.corr_status == CS_LOCKED)
            multi_master_bit(type == 3 || type == 5 || type == 7 || type == 9);
    }
    if (slave_mode() && (type == 2 || type == 3 || type == 6 || type == 7))  /* axis-0 syncs only */
        corr_pulse(rise_t, type <= 5, first);
}

/* Locked, the cable's rises replace MR1 as the rotors' reference (stock: MR1 then
 * only watches for loss). Unlike stock, an edge that doesn't fit the signal is
 * counted as stray and ignored, so a glitch can't move the reference. */
static void cable_edge(uint32_t t)
{
    const uint32_t per = P.timebase_period ? P.timebase_period : 800000;
    const uint32_t half = per >> 1;
    if (!gpio_read(PIN_CABLE_IN)) {
        /* the three widths stock sends: half, half -+ CABLE_TDM_WIDTH */
        const uint32_t w = t - cable_rise;
        const int32_t d = (int32_t)(w - half);
        const int32_t off = d < 0 ? -d : d;
        if (off > (int32_t)CABLE_WIDTH_TOL &&
            (off < (int32_t)(CABLE_TDM_WIDTH - CABLE_WIDTH_TOL) ||
             off > (int32_t)(CABLE_TDM_WIDTH + CABLE_WIDTH_TOL))) {
            cable_stray_falls++;
            tb_trace('W', (int32_t)w, 0, t, cable_rise);
            return;
        }
        last_width = w;
        cable_tdm = w <= half + CABLE_TDM_SPLIT;
        return;
    }
    const uint32_t dt = t - cable_rise;
    const uint32_t k = (dt + half) / per;           /* whole periods since the last rise */
    const int32_t err = (int32_t)(dt - k * per);
    const bool fits = cable_locked ? k >= 1 && k < CABLE_LOST_PERIODS : k == 1;
    if (!fits || err > (int32_t)CABLE_TOL || err < -(int32_t)CABLE_TOL) {
        if (cable_locked) {                         /* noise: keep the reference */
            cable_stray_rises++;
            tb_trace('R', (int32_t)dt, 0, t, cable_rise);
            return;
        }
        cable_rise = t;                             /* maybe the first of a new train */
        return;
    }
    if (k == 1) period_cur = dt;
    cable_rise = t;
    cable_good = t;
    cable_edges++;
    cable_locked = true;
    tdm_flag = cable_tdm;
    rotor_reference_edge(t, P.timebase_offset);
    rotor_control();
}

/* Only one pin may be CT32B1_CAP0; the other goes back to GPIO. FUNC is changed
 * read-modify-write, as stock does: P0_12's ADMODE bit must stay digital. */
static void route_capture(bool cable)
{
    lpc_timer_t *t = TMR_CT32B1;
    t->CCR = 0;
    if (cable) {
        *iocon(PIN_OPT_IN) = IOCON_FUNC(0);
        gpio_output(PIN_OPT_SEL, false);
        *iocon(PIN_CABLE_IN) = (*iocon(PIN_CABLE_IN) & ~7u) | IOCON_FUNC(3) | (1u << 5);
    } else {
        *iocon(PIN_CABLE_IN) = (*iocon(PIN_CABLE_IN) & ~7u) | IOCON_FUNC(1);
        gpio_output(PIN_OPT_SEL, true);
        *iocon(PIN_OPT_IN) = IOCON_FUNC(1) | (1u << 5);   /* CT32B1_CAP0, hysteresis */
    }
    cable_in = cable;
    input_since_ms = sys_ms();
    t->IR = IR_CR(0);
    t->CCR = CCR_RISE(0) | (2u << 0) | CCR_I(0);          /* both edges + irq */
}

/* Sync out, P0_14 as MAT1 (set by hardware at MR1, cleared at MR3), in A/b/c as stock.
 * Multi doesn't use the cable and every multi station raises P1_0, so it leaves P0_14
 * undriven: a cable left plugged in would otherwise have several drivers. */
static bool sync_out_on;
static void sync_out(bool on)
{
    lpc_timer_t *t = TMR_CT32B1;
    uint32_t s = irq_save();
    sync_out_on = on;
    if (on) {
        gpio_output(PIN_CABLE_OUT, false);
        *iocon(PIN_CABLE_OUT) = (*iocon(PIN_CABLE_OUT) & ~7u) | IOCON_FUNC(3);
        t->EMR = 2u << 6;           /* EMC1 = set on match */
        if (t->MCR & MCR_I(1)) t->MCR |= MCR_I(3);  /* else timebase_start sets it */
    } else {
        t->MCR &= ~MCR_I(3);
        t->EMR = 0;
        t->IR = IR_MR(3);
        *iocon(PIN_CABLE_OUT) = (*iocon(PIN_CABLE_OUT) & ~7u) | IOCON_FUNC(1);
        GPIO_DIR(PIN_PORT(PIN_CABLE_OUT)) &= ~(1u << PIN_BIT(PIN_CABLE_OUT));
    }
    irq_restore(s);
}

static void reset_sync_state(void)
{
    period_cur = P.timebase_period ? P.timebase_period : 800000;
    mr1_extra = 0;
    ph_f = ph_prev = ph_integ = 0;
    corr.n = 0; corr.sum_ms = 0; corr.est = 0;
    cable_locked = false;
    cable_tdm = true;
}

/* stock timebase_select_input @0xCBA8: optical receiver for slave modes, the cable
 * for slave-cable (lhbs: the cable first, see cable_switch) */
void timebase_select_input(void)
{
    lpc_timer_t *t = TMR_CT32B1;
    sync_out(!multi_active());
    uint32_t s = irq_save();
    t->CCR = 0;                     /* no capture on the old input against reset state */
    reset_sync_state();
    cable_misses = 0;
    rev = 0;
    sf_confirm = 0;
    flash_rev = true;
    P.corr_status = CS_DISABLED;
    irq_restore(s);
    if (slave_mode() || multi_active()) {          /* a multi master listens for its slaves */
        route_capture(cable_mode());
        if (slave_mode() && !cable_in) P.corr_status = CS_SEARCHING;
    } else {
        gpio_output(PIN_OPT_SEL, false);
        t->CCR = 0;
        cable_in = false;
        t->IR = IR_CR(0);
    }
}

/* Every 10 ms from the main loop, while optical (P0_12 is then a plain GPIO). A live
 * cable is high 37-63 % of each period, and 10 ms samples walk across the 16.7 ms
 * period, so both levels show within one supervisor interval; an idle jack holds one.
 * Not done in timebase_supervisor: 500 ms is exactly 30 periods, the same phase each time. */
void timebase_poll_cable(void)
{
    if (cable_mode() && !cable_in) cable_levels |= gpio_read(PIN_CABLE_IN) ? 2 : 1;
}

/* lhbs, slave-cable: unlocked on the cable for CABLE_LISTEN_MS (a loss restarts it),
 * follow optically; back to the cable once its pin toggles, even from an optical lock.
 * Failed tries back off, so a cable that toggles but never locks can't keep breaking
 * the optical lock. */
static void cable_switch(void)
{
    static uint32_t lost_seen;
    const bool signal = cable_levels == 3;
    cable_levels = 0;
    if (!cable_mode()) return;
    if (cable_lost_count != lost_seen) {
        lost_seen = cable_lost_count;
        input_since_ms = sys_ms();
    }
    const uint32_t held = sys_ms() - input_since_ms;
    bool to_cable;
    if (cable_in) {
        if (cable_locked) { cable_misses = 0; return; }
        if (held < CABLE_LISTEN_MS) return;
        if (cable_misses < CABLE_BACKOFF_MAX) cable_misses++;
        to_cable = false;
    } else {
        if (!signal) return;
        uint32_t wait = CABLE_RETRY_MS;
        for (uint8_t i = 1; i < cable_misses && wait < CABLE_BACKOFF_MAX * CABLE_RETRY_MS; i++)
            wait *= 2;
        if (wait > CABLE_BACKOFF_MAX * CABLE_RETRY_MS) wait = CABLE_BACKOFF_MAX * CABLE_RETRY_MS;
        if (held < wait) return;
        to_cable = true;
    }
    uint32_t s = irq_save();
    TMR_CT32B1->CCR = 0;            /* no capture on the old input against reset state */
    reset_sync_state();
    P.corr_status = to_cable ? CS_DISABLED : CS_SEARCHING;
    irq_restore(s);
    route_capture(to_cable);
}

void timebase_dump_trace(void (*out)(const char *fmt, ...))
{
    out("cable: input %s  %s  edges %u  lost %u  failed tries %u  next %s\n",
        cable_in ? "CABLE" : "optical", cable_locked ? "LOCKED" : "no signal",
        cable_edges, cable_lost_count, cable_misses, cable_tdm ? "sweep" : "skip");
    out("cable: stray rises %u  stray falls %u  captures %u  (trace: R rise, W width, L lost)\n",
        cable_stray_rises, cable_stray_falls, cable_caps);
    out("guards %u, trace (oldest first): kind e extra a b\n", tb_guard_count);
    for (int i = 0; i < 16; i++) {
        tb_trace_t *x = &trace[(trace_pos + i) & 15];
        if (x->kind) out("  %c %d %d %u %u\n", x->kind, x->e, x->extra, x->a, x->b);
    }
}

uint32_t timebase_rev(void) { return rev; }
uint8_t  timebase_sf_confirm(void) { return sf_confirm; }

const char *timebase_corr_state(void) { return corr_state_names[P.corr_status < 5 ? P.corr_status : 0]; }
uint32_t timebase_pulses_seen(void) { return pulses_seen; }
uint32_t timebase_pulses_used(void) { return pulses_used; }
uint32_t timebase_last_width(void) { return last_width; }
int32_t  timebase_phase_err(void) { return last_phase_err; }

static bool tdm_mode(void)
{
    return P.timebase_mode == TB_MASTER_TDM || P.timebase_mode == TB_SLAVE_OPTICAL ||
           P.timebase_mode == TB_SLAVE_CABLE;
}

/* stock 0x1000066D: timebase permits emission - masters always, slaves once
 * the correlator is locked to a master. */
bool timebase_permits_emission(void)
{
    /* Stock lets a cable slave emit unlocked, colliding with the master; lhbs waits. */
    if (on_cable()) return cable_locked;
    if (slave_mode())
        /* Not sf_confirm (that gates sweeping, multi_sweeps_now): a slave that stopped
         * flashing whenever the round changed under it dropped out of the master's
         * round, which changed it back - joining took minutes (4 stations). */
        return P.corr_status == CS_LOCKED && (!multi_active() || phase_good >= 20);
    return true;
}
static volatile bool running;

/* Stock timers_sync @0xCFA4: stop, reset and restart CT32B0 and CT32B1 back to
 * back so rotor captures (CT32B0) and timebase edges (CT32B1) share one clock. */
void timers_sync_start(void)
{
    TMR_CT32B0->TCR = 0;
    TMR_CT32B1->TCR = 0;
    TMR_CT32B0->TCR = 2; while (TMR_CT32B0->TC) {}
    TMR_CT32B1->TCR = 2; while (TMR_CT32B1->TC) {}
    TMR_CT32B0->TCR = 0;
    TMR_CT32B1->TCR = 0;
    TMR_CT32B0->TCR = 1;
    TMR_CT32B1->TCR = 1;
}

void timebase_init(void)
{
    SYSAHBCLKCTRL |= AHB_CT32B1;
    lpc_timer_t *t = TMR_CT32B1;
    t->TCR = 0;
    t->PR = 0;
    t->MCR = 0;
    t->EMR = 0;                     /* sync out: set per mode by timebase_select_input */
    t->CCR = 0;                     /* optical capture enabled in phase 3 */
    t->IR = 0xFF;
    period_cur = P.timebase_period ? P.timebase_period : 800000;
    timebase_select_input();
    nvic_set_priority(19, 1);       /* stock: CT32B1 priority 1 */
    NVIC_ISER = 1u << 19;
}

void timebase_start(void)
{
    lpc_timer_t *t = TMR_CT32B1;
    t->MR[1] = t->TC + period_cur;
    t->IR = IR_MR(1) | IR_MR(3);
    t->MCR = MCR_I(1) | (sync_out_on ? MCR_I(3) : 0);
    running = true;
}

/* Stock timebase_mr1_isr @0xCC8C, master-async case: re-arm one period later,
 * feed both rotor PLLs a reference edge, run the rotor control loop.
 * (Sync gate P1_0 is deliberately not touched in phase 2.) */
void CT32B1_IRQHandler(void)
{
    lpc_timer_t *t = TMR_CT32B1;
    uint32_t ir = t->IR;
    t->IR = ir;
    if (ir & IR_CR(0)) {
        if (!cable_in) optical_edge(t->CR[0]);
        else if (cable_mode()) { cable_caps++; cable_edge(t->CR[0]); }
    }
    if (ir & IR_MR(3)) t->EMR &= ~(1u << 1);   /* sync out pulse ends (stock MR3 isr @0xCA9C) */
    if (!(ir & IR_MR(1))) return;
    uint32_t now = t->MR[1];
    int32_t extra = mr1_extra;
    uint32_t next = now + period_cur + extra;   /* stock: phase correction consumed */
    /* lhbs guard: a correction beyond half a period, or a next edge already in
     * the past, would stall the timebase until the 32-bit counter wraps */
    if (extra > (int32_t)(period_cur / 2) || extra < -(int32_t)(period_cur / 2) ||
        (int32_t)(next - t->TC) < 1000) {
        tb_guard_count++;
        next = t->TC + period_cur;
    }
    t->MR[1] = next;
    if (!cable_in) tb_trace('E', 0, extra, now, next);  /* on the cable: keep the ring for R/W/L */
    mr1_extra = 0;
    {
        /* sync out width, from the skip flag before it alternates below */
        uint32_t w = period_cur >> 1;
        if (!multi_active() && P.timebase_mode == TB_MASTER_TDM)
            w = tdm_flag ? w + CABLE_TDM_WIDTH : w - CABLE_TDM_WIDTH;
        else if (!multi_active() && P.timebase_mode == TB_SLAVE_OPTICAL)
            w = tdm_flag ? w - CABLE_TDM_WIDTH : w + CABLE_TDM_WIDTH;
        t->MR[3] = now + w;
    }
    /* signed: a cable edge handled in this same interrupt can be later than `now` */
    if (cable_locked && (int32_t)(now - cable_good) > (int32_t)(CABLE_LOST_PERIODS * period_cur)) {
        cable_locked = false;
        cable_lost_count++;
        tb_trace('L', (int32_t)(now - cable_good), 0, now, cable_good);
        period_cur = P.timebase_period ? P.timebase_period : 800000;
    }
    if (slave_mode() && !cable_in) corr_tick(now);
    edges++;
    last_edge = now;
    if (cable_locked && cable_mode()) return;   /* the cable's edges drive the rotors and the skip flag */
    if (multi_active()) {
        rev++;
        multi_rev_start();
        if (rev % (uint32_t)multi_count() == 0) {
            const int before = multi_count();
            multi_superframe();
            if (multi_count() != before) rev = 0;   /* the new round starts here */
        }
        /* ALL: every station flashes every revolution (and sends an OOTX bit
         * each time). MASTER+ACTIVE: the master every revolution, others only
         * when they sweep; the gate is set ahead of this revolution's index
         * edges. Measured (openhouse captures/ms-a): the gate does not stop a
         * slave's flash, so MASTER+ACTIVE slaves still flash, with no OOTX bit. */
        flash_rev = MC->policy == MP_ALL || MC->slot == 0 || multi_sweeps_now() ||
                    !multi_mask_known() || !(multi_mask_live() & (1u << MC->slot));  /* seen to join */
        gpio_write(PIN_SYNC_GATE, emitting && flash_rev);
    } else {
        tdm_flag = tdm_mode() ? !tdm_flag : true;   /* stock: alternate in TDM modes */
        flash_rev = true;
    }
    rotor_reference_edge(now, P.timebase_offset);
    rotor_control();
}

uint32_t timebase_period(void) { return period_cur; }
uint32_t timebase_edges(void) { return edges; }

int timebase_skip(void) { return multi_active() ? !multi_sweeps_now() : !tdm_flag; }
bool timebase_flash_this_rev(void) { return flash_rev; }

/* stock task @0xCFE8, every 500 ms: re-arm a stalled timebase, enter stand-by
 * after 60 s without timebase lock. */
void timebase_supervisor(void)
{
    lpc_timer_t *t = TMR_CT32B1;
    if (TMR_CT32B1->TC - last_edge > 480000000u) {     /* 10 s: "timebase error, kicking" */
        t->MR[1] = t->TC + period_cur;
        last_edge = t->TC;
    }
    /* lhbs: a slave's receiver keeps listening in stand-by (rotors and emitter off,
     * as while searching): the master's flash back, locked onto, wakes it. Stock
     * waits for the button (or Bluetooth), so a master that came back after its slaves
     * timed out (reflashed, power cycled) left them all dark. Nor does a slave go to
     * stand-by while it hears a master (syncing, or locked and settling in). */
    if (cable_in && !cable_mode()) timebase_select_input();     /* mode changed by `set` */
    cable_switch();
    const bool hears = cable_locked ||
                       (slave_mode() && (P.corr_status == CS_SYNCING || P.corr_status == CS_LOCKED));
    if (P.sys_standby && (cable_locked || (slave_mode() && P.corr_status == CS_LOCKED)))
        timebase_exit_standby();
    if (timebase_permits_emission() || P.sys_standby || hears) last_lock_ms = sys_ms();
    else if (sys_ms() - last_lock_ms > 60000u) P.sys_standby = true;
}

void timebase_exit_standby(void)
{
    P.sys_standby = false;
    last_lock_ms = sys_ms();
}

volatile bool sync_requested = true;   /* stock: on whenever conditions allow */

bool emission_active(void) { return emitting; }

bool emission_update(void)
{
    bool locked = rotor[0].locked && rotor[1].locked;
    bool on = sync_requested && rotors_armed && P.sys_emission_enable && P.ootx_enable &&
              !P.sys_standby && timebase_permits_emission() &&
              (locked || !P.ootx_lock_required);
    gpio_write(PIN_EMITTER_PWR_N, !on);
    /* Stock clears P1_0 as slave-cable (timebase_mr1_isr @0xCD36); raised, it silences
     * our own cable input (ai/firmware/HARDWARE.md, P1_0). */
    gpio_write(PIN_SYNC_GATE, on && flash_rev && !on_cable());
    carrier_enable(on && P.carrier_enable);
    emitting = on;
    return on;
}
