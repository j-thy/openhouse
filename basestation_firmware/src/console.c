/* USB debug console (lhbs addition; stock has a UART console we leave out). */
#include <stdarg.h>
#include <string.h>
#include "console.h"
#include "usb_cdc.h"
#include "params.h"
#include "modes.h"
#include "iap.h"
#include "sys.h"
#include "board.h"
#include "rotor.h"
#include "timebase.h"
#include "ootx.h"
#include "laser.h"
#include "uart_sniff.h"
#include "multi.h"

/* ---- tiny printf: %d %u %x %X %s %c %f (%.Nf), width + '0' pad ---- */
static void out(const char *s, unsigned n) { usb_cdc_write(s, n); }

static void put_uint(uint32_t v, unsigned base, int width, char pad, bool upper)
{
    char b[12]; int n = 0;
    const char *dig = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    do { b[n++] = dig[v % base]; v /= base; } while (v);
    while (n < width) b[n++] = pad;
    while (n) out(&b[--n], 1);
}

static void put_float(float f, int prec)
{
    if (f != f) { out("nan", 3); return; }
    if (f < 0) { out("-", 1); f = -f; }
    uint32_t scale = 1;
    for (int i = 0; i < prec; i++) scale *= 10;
    uint32_t ip = (uint32_t)f;
    uint32_t fp = (uint32_t)((f - ip) * scale + 0.5f);
    if (fp >= scale) { ip++; fp -= scale; }
    put_uint(ip, 10, 0, ' ', false);
    if (prec) { out(".", 1); put_uint(fp, 10, prec, '0', false); }
}

void con_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    for (const char *p = fmt; *p; p++) {
        if (*p != '%') {
            if (*p == '\n') out("\r", 1);
            out(p, 1);
            continue;
        }
        p++;
        char pad = ' '; int width = 0, prec = 6;
        if (*p == '0') { pad = '0'; p++; }
        while (*p >= '0' && *p <= '9') width = width * 10 + *p++ - '0';
        if (*p == '.') { p++; prec = 0; while (*p >= '0' && *p <= '9') prec = prec * 10 + *p++ - '0'; }
        switch (*p) {
        case 'd': { int v = va_arg(ap, int); if (v < 0) { out("-", 1); v = -v; } put_uint(v, 10, width, pad, false); break; }
        case 'u': put_uint(va_arg(ap, unsigned), 10, width, pad, false); break;
        case 'x': put_uint(va_arg(ap, unsigned), 16, width, pad, false); break;
        case 'X': put_uint(va_arg(ap, unsigned), 16, width, pad, true); break;
        case 'c': { char c = va_arg(ap, int); out(&c, 1); break; }
        case 's': { const char *s = va_arg(ap, const char *); int n = strlen(s); out(s, n); while (n++ < width) out(" ", 1); break; }
        case 'f': put_float((float)va_arg(ap, double), prec); break;
        case '%': out("%", 1); break;
        }
    }
    va_end(ap);
}

/* ---- minimal number parsing (newlib's strto* would pull in malloc/syscalls) ---- */
static bool parse_long(const char *s, long *out)
{
    bool neg = false; long v = 0; int base = 10, nd = 0;
    while (*s == ' ') s++;
    if (*s == '-' || *s == '+') neg = *s++ == '-';
    if (s[0] == '0' && (s[1] | 0x20) == 'x') { base = 16; s += 2; }
    for (;; s++, nd++) {
        int d;
        if (*s >= '0' && *s <= '9') d = *s - '0';
        else if (base == 16 && (*s | 0x20) >= 'a' && (*s | 0x20) <= 'f') d = (*s | 0x20) - 'a' + 10;
        else break;
        v = v * base + d;
    }
    if (!nd || *s) return false;
    *out = neg ? -v : v;
    return true;
}

static bool parse_float(const char *s, float *out)
{
    bool neg = false; float v = 0, scale = 1; int nd = 0, ex = 0;
    if (*s == '-' || *s == '+') neg = *s++ == '-';
    for (; *s >= '0' && *s <= '9'; s++, nd++) v = v * 10 + (*s - '0');
    if (*s == '.') for (s++; *s >= '0' && *s <= '9'; s++, nd++) { scale /= 10; v += (*s - '0') * scale; }
    if ((*s | 0x20) == 'e') {
        long e; if (!parse_long(s + 1, &e)) return false;
        ex = e; s += strlen(s);
    }
    if (!nd || *s) return false;
    while (ex > 0) { v *= 10; ex--; }
    while (ex < 0) { v /= 10; ex++; }
    *out = neg ? -v : v;
    return true;
}

static char *next_tok(char **p)
{
    while (**p == ' ') (*p)++;
    if (!**p) return 0;
    char *t = *p;
    while (**p && **p != ' ') (*p)++;
    if (**p) *(*p)++ = 0;
    return t;
}

static unsigned parse_uint_or(const char *s, unsigned dflt)
{
    long v;
    return s && parse_long(s, &v) && v >= 0 ? (unsigned)v : dflt;
}

/* ---- parameter value printing / parsing ---- */
static const char *const tb_mode_names[] = { ENUM_E5CC_NAMES };
static const char *const tb_src_names[]  = { ENUM_E5E4_NAMES };
static const char *const corr_names[]    = { ENUM_11E7C_NAMES };
static const char *const ootx_names[]    = { ENUM_E580_NAMES };

static const char *const *enum_for(const pdesc_t *d, int *count)
{
    if (strstr(d->name, "timebase.mode")) { *count = 5; return tb_mode_names; }
    if (strstr(d->name, "timebase.source")) { *count = 3; return tb_src_names; }
    if (strstr(d->name, "corr.status")) { *count = 5; return corr_names; }
    if (strstr(d->name, ".ootx")) { *count = 3; return ootx_names; }
    *count = 0;
    return 0;
}

static void print_value(const pdesc_t *d)
{
    switch (d->type) {
    case PT_BOOL:  con_printf("%s", *(bool *)d->ptr ? "true" : "false"); break;
    case PT_U8:    con_printf("%u", *(uint8_t *)d->ptr); break;
    case PT_U16:   con_printf("%u", *(uint16_t *)d->ptr); break;
    case PT_U32:   con_printf("%u", *(uint32_t *)d->ptr); break;
    case PT_I8:    con_printf("%d", *(int8_t *)d->ptr); break;
    case PT_I16:   con_printf("%d", *(int16_t *)d->ptr); break;
    case PT_I32:   con_printf("%d", *(int32_t *)d->ptr); break;
    case PT_FLOAT: con_printf("%.6f", *(float *)d->ptr); break;
    case PT_STRING: con_printf("'%s'", (char *)d->ptr); break;
    case PT_ENUM: {
        int n; const char *const *names = enum_for(d, &n);
        uint8_t v = *(uint8_t *)d->ptr;
        if (v < n) con_printf("%s", names[v]); else con_printf("%u", v);
        break;
    }
    default: break;
    }
}

static const pdesc_t *find_param(const char *name)
{
    for (unsigned i = 0; i < param_count; i++)
        if (param_desc[i].name[0] && !strcmp(param_desc[i].name, name)) return &param_desc[i];
    return 0;
}

static bool set_value(const pdesc_t *d, const char *v)
{
    if (d->type == PT_FLOAT) {
        float f;
        if (!parse_float(v, &f)) return false;
        *(float *)d->ptr = f;
        return true;
    }
    if (d->type == PT_STRING) {
        strncpy((char *)d->ptr, v, d->size - 1);
        ((char *)d->ptr)[d->size - 1] = 0;
        return true;
    }
    long x;
    if (d->type == PT_BOOL && (!strcmp(v, "true") || !strcmp(v, "false"))) x = v[0] == 't';
    else if (d->type == PT_ENUM && !(v[0] >= '0' && v[0] <= '9')) {
        int n; const char *const *names = enum_for(d, &n);
        for (x = 0; x < n && strcmp(names[x], v); x++) {}
        if (x == n) return false;
    } else if (!parse_long(v, &x)) {
        return false;
    }
    switch (d->size) {
    case 1: *(uint8_t *)d->ptr = x; break;
    case 2: *(uint16_t *)d->ptr = x; break;
    case 4: *(uint32_t *)d->ptr = x; break;
    default: return false;
    }
    return true;
}

/* Shared with the UART console (Bluetooth chip types stock commands there). */
bool console_param_set(const char *name, const char *value)
{
    const pdesc_t *d = find_param(name);
    return d && set_value(d, value);
}

/* Value in stock 'param get' format (cparam print @0x7248): bools true/false,
 * integers %u/%d, enums by number, strings %s. */
bool console_param_format(const char *name, char *out, unsigned len)
{
    const pdesc_t *d = find_param(name);
    if (!d || len < 16) return false;
    long v = 0; bool sgn = false;
    switch (d->type) {
    case PT_BOOL:   strcpy(out, *(bool *)d->ptr ? "true" : "false"); return true;
    case PT_STRING: strncpy(out, d->ptr, len - 1); out[len - 1] = 0; return true;
    case PT_U8: case PT_ENUM: v = *(uint8_t *)d->ptr; break;
    case PT_U16:    v = *(uint16_t *)d->ptr; break;
    case PT_U32:    v = (long)*(uint32_t *)d->ptr; break;
    case PT_I8:     v = *(int8_t *)d->ptr; sgn = true; break;
    case PT_I16:    v = *(int16_t *)d->ptr; sgn = true; break;
    case PT_I32:    v = *(int32_t *)d->ptr; sgn = true; break;
    case PT_FLOAT:  v = (long)(*(float *)d->ptr * 1000.0f); sgn = true; break;  /* rough */
    default: return false;
    }
    char tmp[16]; int n = 0; unsigned long u;
    bool neg = sgn && v < 0;
    u = neg ? (unsigned long)-v : (unsigned long)v;
    if (d->type == PT_U32) u = *(uint32_t *)d->ptr;
    do { tmp[n++] = '0' + u % 10; u /= 10; } while (u);
    int o = 0;
    if (neg) out[o++] = '-';
    while (n) out[o++] = tmp[--n];
    out[o] = 0;
    return true;
}

/* ---- commands ---- */
static void cmd_params(const char *pat)
{
    for (unsigned i = 0; i < param_count; i++) {
        const pdesc_t *d = &param_desc[i];
        if (!d->name[0] || (pat && !strstr(d->name, pat))) continue;
        con_printf("%s%s = ", d->name, (d->flags & PF_RUNTIME) ? " [rt]" : "");
        print_value(d);
        con_printf("\n");
    }
}

static void cmd_eeprom(const char *a, const char *l)
{
    unsigned addr = parse_uint_or(a, 0), len = parse_uint_or(l, 256);
    if (addr >= EEPROM_SIZE) return;
    if (addr + len > EEPROM_SIZE) len = EEPROM_SIZE - addr;
    uint8_t b[16];
    for (unsigned o = 0; o < len; o += 16) {
        unsigned n = len - o < 16 ? len - o : 16;
        if (iap_eeprom_read(addr + o, b, n)) { con_printf("read error\n"); return; }
        con_printf("%03x:", addr + o);
        for (unsigned i = 0; i < n; i++) con_printf(" %02x", b[i]);
        con_printf("\n");
    }
}

static void rotor_line(int r)
{
    rotor_state_t *s = &rotor[r];
    uint32_t per = s->last_period, exp = timebase_period();
    int32_t diff = (int32_t)(per - exp);
    float ppm = per ? (float)diff / (float)exp * 1e6f : 0;
    float hz = per ? 48e6f / (float)per : 0;
    con_printf("r%d %s%s%s  %.3f Hz  %d ppm  coarse %u fine %u  pll err %d out %d  lock %u%s\n",
               r, s->fault ? "FAULT " : "", s->running ? "run" : "stop", s->locked ? " LOCK" : "",
               hz, (int)ppm, s->mr_coarse, s->mr_fine, s->pll_err, s->pll_out, s->lock_count,
               s->saturated ? " SAT" : "");
}

static void cmd_rotor(void)
{
    con_printf("armed %s  timebase period %u  edges %u\n", rotors_armed ? "yes" : "no",
               timebase_period(), timebase_edges());
    rotor_line(0);
    rotor_line(1);
}

static void cmd_watch(const char *secs)
{
    unsigned n = parse_uint_or(secs, 10) * 2;
    for (unsigned i = 0; i < n; i++) {
        con_printf("[%u] ", sys_ms() / 1000);
        rotor_line(0);
        con_printf("    ");
        rotor_line(1);
        uint32_t t = sys_ms();
        while (sys_ms() - t < 500)
            if (usb_cdc_getc() >= 0) return;    /* any key stops */
    }
}

static void cmd_ootx(void)
{
    extern bool carrier_ok;
    con_printf("serial 0x%08X  frame %u bits  frames sent %u  late syncs %u  carrier %s %u Hz\n",
               ootx_serial(), ootx_bits(), ootx_frames_sent(), sync_late_count,
               carrier_ok ? "ok" : "I2C FAIL", P.carrier_frequency);
    const uint8_t *f = ootx_frame();
    con_printf("frame:");
    for (unsigned i = 0; i < (ootx_bits() + 7) / 8; i++) con_printf(" %02x", f[i]);
    con_printf("\n");
}

static void cmd_laser_status(void)
{
    con_printf("ISL58303 %s (id 0x%02x)  requested %s  emission %s\n",
               laser_driver_ok() ? "ok" : "FAIL", laser_driver_id(), lasers_requested ? "yes" : "no",
               emission_active() ? "on" : "off");
    con_printf("regs: 07=%02x 03=%02x  ch1(laser0) 22=%02x 23=%02x 26=%02x 27=%02x  ch0(laser1) 12=%02x 13=%02x 16=%02x 17=%02x\n",
               isl_read(7), isl_read(3), isl_read(0x22), isl_read(0x23), isl_read(0x26), isl_read(0x27),
               isl_read(0x12), isl_read(0x13), isl_read(0x16), isl_read(0x17));
    con_printf("sweeps: laser0 %u  laser1 %u  safety kills %u%s\n",
               laser_on_count[0], laser_on_count[1], laser_kill_count, laser_fault ? "  FAULT" : "");
    for (int n = 0; n < 2; n++)
        con_printf("laser%d: adc %u  detected %.1f  power %.2f mW (target %.1f)  current %u  apc %s\n", n,
                   apc_adc[n], apc_detected[n], apc_power[n], n ? P.laser_1_pwr : P.laser_0_pwr,
                   n ? P.laser_1_current : P.laser_0_current, apc_enabled ? "on" : "monitor");
}

static void cmd_corr(void)
{
    con_printf("mode %s  correlator %s  est period %.1f  missed %u\n",
               P.timebase_mode < 5 ? tb_mode_names[P.timebase_mode] : "?", timebase_corr_state(),
               P.corr_period, P.corr_missed);
    con_printf("optical pulses seen %u  used %u  last width (optical or cable) %u ticks (%.1f us)  phase err %d ticks\n",
               timebase_pulses_seen(), timebase_pulses_used(), timebase_last_width(),
               (float)timebase_last_width() / 48.0f, timebase_phase_err());
    timebase_dump_trace(con_printf);
}

static void cmd_multi(char *a1, char *a2)
{
    if (a1 && !strcmp(a1, "enable") && a2) {
        MC->enable = !strcmp(a2, "on");
    } else if (a1 && !strcmp(a1, "n")) {
        con_printf("n is automatic: the master counts the stations it sees\n");
        return;
    } else if (a1 && !strcmp(a1, "slot") && a2) {
        long s;
        if (!parse_long(a2, &s) || s < 0 || s >= MULTI_SLOTS) { con_printf("slot must be 0..3 (shown as 1..4)\n"); return; }
        multi_enter((int)s);
    } else if (a1 && !strcmp(a1, "policy") && a2) {
        if (!strcmp(a2, "all")) MC->policy = MP_ALL;
        else if (!strcmp(a2, "master+active")) MC->policy = MP_MASTER_ACTIVE;
        else { con_printf("policy must be all or master+active\n"); return; }
        if (multi_active()) multi_enter(MC->slot);
    } else if (a1 && !strcmp(a1, "off")) {
        multi_leave();
        mode_apply(P.mode_current % mode_count());
    }
    if (a1) {                                   /* any change: apply and persist */
        timebase_select_input();
        console_mode_changed();
        laser_boot_currents_for_save(true);
        params_save(0);
        laser_boot_currents_for_save(false);
    }
    con_printf("multi %s  enable %s  slot %u (label %c)  stagger %u ticks  policy %s\n",
               multi_active() ? "ACTIVE" : "off", MC->enable ? "on" : "off", MC->slot,
               multi_label(), MC->stagger, MC->policy == MP_MASTER_ACTIVE ? "master+active" : "all");
    if (multi_active()) {
        const uint8_t m = multi_mask_live();
        con_printf("round: slots");
        for (int s = 0; s < MULTI_SLOTS; s++) if (m & (1u << s)) con_printf(" %d", s + 1);
        con_printf("  (%d stations)%s  announced 0x%x  guard %u%s\n", multi_count(),
                   multi_mask_known() ? "" : "  [no round from the master yet: not sweeping]",
                   multi_mask_announced(), multi_guard(),
                   (m & (1u << MC->slot)) ? "" : "  [not in the round: flashing only]");
        con_printf("flashes seen (ms ago):");
        for (int s = 0; s < MULTI_SLOTS; s++) {
            const uint32_t a = multi_seen_age_ms(s);
            if (a == 0xFFFFFFFFu) con_printf("  %d -", s + 1); else con_printf("  %d %u", s + 1, a);
        }
        con_printf("\n");
        if (MC->slot) con_printf("master frames decoded %u  bad %u  from a slave (relocked) %u  correlator %s\n", multi_frames_decoded(), multi_frames_bad(), multi_frames_wrong(), timebase_corr_state());
        if (MC->slot) con_printf("turns swept without that revolution's master flash %u\n", multi_blind_sweeps());
        con_printf("rev %u (turn %u)  this rev: %s, %s  superframe confirms %u  realigns %u  ootx mode 0x%02x\n",
                   timebase_rev(), timebase_rev() % (uint32_t)multi_count(), timebase_skip() ? "skip" : "SWEEP",
                   timebase_flash_this_rev() ? "flash" : "dark", timebase_sf_confirm(), sf_realign_count,
                   multi_ootx_mode());
    }
}

static void cmd_help(void)
{
    con_printf("lhbs console\n"
               "  status                 firmware/channel/params summary\n"
               "  params [pattern]       list parameters ([rt] = runtime, not saved)\n"
               "  get <name>             show one parameter\n"
               "  set <name> <value>     change a parameter in RAM\n"
               "  save                   write parameters to EEPROM (stock format)\n"
               "  next                   next channel (same as the mode button)\n"
               "  spin / stop            arm or stop the rotor motors (lasers stay off)\n"
               "  rotor                  rotor/PLL status\n"
               "  sync on|off            IR sync flashes (only while rotors are locked)\n"
               "  ootx                   OOTX payload/frame status\n"
               "  corr                   optical slave correlator status\n"
               "  multi [enable on|off | slot 0-3 | policy all|master+active | off]  (N: automatic)\n"
               "  uartlog                bytes received on the stock UART (Bluetooth chip?)\n"
               "  laser on|off           laser sweeps (needs sync on + both rotors locked)\n"
               "  laser                  laser driver status + measured power\n"
               "  apc on|off             laser power regulation (default: monitor only)\n"
               "  watch [s]              stream rotor status every 0.5 s (any key stops)\n"
               "  eeprom [addr] [len]    hex dump EEPROM\n"
               "  isp                    enter USB bootloader\n"
               "  reset                  reboot\n");
}

static int load_rc, load_part;
void console_set_boot_info(int rc, int part) { load_rc = rc; load_part = part; }

static void cmd_status(void)
{
    con_printf("lhbs " __DATE__ " " __TIME__ "\n");
    con_printf("params: load rc %d from partition %d, payload %u bytes\n",
               load_rc, load_part, params_payload_size());
    con_printf("name %s  channel %c (mode.current %u of %u)  %s\n", P.sys_name, mode_label(),
               P.mode_current, P.mode_count, mode_matches_live() ? "matches table" : "custom config");
    con_printf("timebase.mode %s  period %u  offset %d  carrier %u Hz\n",
               P.timebase_mode < 5 ? tb_mode_names[P.timebase_mode] : "?", P.timebase_period,
               P.timebase_offset, P.carrier_frequency);
    extern uint32_t boot_mailbox, boot_rststat, _mailbox;
    extern bool accel_ok, carrier_ok;
    extern float accel_f[3];
    con_printf("emission %s (timebase %s)  standby %s  faults 0x%02x  carrier %s  accel %s\n",
               emission_active() ? "ON" : "off", timebase_permits_emission() ? "master" : "waiting for master",
               P.sys_standby ? "YES" : "no", P.sys_faults, carrier_ok ? "ok" : "FAIL", accel_ok ? "ok" : "FAIL");
    con_printf("gravity %s  |a| %.2f m/s2  dir %d %d %d  (%.2f %.2f %.2f)\n",
               P.accel_is_gravity ? "yes" : "no", P.accel_magnitude, P.accel_dir_x, P.accel_dir_y,
               P.accel_dir_z, accel_f[0], accel_f[1], accel_f[2]);
    con_printf("uptime %u ms  reset status 0x%02x  boot mailbox 0x%08x (now 0x%08x @%x)\n",
               sys_ms(), boot_rststat, boot_mailbox, _mailbox, (unsigned)&_mailbox);
}

static void handle(char *line)
{
    char *p = line;
    char *cmd = next_tok(&p);
    char *a1 = next_tok(&p);
    while (*p == ' ') p++;
    char *a2 = *p ? p : 0;      /* rest of line (values may contain spaces) */
    if (!cmd) return;
    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) cmd_help();
    else if (!strcmp(cmd, "status")) cmd_status();
    else if (!strcmp(cmd, "params")) cmd_params(a1);
    else if (!strcmp(cmd, "get") && a1) {
        const pdesc_t *d = find_param(a1);
        if (!d) { con_printf("no such parameter\n"); return; }
        print_value(d); con_printf("\n");
    } else if (!strcmp(cmd, "set") && a1 && a2) {
        const pdesc_t *d = find_param(a1);
        if (!d) { con_printf("no such parameter\n"); return; }
        if (!set_value(d, a2)) { con_printf("bad value\n"); return; }
        con_printf("%s = ", d->name); print_value(d); con_printf("  (RAM only, 'save' to keep)\n");
    } else if (!strcmp(cmd, "save")) {
        int part; cp_result_t r = params_save(&part);
        con_printf(r ? "save failed: %d\n" : "saved to partition %d\n", r ? r : part);
    } else if (!strcmp(cmd, "next")) {
        mode_next();
        timebase_select_input();
        con_printf("channel %c\n", mode_label());
        console_mode_changed();
    } else if (!strcmp(cmd, "eeprom")) cmd_eeprom(a1, a2);
    else if (!strcmp(cmd, "spin")) {
        rotor[0].fault = rotor[1].fault = false;
        rotors_armed = true;
        con_printf("rotors armed\n");
    } else if (!strcmp(cmd, "stop")) { rotor_stop_all(); con_printf("rotors stopped\n"); }
    else if (!strcmp(cmd, "rotor")) cmd_rotor();
    else if (!strcmp(cmd, "ootx")) cmd_ootx();
    else if (!strcmp(cmd, "corr")) cmd_corr();
    else if (!strcmp(cmd, "multi")) cmd_multi(a1, a2);
    else if (!strcmp(cmd, "uartlog")) uart_sniff_dump(con_printf);
    else if (!strcmp(cmd, "apc") && a1) {
        apc_enabled = !strcmp(a1, "on");
        con_printf("apc %s\n", apc_enabled ? "regulating" : "monitor only");
    } else if (!strcmp(cmd, "laser")) {
        if (a1) {
            lasers_requested = !strcmp(a1, "on");
            if (lasers_requested) laser_fault = false;     /* acknowledge APC fault */
            if (!lasers_requested) lasers_off();
            con_printf("lasers %s\n", lasers_requested ? "armed (sweep only while locked + emitting)" : "off");
        } else cmd_laser_status();
    }
    else if (!strcmp(cmd, "sync") && a1) {
        sync_requested = !strcmp(a1, "on");
        con_printf("sync %s%s\n", sync_requested ? "on" : "off",
                   sync_requested && !(rotor[0].locked && rotor[1].locked) ? " (waits for rotor lock)" : "");
    }
    else if (!strcmp(cmd, "watch")) cmd_watch(a1);
    else if (!strcmp(cmd, "isp")) { con_printf("entering ISP\n"); sys_delay_ms(50); iap_reinvoke_isp(); }
    else if (!strcmp(cmd, "reset")) { sys_delay_ms(50); SCB_AIRCR = 0x05FA0004; }
    else con_printf("unknown command, try 'help'\n");
}

void console_poll(void)
{
    static char line[80];
    static unsigned n;
    static bool was_connected;

    if (usb_cdc_connected() && !was_connected) {
        sys_delay_ms(20);
        con_printf("\nlhbs - Lighthouse 1.0 firmware rewrite. 'help' for commands\nlhbs> ");
    }
    was_connected = usb_cdc_connected();

    int c;
    while ((c = usb_cdc_getc()) >= 0) {
        if (c == '\r' || c == '\n') {
            con_printf("\n");
            line[n] = 0;
            handle(line);
            n = 0;
            con_printf("lhbs> ");
        } else if ((c == 8 || c == 127) && n) {
            n--; con_printf("\b \b");
        } else if (c >= ' ' && n < sizeof line - 1) {
            line[n++] = c;
            char ch = c; usb_cdc_write(&ch, 1);
        }
    }
}
