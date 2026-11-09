/* lhbs - readable reimplementation of the Lighthouse 1.0 base station firmware.
 * Stock reference: main @0xA52C and its periodic tasks (table set up in main):
 *   10 ms  accel_task        (stock @0x2B08)
 *  100 ms  ui_task           (stock @0xDADC: label, fault-driven OOTX rebuild)
 *  100 ms  laser_poll        (stock laser_update_task @0x9DE0: push APC current)
 *  500 ms  carrier_task      (stock @0x372C)
 *  500 ms  timebase_supervisor (stock @0xCFE8: stall kick, 60 s stand-by)
 * Interrupts: CT32B1 timebase (prio 1), CT32B0 rotor index/sync/laser events
 * (prio 2), USB console (prio 3), SysTick (LEDs, emission + laser watchdogs
 * every 20 ms). lhbs additions: USB console, extra laser safety layers. */
#include "board.h"
#include "sys.h"
#include "params.h"
#include "modes.h"
#include "ui.h"
#include "debug.h"
#include "usb_cdc.h"
#include "console.h"
#include "rotor.h"
#include "timebase.h"
#include "timers.h"
#include "i2c.h"
#include "carrier.h"
#include "ootx.h"
#include "laser.h"
#include "accel.h"
#include "uart_sniff.h"
#include "multi.h"

bool carrier_ok, accel_ok;
static volatile bool ui_ready;
static uint32_t carrier_programmed;
static uint8_t last_faults;

/* stock disp_update_mode_label @0xC2C0: label if the live config is a table entry */
static void show_channel(void)
{
    if (mode_matches_live()) disp_char(mode_label(), accel_flipped());
    else disp_clear();
}

void console_mode_changed(void) { show_channel(); ootx_build(); }

/* stock ui_update @0xC3C8 colour states; runs from SysTick every 20 ms so a
 * blocking console command can never freeze it */
void sys_tick_20ms(void)
{
    if (!ui_ready) return;
    emission_update();
    laser_watchdog();

    bool blink = (sys_ms() / 800) & 1;
    bool locked = rotor[0].locked && rotor[1].locked;
    bool sat = rotor[0].saturated || rotor[1].saturated;
    unsigned c0 = C_OFF, c1 = C_OFF;
    if (P.sys_faults || rotor[0].fault || rotor[1].fault)
        c0 = blink ? C_RED : C_OFF;                 /* fault: blinking red */
    else if (P.sys_standby)
        c1 = C_GREEN;                               /* stand-by: LED1 green */
    else if (!timebase_permits_emission())
        c0 = blink ? C_PURPLE : C_OFF;              /* waiting for a master */
    else
        c0 = locked && !sat ? C_GREEN : C_BLUE;
    led_set(0, c0);
    led_set(1, c1);
    gpio_write(PIN_IND_ROTOR0, rotor[0].running && rotor[0].locked);
    gpio_write(PIN_IND_ROTOR1, rotor[1].running && rotor[1].locked);
    gpio_write(PIN_IND_TIMEBASE, timebase_permits_emission());
    gpio_write(PIN_IND_STATUS_N, c0 != C_GREEN);
}

static void ui_task(void)                           /* 100 ms */
{
    static bool was_flipped;
    P.sys_faults = laser_fault ? 0x11 : 0;          /* stock bits: 0 laser0, 4 laser1 */
    if (P.sys_faults != last_faults) {              /* faults are part of OOTX */
        last_faults = P.sys_faults;
        ootx_build();
    }
    if (accel_flipped() != was_flipped) {
        was_flipped = accel_flipped();
        if (!P.sys_standby) show_channel();
    }
    /* stand-by (no master for 60 s): the channel display goes dark too, back on wake */
    static bool was_standby;
    if (P.sys_standby != was_standby) {
        was_standby = P.sys_standby;
        if (was_standby) disp_clear(); else show_channel();
    }
}

static void carrier_task(void)                      /* 500 ms */
{
    if (P.carrier_frequency != carrier_programmed) {
        carrier_ok = carrier_set(P.carrier_frequency);
        carrier_programmed = P.carrier_frequency;
    }
}

int main(void)
{
    sys_clock_init();
    sys_tick_init();
    ui_init();

    int part = -1;
    cp_result_t rc = params_load(&part);
    console_set_boot_info(rc, part);
    multi_init();

    /* snapshot before anything can write EEPROM (read back via the ISP drive) */
    if (debug_snapshot(rc, part, 0)) led_set(0, C_RED);
    debug_dump_rom();

    i2c_init();
    carrier_enable(false);
    carrier_task();                                  /* stock carrier_init @0x36B8 */
    accel_ok = accel_init();                         /* stock @0x2A70 */
    gpio_output(PIN(1, 5), P.ootx_stretch);          /* sync pulse stretching */
    for (int i = 0; i < 100; i++) { accel_task(); sys_delay_ms(10); }  /* settle filter */
    show_channel();
    ootx_build();
    laser_init();                                    /* ISL58303; enable pins stay low */

    /* stock order: timebase_init, rotor_reset, rotor_init (ends with timer sync) */
    timebase_init();
    rotor_init();
    timers_sync_start();
    timebase_start();

    usb_cdc_init();
    uart_sniff_init();
    ui_ready = true;

    uint32_t t10 = 0, t100 = 0, t500 = 0;
    for (;;) {
        uint32_t now = sys_ms();
        if (now - t10 >= 10)   { t10 = now;  accel_task(); timebase_poll_cable(); }
        if (now - t100 >= 100) { t100 = now; ui_task(); laser_poll(); multi_task(); }
        if (now - t500 >= 500) { t500 = now; carrier_task(); timebase_supervisor(); }

        if (button_poll()) {
            if (P.sys_standby) {
                timebase_exit_standby();            /* stock: a press only wakes up */
            } else {
                mode_next();                        /* stock: advance, apply, save */
                timebase_select_input();
                show_channel();
                carrier_task();
                ootx_build();
            }
        }
        if (rotor_lock_event) {                     /* stock rebuilds on every lock */
            rotor_lock_event = false;
            ootx_build();
        }
        multi_poll();
        if (multi_take_round_changed()) ootx_build();   /* announce the round we now sweep in */
        ootx_refresh_accel();
        console_poll();
        uart_sniff_poll();
    }
}
