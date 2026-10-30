
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

/* Isolate IMU task timing first. Set to 0 and rebuild for Buddy 1/4/5. */
#ifndef BUDDY4_TRIAL_IMU_ONLY
#define BUDDY4_TRIAL_IMU_ONLY 1
#endif
#if !BUDDY4_TRIAL_IMU_ONLY
#include "buddy1/buddy1.h"
#endif
#if !APP_BUDDY1_ONLY && !BUDDY4_TRIAL_IMU_ONLY
#include "buddy5/buddy5.h"
#endif
#if APP_BUDDY4_ENABLED && !APP_BUDDY1_ONLY
#include "buddy4/buddy4_task.h"
/* Core label functions use no SDK or standard size_t types. Keep the
 * standard-library core header separate from kernel headers. */
extern const char *buddy4_terrain_name(int state);
extern const char *buddy4_side_name(int state);
extern const char *buddy4_quality_name(int quality);
/* 0: readable status (default); 1: original numeric CSV for recording.
 * Rebuild this trial after changing the setting. No serial commands needed. */
#ifndef BUDDY4_CONSOLE_CSV
#define BUDDY4_CONSOLE_CSV 0
#endif
static buddy4_task_snapshot_t buddy4_diagnostic;
#if !BUDDY4_CONSOLE_CSV
/* tm_printf has no float format. Bounded decimal conversion avoids float
 * printf support and prints small negative angles correctly. */
static void buddy4_decimal(char out[16], float value)
{
    unsigned int hundredths, whole, divisor;
    unsigned int pos = 0;
    bool negative = value < 0.0f;
    if (!(value >= -9999.0f && value <= 9999.0f)) {
        out[0] = 'N'; out[1] = '/'; out[2] = 'A'; out[3] = 0;
        return;
    }
    if (negative) value = -value;
    hundredths = (unsigned int)(value * 100.0f + 0.5f);
    if (negative && hundredths != 0) out[pos++] = '-';
    whole = hundredths / 100;
    divisor = 1;
    while (whole / divisor >= 10) divisor *= 10;
    do {
        out[pos++] = (char)('0' + whole / divisor);
        whole %= divisor;
        divisor /= 10;
    } while (divisor != 0);
    out[pos++] = '.';
    out[pos++] = (char)('0' + (hundredths / 10) % 10);
    out[pos++] = (char)('0' + hundredths % 10);
    out[pos] = 0;
}

/* Print each newly observed completed side event once, never as NOW data.
 * The snapshot retains only the latest result; this is not a lossless queue. */
static uint32_t buddy4_reported_events;
static uint32_t buddy4_reported_impacts;
static void buddy4_print_new_peak(const buddy4_task_snapshot_t *s)
{
    if (s->completed_events == 0 || s->completed_events == buddy4_reported_events) return;
    char stable[16], median[16];
    if (s->last_event.stable_valid) buddy4_decimal(stable, s->last_event.stable_cm);
    if (s->last_event.median_valid) buddy4_decimal(median, s->last_event.median_cm);
    if (s->completed_events > buddy4_reported_events &&
        s->completed_events - buddy4_reported_events > 1u) {
        tm_printf((UB *)"[B4 RESULT] %u intermediate event(s) not captured by this slow monitor; latest result follows.\n",
            (UINT)(s->completed_events - buddy4_reported_events - 1u));
    }
    tm_printf((UB *)"[B4 RESULT] Completed side-tilt #%u | Side: %s | Ended at: %u s since boot | SAVED RESULT, not current tilt | OBSERVATIONAL TILT, NOT CONFIRMED HUMP CROSSING\n",
        (UINT)s->completed_events, buddy4_side_name(s->last_event.side),
        (UINT)(s->last_event.end_us / 1000000u));
    tm_printf((UB *)"[B4 RESULT] Stable peak (cm): %s | Median peak candidate (cm): %s | PRELIMINARY wheel-height difference\n",
        s->last_event.stable_valid ? stable : "UNCONFIRMED",
        s->last_event.median_valid ? median : "UNCONFIRMED");
    buddy4_reported_events = s->completed_events;
}

/* The 1 Hz monitor cannot reliably observe possible_impact's one-sample pulse.
 * Report changes in the retained count and timestamp instead. The snapshot keeps
 * only the latest timestamp, so a count jump reveals lost intermediate details. */
static void buddy4_print_new_impact(const buddy4_task_snapshot_t *s)
{
    if (s->impact_count == buddy4_reported_impacts) return;
    if (s->impact_count > buddy4_reported_impacts &&
        s->impact_count - buddy4_reported_impacts > 1u) {
        tm_printf((UB *)"[B4 POSSIBLE IMPACT] %u intermediate possible-impact event timestamp(s) not retained by this snapshot; latest follows.\n",
            (UINT)(s->impact_count - buddy4_reported_impacts - 1u));
    }
    tm_printf((UB *)"[B4 POSSIBLE IMPACT] Observational event #%u | Retained event time: %u.%03u s since boot | SAVED EVENT, NOT CURRENT DATA, NOT CONFIRMED COLLISION\n",
        (UINT)s->impact_count, (UINT)(s->impact_time_us / 1000000u),
        (UINT)((s->impact_time_us / 1000u) % 1000u));
    buddy4_reported_impacts = s->impact_count;
}

static void buddy4_print_readable(const buddy4_task_snapshot_t *s)
{
    char pitch[16], roll[16], height[16];
    if (s->valid && s->tilt_valid) {
        buddy4_decimal(pitch, s->pitch_deg);
        buddy4_decimal(roll, s->roll_deg);
        buddy4_decimal(height, s->height_cm);
        tm_printf((UB *)"[B4] NOW | IMU: OK | Terrain: %s | Side: %s | Pitch: %s deg | Roll: %s deg\n",
            buddy4_terrain_name(s->terrain), buddy4_side_name(s->side), pitch, roll);
        tm_printf((UB *)"     Estimated wheel height difference: %s cm (preliminary)\n", height);
    } else {
        /* Never present retained sensor readings as current measurements. */
        tm_printf((UB *)"[B4] NOW | IMU: INVALID for tilt / waiting for suitable data | Current tilt/height: N/A\n");
    }
    char ax[16], ay[16], az[16], mag[16];
    buddy4_decimal(ax, s->acceleration_g[0]);
    buddy4_decimal(ay, s->acceleration_g[1]);
    buddy4_decimal(az, s->acceleration_g[2]);
    buddy4_decimal(mag, s->magnitude_g);
    tm_printf((UB *)"     Acquisition fresh=%u | Tilt usable=%u | Quality=%s | Clipped axes=0x%X | STATUS_A=0x%02X\n",
        (UINT)s->valid, (UINT)s->tilt_valid, buddy4_quality_name(s->measurement_quality),
        (UINT)s->clipped_axes, (UINT)s->status_a);
    tm_printf((UB *)"     Raw XYZ=(%d,%d,%d) | Corrected g=(%s,%s,%s) | Magnitude=%s g | sample_ms=%u\n",
        (INT)s->raw[0], (INT)s->raw[1], (INT)s->raw[2], ax, ay, az, mag,
        (UINT)(s->sample_us / 1000u));
    tm_printf((UB *)"     Saved clipped samples=%u (impact assessment indeterminate) | Unsuitable tilt samples=%u\n",
        (UINT)s->clipped_samples, (UINT)s->unsuitable_samples);
    tm_printf((UB *)"     Sensor readback: verified=%u at_ms=%u WHO=0x%02X CTRL0/1/2/3/4/7=%02X/%02X/%02X/%02X/%02X/%02X\n",
        (UINT)s->sensor_config_valid, (UINT)(s->config_checked_us / 1000u), (UINT)s->who_am_i,
        (UINT)s->sensor_ctrl[0], (UINT)s->sensor_ctrl[1], (UINT)s->sensor_ctrl[2],
        (UINT)s->sensor_ctrl[3], (UINT)s->sensor_ctrl[4], (UINT)s->sensor_ctrl[5]);
    tm_printf((UB *)"     Timing: poll=%s | Reading age=%u ms | Poll interval=%u ms | Transfer time=%u ms | Timing valid=%u\n",
        s->poll_result == 0 ? "FRESH" : s->poll_result == 1 ? "NO_NEW_DATA" :
        s->poll_result == 2 ? "I2C_ERROR" : "NOT_READY",
        (UINT)s->sample_age_ms, (UINT)s->poll_interval_ms,
        (UINT)s->poll_duration_ms, (UINT)s->timing_valid);
    tm_printf((UB *)"     Possible-impact detector NOW: %s | Saved cumulative possible-impact count: %u\n",
        s->valid && s->clipped_axes ? "UNKNOWN_CLIPPED" : s->impact_valid ? "VALID" : "INVALID", (UINT)s->impact_count);
    if (s->impact_count != 0) {
        tm_printf((UB *)"     Saved latest possible-impact time: %u.%03u s since boot | OBSERVATIONAL, NOT CONFIRMED COLLISION\n",
            (UINT)(s->impact_time_us / 1000000u),
            (UINT)((s->impact_time_us / 1000u) % 1000u));
    }
    if (s->saved_error_present) {
        tm_printf((UB *)"     Saved IMU failure (previous): %s | Code=%d | %s register=0x%02X | Observed at=%u s | Time valid=%u\n",
            s->saved_error_code == E_TMOUT ? "TIMEOUT" :
            s->saved_error_code == E_IO ? "BUS_ERROR_OR_SHORT_TRANSFER" :
            s->saved_error_code < E_OK ? "DEVICE_ERROR" : "SENSOR_VERIFICATION_FAILED",
            (INT)s->saved_error_code, s->saved_error_read ? "READ" : "WRITE",
            (UINT)s->saved_error_register, (UINT)(s->saved_error_us / 1000000u),
            (UINT)s->saved_error_time_valid);
    }
    if (s->saved_error_present && s->saved_error_code == E_TMOUT) {
        const unsigned int *t = s->saved_timeout_trace;
        tm_printf((UB *)"     Timeout trace: state=%u read_left=%u write_left=%u raw=0x%X mask=0x%X abort=0x%X status=0x%X rx_fifo=%u\n",
            (UINT)t[0], (UINT)t[1], (UINT)t[2], (UINT)t[3], (UINT)t[4],
            (UINT)t[5], (UINT)t[6], (UINT)t[7]);
        tm_printf((UB *)"     IRQ trace (us from request): elapsed=%u last_irq=%u count=%u bits=0x%X done=%u entry_state=%u NVIC enabled=%u pending=%u\n",
            (UINT)t[8], (UINT)t[9], (UINT)t[10], (UINT)t[11],
            (UINT)t[12], (UINT)t[13], (UINT)t[14], (UINT)t[15]);
        tm_printf((UB *)"     Wait registration: at=%u us kernel=%u ms deadline=%u ms tick_remaining=%u us ICSR=0x%X\n",
            (UINT)t[16], (UINT)t[17], (UINT)t[18], (UINT)t[19], (UINT)t[20]);
        tm_printf((UB *)"     Wait ordering (us): expiry=%u expiry_kernel=%u ms release_ok=%u signal=%u resume=%u signal_rc=%d wait_rc=%d\n",
            (UINT)t[21], (UINT)t[22], (UINT)t[23], (UINT)t[24], (UINT)t[25], (INT)t[28], (INT)t[29]);
        tm_printf((UB *)"     IRQ gaps (us): max=%u first=%u | At expiry: state=%u read_left=%u\n",
            (UINT)t[26], (UINT)t[27], (UINT)t[30], (UINT)t[31]);
    }
    tm_printf((UB *)"     I2C errors: %u | Skipped sampling slots: %u | Build: B4-MEASUREMENT-QUALITY-I USB-only\n",
        (UINT)s->io_errors, (UINT)s->skipped_slots);
    buddy4_print_new_peak(s);
    buddy4_print_new_impact(s);
}
#endif

#endif

/*
 * =====================================================
 * APPLICATION ENTRY POINT
 * =====================================================
 *
 * µT-Kernel calls usermain() after system startup.
 *
 * The application follows two phases:
 *
 * Phase 1: Initialise all subsystem resources.
 * Phase 2: Start all subsystem tasks.
 *
 * Once the other buddies are implemented,
 * they will be integrated into both phases.
 */

EXPORT INT usermain(void)
{
    ER error_code;

    /*
     * =================================================
     * PHASE 1: SUBSYSTEM INITIALISATION
     * =================================================
     *
     * All kernel IPC objects must be created
     * before application tasks begin running.
     */

#if !BUDDY4_TRIAL_IMU_ONLY
    error_code = buddy1_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 1 init failed: %d\n",
            error_code
        );

        return 1;
    }


#endif
#if !APP_BUDDY1_ONLY && !BUDDY4_TRIAL_IMU_ONLY
    error_code = buddy5_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 5 init failed: %d\n",
            error_code
        );

        return 1;
    }

#endif
#if APP_BUDDY4_ENABLED && !APP_BUDDY1_ONLY
    error_code = buddy4_task_init();
    if (error_code < E_OK) {
        tm_printf((UB *)"[APP] Buddy 4 init failed: %d\n", error_code);
        return 1;
    }
#endif
    /*
     * Future subsystem initialisation:
     *
     * buddy2_init();
     * buddy3_init();
     *
     * These calls will be added when
     * the respective buddies are ready.
     */


    /*
     * =================================================
     * PHASE 2: SUBSYSTEM TASK STARTUP
     * =================================================
     *
     * All required shared kernel objects
     * should now be available.
     */

#if !BUDDY4_TRIAL_IMU_ONLY
    error_code = buddy1_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 1 start failed: %d\n",
            error_code
        );

        return 1;
    }

#endif
#if !APP_BUDDY1_ONLY && !BUDDY4_TRIAL_IMU_ONLY
    error_code = buddy5_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 5 start failed: %d\n",
            error_code
        );

        return 1;
    }

#endif
#if APP_BUDDY4_ENABLED && !APP_BUDDY1_ONLY
    error_code = buddy4_task_start();
    if (error_code < E_OK) {
        tm_printf((UB *)"[APP] Buddy 4 start failed: %d\n", error_code);
        return 1;
    }
#endif
    /*
     * Future subsystem startup:
     *
     * buddy2_start();
     * buddy3_start();
     */


    /*
     * =================================================
     * SYSTEM RUNNING
     * =================================================
     */

#if BUDDY4_TRIAL_IMU_ONLY
    tm_printf((UB *)"[APP] BUDDY 4 ONLY: Buddy 1/5 tasks disabled; motor pins held low.\n");
#else
    tm_printf((UB *)"[APP] Buddy 1/4/5 trial started.\n");
#endif

    /*
     * Keep the initial task asleep.
     *
     * Returning from usermain() would shut down
     * the µT-Kernel application.
     */
#if APP_BUDDY4_ENABLED && !APP_BUDDY1_ONLY
    /* Read-only diagnostics once per second; no motor actions. */
    (void)tk_chg_pri(TSK_SELF, 14);
    #if BUDDY4_CONSOLE_CSV
    tm_printf((UB *)"# B4_MTK: seq valid errors pitch_mdeg roll_mdeg height_milli_cm events stable_milli_cm median_milli_cm skipped tilt_valid acceleration_valid quality clipped_axes clipped_samples unsuitable_samples\n");
    #else
    tm_printf((UB *)"[B4] Tilt/peak and possible-impact monitor; NOW is current. Saved results are retained observations. Possible impact does not mean confirmed collision.\n");
    #endif
    for (;;) {
        if (buddy4_task_get(&buddy4_diagnostic) >= E_OK) {
            #if BUDDY4_CONSOLE_CSV
            tm_printf((UB *)"B4,%u,%u,%u,%d,%d,%d,%u,%d,%d,%u,%u,%u,%d,%u,%u,%u\n",
                (UINT)buddy4_diagnostic.sequence, (UINT)buddy4_diagnostic.valid,
                (UINT)buddy4_diagnostic.io_errors,
                buddy4_diagnostic.valid && buddy4_diagnostic.tilt_valid ? (INT)(buddy4_diagnostic.pitch_deg * 1000) : -2147483647,
                buddy4_diagnostic.valid && buddy4_diagnostic.tilt_valid ? (INT)(buddy4_diagnostic.roll_deg * 1000) : -2147483647,
                buddy4_diagnostic.valid && buddy4_diagnostic.tilt_valid ? (INT)(buddy4_diagnostic.height_cm * 1000) : -2147483647,
                (UINT)buddy4_diagnostic.completed_events,
                buddy4_diagnostic.last_event.stable_valid ?
                    (INT)(buddy4_diagnostic.last_event.stable_cm * 1000) : -1,
                buddy4_diagnostic.last_event.median_valid ?
                    (INT)(buddy4_diagnostic.last_event.median_cm * 1000) : -1,
                (UINT)buddy4_diagnostic.skipped_slots,
                (UINT)buddy4_diagnostic.tilt_valid, (UINT)buddy4_diagnostic.acceleration_valid,
                (INT)buddy4_diagnostic.measurement_quality, (UINT)buddy4_diagnostic.clipped_axes,
                (UINT)buddy4_diagnostic.clipped_samples, (UINT)buddy4_diagnostic.unsuitable_samples);
            #else
            buddy4_print_readable(&buddy4_diagnostic);
            #endif
        } else {
            tm_printf((UB *)"# B4: snapshot unavailable; current measurements N/A\n");
        }
        (void)tk_dly_tsk(1000);
    }
#else
    tk_slp_tsk(TMO_FEVR);
#endif

    return 0;
}
