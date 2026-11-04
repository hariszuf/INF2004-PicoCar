#include <tk/tkernel.h>

#if defined(B5_FULL_TEST) && TM_CONSOLE_USB_CDC
#include <tm/tmonitor.h>
#include "barcode_task.h"
#include "barcode_scan.h"
#include "line_sensors.h"
#include "platform.h"

#define BARCODE_REPORT_US 2000000u

void barcode_scan_on_run(bool black, uint32_t width_us)
{
    tm_printf((UB *)"[B3 BARCODE] %s %u us\n",
              (UB *)(black ? "BLACK" : "white"), (unsigned)width_us);
}

void barcode_scan_on_decoded(const barcode39_result_t *result)
{
    tm_printf((UB *)"[B3 BARCODE] *** BARCODE: %s (crossed %s) ***\n",
              (const UB *)result->text,
              (UB *)(result->reversed ? "backwards" : "forwards"));
}

static void barcode_task(INT start_code, void *exinf)
{
    (void)start_code;
    (void)exinf;
    barcode_scan_init();
    uint32_t last_report = (uint32_t)b5p_time_us_64();
    for (;;) {
        bool black = (line_sensors_read() & LINE_SENSOR_BARCODE) != 0u;
        uint32_t now = (uint32_t)b5p_time_us_64();
        barcode_scan_update(black, now);
        if (now - last_report >= BARCODE_REPORT_US) {
            last_report = now;
            tm_printf((UB *)"[B3 BARCODE] sensor=%s decoded=%u rejected=%u last_reject=%u\n",
                      (UB *)(black ? "BLACK" : "white"),
                      (unsigned)barcode_scan_count(),
                      (unsigned)barcode_scan_fail_count(),
                      (unsigned)barcode_scan_last_fail());
        }
        (void)tk_dly_tsk(1);
    }
}

ER buddy3_barcode_start(void)
{
    T_CTSK config = {
        .tskatr = TA_HLNG | TA_RNG3,
        .task = barcode_task,
        .itskpri = 6,
        .stksz = 2048
    };
    ID task = tk_cre_tsk(&config);
    if (task < E_OK) return task;
    ER result = tk_sta_tsk(task, 0);
    if (result < E_OK) (void)tk_del_tsk(task);
    return result;
}
#endif
