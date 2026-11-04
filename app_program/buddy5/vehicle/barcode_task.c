#include <tk/tkernel.h>

#if defined(B5_FULL_TEST) && (TM_CONSOLE_USB_CDC || TM_WIFI_MQTT)
#include <tm/tmonitor.h>
#include "barcode_task.h"
#include "barcode_scan.h"
#include "line_sensors.h"
#include "platform.h"

#define BARCODE_REPORT_US 2000000u

static char latest_letter;
static uint32_t latest_read_ms;

bool buddy3_barcode_latest(char *letter, uint32_t *read_ms)
{
    UW irq;
    DI(irq);
    *letter = latest_letter;
    *read_ms = latest_read_ms;
    EI(irq);
    return *letter != 0;
}

#if TM_CONSOLE_USB_CDC
void barcode_scan_on_run(bool black, uint32_t width_us)
{
    tm_printf((UB *)"[B3 BARCODE] %s %u us\n",
              (UB *)(black ? "BLACK" : "white"), (unsigned)width_us);
}

#endif

void barcode_scan_on_decoded(const barcode39_result_t *result)
{
    if (result->length == 1 && result->text[0] >= 'A' && result->text[0] <= 'Z') {
        uint32_t now = (uint32_t)(b5p_time_us_64() / 1000u);
        UW irq;
        DI(irq);
        latest_letter = result->text[0];
        latest_read_ms = now;
        EI(irq);
    }
#if TM_CONSOLE_USB_CDC
    tm_printf((UB *)"[B3 BARCODE] *** BARCODE: %s (crossed %s) ***\n",
              (const UB *)result->text,
              (UB *)(result->reversed ? "backwards" : "forwards"));
#endif
}

static void barcode_task(INT start_code, void *exinf)
{
    (void)start_code;
    (void)exinf;
    barcode_scan_init();
#if TM_CONSOLE_USB_CDC
    uint32_t last_report = (uint32_t)b5p_time_us_64();
#endif
    for (;;) {
        bool black = (line_sensors_read() & LINE_SENSOR_BARCODE) != 0u;
        uint32_t now = (uint32_t)b5p_time_us_64();
        barcode_scan_update(black, now);
#if TM_CONSOLE_USB_CDC
        if (now - last_report >= BARCODE_REPORT_US) {
            last_report = now;
            tm_printf((UB *)"[B3 BARCODE] sensor=%s decoded=%u rejected=%u last_reject=%u\n",
                      (UB *)(black ? "BLACK" : "white"),
                      (unsigned)barcode_scan_count(),
                      (unsigned)barcode_scan_fail_count(),
                      (unsigned)barcode_scan_last_fail());
        }
#endif
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
