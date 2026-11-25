#include "../barrc_types.h"
#include <tk/tkernel.h>

#if defined(B5_FULL_TEST) && (TM_CONSOLE_USB_CDC || TM_WIFI_MQTT)
#include <tm/tmonitor.h>
#include "barcode_task.h"
#include "barcode_scan.h"
#include "line_sensors.h"
#include "platform.h"

#define BARCODE_REPORT_US 2000000u

static char s_latest_letter;
static uint32_t s_latest_read_ms;

bool buddy3_barcode_latest (char * letter, uint32_t * p_read_ms)
{
    UW irq;
    DI(irq);
    *letter = s_latest_letter;
    *p_read_ms = s_latest_read_ms;
    EI(irq);
    return 0 != *letter;
}

#if TM_CONSOLE_USB_CDC
void barcode_scan_on_run (bool black, uint32_t width_us)
{
    tm_printf((UB *)"[B3 BARCODE] %s %u us\n",
              (UB *)(black ? "BLACK" : "white"), (uint32_t)width_us);
}

#endif

#if TM_CONSOLE_USB_CDC
/* Print the element widths (us) of a window that looked like a barcode but
 * failed.
 * Black bars are even positions (0,2,4..), white gaps odd. */
void barcode39_on_fail (const uint32_t * p_w, uint8_t n, uint8_t reason)
{
    tm_printf((UB *)"[B3 BARCODE] FAIL reason=%u elements=%u\n",
              (uint32_t)reason, (uint32_t)n);

    for (uint8_t idx = 0; idx < n; idx += 10)
    {
        uint32_t v[10];

        for (uint8_t k = 0; k < 10; k++)
        {
            v[k] = ((uint8_t)(idx + k) < n) ? (uint32_t)p_w[idx + k] : 0u;
        }

        tm_printf((UB *)"[B3 BARCODE] w[%u..]: %u %u %u %u %u %u %u %u %u %u\n",
                  (uint32_t)idx,
                  v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9]);
    }
}
#endif

void barcode_scan_on_decoded (const barcode39_result_t * p_result)
{
    if ((1 == p_result->length) && (p_result->text[0] >= 'A')
        && (p_result->text[0] <= 'Z'))
    {
        uint32_t now = (uint32_t)(b5p_time_us_64() / 1000u);
        UW irq;
        DI(irq);
        s_latest_letter = p_result->text[0];
        s_latest_read_ms = now;
        EI(irq);
    }
#if TM_CONSOLE_USB_CDC
    tm_printf((UB *)"[B3 BARCODE] *** BARCODE: %s (crossed %s) ***\n",
              (const UB *)p_result->text,
              (UB *)(p_result->reversed ? "backwards" : "forwards"));
#endif
}

static void barcode_task (INT start_code, void * p_exinf)
{
    (void)start_code;
    (void)p_exinf;
    barcode_scan_init();
#if TM_CONSOLE_USB_CDC
    uint32_t last_report = (uint32_t)b5p_time_us_64();
#endif
    for (;;)
    {
        bool black = 0u != (line_sensors_read() & LINE_SENSOR_BARCODE);
        uint32_t now = (uint32_t)b5p_time_us_64();
        barcode_scan_update(black, now);
#if TM_CONSOLE_USB_CDC
        if (now - last_report >= BARCODE_REPORT_US)
        {
            last_report = now;
            tm_printf((UB *)"[B3 BARCODE] sensor=%s decoded=%u rejected=%u "
                      "last_reject=%u\n",
                      (UB *)(black ? "BLACK" : "white"),
                      (uint32_t)barcode_scan_count(),
                      (uint32_t)barcode_scan_fail_count(),
                      (uint32_t)barcode_scan_last_fail());
        }
#endif
        (void)tk_dly_tsk(1);
    }
}

ER buddy3_barcode_start (void)
{
    T_CTSK config = {
        .tskatr = TA_HLNG | TA_RNG3,
        .task = barcode_task,
        .itskpri = 6,
        .stksz = 2048
    };
    ID task = tk_cre_tsk(&config);

    if (task < E_OK)
    {
        return task;
    }

    ER result = tk_sta_tsk(task, 0);

    if (result < E_OK)
    {
        (void)tk_del_tsk(task);
    }

    return result;
}
#endif

/*** end of file ***/
