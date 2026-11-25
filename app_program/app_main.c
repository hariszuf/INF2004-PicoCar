#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/buddy1.h"

#ifdef B5_FULL_TEST
#ifndef B5_DISABLED
#define B5_DISABLED 1   /* Buddy 5 (obstacle detection) removed */
#endif
#include "buddy2/motion_control.h"
#include "buddy3/line_follow.h"
#include "buddy4/rtos/buddy4_task.h"
#include "barcode_task.h"
#include "platform.h"
#include "scan_link.h"
#include "vehicle/vehicle_controller.h"

#if TK_SUPPORT_SMP
#error "This integration profile requires SMP=0"
#endif
#if CNF_TIMER_PERIOD != 1
#error "Use picocar_integration.mk (1ms kernel tick required)"
#endif
#if TM_CONSOLE_UART
#error "GP0 belongs to the line sensor: this profile requires USB-only console"
#endif

static INT start_application (void)
{
    /* Phase 1: Initialise hardware and subsystem resources. */
    if ((b5_platform_init() < E_OK) || (MOTION_OK != motion_init()))
    {
        return 1;
    }

    motion_emergency_stop();
    line_follow_init();

    ER buddy1_init_result = buddy1_init();

    if (buddy1_init_result < E_OK)
    {
        tm_printf((UB *)"[B1] Telemetry unavailable (%d); driving continues.\n",
                  buddy1_init_result);
    }

    ER buddy4_init_result = buddy4_task_init();
    bool buddy4_ready = buddy4_init_result >= E_OK;

    if (!buddy4_ready)
    {
        tm_printf((UB *)"[B4] IMU task unavailable (%d); driving continues.\n",
                  buddy4_init_result);
    }

    /* Phase 2: Start the application tasks. */
#if !B5_DISABLED
    if (line_scan_start() < E_OK)
    {
        tm_printf((UB *)"[B5] Obstacle scanner failed to start.\n");
        return 1;
    }
#endif
    if (vehicle_controller_start(buddy4_ready) < E_OK)
    {
        return 1;
    }

    if (buddy1_init_result >= E_OK)
    {
        ER error = buddy1_start();

        if (error < E_OK)
        {
            tm_printf((UB *)"[B1] Telemetry start failed (%d); driving "
                      "continues.\n",
                      error);
        }
    }

    if (buddy4_ready)
    {
        ER error = buddy4_task_start();

        if (error < E_OK)
        {
            vehicle_controller_disable_imu();
            tm_printf((UB *)"[B4] IMU task did not start (%d); driving "
                      "continues.\n",
                      error);
        }
    }

#if (TM_CONSOLE_USB_CDC || TM_WIFI_MQTT) && !B3_NO_BARCODE
    ER barcode_error = buddy3_barcode_start();

    if (barcode_error < E_OK)
    {
        tm_printf((UB *)"[B3 BARCODE] Task start failed (%d); driving "
                  "continues.\n",
                  barcode_error);
    }
#endif

    return 0;
}
#else
/* Legacy Buddy 1 development profile (Buddy 5 removed). */
static INT start_application (void)
{
    /* Phase 1: Initialise subsystem resources. */
    ER error = buddy1_init();

    if (error < E_OK)
    {
        tm_printf((UB *)"[APP] Buddy 1 init failed: %d\n", error);
        return 1;
    }

    /* Phase 2: Start the application tasks. */
    error = buddy1_start();

    if (error < E_OK)
    {
        tm_printf((UB *)"[APP] Buddy 1 start failed: %d\n", error);
        return 1;
    }

    tm_printf((UB *)"[APP] All registered buddies started\n");
    return 0;
}
#endif

EXPORT INT usermain (void)
{
    INT result = start_application();

    if (0 != result)
    {
        return result;
    }

    /* Returning from usermain() shuts down the application. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
