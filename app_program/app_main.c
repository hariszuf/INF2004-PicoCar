#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/buddy1.h"

#ifdef B5_FULL_TEST
#include "buddy2/motion_control.h"
#include "buddy3/line_follow.h"
#include "buddy4/rtos/buddy4_task.h"
#include "buddy5/ultrasonic.h"
#include "buddy5/rtos/buddy5_servo.h"
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

static INT start_application(void)
{
    /* Phase 1: Initialise hardware and subsystem resources. */
    if (b5_platform_init() < E_OK || motion_init() != MOTION_OK) {
        return 1;
    }

    motion_emergency_stop();
    line_follow_init();
    ultrasonic_init();
    if (!buddy5_servo_init()) {
        return 1;
    }

    ER buddy1_init_result = buddy1_init();
    if (buddy1_init_result < E_OK) {
        tm_printf((UB *)"[B1] Telemetry unavailable (%d); driving continues.\n",
                  buddy1_init_result);
    }

    ER buddy4_init_result = buddy4_task_init();
    bool buddy4_ready = buddy4_init_result >= E_OK;
    if (!buddy4_ready) {
        tm_printf((UB *)"[B4] IMU task unavailable (%d); driving continues.\n",
                  buddy4_init_result);
    }

    /* Phase 2: Start the application tasks. */
    if (vehicle_controller_start(buddy4_ready) < E_OK) {
        return 1;
    }
    if (line_scan_start() < E_OK) {
        motion_emergency_stop();
        return 1;
    }

    if (buddy1_init_result >= E_OK) {
        ER error = buddy1_start();
        if (error < E_OK) {
            tm_printf((UB *)"[B1] Telemetry start failed (%d); driving continues.\n",
                      error);
        }
    }

    if (buddy4_ready) {
        ER error = buddy4_task_start();
        if (error < E_OK) {
            vehicle_controller_disable_imu();
            tm_printf((UB *)"[B4] IMU task did not start (%d); driving continues.\n",
                      error);
        }
    }

#if TM_CONSOLE_USB_CDC || TM_WIFI_MQTT
    ER barcode_error = buddy3_barcode_start();
    if (barcode_error < E_OK) {
        tm_printf((UB *)"[B3 BARCODE] Task start failed (%d); driving continues.\n",
                  barcode_error);
    }
#endif

    return 0;
}
#else
#include "buddy5/buddy5.h"

/* Legacy Buddy 1 and Buddy 5 development profile. */
static INT start_application(void)
{
    /* Phase 1: Initialise subsystem resources. */
    ER error = buddy1_init();
    if (error < E_OK) {
        tm_printf((UB *)"[APP] Buddy 1 init failed: %d\n", error);
        return 1;
    }

    error = buddy5_init();
    if (error < E_OK) {
        tm_printf((UB *)"[APP] Buddy 5 init failed: %d\n", error);
        return 1;
    }

    /* Phase 2: Start the application tasks. */
    error = buddy1_start();
    if (error < E_OK) {
        tm_printf((UB *)"[APP] Buddy 1 start failed: %d\n", error);
        return 1;
    }

    error = buddy5_start();
    if (error < E_OK) {
        tm_printf((UB *)"[APP] Buddy 5 start failed: %d\n", error);
        return 1;
    }

    tm_printf((UB *)"[APP] All registered buddies started\n");
    return 0;
}
#endif

EXPORT INT usermain(void)
{
    INT result = start_application();
    if (result != 0) {
        return result;
    }

    /* Returning from usermain() shuts down the application. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
