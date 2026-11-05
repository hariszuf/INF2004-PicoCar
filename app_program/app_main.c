#ifdef B5_FULL_TEST
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/buddy1.h"
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

EXPORT INT usermain(void) {
 if(b5_platform_init()<E_OK || motion_init()!=MOTION_OK) return 1;
 motion_emergency_stop(); line_follow_init(); ultrasonic_init();
 if(!buddy5_servo_init()) return 1;
 ER buddy1_init_result=buddy1_init();
 if(buddy1_init_result<E_OK)
  tm_printf((UB*)"[B1] Telemetry unavailable (%d); driving continues.\n",buddy1_init_result);
 ER buddy4_init_result=buddy4_task_init();
 bool buddy4_ready=buddy4_init_result>=E_OK;
 if(!buddy4_ready) tm_printf((UB*)"[B4] IMU task unavailable (%d); driving continues.\n",buddy4_init_result);
 if(vehicle_controller_start(buddy4_ready)<E_OK) return 1;
 if(line_scan_start()<E_OK) { motion_emergency_stop(); return 1; }
 if(buddy1_init_result>=E_OK) {
  ER e=buddy1_start();
  if(e<E_OK) tm_printf((UB*)"[B1] Telemetry start failed (%d); driving continues.\n",e);
 }
 if(buddy4_ready) {
  ER e=buddy4_task_start();
  if(e<E_OK) { vehicle_controller_disable_imu(); tm_printf((UB*)"[B4] IMU task did not start (%d); driving continues.\n",e); }
 }
#if TM_CONSOLE_USB_CDC || TM_WIFI_MQTT
 ER barcode_error=buddy3_barcode_start();
 if(barcode_error<E_OK)
  tm_printf((UB*)"[B3 BARCODE] Task start failed (%d); driving continues.\n",barcode_error);
#endif
 tk_slp_tsk(TMO_FEVR); return 0;
}
#else

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/buddy1.h"
#include "buddy5/buddy5.h"

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

    error_code = buddy1_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 1 init failed: %d\n",
            error_code
        );

        return 1;
    }


    error_code = buddy5_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 5 init failed: %d\n",
            error_code
        );

        return 1;
    }

    /*
     * Future subsystem initialisation:
     *
     * buddy2_init();
     * buddy3_init();
     * buddy4_init();
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

    error_code = buddy1_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 1 start failed: %d\n",
            error_code
        );

        return 1;
    }

    error_code = buddy5_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 5 start failed: %d\n",
            error_code
        );

        return 1;
    }

    /*
     * Future subsystem startup:
     *
     * buddy2_start();
     * buddy3_start();
     * buddy4_start();
     */


    /*
     * =================================================
     * SYSTEM RUNNING
     * =================================================
     */

    tm_printf(
        (UB *)"[APP] All registered buddies started\n"
    );

    /*
     * Keep the initial task asleep.
     *
     * Returning from usermain() would shut down
     * the µT-Kernel application.
     */
    tk_slp_tsk(TMO_FEVR);

    return 0;
}
#endif
