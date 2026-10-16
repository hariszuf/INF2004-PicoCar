
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy5.h"
#include "ultrasonic.h"
#include "obstacle_state.h"

/* Temporary task priority for testing */
#define BUDDY5_PRIORITY 11

static ID buddy5_task_id = 0;

/* Continuously read ultrasonic distance */
static void buddy5_task(INT stacd, void *exinf) // repeatedly takes distance measurements in mm cause accurate
{
    buddy5_state_t state = BUDDY5_SENSOR_FAULT;

    (void)stacd;
    (void)exinf;

    tm_printf((UB *)"[Buddy5] Sensor task started\n");

    while (1) {
        ultrasonic_result_t result = ultrasonic_read();

        state = buddy5_next_state(state, result);
        const char *state_name = state == BUDDY5_OBSTACLE ? "OBSTACLE" :
                                 state == BUDDY5_CLEAR ? "CLEAR" : "SENSOR FAULT";

        if (result.status == US_OK) {
            tm_printf(
                (UB *)"[Buddy5] Distance: %u mm | State: %s\n",
                (unsigned int)result.distance_mm, state_name
            );
        } else {
            tm_printf(
                (UB *)"[Buddy5] State: SENSOR FAULT | Sensor error: %d\n",
                (int)result.status
            );
        }

        /* Wait 100 milliseconds */
        tk_dly_tsk(100); //this is to allo other tasks to execute btwn measurements
    }
}

/* RTOS task configuration */
static T_CTSK buddy5_task_config = {
    .itskpri = BUDDY5_PRIORITY,
    .stksz = 2048,
    .task = buddy5_task,
    .tskatr = TA_HLNG | TA_RNG3
};

/* Phase 1: Initialize GPIO */
ER buddy5_init(void)
{
    ultrasonic_init();

    tm_printf((UB *)"[Buddy5] Ultrasonic initialized\n");

    return E_OK;
}

/* Phase 2: Create and start task */
ER buddy5_start(void)
{
    if (buddy5_task_id > 0) {
        return E_OBJ;
    }

    buddy5_task_id = tk_cre_tsk(&buddy5_task_config);

    if (buddy5_task_id < E_OK) {
        return buddy5_task_id;
    }

    return tk_sta_tsk(buddy5_task_id, 0);
}

