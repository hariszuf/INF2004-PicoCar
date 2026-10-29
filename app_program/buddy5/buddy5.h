#ifndef BUDDY5_H
#define BUDDY5_H
#include <tk/tkernel.h>
#include "buddy5_core.h"
typedef struct {
    void *context;
    /* Nonblocking; fresh real motor/pose/line status, not an assumed success. */
    bool (*read_feedback)(void *context,b5_feedback_t *feedback);
    /* Accept once per id. Execute asynchronously. Refuse unverified swept paths.
     * STOP must work even when another request is active or feedback is stale. */
    bool (*submit_request)(void *context,const b5_request_t *request);
} buddy5_controller_t;
typedef struct {
    b5_state_t state;
    b5_fault_t fault;
    b5_profile_t profile;
    b5_request_t request;
    int32_t front_clearance_mm;
    uint32_t timestamp_ms, telemetry_dropped;
    int16_t servo_angle_deg;
    bool bench_mode;
} buddy5_status_t;
/* Bind before buddy5_start(), after buddy5_init(). Required if BENCH_MODE=0. */
ER buddy5_bind_controller(const buddy5_controller_t *controller);
ER buddy5_init(void);
ER buddy5_start(void);
ER buddy5_get_status(buddy5_status_t *status);
/* Queue explicit reset. Integrated mode only resets ERROR when fresh/stopped.
 * Bench mode can also restart a finished stationary scan. */
ER buddy5_reset(void);
#endif