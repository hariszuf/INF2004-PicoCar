#ifndef BUDDY4_MOTION_CLASS_H
#define BUDDY4_MOTION_CLASS_H
#include <stdbool.h>
#include <stdint.h>
/* Encoder-based estimated motion; no SDK, Buddy 2 or motor-control dependency. */
typedef enum {
    BUDDY4_MOTION_UNKNOWN = 0, BUDDY4_MOTION_STOPPED,
    BUDDY4_MOTION_FORWARD, BUDDY4_MOTION_REVERSE,
    BUDDY4_MOTION_TURNING_LEFT, BUDDY4_MOTION_TURNING_RIGHT
} buddy4_motion_state_t;
typedef struct {
    float stop_enter_cm_s, stop_exit_cm_s; /* max absolute wheel speed */
    float turn_enter_cm_s, turn_exit_cm_s; /* absolute right-minus-left */
    uint32_t confirm_us, stale_us;
} buddy4_motion_config_t;
typedef struct {
    buddy4_motion_state_t state;
    bool valid; /* false implies UNKNOWN, including startup/initial confirmation */
    uint64_t sample_us;
} buddy4_motion_output_t;
typedef struct {
    buddy4_motion_config_t config;
    buddy4_motion_state_t state, pending;
    bool configured, has_sample, has_clock;
    uint64_t sample_us, pending_since_us, clock_us;
} buddy4_motion_t;
/* All defaults are provisional; not measured vehicle thresholds. */
buddy4_motion_config_t buddy4_motion_default_config(void);
bool buddy4_motion_init(buddy4_motion_t *c, buddy4_motion_config_t config);
/* Call once per new speed observation. sample_us must increase strictly.
 * now_us must be monotonic; future/old samples and nonfinite speeds invalidate
 * immediately. Recovery requires a new full confirmation period. */
void buddy4_motion_update(buddy4_motion_t *c, float left_cm_s, float right_cm_s,
                         bool input_valid, uint64_t sample_us, uint64_t now_us);
/* Poll even without new observations to expire stale state. During a pending
 * transition the previous confirmed label persists while inputs remain fresh. */
buddy4_motion_output_t buddy4_motion_get(buddy4_motion_t *c, uint64_t now_us);
const char *buddy4_motion_name(buddy4_motion_state_t state);
#endif
