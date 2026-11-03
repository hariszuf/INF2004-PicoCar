#ifndef BUDDY4_OBSTRUCTION_H
#define BUDDY4_OBSTRUCTION_H
#include <stdbool.h>
#include <stdint.h>

/* Supporting wheel-motion observation, NOT a collision or motor command.
 * Platform-independent: an eventual MTK producer must supply real observation
 * timing/validity and contemporaneous command context, not retimestamp a getter. */
typedef struct {
    bool valid;       /* NEW coherent finite speed pair with qualified timing. */
    bool eligible;    /* Established command context: straight forward, no brake,
                       * turn/fault/mode transition, usable flat-terrain context. */
    bool pause_motion; /* Fresh otherwise-eligible input, brief pitch guard only.
                        * Cannot establish or accumulate slowdown while paused. */
    uint64_t observation_us; /* Producer observation time; see SDK adapter contract.
                             * Not necessarily encoder acquisition time. */
    uint64_t demand_id; /* Changes on every new drive/mode session. */
    float command_left_cm_s, command_right_cm_s;
    float left_cm_s, right_cm_s;
} buddy4_obstruction_input_t;

typedef struct {
    bool valid, established, possible_obstruction;
    uint32_t event_count, slow_ms;
    uint64_t observation_us, event_us;
} buddy4_obstruction_output_t;

typedef struct {
    buddy4_obstruction_output_t output;
    bool has_clock, has_observation, has_demand, establishing, pending, asymmetric, paused;
    uint64_t clock_us, last_observation_us, moving_since_us, slow_since_us;
    uint64_t demand_id;
    uint64_t asymmetric_since_us;
    uint64_t paused_since_us;
    float command_left_cm_s, command_right_cm_s;
} buddy4_obstruction_t;

void buddy4_obstruction_init(buddy4_obstruction_t *d);
/* Immediate cancellation between observations; retains count/time/cooldown.
 * Recovery always needs a complete fresh established-motion window again. */
void buddy4_obstruction_cancel(buddy4_obstruction_t *d, uint64_t now_us);
/* Under unchanged eligible demand only, an already established observer may
 * retain history through <=200 ms of measured asymmetry. No slowdown evidence
 * accumulates during that transition. Persistent asymmetry/faults cancel it;
 * a subsequent two-wheel stop still needs the full 100 ms confirmation. */
/* A pitch-guard pause may retain ALREADY established history for <=40 ms,
 * with fresh inputs throughout. valid/established are false during the pause;
 * all pending evidence is discarded. Persistent pause, asymmetry, context or
 * time faults cancel. Recovery starts a new full slowdown confirmation. */
void buddy4_obstruction_update(buddy4_obstruction_t *d,
    const buddy4_obstruction_input_t *input, uint64_t now_us);
/* Getters age the last observation; they neither consume a pulse nor count it
 * twice. A pulse belongs to one input until the next update/cancel/expiry. */
buddy4_obstruction_output_t buddy4_obstruction_get(buddy4_obstruction_t *d,
    uint64_t now_us);
#endif
