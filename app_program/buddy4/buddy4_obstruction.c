#include "buddy4_obstruction.h"
#include <math.h>

/* Provisional envelope around the measured ~3 cm/s trial, not general tuning.
 * 0.5 cm/s is Buddy 2/4's existing near-stopped scale, far below 3 cm/s.
 * Also require <=20% of demand. The reported 0.12 -> 0.004 spans 100 ms. */
#define MAX_GAP_US       40000u
#define ESTABLISH_US    500000u
#define CONFIRM_US      100000u
#define COOLDOWN_US    1000000u
/* Allow up to two existing confirmation intervals for a brief unequal-wheel
 * transition to resolve. This does not arm from asymmetric motion or count it
 * as stopped-wheel evidence. Recorded foam transition resolves in ~140 ms. */
#define ASYMMETRY_US    (2u * CONFIRM_US)

static void clear_evidence(buddy4_obstruction_t *d) {
    d->output.valid=d->output.established=d->output.possible_obstruction=false;
    d->output.slow_ms=0;
    d->establishing=d->pending=d->has_demand=d->asymmetric=d->paused=false;
}
static bool clock_ok(buddy4_obstruction_t *d,uint64_t now) {
    if(d->has_clock && now<d->clock_us) { clear_evidence(d); return false; }
    d->has_clock=true;d->clock_us=now;return true;
}
void buddy4_obstruction_init(buddy4_obstruction_t *d) {
    *d=(buddy4_obstruction_t){0};
}
void buddy4_obstruction_cancel(buddy4_obstruction_t *d,uint64_t now) {
    (void)clock_ok(d,now);clear_evidence(d);
}
void buddy4_obstruction_update(buddy4_obstruction_t *d,
    const buddy4_obstruction_input_t *in,uint64_t now) {
    d->output.possible_obstruction=false;
    if(!clock_ok(d,now))return;
    if(!in->valid || !in->eligible || !isfinite(in->left_cm_s) ||
       !isfinite(in->right_cm_s) || !isfinite(in->command_left_cm_s) ||
       !isfinite(in->command_right_cm_s) || in->observation_us>now ||
       now-in->observation_us>MAX_GAP_US ||
       (d->has_observation && in->observation_us<=d->last_observation_us)) {
        clear_evidence(d);return;
    }
    if(d->has_observation && in->observation_us-d->last_observation_us>MAX_GAP_US)
        clear_evidence(d);
    d->last_observation_us=in->observation_us;d->has_observation=true;
    d->output.observation_us=in->observation_us;
    float cl=in->command_left_cm_s, cr=in->command_right_cm_s;
    /* Commanded turns, reversals and out-of-envelope demand still invalidate
     * immediately. Measured asymmetry is not proof of changed command intent. */
    if(cl<2 || cl>4 || cr!=cl || in->left_cm_s<0 || in->right_cm_s<0) {
        clear_evidence(d);return;
    }
    if(!d->has_demand || in->demand_id!=d->demand_id ||
       cl!=d->command_left_cm_s || cr!=d->command_right_cm_s) {
        clear_evidence(d);
        d->demand_id=in->demand_id;
        d->command_left_cm_s=cl;d->command_right_cm_s=cr;d->has_demand=true;
    }
    d->output.valid=true;
    /* A single contact sample can perturb accelerometer-derived pitch without
     * changing the established terrain state. Quarantine it for at most one
     * existing freshness interval, never counting it as movement or slowdown.
     * Fresh encoder/IMU pairs still pass every normal validity/context check.
     * This cannot extend the separate asymmetric-wheel transition budget. */
    if(in->pause_motion) {
        if(d->asymmetric || fabsf(in->left_cm_s-in->right_cm_s)>0.75f ||
           (!d->paused&&!d->output.established)) {
            clear_evidence(d);return;
        }
        if(!d->paused) { d->paused=true;d->paused_since_us=in->observation_us; }
        d->output.valid=d->output.established=false;
        d->pending=d->establishing=false;d->output.slow_ms=0;
        if(in->observation_us-d->paused_since_us>=MAX_GAP_US)clear_evidence(d);
        return;
    }
    if(d->paused) {
        if(in->observation_us-d->paused_since_us>MAX_GAP_US ||
           fabsf(in->left_cm_s-in->right_cm_s)>0.75f) {
            clear_evidence(d);return;
        }
        d->paused=false;d->output.established=true;
    }
    bool moving=in->left_cm_s>=0.7f*cl && in->left_cm_s<=1.3f*cl &&
                in->right_cm_s>=0.7f*cr && in->right_cm_s<=1.3f*cr &&
                fabsf(in->left_cm_s-in->right_cm_s)<=0.75f;
    bool slow=in->left_cm_s<=fminf(0.5f,0.2f*cl) &&
              in->right_cm_s<=fminf(0.5f,0.2f*cr);
    if(fabsf(in->left_cm_s-in->right_cm_s)>0.75f) {
        if(!d->output.established) { clear_evidence(d);return; }
        d->pending=d->establishing=false;d->output.slow_ms=0;
        if(!d->asymmetric) { d->asymmetric=true;d->asymmetric_since_us=in->observation_us; }
        if(in->observation_us-d->asymmetric_since_us>ASYMMETRY_US)clear_evidence(d);
        return;
    }
    if(d->asymmetric) {
        if(in->observation_us-d->asymmetric_since_us>ASYMMETRY_US) {
            clear_evidence(d);return;
        }
        if(moving||slow)d->asymmetric=false;
        else return; /* Still transitioning: no establishment/slow evidence. */
    }
    if(moving) {
        d->pending=false;d->output.slow_ms=0;
        if(!d->establishing) { d->establishing=true;d->moving_since_us=in->observation_us; }
        if(in->observation_us-d->moving_since_us>=ESTABLISH_US &&
           (!d->output.event_count || (in->observation_us>=d->output.event_us &&
            in->observation_us-d->output.event_us>=COOLDOWN_US)))
            d->output.established=true;
    } else {
        d->establishing=false;
        if(slow && d->output.established) {
            if(!d->pending) { d->pending=true;d->slow_since_us=in->observation_us; }
            uint64_t span=in->observation_us-d->slow_since_us;
            d->output.slow_ms=(uint32_t)(span/1000u);
            if(span>=CONFIRM_US) {
                d->output.possible_obstruction=true;
                if(d->output.event_count!=UINT32_MAX)++d->output.event_count;
                d->output.event_us=in->observation_us;
                d->output.established=false;d->pending=false;
            }
        } else { d->pending=false;d->output.slow_ms=0; }
    }
}
buddy4_obstruction_output_t buddy4_obstruction_get(buddy4_obstruction_t *d,uint64_t now) {
    if(!clock_ok(d,now) || !d->has_observation || now<d->output.observation_us ||
       now-d->output.observation_us>MAX_GAP_US)clear_evidence(d);
    return d->output;
}
