#include "buddy4_motion.h"
#include <math.h>
static void invalidate(buddy4_motion_t *c) {
    c->state = c->pending = BUDDY4_MOTION_UNKNOWN;
    c->has_sample = false;
}
static bool clock_ok(buddy4_motion_t *c, uint64_t now) {
    if (c->has_clock && now < c->clock_us) { invalidate(c); return false; }
    c->has_clock = true; c->clock_us = now; return true;
}
buddy4_motion_config_t buddy4_motion_default_config(void) {
    return (buddy4_motion_config_t){0.5f,0.8f,1.5f,1.0f,100000u,100000u};
}
bool buddy4_motion_init(buddy4_motion_t *c, buddy4_motion_config_t cfg) {
    *c = (buddy4_motion_t){.config = cfg};
    c->configured = isfinite(cfg.stop_enter_cm_s) && isfinite(cfg.stop_exit_cm_s) &&
        isfinite(cfg.turn_enter_cm_s) && isfinite(cfg.turn_exit_cm_s) &&
        cfg.stop_enter_cm_s >= 0 && cfg.stop_exit_cm_s > cfg.stop_enter_cm_s &&
        cfg.turn_exit_cm_s > 0 && cfg.turn_enter_cm_s > cfg.turn_exit_cm_s &&
        cfg.confirm_us > 0 && cfg.stale_us > 0;
    return c->configured;
}
void buddy4_motion_update(buddy4_motion_t *c, float left, float right,
                         bool valid, uint64_t sample, uint64_t now) {
    if (!clock_ok(c,now)) return;
    float mean = left * 0.5f + right * 0.5f, diff = right - left;
    if (!c->configured || !valid || !isfinite(left) || !isfinite(right) ||
        !isfinite(mean) || !isfinite(diff) || sample > now ||
        now - sample >= c->config.stale_us ||
        (c->has_sample && sample <= c->sample_us)) {
        invalidate(c); return;
    }
    if (c->has_sample && sample - c->sample_us >= c->config.stale_us)
        invalidate(c); /* Never confirm across a missing-data interval. */
    c->sample_us = sample; c->has_sample = true;
    float stop_limit = c->state == BUDDY4_MOTION_STOPPED
        ? c->config.stop_exit_cm_s : c->config.stop_enter_cm_s;
    buddy4_motion_state_t target;
    if (fmaxf(fabsf(left),fabsf(right)) <= stop_limit)
        target = BUDDY4_MOTION_STOPPED;
    else if (diff >= c->config.turn_enter_cm_s ||
             (c->state == BUDDY4_MOTION_TURNING_LEFT && diff >= c->config.turn_exit_cm_s))
        target = BUDDY4_MOTION_TURNING_LEFT;
    else if (diff <= -c->config.turn_enter_cm_s ||
             (c->state == BUDDY4_MOTION_TURNING_RIGHT && diff <= -c->config.turn_exit_cm_s))
        target = BUDDY4_MOTION_TURNING_RIGHT;
    else if (mean > c->config.stop_enter_cm_s) target = BUDDY4_MOTION_FORWARD;
    else if (mean < -c->config.stop_enter_cm_s) target = BUDDY4_MOTION_REVERSE;
    else target = BUDDY4_MOTION_UNKNOWN; /* cancellation is not stopped */
    if (target == BUDDY4_MOTION_UNKNOWN) {
        c->state = c->pending = target; return;
    }
    if (target != c->pending) { c->pending = target; c->pending_since_us = sample; }
    if (sample - c->pending_since_us >= c->config.confirm_us) c->state = target;
}
buddy4_motion_output_t buddy4_motion_get(buddy4_motion_t *c, uint64_t now) {
    if (!clock_ok(c,now) || !c->has_sample || now < c->sample_us ||
        now - c->sample_us >= c->config.stale_us) invalidate(c);
    bool valid = c->configured && c->has_sample && c->state != BUDDY4_MOTION_UNKNOWN;
    return (buddy4_motion_output_t){valid ? c->state : BUDDY4_MOTION_UNKNOWN,
                                   valid,c->has_sample ? c->sample_us : 0};
}
const char *buddy4_motion_name(buddy4_motion_state_t s) {
    switch(s) {
    case BUDDY4_MOTION_STOPPED: return "STOPPED";
    case BUDDY4_MOTION_FORWARD: return "FORWARD";
    case BUDDY4_MOTION_REVERSE: return "REVERSE";
    case BUDDY4_MOTION_TURNING_LEFT: return "TURNING_LEFT";
    case BUDDY4_MOTION_TURNING_RIGHT: return "TURNING_RIGHT";
    default: return "UNKNOWN";
    }
}
