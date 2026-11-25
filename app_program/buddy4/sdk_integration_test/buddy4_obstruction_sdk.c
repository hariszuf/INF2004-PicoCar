#include "buddy4_obstruction_sdk.h"
#include <math.h>

void buddy4_obstruction_sdk_init (buddy4_obstruction_sdk_t * a)
{
    *a = (buddy4_obstruction_sdk_t){0};
    buddy4_obstruction_init(&a->observer);
}

void buddy4_obstruction_sdk_note_update (buddy4_obstruction_sdk_t * a,
                                         const motion_status_t * status,
                                         const drive_control_t
                                         * p_demand,
                                         uint64_t begin, uint64_t end,
                                         uint32_t skipped)
{
    a->timing_ok = (a->have_update) && (!a->pending) && (begin > a->begin_us) &&
            (begin - a->begin_us >= 10000u)
            && (begin - a->begin_us <= 30000u) &&
            (end >= begin) && (end - begin <= 5000u) && (skipped == a->skipped);
    a->status = *status;
    a->demand = *p_demand;
    a->begin_us = begin;
    a->end_us = end;
    a->skipped = skipped;
    a->have_update = a->pending = true;
}

buddy4_obstruction_output_t buddy4_obstruction_sdk_poll (
        buddy4_obstruction_sdk_t * a,
        const drive_control_t * c, const buddy4_imu_t * p_imu, bool b_usb,
        uint64_t now)
{
    const buddy4_output_t * p_o = &p_imu->output;
    const motion_status_t * m = &a->status;
    bool b_usable = (a->have_update) && b_usb && (c->running)
            && (DRIVE_MANUAL == c->mode) &&
            (MOTION_FAULT_NONE == c->stop_fault) && (a->demand.running) &&
            (a->demand.mode == c->mode)
            && (a->demand.started_us == c->started_us) &&
            (a->demand.selected_left_cm_s == c->selected_left_cm_s) &&
            (a->demand.selected_right_cm_s == c->selected_right_cm_s) &&
            (MOTION_STATE_EXECUTING == m->state)
            && (MOTION_FAULT_NONE == m->fault) &&
            (isfinite(m->left_speed_cm_s)) && (isfinite(m->right_speed_cm_s)) &&
            (MOTION_TERRAIN_FLAT == m->terrain)
            && (isfinite(m->terrain_speed_scale)) &&
            (fabsf(m->terrain_speed_scale - 1) < 0.001f) &&
            (isfinite(m->left_terrain_duty))
            && (isfinite(m->right_terrain_duty)) &&
            (fabsf(m->left_terrain_duty) < 0.001f)
            && (fabsf(m->right_terrain_duty) < 0.001f) &&
            (p_imu->ready) && (p_o->valid) && (p_o->acceleration_valid)
            && (p_o->tilt_valid) &&
            (!p_o->clipped_axes) && (0 == p_o->terrain) && (0 == p_o->side) &&
            (isfinite(p_o->pitch_deg)) && (isfinite(p_o->roll_deg)) &&
            (isfinite(p_o->filtered_pitch_deg))
            && (isfinite(p_o->filtered_roll_deg)) &&
            /* Roll/terrain/quality guards cancel immediately. A brief
             * pitch-only
             * excursion is passed as an explicit pause, never as level
             * evidence.
             * Invalid tilt is never treated as flat, even if state flags are
             * zero.
             */
            (fabsf(p_o->filtered_roll_deg) <= 2) &&
            (p_o->timestamp_us <= now) && (now - p_o->timestamp_us <= 40000u) &&
            (a->end_us <= now) && (now - a->end_us <= 40000u);

    if (b_usable)
    {
        uint64_t skew =
                p_o->timestamp_us > a->end_us ? p_o->timestamp_us - a->end_us :
        a->end_us - p_o->timestamp_us;
        b_usable = skew <= 30000u;
    }

    if (!b_usable)
    {
        buddy4_obstruction_cancel(&a->observer, now);
        a->pending = false;
    }
    else if (a->pending)
    {
        bool b_imu_new = (!a->has_imu) || (p_o->timestamp_us > a->last_imu_us);

        if ((!a->timing_ok)
            || ((a->has_imu) && (p_o->timestamp_us < a->last_imu_us)))
        {
            buddy4_obstruction_cancel(&a->observer, now);
            a->pending = false;
        }
        else if (b_imu_new)
        {
            buddy4_obstruction_input_t in = {
                .valid = true, .eligible = true, .observation_us = a->end_us,
                .pause_motion = fabsf(p_o->filtered_pitch_deg) > 3,
                .demand_id = c->started_us,
                .command_left_cm_s = c->selected_left_cm_s,
                .command_right_cm_s = c->selected_right_cm_s,
                .left_cm_s = m->left_speed_cm_s,
                .right_cm_s = m->right_speed_cm_s};
            buddy4_obstruction_update(&a->observer, &in, now);
            a->pending = false;
            a->has_imu = true;
            a->last_imu_us = p_o->timestamp_us;
        }
        else
        {
            /* no action required */
        }

        /* Equal IMU timestamp is ordinary phase skew between two 20 ms tasks.
         * Keep the motor snapshot pending, without advancing evidence, until a
         * NEW IMU arrives. All eligibility/age/skew checks above and the core's
         * getter expiry still apply on every poll. A second motor callback
         * while
         * waiting is unqualified by note_update(), never silently combined. */
    }
    else
    {
        /* no action required */
    }

    return buddy4_obstruction_get(&a->observer, now);
}

/*** end of file ***/
