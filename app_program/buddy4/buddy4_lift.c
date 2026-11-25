#include "../barrc_types.h"
#include "buddy4_lift.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f

buddy4_lift_config_t buddy4_lift_default_config (void)
{
    /* Picking the car up by hand tilts it ~25-30 deg (seen on the car), so a
     * tilt of
     * 20 deg for 100 ms counts as lifted; the acceleration cue (0.10 g for 100
     * ms)
     * covers a level lift. Provisional values: tune on the car. */
    return (buddy4_lift_config_t)
    {
        -1.0f, 20.0f, 10.0f, 100.0f, 0.10f, 0.06f,
        100000u, 100000u, 1000000u, 60000u, 2000000u
    };
}

bool buddy4_lift_init (buddy4_lift_t * d, buddy4_lift_config_t c)
{
    *d = (buddy4_lift_t){.config = c};
    d->configured = ((1.0f == c.up_sign) || (-1.0f == c.up_sign))
            && (isfinite(c.tilt_enter_deg)) && (isfinite(c.tilt_exit_deg)) &&
            (isfinite(c.overturn_deg)) && (isfinite(c.accel_enter_g))
            && (isfinite(c.accel_exit_g)) &&
            (c.tilt_exit_deg > 0) && (c.tilt_enter_deg > c.tilt_exit_deg) &&
            (c.overturn_deg >= c.tilt_enter_deg) && (c.accel_exit_g >= 0) &&
            (c.accel_enter_g > c.accel_exit_g) && (c.tilt_confirm_us > 0) &&
            (c.accel_confirm_us > 0) && (c.clear_us > 0)
                    && (c.max_gap_us > 0) &&
            (c.invalid_release_us > 0);
    return d->configured;
}

static void clear_pending (buddy4_lift_t * d)
{
    d->tilt_pending = d->accel_pending = d->calm_pending = false;
}

const char * buddy4_lift_reason_name (int32_t reason)
{
    switch (reason)
    {
        case BUDDY4_LIFT_TILTED:
        return "TILTED";
        case BUDDY4_LIFT_OVERTURNED:
        return "OVERTURNED";
        case BUDDY4_LIFT_MOTION:
        return "LIFT_MOTION";
        default:
        return "NONE";
    }
}

void buddy4_lift_update (buddy4_lift_t * d, float32_t x, float32_t y,
                         float32_t z,
                         bool valid, uint64_t sample_us, uint64_t now_us)
{
    buddy4_lift_output_t * o = &d->output;
    float32_t mag = sqrtf(x * x + y * y + z * z);
    float32_t c_up = d->config.up_sign;
    bool ok = (d->configured) && valid && (isfinite(mag))
            && (sample_us <= now_us) &&
            (now_us - sample_us <= d->config.max_gap_us) &&
            ((!d->has_time) || (sample_us > d->last_time_us));

    if (!ok)
    {
        /* One poll without a new sample (the IMU updates slower than the 20 ms
         * poll) must not
         * flip the status to unavailable: keep the last verdict for hold_us. */
        if ((!d->has_time) || (now_us < d->last_time_us)
            || (now_us - d->last_time_us > 250000u))
        {
            o->valid = false;
        }

        clear_pending(d);

        if (!d->invalid_pending)
        {
            d->invalid_pending = true;
            d->invalid_since_us = now_us;
        }
        else if ((o->lifted) && (now_us >= d->invalid_since_us) &&
                 (now_us - d->invalid_since_us >= d->config.invalid_release_us))
        {
            o->lifted = false; /* sensor gone: unknown is not "lifted" */
        }
        else
        {
            /* no action required */
        }

        return;
    }

    d->invalid_pending = false;
    d->has_time = true;
    d->last_time_us = sample_us;
    o->valid = true;
    o->tilt_deg = atan2f(sqrtf(x * x + y * y), c_up * z) * RAD_TO_DEG;
    o->deviation_g = fabsf(mag - 1.0f);
    const buddy4_lift_config_t * c = &d->config;

    if (!o->lifted)
    {
        bool tilted = o->tilt_deg >= c->tilt_enter_deg;
        bool moving = o->deviation_g >= c->accel_enter_g;

        if (tilted)
        {
            if (!d->tilt_pending)
            {
                d->tilt_pending = true;
                d->tilt_since_us = sample_us;
            }
        }
        else
        {
            d->tilt_pending = false;
        }

        if (moving)
        {
            if (!d->accel_pending)
            {
                d->accel_pending = true;
                d->accel_since_us = sample_us;
            }
        }
        else
        {
            d->accel_pending = false;
        }

        uint8_t reason = BUDDY4_LIFT_NONE;

        if ((d->tilt_pending)
            && (sample_us - d->tilt_since_us >= c->tilt_confirm_us))
        {
            reason = o->tilt_deg >= c->overturn_deg ? BUDDY4_LIFT_OVERTURNED
                    : BUDDY4_LIFT_TILTED;
        }
        else if ((d->accel_pending)
                 && (sample_us - d->accel_since_us >= c->accel_confirm_us))
        {
            reason = BUDDY4_LIFT_MOTION;
        }
        else
        {
            /* no action required */
        }

        if (reason)
        {
            o->lifted = true;
            o->reason = reason;
            o->since_us = sample_us;

            if (UINT32_MAX != o->count)
            {
                o->count++;
            }

            clear_pending(d);
        }
    }
    else
    {
        /* Latched: release only after a continuous level, still period. */
        bool calm = (o->tilt_deg <= c->tilt_exit_deg)
                && (o->deviation_g <= c->accel_exit_g);

        if (!calm)
        {
            d->calm_pending = false;
        }
        else if (!d->calm_pending)
        {
            d->calm_pending = true;
            d->calm_since_us = sample_us;
        }
        else if (sample_us - d->calm_since_us >= c->clear_us)
        {
            o->lifted = false;
            clear_pending(d);
        }
        else
        {
            /* no action required */
        }
    }
}

/*** end of file ***/
