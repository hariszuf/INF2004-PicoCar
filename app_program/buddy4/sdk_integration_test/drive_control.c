#include "../../barrc_types.h"
#include "drive_control.h"
#include "line_follow.h"
#include "motion_config.h"
#include <math.h>
void drive_stop (drive_control_t * c, const char * why, motion_fault_t fault)
{
    line_follow_stop();
    motion_emergency_stop();
    c->running = c->armed = false;
    c->reason = why;
    c->stop_fault = fault;
    c->event_pending = true;
}

void drive_init (drive_control_t * c)
{
    *c = (drive_control_t){.mode = DRIVE_MANUAL};
    c->selected_left_cm_s = c->selected_right_cm_s = MANUAL_SPEED_CM_S;
    drive_stop(c, "BOOT", MOTION_FAULT_NONE);
}

bool drive_imu_usable (const buddy4_imu_t * imu)
{
    const buddy4_output_t * o = &imu->output;
    return (imu->ready) && (o->valid) && (o->acceleration_valid)
            && (o->tilt_valid) &&
            (BUDDY4_QUALITY_GRAVITY_LIKE == o->quality) && (!o->clipped_axes) &&
            (isfinite(o->magnitude_g)) && (isfinite(o->accel_g[0])) &&
            (isfinite(o->accel_g[1])) && (isfinite(o->accel_g[2])) &&
            (isfinite(o->pitch_deg)) && (isfinite(o->roll_deg)) &&
            (isfinite(o->filtered_pitch_deg))
                    && (isfinite(o->filtered_roll_deg));
}

bool drive_imu_poll (drive_control_t * c, const buddy4_imu_t * imu,
                     buddy4_result_t result)
{
    bool usable = (BUDDY4_FRESH == result) && (drive_imu_usable(imu));

    if ((BUDDY4_FRESH == result) && (!usable) && ((c->running) || (c->armed)))
    {
        drive_stop(c, "IMU_QUALITY", MOTION_FAULT_NONE);
    }

    return usable;
}

void drive_safety (drive_control_t * c, bool usb, bool has_fresh,
                   uint64_t now, uint64_t last_fresh)
{
    motion_status_t s;
    motion_get_status(&s);

    if ((!usb) && ((c->running) || (c->armed)))
    {
        drive_stop(c, "USB_DISCONNECTED", MOTION_FAULT_NONE);
    }
    else if (((c->running) || (c->armed)) &&
             ((!has_fresh) || (now - last_fresh >= INVALID_LIMIT_US)))
    {
        drive_stop(c, "IMU_INVALID_TIMEOUT", MOTION_FAULT_NONE);
    }
    else if (((c->running) || (c->armed)) &&
             ((MOTION_FAULT_NONE != s.fault)
              || (MOTION_STATE_ERROR == s.state)))
    {
        drive_stop(c, "MOTOR_FAULT", s.fault);
    }
    else if ((c->running)
             && ((DRIVE_TURN_LEFT == c->mode)
                 || (DRIVE_TURN_RIGHT == c->mode)) &&
             ((now < c->started_us)
              || (now - c->started_us >= TURN_TEST_LIMIT_US)))
    {
        drive_stop(c, "TURN_TEST_TIMEOUT", MOTION_FAULT_NONE);
    }
    else
    {
        /* no action required */
    }
}

void drive_command (drive_control_t * c, int32_t key, bool usb, bool healthy,
                    uint64_t now)
{
    if (('S' == key) || (' ' == key))
    {
        drive_stop(c, "SERIAL_STOP", MOTION_FAULT_NONE);
        return;
    }

    if (!usb)
    {
        return;
    }

    if ((('M' == key) || ('L' == key) || ('A' == key) || ('D' == key))
        && (!c->running))
    {
        drive_stop(c, "MODE_SELECTED", MOTION_FAULT_NONE);
        c->mode = 'M' == key ? DRIVE_MANUAL : 'L' == key ? DRIVE_LINE :
        'A' == key ? DRIVE_TURN_LEFT : DRIVE_TURN_RIGHT;
        c->selected_left_cm_s = DRIVE_MANUAL == c->mode ? MANUAL_SPEED_CM_S :
        DRIVE_TURN_LEFT == c->mode ? -TURN_TEST_SPEED_CM_S :
        DRIVE_TURN_RIGHT == c->mode ? TURN_TEST_SPEED_CM_S : 0;
        c->selected_right_cm_s = DRIVE_MANUAL == c->mode ? MANUAL_SPEED_CM_S :
        DRIVE_TURN_LEFT == c->mode ? TURN_TEST_SPEED_CM_S :
        DRIVE_TURN_RIGHT == c->mode ? -TURN_TEST_SPEED_CM_S : 0;
        return;
    }

    motion_status_t s;
    motion_get_status(&s);

    if (('R' == key) && (!c->running) && healthy
        && (MOTION_STATE_ERROR == s.state))
    {
        if (MOTION_OK == motion_reset())
        {
            c->armed = true;
            c->reason = "ARMED";
            c->stop_fault = MOTION_FAULT_NONE;
            c->event_pending = true;
        }
    }
    else if (('G' == key) && (!c->running) && (c->armed) && healthy &&
             (MOTION_STATE_IDLE == s.state))
    {
        if ((DRIVE_MANUAL != c->mode) && (DRIVE_LINE != c->mode) &&
            ((!isfinite(TURN_TEST_SPEED_CM_S)) || (TURN_TEST_SPEED_CM_S <= 0) ||
             (TURN_TEST_SPEED_CM_S > MOTION_MAX_SPEED_CM_S)))
        {
            drive_stop(c, "INVALID_TURN_TARGET", MOTION_FAULT_NONE);
            return;
        }

        bool ok = DRIVE_LINE == c->mode ? line_follow_start() :
        MOTION_OK == motion_drive(c->selected_left_cm_s,
                                  c->selected_right_cm_s);

        if (ok)
        {
            c->started_us = now;
            c->running = true;
            c->armed = false;
            c->reason = "START";
            c->event_pending = true;
        }
        else
        {
            drive_stop(c, "START_FAILED", s.fault);
        }
    }
    else
    {
        /* no action required */
    }
}

void drive_line_update (drive_control_t * c)
{
    /* Buddy 3 updates run only in explicitly selected line mode. */
    if ((!c->running) || (DRIVE_LINE != c->mode))
    {
        return;
    }

    line_follow_update();
    line_status_t s;
    line_follow_get_status(&s);

    if (LINE_STATE_STOPPED == s.state)
    {
        drive_stop(c, "LINE_LOST", MOTION_FAULT_NONE);
    }
    else if (LINE_STATE_FAULT == s.state)
    {
        motion_status_t m;
        motion_get_status(&m);
        drive_stop(c, "MOTOR_FAULT", m.fault);
    }
    else
    {
        /* no action required */
    }
}

/*** end of file ***/
