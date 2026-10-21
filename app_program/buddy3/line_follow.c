
/**
 * @file line_follow.c
 * @brief Two-sensor line following (decision logic from ir_motor_curve_test.c)
 *        driving the wheels through Buddy 2's motion_drive().
 *
 *   both on line   -> straight
 *   one on line    -> steer toward the side that still sees it (inside wheel slower)
 *   none on line   -> keep turning toward the last side for LINE_RECOVER_MS, then stop
 */
#include "line_follow.h"

#include "line_config.h"
#include "line_sensors.h"
#include "motion_control.h"

static line_state_t s_state = LINE_STATE_IDLE;
static uint8_t s_mask;
static float s_left_cmd;
static float s_right_cmd;
static int32_t s_last_side;      /* +1 line last seen on the left, -1 on the right */
static uint32_t s_lost_ms;
static uint32_t s_side_ms;       /* how long only one sensor has seen the line */
static bool s_active;

static void drive(float left, float right)
{
    s_left_cmd = left * LINE_LEFT_WHEEL_SCALE;
    s_right_cmd = right * LINE_RIGHT_WHEEL_SCALE;
    (void)motion_drive(s_left_cmd, s_right_cmd);
}

static void steer(int32_t side, float inner)
{
    if (side > 0)
    {
        drive(inner, LINE_TURN_OUTER_SPEED_CM_S);      /* turn left */
    }
    else
    {
        drive(LINE_RIGHT_OUTER_SPEED_CM_S, inner);     /* turn right: left wheel faster */
    }
}

void line_follow_init(void)
{
    line_sensors_init();
    s_state = LINE_STATE_IDLE;
    s_active = false;
    s_last_side = 0;
    s_lost_ms = 0u;
    s_side_ms = 0u;
}

bool line_follow_start(void)
{
    motion_status_t status;

    motion_get_status(&status);
    if (status.state == MOTION_STATE_ERROR)
    {
        s_state = LINE_STATE_FAULT;
        return false;
    }
    s_active = true;
    s_last_side = 0;
    s_lost_ms = 0u;
    s_side_ms = 0u;
    s_state = LINE_STATE_STOPPED;
    return true;
}

void line_follow_stop(void)
{
    s_active = false;
    s_state = LINE_STATE_IDLE;
    motion_stop();
}

void line_follow_update(void)
{
    motion_status_t status;
    bool left;
    bool right;

    if ((!s_active) || (s_state == LINE_STATE_FAULT))
    {
        return;
    }

    motion_get_status(&status);
    if (status.state == MOTION_STATE_ERROR)
    {
        /* Do not automatically clear motor faults. A stuck wheel needs
           inspection before the vehicle can resume. */
        s_state = LINE_STATE_FAULT;
        s_left_cmd = 0.0f;
        s_right_cmd = 0.0f;
        return;
    }

    s_mask = line_sensors_read();
    left = ((s_mask & LINE_SENSOR_LEFT) != 0u);
    right = ((s_mask & LINE_SENSOR_RIGHT) != 0u);

    if (left && right && (LINE_SENSORS_STRADDLE != 0u))
    {
        /* both sensors on black while straddling: junction or barcode area, keep going */
        s_lost_ms = 0u;
        s_side_ms = 0u;
        s_state = LINE_STATE_STRAIGHT;
        drive(LINE_STRAIGHT_SPEED_CM_S, LINE_STRAIGHT_SPEED_CM_S);
    }
    else if (left && right)
    {
        s_lost_ms = 0u;
        s_side_ms = 0u;
        s_state = LINE_STATE_STRAIGHT;
        drive(LINE_STRAIGHT_SPEED_CM_S, LINE_STRAIGHT_SPEED_CM_S);
    }
    else if (left)
    {
        s_lost_ms = 0u;
        s_last_side = 1;
        s_side_ms = (s_state == LINE_STATE_STEER_LEFT) ? (s_side_ms + LINE_UPDATE_PERIOD_MS) : 0u;
        s_state = LINE_STATE_STEER_LEFT;
        steer(1, ((LINE_ESCALATE_AFTER_MS != 0u) && (s_side_ms >= LINE_ESCALATE_AFTER_MS)) ? LINE_ESCALATE_INNER_CM_S
                                                      : LINE_TURN_INNER_SPEED_CM_S);
    }
    else if (right)
    {
        s_lost_ms = 0u;
        s_last_side = -1;
        s_side_ms = (s_state == LINE_STATE_STEER_RIGHT) ? (s_side_ms + LINE_UPDATE_PERIOD_MS) : 0u;
        s_state = LINE_STATE_STEER_RIGHT;
        steer(-1, ((LINE_ESCALATE_AFTER_MS != 0u) && (s_side_ms >= LINE_ESCALATE_AFTER_MS)) ? LINE_ESCALATE_INNER_CM_S
                                                       : LINE_TURN_INNER_SPEED_CM_S);
    }
    else if (LINE_SENSORS_STRADDLE != 0u)
    {
        /* line is between the sensors: this is the normal on-track state */
        s_side_ms = 0u;
        s_lost_ms += LINE_UPDATE_PERIOD_MS;
        if (s_lost_ms < LINE_STRADDLE_STOP_MS)
        {
            s_state = LINE_STATE_STRAIGHT;
            drive(LINE_STRAIGHT_SPEED_CM_S, LINE_STRAIGHT_SPEED_CM_S);
        }
        else if (s_state != LINE_STATE_STOPPED)
        {
            s_state = LINE_STATE_STOPPED;
            s_left_cmd = 0.0f;
            s_right_cmd = 0.0f;
            motion_stop();
        }
        else
        {
            /* already stopped */
        }
    }
    else
    {
        s_lost_ms += LINE_UPDATE_PERIOD_MS;
        if ((s_last_side != 0) && (s_lost_ms <= LINE_RECOVER_MS))
        {
            s_state = LINE_STATE_RECOVERING;
            steer(s_last_side, LINE_RECOVER_INNER_CM_S);
        }
        else
        {
            if (s_state != LINE_STATE_STOPPED)
            {
                s_state = LINE_STATE_STOPPED;
                s_left_cmd = 0.0f;
                s_right_cmd = 0.0f;
                motion_stop();
            }
        }
    }
}

void line_follow_get_status(line_status_t *p_status)
{
    if (p_status != 0)
    {
        p_status->state = s_state;
        p_status->sensor_mask = s_mask;
        p_status->left_cmd_cm_s = s_left_cmd;
        p_status->right_cmd_cm_s = s_right_cmd;
    }
}
