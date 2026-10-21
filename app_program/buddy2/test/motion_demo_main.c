/**
 * @file motion_demo_main.c
 * @brief Autonomous Motion Control test route (no RTOS, no remote control).
 *
 * Stand-in for the Vehicle Controller while Motion Control is tested alone.
 * After power-on the car waits DEMO_START_DELAY_MS (time to put it on the
 * floor), then runs the route in s_route[] by itself and stops. Nothing needs
 * to be typed or pressed. Switch the Robo Pico off to abort.
 *
 * Default route: a square (4 x forward, 4 x left 90) then a U-turn and a short
 * reverse. If Motion Control is accurate, the car ends where it started.
 * Needs roughly 1 m x 1 m of clear floor.
 *
 * Progress is printed over USB serial if a terminal happens to be connected,
 * which is optional.
 */
#include <stdio.h>

#include "pico/stdlib.h"

#include "motion_config.h"
#include "motion_control.h"

#define DEMO_START_DELAY_MS     (5000u)
#define DEMO_PAUSE_MS           (800u)
#define DEMO_SIDE_CM            (50.0f)

typedef enum
{
    DEMO_FORWARD = 0,
    DEMO_BACKWARD,
    DEMO_LEFT,
    DEMO_RIGHT
} demo_action_t;

typedef struct
{
    demo_action_t action;
    float value;            /* cm for forward / backward, degrees for turns */
} demo_step_t;

static const demo_step_t s_route[] =
{
    { DEMO_FORWARD,  DEMO_SIDE_CM }, { DEMO_LEFT, 90.0f },
    { DEMO_FORWARD,  DEMO_SIDE_CM }, { DEMO_LEFT, 90.0f },
    { DEMO_FORWARD,  DEMO_SIDE_CM }, { DEMO_LEFT, 90.0f },
    { DEMO_FORWARD,  DEMO_SIDE_CM }, { DEMO_LEFT, 90.0f },
    { DEMO_LEFT,     180.0f },
    { DEMO_BACKWARD, 30.0f }
};

#define ROUTE_STEPS     ((uint32_t)(sizeof(s_route) / sizeof(s_route[0])))

static void print_status(const char *label)
{
    motion_status_t status;

    motion_get_status(&status);
    printf("%s: state=%d fault=%d dist=%.1f cm heading=%.1f deg countL=%ld countR=%ld\n",
           label, (int)status.state, (int)status.fault,
           (double)status.distance_cm, (double)status.heading_deg,
           (long)status.left_count, (long)status.right_count);
}

static motion_result_t issue_step(const demo_step_t *p_step)
{
    motion_result_t result;

    switch (p_step->action)
    {
        case DEMO_FORWARD:
            result = motion_move_forward(p_step->value);
            break;
        case DEMO_BACKWARD:
            result = motion_move_backward(p_step->value);
            break;
        case DEMO_LEFT:
            result = motion_turn_left(p_step->value);
            break;
        default:
            result = motion_turn_right(p_step->value);
            break;
    }
    return result;
}

int main(void)
{
    uint32_t next_update_ms;
    uint32_t pause_until_ms;
    uint32_t now_ms;
    uint32_t step = 0u;
    bool busy = false;
    bool finished = false;
    motion_status_t status;

    stdio_init_all();
    sleep_ms(DEMO_START_DELAY_MS);

    (void)motion_init();
    (void)motion_reset_odometry();

    now_ms = to_ms_since_boot(get_absolute_time());
    next_update_ms = now_ms;
    pause_until_ms = now_ms;

    for (;;)
    {
        now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms >= next_update_ms)
        {
            motion_update();
            next_update_ms += MOTION_UPDATE_PERIOD_MS;
        }

        motion_get_status(&status);
        if (status.state == MOTION_STATE_ERROR)
        {
            if (!finished)
            {
                print_status("FAULT, route aborted");
                finished = true;
            }
        }
        else if (!motion_is_idle())
        {
            busy = true;
        }
        else if (busy)
        {
            busy = false;
            pause_until_ms = now_ms + DEMO_PAUSE_MS;
            print_status("step done");
        }
        else if ((!finished) && (now_ms >= pause_until_ms))
        {
            if (step < ROUTE_STEPS)
            {
                (void)issue_step(&s_route[step]);
                step++;
            }
            else
            {
                print_status("route finished");
                finished = true;
            }
        }
        else
        {
            /* waiting for the pause between steps, or finished */
        }
    }
}
