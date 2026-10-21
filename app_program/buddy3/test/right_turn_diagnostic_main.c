/**
 * @file right_turn_diagnostic_main.c
 * @brief Controlled, one-time left/right motor test using the original Buddy 2
 *        motion API. No line following and no autonomous repeat.
 *
 * SAFETY: Raise BOTH wheels completely clear of the floor and secure the car.
 * Motor power must be easy to disconnect. This is not a line-follower.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "motion_control.h"
#include "motion_config.h"

#define START_DELAY_MS      5000u
#define STEP_MS             2200u
#define STOP_MS             1200u
#define TEST_SPEED_CM_S     6.0f
#define REPORT_MS          200u

static void run_interval(uint32_t ms, const char *label)
{
    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint32_t next_motion = start;
    uint32_t next_print = start;
    motion_status_t state;

    while ((uint32_t)(to_ms_since_boot(get_absolute_time()) - start) < ms)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((int32_t)(now - next_motion) >= 0)
        {
            next_motion += MOTION_UPDATE_PERIOD_MS;
            motion_update();
        }
        if ((int32_t)(now - next_print) >= 0)
        {
            next_print += REPORT_MS;
            motion_get_status(&state);
            printf("%s actual L/R=%.2f/%.2f cm/s counts L/R=%ld/%ld state=%d fault=%d\n",
                   label, (double)state.left_speed_cm_s,
                   (double)state.right_speed_cm_s,
                   (long)state.left_count, (long)state.right_count,
                   (int)state.state, (int)state.fault);
        }
        sleep_ms(1);
    }
}

static bool no_motor_fault(void)
{
    motion_status_t status;
    motion_get_status(&status);
    return status.state != MOTION_STATE_ERROR;
}

int main(void)
{
    stdio_init_all();
    printf("Right-turn motor diagnostic: WHEELS MUST BE RAISED!\n");
    sleep_ms(START_DELAY_MS);

    if (motion_init() != MOTION_OK)
    {
        printf("motor init failed\n");
        for (;;) sleep_ms(1000);
    }

    printf("TEST 1: LEFT MOTOR only (this is the outside wheel for RIGHT turns)\n");
    (void)motion_drive(TEST_SPEED_CM_S, 0.0f);
    run_interval(STEP_MS, "LEFT MOTOR");
    motion_stop();
    run_interval(STOP_MS, "STOP");

    if (no_motor_fault())
    {
        printf("TEST 2: RIGHT MOTOR only (this is the outside wheel for LEFT turns)\n");
        (void)motion_drive(0.0f, TEST_SPEED_CM_S);
        run_interval(STEP_MS, "RIGHT MOTOR");
        motion_stop();
        run_interval(STOP_MS, "STOP");
    }
    else
    {
        printf("Motor fault during TEST 1: inspect hardware; test aborted.\n");
    }

    printf("DIAGNOSTIC FINISHED. MOTORS STOPPED.\n");
    motion_stop();
    for (;;)
    {
        motion_update();
        sleep_ms(MOTION_UPDATE_PERIOD_MS);
    }
}
