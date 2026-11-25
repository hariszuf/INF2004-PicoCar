/**
 * @file test_motion.c
 * @brief Host simulation tests for the Motion Control state machine.
 *
 * Build and run:  see test/host/run_tests.sh
 * These tests check logic against a simulated robot. They are not evidence of
 * real-robot accuracy.
 */
#include "../../../barrc_types.h"
#include <math.h>
#include <stdio.h>

#include "motion_config.h"
#include "motion_control.h"
#include "sim_hw.h"

static int32_t s_failures = 0;
static int32_t s_checks = 0;

#define CHECK(condition, text)                                              \
    do                                                                      \
    {                                                                       \
        s_checks++;                                                         \
        if (condition)                                                      \
        {                                                                   \
            printf("  PASS  %s\n", (text));                                 \
        }                                                                   \
        else                                                                \
        {                                                                   \
            printf("  FAIL  %s\n", (text));                                 \
            s_failures++;                                                   \
        }                                                                   \
    } while (0)

/* Run the 20 ms control loop until idle. Returns elapsed ms, or -1 on timeout.
 */
static int32_t run_until_idle (int32_t max_ms)
{
    int32_t elapsed = 0;

    while (elapsed < max_ms)
    {
        sim_advance_ms(MOTION_UPDATE_PERIOD_MS);
        motion_update();
        elapsed += (int32_t)MOTION_UPDATE_PERIOD_MS;

        if (motion_is_idle())
        {
            return elapsed;
        }
    }

    return -1;
}

static void run_for (int32_t ms)
{
    int32_t elapsed = 0;

    while (elapsed < ms)
    {
        sim_advance_ms(MOTION_UPDATE_PERIOD_MS);
        motion_update();
        elapsed += (int32_t)MOTION_UPDATE_PERIOD_MS;
    }
}

static void test_move (const char * name, float32_t distance_cm, bool b_forward)
{
    float64_t start = sim_true_distance_cm();
    float64_t start_heading = sim_true_heading_deg();
    int32_t elapsed;
    float64_t travelled;
    float64_t error;
    float64_t drift;
    motion_status_t status;

    printf("\n[%s]\n", name);

    if (b_forward)
    {
        motion_move_forward(distance_cm);
    }
    else
    {
        motion_move_backward(distance_cm);
    }

    elapsed = run_until_idle(15000);
    travelled = fabs(sim_true_distance_cm() - start);
    error = travelled - (float64_t)distance_cm;
    drift = sim_true_heading_deg() - start_heading;
    motion_get_status(&status);

    printf("  target %.1f cm, true %.2f cm, error %+.2f cm, time %d ms, "
           "heading drift %+.2f deg\n",
           (float64_t)distance_cm, travelled, error, elapsed, drift);
    CHECK(elapsed > 0, "move finishes (no timeout)");
    CHECK(fabs(error) < 1.0, "distance error within 1 cm");
    CHECK(fabs(drift) < 3.0,
          "heading drift within 3 deg (straight-line correction)");
    CHECK(MOTION_STATE_IDLE == status.state, "returns to IDLE");
}

static void test_turn (const char * name, float32_t angle_deg, bool left)
{
    float64_t start_heading = sim_true_heading_deg();
    int32_t elapsed;
    float64_t turned;
    /* The simulated robot has no wheel slip, so it turns by commanded angle x
     * MOTION_TURN_SCALE. On the real car the scale compensates for slip. */
    float64_t scaled = (float64_t)angle_deg * (float64_t)MOTION_TURN_SCALE;
    float64_t expected = left ? scaled : -scaled;

    printf("\n[%s]\n", name);

    if (left)
    {
        motion_turn_left(angle_deg);
    }
    else
    {
        motion_turn_right(angle_deg);
    }

    elapsed = run_until_idle(15000);
    turned = sim_true_heading_deg() - start_heading;

    printf("  target %+.0f deg, true %+.2f deg, error %+.2f deg, time %d ms\n",
           expected, turned, turned - expected, elapsed);
    CHECK(elapsed > 0, "turn finishes (no timeout)");
    CHECK(fabs(turned - expected) < 3.0, "angle error within 3 deg");
}

static void test_estop (void)
{
    motion_status_t status;
    motion_result_t result;

    printf("\n[emergency stop]\n");
    motion_move_forward(100.0f);
    run_for(600);
    motion_emergency_stop();

    CHECK((sim_is_braked(MOTION_SIDE_LEFT))
          && (sim_is_braked(MOTION_SIDE_RIGHT)),
          "motors braked immediately by motion_emergency_stop()");
    run_for(MOTION_UPDATE_PERIOD_MS);
    motion_get_status(&status);
    CHECK((MOTION_STATE_ERROR == status.state)
          && (MOTION_FAULT_ESTOP == status.fault),
          "state ERROR with fault ESTOP");

    result = motion_move_forward(50.0f);
    CHECK(MOTION_ERR_FAULT == result,
          "new movement command rejected while in ERROR");
    run_for(200);
    CHECK((sim_is_braked(MOTION_SIDE_LEFT))
          && (sim_is_braked(MOTION_SIDE_RIGHT)),
          "motors stay braked (command cannot override e-stop)");

    CHECK(MOTION_OK == motion_reset(), "motion_reset() accepted");
    run_for(MOTION_UPDATE_PERIOD_MS);
    motion_get_status(&status);
    CHECK((MOTION_STATE_IDLE == status.state)
          && (MOTION_FAULT_NONE == status.fault),
          "back to IDLE after reset");
    run_for(500);   /* let the wheels settle */
    motion_move_forward(20.0f);
    CHECK(run_until_idle(10000) > 0, "can move again after reset");
}

static void test_stall (void)
{
    motion_status_t status;
    int32_t ms = 0;

    printf("\n[stall detection]\n");
    motion_move_forward(100.0f);
    run_for(300);
    sim_set_stall(MOTION_SIDE_LEFT, true);

    for (ms = 0; ms < 3000; ms += (int32_t)MOTION_UPDATE_PERIOD_MS)
    {
        run_for(MOTION_UPDATE_PERIOD_MS);
        motion_get_status(&status);

        if (MOTION_STATE_ERROR == status.state)
        {
            break;
        }
    }

    printf("  fault detected %d ms after the wheel locked\n", ms);
    CHECK((MOTION_STATE_ERROR == status.state)
          && (MOTION_FAULT_STALL == status.fault),
          "locked wheel gives ERROR with fault STALL");
    CHECK((sim_is_braked(MOTION_SIDE_LEFT))
          && (sim_is_braked(MOTION_SIDE_RIGHT)),
          "motors braked on stall");
    sim_set_stall(MOTION_SIDE_LEFT, false);
    motion_reset();
    run_for(500);
}

/* Line following re-sends motion_drive() every 5 ms at a few cm/s. */
static void test_stall_while_line_following (void)
{
    motion_status_t status;
    int32_t ms;
    int32_t sub;
    float64_t before;

    printf("\n[stall detection while line following (drive re-sent every 5 "
           "ms, 3 cm/s)]\n");

    for (ms = 0; ms < 1000; ms += (int32_t)MOTION_UPDATE_PERIOD_MS)
    {
        for (sub = 0; sub < 4; sub++)
        {
            motion_drive(3.0f, 3.0f);
        }

        run_for(MOTION_UPDATE_PERIOD_MS);
    }

    motion_get_status(&status);
    CHECK(MOTION_STATE_EXECUTING == status.state,
          "slow driving alone is not a stall");

    before = sim_true_distance_cm();

    for (ms = 0; ms < 2000; ms += (int32_t)MOTION_UPDATE_PERIOD_MS)
    {
        for (sub = 0; sub < 4; sub++)
        {
            /* steering updates keep the car moving */
            motion_drive(3.5f, 2.5f);
        }

        run_for(MOTION_UPDATE_PERIOD_MS);
    }

    motion_get_status(&status);
    CHECK((MOTION_STATE_EXECUTING == status.state)
          && (sim_true_distance_cm() > before + 3.0),
          "steering updates do not interrupt driving");

    sim_set_stall(MOTION_SIDE_LEFT, true);

    for (ms = 0; ms < 5000; ms += (int32_t)MOTION_UPDATE_PERIOD_MS)
    {
        for (sub = 0; sub < 4; sub++)
        {
            motion_drive(3.0f, 3.0f);
        }

        run_for(MOTION_UPDATE_PERIOD_MS);
        motion_get_status(&status);

        if (MOTION_STATE_ERROR == status.state)
        {
            break;
        }
    }

    printf("  fault detected %d ms after the wheel locked\n", ms);
    CHECK((MOTION_STATE_ERROR == status.state)
          && (MOTION_FAULT_STALL == status.fault),
          "locked wheel detected even though drive commands keep arriving");
    sim_set_stall(MOTION_SIDE_LEFT, false);
    motion_reset();
    run_for(500);
}

static void test_drive_and_stop (void)
{
    motion_status_t status;

    printf("\n[continuous drive + stop]\n");
    motion_drive(15.0f, 15.0f);
    run_for(1500);
    motion_get_status(&status);
    printf("  speeds after 1.5 s: left %.1f, right %.1f cm/s\n",
           (float64_t)status.left_speed_cm_s,
           (float64_t)status.right_speed_cm_s);
    CHECK(fabs(status.left_speed_cm_s - 15.0) < 2.0,
          "left wheel reaches ~15 cm/s");
    CHECK(fabs(status.right_speed_cm_s - 15.0) < 2.0,
          "right wheel reaches ~15 cm/s");
    CHECK(MOTION_STATE_EXECUTING == status.state,
          "stays EXECUTING while driving");

    motion_drive(25.0f, 10.0f);   /* gentle left arc, new command mid-motion */
    run_for(1500);
    motion_get_status(&status);
    CHECK(status.heading_deg > 5.0, "differential speeds curve the robot left");

    motion_stop();
    CHECK(run_until_idle(5000) > 0, "motion_stop() returns to IDLE");
}

static void test_reversal_counts (void)
{
    float64_t start = sim_true_distance_cm();
    motion_status_t status;
    float64_t measured;
    float64_t truth;

    printf("\n[reverse command while moving (single-phase encoder guard)]\n");
    motion_reset_odometry();
    motion_move_forward(60.0f);
    run_for(800);
    motion_move_backward(30.0f);   /* replaces the forward move mid-motion */
    run_until_idle(15000);
    motion_get_status(&status);
    measured = (float64_t)status.distance_cm;
    truth = sim_true_distance_cm() - start;
    printf("  odometry %.2f cm vs true %.2f cm\n", measured, truth);
    CHECK(fabs(measured - truth) < 1.5,
          "odometry still matches true travel after a mid-move reversal");
}

static void test_invalid_args (void)
{
    printf("\n[argument checks]\n");
    CHECK(MOTION_ERR_INVALID_ARG == motion_move_forward(0.0f),
          "zero distance rejected");
    CHECK(MOTION_ERR_INVALID_ARG == motion_move_forward(-5.0f),
          "negative distance rejected");
    CHECK(MOTION_ERR_INVALID_ARG == motion_turn_left(0.0f),
          "zero angle rejected");
    CHECK(MOTION_ERR_INVALID_ARG == motion_turn_right(1000.0f),
          "angle above 720 rejected");
    CHECK(MOTION_ERR_INVALID_ARG == motion_set_speed(0.0f),
          "zero speed rejected");
    CHECK(MOTION_ERR_INVALID_ARG == motion_set_speed(1000.0f),
          "speed above max rejected");
}

int main (void)
{
    motion_init();
    run_for(100);

    test_invalid_args();
    test_move("forward 100 cm", 100.0f, true);
    test_move("forward 30 cm", 30.0f, true);
    test_move("backward 50 cm", 50.0f, false);
    test_turn("turn left 90", 90.0f, true);
    test_turn("turn right 90", 90.0f, false);
    test_turn("U-turn (left 180)", 180.0f, true);
    test_drive_and_stop();
    test_reversal_counts();
    test_estop();
    test_stall();
    test_stall_while_line_following();

    printf("\n%d checks, %d failed\n", s_checks, s_failures);
    return (0 == s_failures) ? 0 : 1;
}

/*** end of file ***/
