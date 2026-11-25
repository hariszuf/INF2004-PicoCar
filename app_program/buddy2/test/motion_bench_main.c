/**
 * @file motion_bench_main.c
 * @brief Stand-alone serial bench for Motion Control (no RTOS).
 *
 * Build this as its OWN executable (see README) and open a serial terminal
 * (115200 baud, USB CDC). Use it to collect the data needed for
 * docs/PID_Tuning_Report.md and docs/Motion_Accuracy_Evaluation.md.
 *
 * SAFETY: put the car on a block (wheels off the ground) for the open-loop
 * tests 'd' and 'k'. Press 'e' for an emergency stop at any time.
 */
#include "../../barrc_types.h"
#include <stdio.h>

#include "pico/stdlib.h"

#include "motion_config.h"
#include "motion_control.h"
#include "motion_encoder.h"
#include "motion_motor.h"

#define LOG_PERIOD_MS       (100u)
#define SWEEP_SETTLE_MS     (600u)
#define SWEEP_MEASURE_MS    (1000u)
#define BRAKE_TEST_DUTY     (0.6f)

static bool s_logging = false;

static float32_t cm_per_count (motion_side_t side)
{
    float32_t cpr = (MOTION_SIDE_LEFT == side) ? MOTION_COUNTS_PER_REV_LEFT
            : MOTION_COUNTS_PER_REV_RIGHT;
    return (MOTION_PI * MOTION_WHEEL_DIAMETER_CM) / cpr;
}

static void print_help (void)
{
    printf("\nMotion bench commands\n");
    printf(" f/F  forward 50/100 cm      b  backward 50 cm\n");
    printf(" l/r  turn left/right 90     u  U-turn (left 180)   L  spin left "
           "360\n");
    printf(" s    stop                   e  EMERGENCY STOP      x  reset "
           "after error\n");
    printf(" o    zero odometry          p  print status once   c  toggle CSV "
           "log\n");
    printf(" t    step test (drive 20 cm/s for 3 s, then stop)\n");
    printf(" d    open-loop duty sweep (WHEELS OFF GROUND)\n");
    printf(" k    brake/coast test     (WHEELS OFF GROUND)\n");
    printf(" h    help\n");
}

static void print_status (void)
{
    motion_status_t status;

    motion_get_status(&status);
    printf("state=%d fault=%d countL=%ld countR=%ld vL=%.1f vR=%.1f dist=%.2f "
           "cm head=%.2f deg\n",
           (int32_t)status.state, (int32_t)status.fault,
                   (int32_t)status.left_count,
           (int32_t)status.right_count, (float64_t)status.left_speed_cm_s,
           (float64_t)status.right_speed_cm_s, (float64_t)status.distance_cm,
           (float64_t)status.heading_deg);
}

static void print_csv_row (uint32_t time_ms)
{
    motion_status_t status;

    motion_get_status(&status);
    printf("%lu,%d,%ld,%ld,%.2f,%.2f,%.2f,%.2f\n", (uint32_t)time_ms,
           (int32_t)status.state, (int32_t)status.left_count,
                   (int32_t)status.right_count,
           (float64_t)status.left_speed_cm_s,
                   (float64_t)status.right_speed_cm_s,
           (float64_t)status.distance_cm, (float64_t)status.heading_deg);
}

/* Measure steady wheel speed at fixed duty. Motors driven directly. */
static void measure_speed (float32_t duty, float32_t * p_left,
                           float32_t * p_right)
{
    int32_t l0;
    int32_t r0;
    float32_t seconds = (float32_t)SWEEP_MEASURE_MS / 1000.0f;

    motion_encoder_set_direction(MOTION_SIDE_LEFT, (duty < 0.0f) ? -1 : 1);
    motion_encoder_set_direction(MOTION_SIDE_RIGHT, (duty < 0.0f) ? -1 : 1);
    motion_motor_set(MOTION_SIDE_LEFT, duty);
    motion_motor_set(MOTION_SIDE_RIGHT, duty);
    sleep_ms(SWEEP_SETTLE_MS);
    l0 = motion_encoder_get_count(MOTION_SIDE_LEFT);
    r0 = motion_encoder_get_count(MOTION_SIDE_RIGHT);
    sleep_ms(SWEEP_MEASURE_MS);
    *p_left = (float32_t)(motion_encoder_get_count(MOTION_SIDE_LEFT) - l0) *
            cm_per_count(MOTION_SIDE_LEFT) / seconds;
    *p_right = (float32_t)(motion_encoder_get_count(MOTION_SIDE_RIGHT) - r0) *
            cm_per_count(MOTION_SIDE_RIGHT) / seconds;
}

static void run_open_loop_sweep (void)
{
    float32_t duty;
    float32_t left;
    float32_t right;

    printf("\nduty,left_cm_s,right_cm_s\n");

    for (duty = 0.10f; duty <= 1.001f; duty += 0.10f)
    {
        measure_speed(duty, &left, &right);
        printf("%.2f,%.2f,%.2f\n", (float64_t)duty, (float64_t)left,
               (float64_t)right);
    }

    motion_motor_brake_all();
    printf("done. Use rows to find the dead zone (lowest duty that moves),\n");
    printf("the max speed, and the feed-forward slope (duty per cm/s).\n");
}

static void run_brake_test (void)
{
    float32_t left;
    float32_t right;
    int32_t l0;
    int32_t r0;
    float32_t coast_left;
    float32_t coast_right;
    float32_t speed;

    measure_speed(BRAKE_TEST_DUTY, &left, &right);
    l0 = motion_encoder_get_count(MOTION_SIDE_LEFT);
    r0 = motion_encoder_get_count(MOTION_SIDE_RIGHT);
    motion_motor_brake_all();
    sleep_ms(1000u);
    coast_left = (float32_t)(motion_encoder_get_count(MOTION_SIDE_LEFT) - l0) *
            cm_per_count(MOTION_SIDE_LEFT);
    coast_right =
            (float32_t)(motion_encoder_get_count(MOTION_SIDE_RIGHT) - r0) *
            cm_per_count(MOTION_SIDE_RIGHT);
    speed = 0.5f * (left + right);

    printf("speed before brake: L %.2f R %.2f cm/s\n", (float64_t)left,
           (float64_t)right);
    printf("distance after brake: L %.2f R %.2f cm\n", (float64_t)coast_left,
           (float64_t)coast_right);

    if (speed > 1.0f)
    {
        printf("suggested MOTION_STOP_LEAD_S = %.3f s\n",
               (float64_t)(0.5f * (coast_left + coast_right) / speed));
    }
}

static void run_step_test (void)
{
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    uint32_t next_update_ms = start_ms;
    uint32_t next_log_ms = start_ms;
    uint32_t now_ms;
    bool b_stopped = false;

    printf("\ntime_ms,state,countL,countR,vL,vR,dist,heading\n");
    (void)motion_drive(20.0f, 20.0f);

    for (;;)
    {
        now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms >= next_update_ms)
        {
            motion_update();
            next_update_ms += MOTION_UPDATE_PERIOD_MS;
        }

        if (now_ms >= next_log_ms)
        {
            print_csv_row(now_ms - start_ms);
            next_log_ms += 20u;
        }

        if ((!b_stopped) && ((now_ms - start_ms) >= 3000u))
        {
            motion_stop();
            b_stopped = true;
        }

        if (b_stopped && (motion_is_idle()) && ((now_ms - start_ms) >= 3500u))
        {
            break;
        }
    }

    printf("step test finished\n");
}

static void handle_command (int32_t key)
{
    switch (key)
    {
        case 'f':
        (void)motion_move_forward(50.0f);
        break;
        case 'F':
        (void)motion_move_forward(100.0f);
        break;
        case 'b':
        (void)motion_move_backward(50.0f);
        break;
        case 'l':
        (void)motion_turn_left(90.0f);
        break;
        case 'r':
        (void)motion_turn_right(90.0f);
        break;
        case 'u':
        (void)motion_turn_left(180.0f);
        break;
        case 'L':
        (void)motion_turn_left(360.0f);
        break;
        case 's':
        motion_stop();
        break;
        case 'e':
        motion_emergency_stop();
        printf("EMERGENCY STOP\n");
        break;
        case 'x':
        (void)motion_reset();
        printf("reset\n");
        break;
        case 'o':
        printf("odometry reset result %d\n", (int32_t)motion_reset_odometry());
        break;
        case 'p':
        print_status();
        break;
        case 'c':
        s_logging = !s_logging;
        printf("log %s\n", s_logging ? "on" : "off");
        break;
        case 't':
        run_step_test();
        break;
        case 'd':
        run_open_loop_sweep();
        break;
        case 'k':
        run_brake_test();
        break;
        case 'h':
        print_help();
        break;
        default:
        break;
    }
}

int main (void)
{
    uint32_t next_update_ms;
    uint32_t next_log_ms;
    uint32_t now_ms;
    int32_t key;

    stdio_init_all();
    sleep_ms(2000u);
    (void)motion_init();
    print_help();

    next_update_ms = to_ms_since_boot(get_absolute_time());
    next_log_ms = next_update_ms;

    for (;;)
    {
        now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms >= next_update_ms)
        {
            motion_update();
            next_update_ms += MOTION_UPDATE_PERIOD_MS;
        }

        if (s_logging && (now_ms >= next_log_ms))
        {
            print_csv_row(now_ms);
            next_log_ms += LOG_PERIOD_MS;
        }

        key = getchar_timeout_us(0u);

        if (PICO_ERROR_TIMEOUT != key)
        {
            handle_command(key);
        }
    }
}

/*** end of file ***/
