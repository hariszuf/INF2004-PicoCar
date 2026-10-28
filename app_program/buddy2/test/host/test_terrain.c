/**
 * @file test_terrain.c
 * @brief Host simulation tests for terrain (slope / hump) speed control:
 *        Buddy 4 IMU report -> motion_set_terrain() -> Buddy 2 speed and power.
 *
 * The simulated slope is a made-up duty-equivalent load, NOT a model of the
 * real ramp. These tests check the control logic only.
 */
#include <math.h>
#include <stdio.h>

#include "motion_config.h"
#include "motion_control.h"
#include "sim_hw.h"

static int s_failures = 0;
static int s_checks = 0;

#define CHECK(condition, text)                                              \
    do                                                                      \
    {                                                                       \
        s_checks++;                                                         \
        if (condition) { printf("  PASS  %s\n", (text)); }                  \
        else { printf("  FAIL  %s\n", (text)); s_failures++; }              \
    } while (0)

static motion_terrain_input_t s_imu;     /* what "Buddy 4" reports */
static bool s_imu_on = false;
static int s_wrong_sign = 0;             /* duty against the commanded direction */

static void step(void)
{
    sim_advance_ms(MOTION_UPDATE_PERIOD_MS);
    if (s_imu_on)
    {
        (void)motion_set_terrain(&s_imu);   /* IMU task: every 20 ms */
    }
    motion_update();
    if ((sim_last_duty(MOTION_SIDE_LEFT) < 0.0) || (sim_last_duty(MOTION_SIDE_RIGHT) < 0.0))
    {
        s_wrong_sign++;
    }
}

static void run_for(int ms)
{
    int t;
    for (t = 0; t < ms; t += (int)MOTION_UPDATE_PERIOD_MS) { step(); }
}

/* Mean wheel speed over a window. */
static double mean_speed(int ms)
{
    motion_status_t st;
    double sum = 0.0;
    int n = 0;
    int t;
    for (t = 0; t < ms; t += (int)MOTION_UPDATE_PERIOD_MS)
    {
        step();
        motion_get_status(&st);
        sum += st.speed_cm_s;
        n++;
    }
    return sum / n;
}

static void set_imu(int8_t pitch_state, float pitch, int8_t roll_state, float roll)
{
    s_imu.valid = true;
    s_imu.pitch_state = pitch_state;
    s_imu.pitch_deg = pitch;
    s_imu.roll_state = roll_state;
    s_imu.roll_deg = roll;
    s_imu_on = true;
}

static void flat_world(void)
{
    s_imu_on = false;
    sim_set_load(MOTION_SIDE_LEFT, 0.0);
    sim_set_load(MOTION_SIDE_RIGHT, 0.0);
}

static void fresh_start(void)
{
    flat_world();
    if (!motion_is_idle()) { motion_stop(); run_for(1500); }
    (void)motion_reset();
    run_for(1500);   /* lets any old terrain report go stale */
}

static void test_api(void)
{
    motion_terrain_input_t in = { true, 0, 0, 0.0f, 0.0f };
    printf("\n[terrain API]\n");
    CHECK(motion_set_terrain((const motion_terrain_input_t *)0) == MOTION_ERR_INVALID_ARG,
          "null report rejected");
    in.pitch_deg = NAN;
    CHECK(motion_set_terrain(&in) == MOTION_ERR_INVALID_ARG, "non-finite angle rejected");
    in.pitch_deg = 0.0f; in.roll_state = 2;
    CHECK(motion_set_terrain(&in) == MOTION_ERR_INVALID_ARG, "out-of-range state rejected");
    in.roll_state = 0;
    CHECK(motion_set_terrain(&in) == MOTION_OK, "valid flat report accepted");
}

static void test_flat_unchanged(void)
{
    motion_status_t st;
    double v;
    printf("\n[flat ground: behaviour unchanged]\n");
    fresh_start();
    set_imu(0, 0.5f, 0, -0.3f);
    (void)motion_drive(15.0f, 15.0f);
    run_for(1500);
    v = mean_speed(500);
    motion_get_status(&st);
    printf("  speed %.2f cm/s, scale %.2f, assist %.3f/%.3f\n", v,
           (double)st.terrain_speed_scale, (double)st.left_terrain_duty, (double)st.right_terrain_duty);
    CHECK(st.terrain == MOTION_TERRAIN_FLAT, "terrain FLAT");
    CHECK(st.terrain_speed_scale == 1.0f, "speed scale 1.0");
    CHECK(fabs(st.left_terrain_duty) < 1e-6 && fabs(st.right_terrain_duty) < 1e-6, "no power assist");
    CHECK(fabs(v - 15.0) < 1.0, "holds commanded 15 cm/s");
}

/* Climb with and without the IMU report: the assist must keep speed better. */
static double uphill_run(bool with_imu, motion_status_t *p_st, double *p_dip)
{
    double v;
    double min_v = 1e9;
    int t;
    motion_status_t st;
    fresh_start();
    (void)motion_drive(10.0f, 10.0f);
    run_for(1500);
    sim_set_load(MOTION_SIDE_LEFT, 0.15);
    sim_set_load(MOTION_SIDE_RIGHT, 0.15);
    if (with_imu) { set_imu(1, 11.0f, 0, 0.0f); }
    for (t = 0; t < 600; t += (int)MOTION_UPDATE_PERIOD_MS)
    {
        step();
        motion_get_status(&st);
        if (st.speed_cm_s < min_v) { min_v = st.speed_cm_s; }
    }
    v = mean_speed(500);
    motion_get_status(p_st);
    *p_dip = min_v;
    return v;
}

static void test_uphill(void)
{
    motion_status_t st;
    motion_status_t st_no;
    double dip;
    double dip_no;
    double v;
    double v_no;
    printf("\n[uphill: power assist]\n");
    v_no = uphill_run(false, &st_no, &dip_no);
    v = uphill_run(true, &st, &dip);
    printf("  without IMU: lowest %.2f cm/s; with IMU: lowest %.2f cm/s, settled %.2f, assist %.3f\n",
           dip_no, dip, v, (double)st.left_terrain_duty);
    (void)v_no;
    CHECK(st.terrain == MOTION_TERRAIN_UPHILL, "terrain UPHILL");
    CHECK(st.left_terrain_duty > 0.05f && st.right_terrain_duty > 0.05f, "both wheels get extra power");
    CHECK(fabs(st.terrain_speed_scale - MOTION_UPHILL_SPEED_SCALE) < 1e-6, "uphill speed scale applied");
    CHECK(dip > dip_no + 0.5, "speed dip on the slope is smaller with the IMU assist");
    CHECK(fabs(v - 10.0 * MOTION_UPHILL_SPEED_SCALE) < 1.0, "holds target speed on the slope");
    CHECK(st.state == MOTION_STATE_EXECUTING, "no stall fault");
}

static void test_downhill(void)
{
    motion_status_t st;
    double v;
    printf("\n[downhill: slow down, less power]\n");
    fresh_start();
    s_wrong_sign = 0;
    (void)motion_drive(10.0f, 10.0f);
    run_for(1500);
    sim_set_load(MOTION_SIDE_LEFT, -0.12);
    sim_set_load(MOTION_SIDE_RIGHT, -0.12);
    set_imu(-1, -9.0f, 0, 0.0f);
    run_for(1500);
    v = mean_speed(500);
    motion_get_status(&st);
    printf("  speed %.2f cm/s (target %.2f), assist %.3f\n", v,
           10.0 * MOTION_DOWNHILL_SPEED_SCALE, (double)st.left_terrain_duty);
    CHECK(st.terrain == MOTION_TERRAIN_DOWNHILL, "terrain DOWNHILL");
    CHECK(st.left_terrain_duty < 0.0f && st.right_terrain_duty < 0.0f, "power reduced on both wheels");
    CHECK(fabs(v - 10.0 * MOTION_DOWNHILL_SPEED_SCALE) < 1.0, "slows to the downhill speed");
    CHECK(s_wrong_sign == 0, "never drives a wheel backwards while going forward");
}

static void test_hump(void)
{
    motion_status_t st;
    double v;
    printf("\n[one-side hump: left wheel raised]\n");
    fresh_start();
    (void)motion_drive(10.0f, 10.0f);
    run_for(1500);
    sim_set_load(MOTION_SIDE_LEFT, 0.08);   /* left wheel climbing the ramp */
    set_imu(0, 1.0f, -1, -5.0f);            /* Buddy 4: LEFT_WHEEL_UP */
    run_for(1000);
    v = mean_speed(400);
    motion_get_status(&st);
    printf("  speed %.2f cm/s (target %.2f), assist L/R %.3f/%.3f\n", v,
           10.0 * MOTION_HUMP_SPEED_SCALE, (double)st.left_terrain_duty, (double)st.right_terrain_duty);
    CHECK(st.terrain == MOTION_TERRAIN_HUMP_LEFT, "terrain HUMP_LEFT");
    CHECK(st.left_terrain_duty > st.right_terrain_duty + 0.03f, "raised (left) wheel gets the extra power");
    CHECK(fabs(v - 10.0 * MOTION_HUMP_SPEED_SCALE) < 1.0, "slows to the hump speed");
    CHECK(fabs(st.left_speed_cm_s - st.right_speed_cm_s) < 1.0, "both wheels keep the same speed (car stays straight)");

    printf("\n[back on flat: IMU reports level, then goes silent]\n");
    sim_set_load(MOTION_SIDE_LEFT, 0.0);
    set_imu(0, 0.0f, 0, 0.0f);
    run_for(600);
    motion_get_status(&st);
    CHECK(st.terrain == MOTION_TERRAIN_FLAT && st.terrain_speed_scale == 1.0f, "returns to FLAT, full speed target");
    set_imu(-1, -9.0f, 1, 5.0f);
    run_for(100);
    s_imu_on = false;                       /* IMU stops reporting */
    run_for((int)MOTION_TERRAIN_STALE_MS + 600);
    v = mean_speed(500);
    motion_get_status(&st);
    CHECK(st.terrain == MOTION_TERRAIN_FLAT, "stale IMU report falls back to FLAT");
    CHECK(fabs(st.left_terrain_duty) < 1e-6 && fabs(st.right_terrain_duty) < 1e-6, "assist fades to zero");
    CHECK(fabs(v - 10.0) < 1.0, "back to normal speed");
    s_imu.valid = false; s_imu_on = true;
    run_for(100);
    motion_get_status(&st);
    CHECK(st.terrain == MOTION_TERRAIN_FLAT, "valid=false report treated as flat");
}

static void test_move_and_turn(void)
{
    double start;
    double travelled;
    double h0;
    double turned;
    int ms_flat;
    int ms_down;
    int t;
    printf("\n[distance move on a downhill: slower but still accurate]\n");
    fresh_start();
    start = sim_true_distance_cm();
    (void)motion_move_forward(30.0f);
    for (ms_flat = 0; ms_flat < 15000 && !(ms_flat > 40 && motion_is_idle()); ms_flat += 20) { step(); }
    fresh_start();
    start = sim_true_distance_cm();
    set_imu(-1, -9.0f, 0, 0.0f);
    (void)motion_move_forward(30.0f);
    for (ms_down = 0; ms_down < 15000 && !(ms_down > 40 && motion_is_idle()); ms_down += 20) { step(); }
    travelled = sim_true_distance_cm() - start;
    printf("  flat %d ms, downhill-scaled %d ms, travelled %.2f cm\n", ms_flat, ms_down, travelled);
    CHECK(ms_down > ms_flat, "move takes longer at the reduced downhill speed");
    CHECK(fabs(travelled - 30.0) < 1.0, "distance still within 1 cm");

    printf("\n[spin turn on a hump: not slowed]\n");
    fresh_start();
    h0 = sim_true_heading_deg();
    set_imu(0, 0.0f, 1, 5.0f);
    (void)motion_turn_left(90.0f);
    for (t = 0; t < 15000 && !(t > 40 && motion_is_idle()); t += 20) { step(); }
    turned = sim_true_heading_deg() - h0;
    CHECK(fabs(turned - 90.0 * MOTION_TURN_SCALE) < 4.0, "turn angle still within 4 deg");
}

int main(void)
{
    printf("Terrain speed control host tests (simulated load, logic only)\n");
    (void)motion_init();
    test_api();
    test_flat_unchanged();
    test_uphill();
    test_downhill();
    test_hump();
    test_move_and_turn();
    printf("\n%d checks, %d failed\n", s_checks, s_failures);
    return (s_failures == 0) ? 0 : 1;
}
