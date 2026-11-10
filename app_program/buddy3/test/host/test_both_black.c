/**
 * Host test: straddle mode white = move, black = stop.
 * Stubs the sensors and Buddy 2 so only the decision logic in line_follow.c runs.
 *
 *   gcc -std=c11 -Wall -Wextra -DLINE_JUNCTION=0 -I. -I../buddy2 line_follow.c test/host/test_both_black.c -o /tmp/both_black && /tmp/both_black
 * (junction logic is off here so both-black is the plain stop; see test_junction.c for the junction)
 *   (run from app_program/buddy3)
 */
#include <stdio.h>

#include "line_config.h"
#include "line_follow.h"
#include "line_sensors.h"
#include "motion_control.h"

static uint8_t s_mask;
static int s_drive_calls;
static int s_stop_calls;
static float s_last_left;
static float s_last_right;
static int s_failures;
static float s_stub_pitch;
static float s_stub_roll;
static bool s_stub_tilt_valid;

void line_sensors_init(void) {}
uint8_t line_sensors_read(void) { return s_mask; }

void motion_get_status(motion_status_t *p_status)
{
    *p_status = (motion_status_t){0};
    p_status->state = MOTION_STATE_IDLE;
}

bool motion_get_tilt(float *p_pitch_deg, float *p_roll_deg)
{
    *p_pitch_deg = s_stub_pitch;
    *p_roll_deg = s_stub_roll;
    return s_stub_tilt_valid;
}

motion_result_t motion_drive(float left_cm_s, float right_cm_s)
{
    s_drive_calls++;
    s_last_left = left_cm_s;
    s_last_right = right_cm_s;
    return MOTION_OK;
}

void motion_stop(void) { s_stop_calls++; }

#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

static void tick(uint8_t mask)
{
    s_mask = mask;
    s_drive_calls = 0;
    s_stop_calls = 0;
    line_follow_update();
}

int main(void)
{
    line_status_t st;

    line_follow_init();
    CHECK(line_follow_start(), "start accepted");

    tick(0u);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STRAIGHT && s_drive_calls == 1 && s_last_left > 0.0f
          && s_last_left == s_last_right, "both white -> drive straight");

    tick(LINE_SENSOR_LEFT);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STEER_LEFT && s_last_left < s_last_right && s_last_left == 0.0f,
          "left black -> left wheel stops, car turns left");

    tick(LINE_SENSOR_RIGHT);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STEER_RIGHT && s_last_right < s_last_left && s_last_right == 0.0f,
          "right black -> right wheel stops, car turns right");

    tick(LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT);
    line_follow_get_status(&st);
#if LINE_BOTH_BLACK_STOP
    CHECK(st.state == LINE_STATE_STOPPED && s_stop_calls == 1 && s_drive_calls == 0
          && st.left_cmd_cm_s == 0.0f && st.right_cmd_cm_s == 0.0f, "both black -> stop");
    tick(LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT);
    CHECK(s_stop_calls == 0 && s_drive_calls == 0, "both black held -> stays stopped, no repeated commands");
    tick(0u);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STRAIGHT && s_drive_calls == 1, "both white again -> moves again");
    tick(LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT);
    tick(LINE_SENSOR_RIGHT);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STEER_RIGHT && s_drive_calls == 1, "one sensor white again -> moves (steers)");
#else
    CHECK(st.state == LINE_STATE_STRAIGHT, "both black -> straight (LINE_BOTH_BLACK_STOP off)");
#endif

    tick(LINE_SENSOR_BARCODE);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STRAIGHT && s_stop_calls == 0, "barcode sensor alone never stops or steers the car");

#if LINE_TERRAIN_HOLD
    /* Hump freeze: tilted car ignores the sensors and keeps the last commands. */
    tick(0u);
    s_stub_tilt_valid = true;
    s_stub_roll = LINE_FREEZE_ROLL_DEG + 1.0f;
    tick(LINE_SENSOR_LEFT);
    line_follow_get_status(&st);
    CHECK(s_drive_calls == 0 && s_stop_calls == 0 && st.state == LINE_STATE_FROZEN,
          "tilted + one sensor black -> frozen, wheel does not stop");
    tick(LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT);
    CHECK(s_drive_calls == 0 && s_stop_calls == 0, "tilted + both black -> frozen, no stop");
    s_stub_roll = 0.0f;
    tick(LINE_SENSOR_LEFT);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STEER_LEFT && s_drive_calls == 1, "level again -> follows the line immediately (no delay)");
    s_stub_roll = 0.0f;
    s_stub_pitch = -(LINE_FREEZE_PITCH_DEG + 1.0f);
    tick(LINE_SENSOR_RIGHT);
    CHECK(s_drive_calls == 0 && s_stop_calls == 0, "steep pitch -> frozen");
    s_stub_pitch = 0.0f;
    s_stub_tilt_valid = false;
    s_stub_roll = 20.0f;
    tick(LINE_SENSOR_RIGHT);
    CHECK(s_drive_calls == 1, "no fresh IMU data -> no freeze, normal following");
#endif

    printf("%s\n", s_failures == 0 ? "ALL PASSED" : "SOME FAILED");
    return s_failures == 0 ? 0 : 1;
}
