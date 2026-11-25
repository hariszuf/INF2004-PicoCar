/**
 * Host test: straddle mode white = move, black = stop.
 * Stubs the sensors and Buddy 2 so only the decision logic in line_follow.c
 * runs.
 *
 * gcc -std=c11 -Wall -Wextra -DLINE_JUNCTION=0 -I. -I../buddy2 line_follow.c
 * test/host/test_both_black.c -o /tmp/both_black && /tmp/both_black
 * (junction logic is off here so both-black is the plain stop; see
 * test_junction.c for the junction)
 *   (run from app_program/buddy3)
 */
#include "../../../barrc_types.h"
#include <stdio.h>

#include "line_config.h"
#include "line_follow.h"
#include "line_sensors.h"
#include "motion_control.h"

static uint8_t s_mask;
static int32_t s_drive_calls;
static int32_t s_stop_calls;
static float32_t s_last_left;
static float32_t s_last_right;
static int32_t s_failures;
static float32_t s_stub_pitch;
static float32_t s_stub_roll;
static bool s_stub_tilt_valid;

void line_sensors_init (void)
{
}

uint8_t line_sensors_read (void)
{
    return s_mask;
}

void motion_get_status (motion_status_t * p_status)
{
    *p_status = (motion_status_t){0};
    p_status->state = MOTION_STATE_IDLE;
}

bool motion_get_tilt (float32_t * p_pitch_deg, float32_t * p_roll_deg)
{
    *p_pitch_deg = s_stub_pitch;
    *p_roll_deg = s_stub_roll;
    return s_stub_tilt_valid;
}

motion_result_t motion_drive (float32_t left_cm_s, float32_t right_cm_s)
{
    s_drive_calls++;
    s_last_left = left_cm_s;
    s_last_right = right_cm_s;
    return MOTION_OK;
}

void motion_stop (void)
{
    s_stop_calls++;
}

#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

static void tick (uint8_t mask)
{
    s_mask = mask;
    s_drive_calls = 0;
    s_stop_calls = 0;
    line_follow_update();
}

int main (void)
{
    line_status_t st;

    line_follow_init();
    CHECK(line_follow_start(), "start accepted");

    tick(0u);
    line_follow_get_status(&st);
    CHECK((LINE_STATE_STRAIGHT == st.state) && (1 == s_drive_calls)
          && (s_last_left > 0.0f)
          && (s_last_left == s_last_right), "both white -> drive straight");

    tick(LINE_SENSOR_LEFT);
    line_follow_get_status(&st);
    CHECK((LINE_STATE_STEER_LEFT == st.state) && (s_last_left < s_last_right)
          && (0.0f == s_last_left),
          "left black -> left wheel stops, car turns left");

    tick(LINE_SENSOR_RIGHT);
    line_follow_get_status(&st);
    CHECK((LINE_STATE_STEER_RIGHT == st.state) && (s_last_right < s_last_left)
          && (0.0f == s_last_right),
          "right black -> right wheel stops, car turns right");

    tick(LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT);
    line_follow_get_status(&st);
#if LINE_BOTH_BLACK_STOP
    CHECK((LINE_STATE_STOPPED == st.state) && (1 == s_stop_calls)
          && (0 == s_drive_calls)
          && (0.0f == st.left_cmd_cm_s) && (0.0f == st.right_cmd_cm_s),
          "both black -> stop");
    tick(LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT);
    CHECK((0 == s_stop_calls) && (0 == s_drive_calls),
          "both black held -> stays stopped, no repeated commands");
    tick(0u);
    line_follow_get_status(&st);
    CHECK((LINE_STATE_STRAIGHT == st.state) && (1 == s_drive_calls),
          "both white again -> moves again");
    tick(LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT);
    tick(LINE_SENSOR_RIGHT);
    line_follow_get_status(&st);
    CHECK((LINE_STATE_STEER_RIGHT == st.state) && (1 == s_drive_calls),
          "one sensor white again -> moves (steers)");
#else
    CHECK(LINE_STATE_STRAIGHT == st.state,
          "both black -> straight (LINE_BOTH_BLACK_STOP off)");
#endif

    tick(LINE_SENSOR_BARCODE);
    line_follow_get_status(&st);
    CHECK((LINE_STATE_STRAIGHT == st.state) && (0 == s_stop_calls),
          "barcode sensor alone never stops or steers the car");

#if LINE_TERRAIN_HOLD
    /* Hump freeze: tilted car ignores the sensors and keeps the last commands.
     */
    tick(0u);
    s_stub_tilt_valid = true;
    s_stub_roll = LINE_FREEZE_ROLL_DEG + 1.0f;
    tick(LINE_SENSOR_LEFT);
    line_follow_get_status(&st);
    CHECK((0 == s_drive_calls) && (0 == s_stop_calls)
          && (LINE_STATE_FROZEN == st.state),
          "tilted + one sensor black -> frozen, wheel does not stop");
    tick(LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT);
    CHECK((0 == s_drive_calls) && (0 == s_stop_calls),
          "tilted + both black -> frozen, no stop");
    s_stub_roll = 0.0f;
    tick(LINE_SENSOR_LEFT);
    line_follow_get_status(&st);
    CHECK((LINE_STATE_STEER_LEFT == st.state) && (1 == s_drive_calls),
          "level again -> follows the line immediately (no delay)");
    s_stub_roll = 0.0f;
    s_stub_pitch = -(LINE_FREEZE_PITCH_DEG + 1.0f);
    tick(LINE_SENSOR_RIGHT);
    CHECK((0 == s_drive_calls) && (0 == s_stop_calls), "steep pitch -> frozen");
    s_stub_pitch = 0.0f;
    s_stub_tilt_valid = false;
    s_stub_roll = 20.0f;
    tick(LINE_SENSOR_RIGHT);
    CHECK(1 == s_drive_calls,
          "no fresh IMU data -> no freeze, normal following");
#endif

    printf("%s\n", 0 == s_failures ? "ALL PASSED" : "SOME FAILED");
    return 0 == s_failures ? 0 : 1;
}

/*** end of file ***/
