/* Host test for the "+" junction: both sensors black -> stop -> advance -> turn -> carry on.
 *   gcc -std=c11 -Wall -Wextra -DLINE_JUNCTION_USE_BARCODE=0 -DLINE_JUNCTION_DEFAULT_ACTION=JUNCTION_RIGHT -I../.. -I../../../buddy2 ../../line_follow.c ../../junction_map.c test_junction.c -o /tmp/junction && /tmp/junction
 * Barcode variant (letters choose the direction):
 *   gcc -std=c11 -Wall -Wextra -DLINE_JUNCTION_USE_BARCODE=1 -DTEST_BARCODE=1 -I../.. -I../../../buddy2 ../../line_follow.c ../../junction_map.c test_junction.c -o /tmp/junction_bc && /tmp/junction_bc
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "line_follow.h"
#include "line_config.h"
#include "line_sensors.h"
#include "motion_control.h"

static uint8_t s_mask;
static bool s_idle = true;
static int s_stop_calls, s_drive_calls, s_forward_calls, s_left_calls, s_right_calls;
static float s_last_forward;
static float s_last_turn;
static int s_failures;

void line_sensors_init(void) {}
uint8_t line_sensors_read(void) { return s_mask; }
bool motion_get_tilt(float *p, float *r) { (void)p; (void)r; return false; }
void motion_get_status(motion_status_t *s) { *s = (motion_status_t){0}; s->state = MOTION_STATE_IDLE; }
bool motion_is_idle(void) { return s_idle; }
void motion_stop(void) { s_stop_calls++; }
motion_result_t motion_drive(float l, float r) { (void)l; (void)r; s_drive_calls++; return MOTION_OK; }
motion_result_t motion_move_forward(float d) { s_forward_calls++; s_last_forward = d; s_idle = false; return MOTION_OK; }
motion_result_t motion_turn_left(float a) { s_left_calls++; s_last_turn = a; s_idle = false; return MOTION_OK; }
motion_result_t motion_turn_right(float a) { s_right_calls++; s_last_turn = a; s_idle = false; return MOTION_OK; }

#if TEST_BARCODE
static char s_letter;
static uint32_t s_letter_ms;
bool buddy3_barcode_latest(char *letter, uint32_t *read_ms)
{
    *letter = s_letter;
    *read_ms = s_letter_ms;
    return s_letter != 0;
}
#endif

#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

static void counters_reset(void)
{
    s_stop_calls = s_drive_calls = s_forward_calls = s_left_calls = s_right_calls = 0;
}

static void tick(uint8_t mask)
{
    s_mask = mask;
    counters_reset();
    line_follow_update();
}

#define BOTH (LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT)

/* Run one full junction. Returns after the car is following the line again. */
static void run_junction(void)
{
    line_status_t st;

    tick(BOTH);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_JUNCTION && s_stop_calls == 1, "both black -> stops, state JUNCTION");

    s_idle = false;
    tick(0u);   /* sensors change while the car is still stopping: ignored */
    CHECK(s_forward_calls == 0 && s_left_calls == 0 && s_right_calls == 0 && s_drive_calls == 0,
          "still stopping -> no new command");

    s_idle = true;
    tick(0u);   /* stopped: starts the pause */
    CHECK(s_forward_calls == 0 && s_left_calls == 0 && s_right_calls == 0, "stopped -> pause, no command yet");
    for (uint32_t ms = 0u; ms < LINE_JUNCTION_PAUSE_MS - LINE_UPDATE_PERIOD_MS; ms += LINE_UPDATE_PERIOD_MS)
    {
        tick(BOTH);
        if (s_forward_calls || s_left_calls || s_right_calls) { break; }
    }
    CHECK(s_forward_calls == 0 && s_left_calls == 0 && s_right_calls == 0, "still pausing before the full pause time");
    tick(0u);   /* pause over: advance / cross is commanded on this tick */
}

int main(void)
{
    line_status_t st;

    line_follow_init();
    CHECK(line_follow_start(), "start accepted");
    tick(0u);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STRAIGHT && s_drive_calls == 1, "both white -> drive straight");

#if !TEST_BARCODE
    /* Test default: turn RIGHT. */
    run_junction();
    CHECK(s_forward_calls == 1 && s_last_forward == LINE_JUNCTION_ADVANCE_CM, "stopped -> advances to the middle of the junction");
    s_idle = true;
    tick(0u);
    CHECK(s_right_calls == 1 && s_left_calls == 0 &&
          s_last_turn == LINE_JUNCTION_TURN_DEG - LINE_JUNCTION_PRETURN_MARGIN_DEG, "advance done -> first part of the RIGHT turn");
    s_idle = false;
    tick(BOTH);
    CHECK(s_stop_calls == 0 && s_drive_calls == 0, "turning -> sensors ignored");
    s_idle = true;
    tick(0u);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_JUNCTION && s_drive_calls == 1, "first part done -> slow pivot search starts");
    tick(0u);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_JUNCTION && s_drive_calls == 0, "no line yet -> keeps pivoting");
    tick(LINE_SENSOR_RIGHT);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STRAIGHT, "sensor sees the new line -> following the line again");
    tick(LINE_SENSOR_RIGHT);
    CHECK(s_drive_calls == 1, "normal steering has taken over");
    tick(BOTH);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_STRAIGHT && s_stop_calls == 0 && s_drive_calls == 1,
          "still on the junction bar -> keeps going, no second junction");
    tick(0u);   /* line centred between the sensors: both white */
    tick(BOTH);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_JUNCTION && s_stop_calls == 1, "both white, then both black again -> next junction");
#else
    char letters[] = {'A', 'B', 'C', 'D', 'Z'};
    int exp_left[] = {1, 0, 0, 1, 0};
    int exp_right[] = {0, 0, 1, 0, 0};
    int exp_cross[] = {0, 1, 0, 0, 1};

    for (int i = 0; i < 5; i++)
    {
        char name[64];

        s_letter = letters[i];
        s_letter_ms = 100u + (uint32_t)i;   /* a new scan each time */
        run_junction();
        s_idle = true;
        if (exp_cross[i])
        {
            CHECK(s_forward_calls == 1 && s_last_forward == LINE_JUNCTION_CROSS_CM, "STRAIGHT -> drives across");
        }
        else
        {
            s_idle = true;
            tick(0u);
            snprintf(name, sizeof name, "letter %c -> %s", letters[i], exp_left[i] ? "LEFT" : "RIGHT");
            CHECK((s_left_calls == exp_left[i]) && (s_right_calls == exp_right[i]), name);
            s_idle = true;
            tick(0u);              /* first part done: search starts */
            tick(LINE_SENSOR_LEFT);   /* sees the new line */
        }
        s_idle = true;
        tick(0u);
        tick(0u);
        line_follow_get_status(&st);
        CHECK(st.state == LINE_STATE_STRAIGHT, "back to following the line");
        tick(0u);
    }
    /* No new scan: stop at the junction and stay stopped, no turn, no driving. */
    tick(BOTH);
    line_follow_get_status(&st);
    CHECK(st.state == LINE_STATE_JUNCTION && s_stop_calls == 1, "no new barcode -> stops at the junction");
    int moved = 0;
    for (int i = 0; i < 2000; i++)
    {
        s_idle = true;
        tick((i % 3 == 0) ? 0u : BOTH);
        if (s_forward_calls || s_left_calls || s_right_calls || s_drive_calls) { moved = 1; }
    }
    line_follow_get_status(&st);
    CHECK(!moved && st.state == LINE_STATE_JUNCTION, "no barcode -> stays stopped (10 s), sends no motion commands");
#endif
    printf("%s\n", s_failures == 0 ? "ALL PASSED" : "SOME FAILED");
    return s_failures == 0 ? 0 : 1;
}
