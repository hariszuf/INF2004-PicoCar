#include "buddy4_demo_display.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void contains(const char *text, const char *expected)
{
    assert(strstr(text, expected));
}

int main(void)
{
    char display[320];
    buddy4_output_t output = {0};
    output.valid = output.tilt_valid = true;

    /* These sign mappings match the existing classifier and physical
     * left/right captures, while leaving the signed angles untouched. */
    output.filtered_pitch_deg = 8.25f;
    output.filtered_roll_deg = -5.20f;
    output.terrain = 1;
    output.side = -1;
    buddy4_demo_format(display, sizeof(display), BUDDY4_FRESH, &output);
    contains(display, "Pitch         : 8.25 deg (FRONT UP)");
    contains(display, "Roll          : -5.20 deg (LEFT WHEEL UP)");
    contains(display, "Terrain       : UPHILL");
    contains(display, "Wheel Raised  : LEFT_WHEEL_UP");

    output.filtered_pitch_deg = -7.25f;
    output.filtered_roll_deg = 5.20f;
    output.terrain = -1;
    output.side = 1;
    buddy4_demo_format(display, sizeof(display), BUDDY4_FRESH, &output);
    contains(display, "Pitch         : -7.25 deg (FRONT DOWN)");
    contains(display, "Roll          : 5.20 deg (RIGHT WHEEL UP)");
    contains(display, "Terrain       : DOWNHILL");
    contains(display, "Wheel Raised  : RIGHT_WHEEL_UP");

    output.filtered_pitch_deg = -0.97f;
    output.filtered_roll_deg = 0.61f;
    output.terrain = output.side = 0;
    buddy4_demo_format(display, sizeof(display), BUDDY4_FRESH, &output);
    contains(display, "Pitch         : -0.97 deg (NEAR LEVEL)");
    contains(display, "Roll          : 0.61 deg (NEAR LEVEL)");
    contains(display, "Wheel Raised  : NONE");
    assert(strcmp(buddy4_demo_pitch_direction(-1.01f), "FRONT DOWN") == 0);
    assert(strcmp(buddy4_demo_roll_direction(1.01f), "RIGHT WHEEL UP") == 0);

    output.tilt_valid = false;
    buddy4_demo_format(display, sizeof(display), BUDDY4_FRESH, &output);
    contains(display, "Pitch         : N/A");
    contains(display, "Roll          : N/A");
    assert(!strstr(display, "NEAR LEVEL"));
    output.tilt_valid = true;
    buddy4_demo_format(display, sizeof(display), BUDDY4_STALE, &output);
    contains(display, "Sensor        : STALE");
    contains(display, "Pitch         : N/A");
    buddy4_demo_format(display, sizeof(display), BUDDY4_IO_ERROR, &output);
    contains(display, "Sensor        : SENSOR ERROR");
    output.filtered_pitch_deg = NAN;
    buddy4_demo_format(display, sizeof(display), BUDDY4_FRESH, &output);
    contains(display, "Pitch         : N/A");

    /* A live height never becomes a saved hump; only a completed event with
     * an independently valid estimator is recorded. */
    buddy4_demo_state_t state = {0};
    output.filtered_pitch_deg = 0.0f;
    output.filtered_roll_deg = -5.2f;
    output.wheel_height_cm = 0.98f;
    output.side = -1;
    output.event_ended = false;
    buddy4_demo_format_full(display, sizeof(display), BUDDY4_FRESH,
                            &output, &state);
    contains(display, "Live Height   : 0.98 cm");
    contains(display, "Last Hump     : NONE");
    assert(!buddy4_demo_record_event(&state, BUDDY4_FRESH, &output));

    output.event_ended = true;
    output.timestamp_us = 100;
    output.ended_event.side = -1;
    assert(!buddy4_demo_record_event(&state, BUDDY4_FRESH, &output));
    output.ended_event.peak_confirmed = true;
    output.ended_event.peak_height_cm = 1.29f;
    assert(buddy4_demo_record_event(&state, BUDDY4_FRESH, &output));
    assert(!buddy4_demo_record_event(&state, BUDDY4_FRESH, &output));
    buddy4_demo_format_event(display, sizeof(display), &state);
    contains(display, "Wheel         : LEFT");
    contains(display, "Stable Peak   : 1.29 cm");
    contains(display, "Median Peak   : N/A");
    contains(display, "Status        : COMPLETED");

    output.timestamp_us = 200;
    output.ended_event.side = 1;
    output.ended_event.peak_confirmed = false;
    output.ended_event.median_candidate_valid = true;
    output.ended_event.median_candidate_height_cm = 1.36f;
    assert(buddy4_demo_record_event(&state, BUDDY4_FRESH, &output));
    buddy4_demo_format_full(display, sizeof(display), BUDDY4_FRESH,
                            &output, &state);
    contains(display, "Last Hump     : RIGHT");
    contains(display, "Stable Peak   : N/A");
    contains(display, "Median Peak   : 1.36 cm");
    contains(display, "Highest Hump  : 1.29 cm");

    output.timestamp_us = 300;
    output.ended_event.peak_confirmed = true;
    output.ended_event.peak_height_cm = 1.50f;
    output.tilt_valid = false;
    assert(!buddy4_demo_record_event(&state, BUDDY4_FRESH, &output));
    output.tilt_valid = true;
    assert(buddy4_demo_record_event(&state, BUDDY4_FRESH, &output));
    buddy4_demo_format_full(display, sizeof(display), BUDDY4_FRESH,
                            &output, &state);
    contains(display, "Highest Hump  : 1.50 cm");

    puts("Buddy 4 demo direction display tests passed");
    return 0;
}
