#include "buddy4_demo_display.h"

#include <math.h>
#include <stdio.h>

/* Display-only tolerance for small mounting/surface offsets. It is below the
 * IMU's pitch/roll return-to-level thresholds (3 and 2 degrees). */
#define DEMO_NEAR_LEVEL_DEG 1.0f

const char *buddy4_demo_pitch_direction(float pitch_deg)
{
    if (!isfinite(pitch_deg)) return "N/A";
    if (pitch_deg >= -DEMO_NEAR_LEVEL_DEG &&
        pitch_deg <= DEMO_NEAR_LEVEL_DEG) return "NEAR LEVEL";
    return pitch_deg > 0.0f ? "FRONT UP" : "FRONT DOWN";
}

const char *buddy4_demo_roll_direction(float roll_deg)
{
    if (!isfinite(roll_deg)) return "N/A";
    if (roll_deg >= -DEMO_NEAR_LEVEL_DEG &&
        roll_deg <= DEMO_NEAR_LEVEL_DEG) return "NEAR LEVEL";
    return roll_deg > 0.0f ? "RIGHT WHEEL UP" : "LEFT WHEEL UP";
}

void buddy4_demo_format(char *buffer, size_t size, buddy4_result_t result,
                        const buddy4_output_t *output)
{
    buddy4_demo_format_full(buffer, size, result, output, NULL);
}

bool buddy4_demo_record_event(buddy4_demo_state_t *state,
                              buddy4_result_t result,
                              const buddy4_output_t *output)
{
    if (!state || !output || result != BUDDY4_FRESH || !output->valid ||
        !output->tilt_valid || !output->event_ended ||
        (output->ended_event.side != -1 && output->ended_event.side != 1) ||
        (state->has_last && output->timestamp_us <= state->last_event_timestamp_us))
        return false;

    const side_event_t *event = &output->ended_event;
    const bool stable = event->peak_confirmed &&
        isfinite(event->peak_height_cm) && event->peak_height_cm >= 0.0f;
    const bool median = event->median_candidate_valid &&
        isfinite(event->median_candidate_height_cm) &&
        event->median_candidate_height_cm >= 0.0f;
    if (!stable && !median) return false;

    state->has_last = true;
    state->last_side = event->side;
    state->stable_valid = stable;
    state->stable_height_cm = stable ? event->peak_height_cm : 0.0f;
    state->median_valid = median;
    state->median_height_cm = median ? event->median_candidate_height_cm : 0.0f;
    state->last_event_timestamp_us = output->timestamp_us;
    if (stable && (!state->highest_valid ||
                   event->peak_height_cm > state->highest_height_cm)) {
        state->highest_valid = true;
        state->highest_height_cm = event->peak_height_cm;
    }
    return true;
}

static void format_height(char *buffer, size_t size, bool valid, float height_cm)
{
    if (valid && isfinite(height_cm))
        snprintf(buffer, size, "%.2f cm", (double)height_cm);
    else
        snprintf(buffer, size, "N/A");
}

void buddy4_demo_format_full(char *buffer, size_t size, buddy4_result_t result,
                             const buddy4_output_t *output,
                             const buddy4_demo_state_t *state)
{
    if (!buffer || !size || !output) return;

    const bool angles_valid = result == BUDDY4_FRESH && output->valid &&
        output->tilt_valid && isfinite(output->filtered_pitch_deg) &&
        isfinite(output->filtered_roll_deg);
    const char *status = result == BUDDY4_IO_ERROR ? "SENSOR ERROR" :
                         result == BUDDY4_STALE ? "STALE" :
                         angles_valid ? "VALID" : "INVALID";
    char live_height[24], stable_height[24], median_height[24], highest_height[24];
    format_height(live_height, sizeof(live_height),
                  angles_valid && isfinite(output->wheel_height_cm),
                  output->wheel_height_cm);
    format_height(stable_height, sizeof(stable_height),
                  state && state->has_last && state->stable_valid,
                  state ? state->stable_height_cm : 0.0f);
    format_height(median_height, sizeof(median_height),
                  state && state->has_last && state->median_valid,
                  state ? state->median_height_cm : 0.0f);
    format_height(highest_height, sizeof(highest_height),
                  state && state->highest_valid,
                  state ? state->highest_height_cm : 0.0f);
    const char *last_side = state && state->has_last ?
        (state->last_side < 0 ? "LEFT" : "RIGHT") : "NONE";
    if (!angles_valid) {
        snprintf(buffer, size,
                 "========== BUDDY 4 IMU ==========\n"
                 "Sensor        : %s\n"
                 "Pitch         : N/A\n"
                 "Roll          : N/A\n"
                 "Terrain       : N/A\n"
                 "Wheel Raised  : N/A\n"
                 "Live Height   : N/A\n"
                 "Last Hump     : %s\n"
                 "Stable Peak   : %s\n"
                 "Median Peak   : %s\n"
                 "Highest Hump  : %s\n"
                 "=================================\n", status, last_side,
                 stable_height, median_height, highest_height);
        return;
    }

    const float pitch = output->filtered_pitch_deg;
    const float roll = output->filtered_roll_deg;
    snprintf(buffer, size,
             "========== BUDDY 4 IMU ==========\n"
             "Sensor        : %s\n"
             "Pitch         : %.2f deg (%s)\n"
             "Roll          : %.2f deg (%s)\n"
             "Terrain       : %s\n"
             "Wheel Raised  : %s\n"
             "Live Height   : %s\n"
             "Last Hump     : %s\n"
             "Stable Peak   : %s\n"
             "Median Peak   : %s\n"
             "Highest Hump  : %s\n"
             "=================================\n",
             status, (double)pitch, buddy4_demo_pitch_direction(pitch),
             (double)roll, buddy4_demo_roll_direction(roll),
             buddy4_terrain_name(output->terrain),
             output->side == 0 ? "NONE" : buddy4_side_name(output->side),
             live_height, last_side, stable_height, median_height,
             highest_height);
}

void buddy4_demo_format_event(char *buffer, size_t size,
                              const buddy4_demo_state_t *state)
{
    if (!buffer || !size) return;
    if (!state || !state->has_last) {
        snprintf(buffer, size, "Last Hump: NONE\n");
        return;
    }
    char stable_height[24], median_height[24];
    format_height(stable_height, sizeof(stable_height), state->stable_valid,
                  state->stable_height_cm);
    format_height(median_height, sizeof(median_height), state->median_valid,
                  state->median_height_cm);
    snprintf(buffer, size,
             "======= HUMP MEASUREMENT =======\n"
             "Wheel         : %s\n"
             "Stable Peak   : %s\n"
             "Median Peak   : %s\n"
             "Status        : COMPLETED\n"
             "================================\n",
             state->last_side < 0 ? "LEFT" : "RIGHT",
             stable_height, median_height);
}
