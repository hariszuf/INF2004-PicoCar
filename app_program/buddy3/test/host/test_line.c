/**
 * @file test_line.c
 * @brief Host simulation of line following on a track with 10 cm radius curves.
 *
 * Checks the control logic against a made-up robot and track. It is not
 * evidence of how the real car behaves.
 */
#include <math.h>
#include <stdio.h>

#include "line_config.h"
#include "line_follow.h"
#include "motion_config.h"
#include "motion_control.h"
#include "sim_hw.h"
#include "sim_track.h"

static int s_failures;
static int s_checks;

#define CHECK(condition, text)                                          \
    do {                                                                \
        s_checks++;                                                     \
        if (condition) { printf("  PASS  %s\n", (text)); }              \
        else { printf("  FAIL  %s\n", (text)); s_failures++; }          \
    } while (0)

typedef struct
{
    double max_err_mm;
    double mean_err_mm;
    int time_ms;
    int lost_samples;
    line_state_t final_state;
    double progress;
} run_result_t;

static run_result_t run(int max_ms, int verbose)
{
    run_result_t r = {0.0, 0.0, 0, 0, LINE_STATE_IDLE, 0.0};
    line_status_t status;
    int ms;
    int n = 0;
    double sum = 0.0;

    for (ms = 0; ms < max_ms; ms++)
    {
        sim_advance_ms(1u);
        track_step_1ms();
        if ((ms % (int)LINE_UPDATE_PERIOD_MS) == ((int)LINE_UPDATE_PERIOD_MS - 1))
        {
            if ((ms % (int)MOTION_UPDATE_PERIOD_MS) == ((int)MOTION_UPDATE_PERIOD_MS - 1))
            {
                motion_update();
            }
            line_follow_update();
            line_follow_get_status(&status);
            if ((status.state == LINE_STATE_RECOVERING) && (track_progress() < 0.97))
            {
                r.lost_samples++;
            }
            if ((status.state == LINE_STATE_FAULT) ||
                ((status.state == LINE_STATE_STOPPED) && (track_progress() > 0.97)))
            {
                break;
            }
            if (verbose && ((ms % 500) == 499))
            {
                double x, y, h;
                track_pose(&x, &y, &h);
                printf("    t=%5d ms pose(%.0f,%.0f,%.0f) mask=%u state=%d err=%.1f mm\n",
                       ms + 1, x, y, h, (unsigned)status.sensor_mask,
                       (int)status.state, track_error_mm());
            }
        }
        if ((ms % 5) == 0)
        {
            double e = track_error_mm();
            if (track_progress() < 0.9)
            {
                if (e > r.max_err_mm) { r.max_err_mm = e; }
                sum += e;
                n++;
            }
        }
    }
    line_follow_get_status(&status);
    r.time_ms = ms;
    r.final_state = status.state;
    r.progress = track_progress();
    r.mean_err_mm = (n > 0) ? (sum / (double)n) : 0.0;
    return r;
}

int main(int argc, char **argv)

{
    run_result_t r;
    int verbose = (argc > 1) ? 1 : 0;

    (void)argv;

    motion_init();
    track_reset();
    line_follow_init();
    CHECK(line_follow_start(), "line_follow_start() accepted");

    printf("\n[follow the whole track: straights, 10 cm radius U-turn, S-curve, 270 deg loop, bend]\n");
    r = run(240000, verbose);
    printf("  time %d ms, progress %.0f %%, mean off-target %.1f mm, max %.1f mm, recovering ticks %d, final state %d\n",
           r.time_ms, r.progress * 100.0, r.mean_err_mm, r.max_err_mm, r.lost_samples, (int)r.final_state);
    CHECK(r.progress > 0.97, "reaches the end of the line");
    CHECK(r.final_state == LINE_STATE_STOPPED, "stops by itself after the line ends");
    CHECK(r.max_err_mm < 20.0, "never farther than 20 mm from the line while following");
    CHECK(r.lost_samples < 400, "brief line losses only (under 2 s of recovery turns in total)");
    return (s_failures == 0) ? 0 : 1;
}
