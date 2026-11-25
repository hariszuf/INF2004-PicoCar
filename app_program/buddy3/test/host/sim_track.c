/**
 * @file sim_track.c
 * @brief Track = list of straights and arcs, sampled every 1 mm. Implements
 *        line_sensors.h for the host build.
 */
#include "../../../barrc_types.h"
#include <math.h>
#include <stdint.h>

#include "sim_track.h"
#include "sim_hw.h"
#include "line_config.h"
#include "line_sensors.h"
#include "motion_config.h"

#define MAX_POINTS      (20000)
#define PI_D            (3.14159265358979)

static float64_t s_px[MAX_POINTS];
static float64_t s_py[MAX_POINTS];
static int32_t s_points;

static float64_t s_x;
static float64_t s_y;
/* robot centre (mm), heading (rad, left +) */
static float64_t s_theta;
static float64_t s_last_left;
static float64_t s_last_right;  /* wheel positions in mm at last step */
static uint8_t s_mask;
static int32_t s_progress_idx;

static float64_t s_cx;
static float64_t s_cy;
static float64_t s_ch;           /* track builder cursor */

static void add_point (void)
{
    if (s_points < MAX_POINTS)
    {
        s_px[s_points] = s_cx;
        s_py[s_points] = s_cy;
        s_points++;
    }
}

static void add_straight (float64_t length_mm)
{
    int32_t i;
    int32_t n = (int32_t)length_mm;

    for (i = 0; i < n; i++)
    {
        s_cx += cos(s_ch);
        s_cy += sin(s_ch);
        add_point();
    }
}

/** turn_deg positive = left. */
static void add_arc (float64_t radius_mm, float64_t turn_deg)
{
    float64_t arc_len = radius_mm * fabs(turn_deg) * PI_D / 180.0;
    float64_t dtheta = ((turn_deg > 0.0) ? 1.0 : -1.0) / radius_mm;
    int32_t i;
    int32_t n = (int32_t)arc_len;

    for (i = 0; i < n; i++)
    {
        s_ch += dtheta * 0.5;
        s_cx += cos(s_ch);
        s_cy += sin(s_ch);
        s_ch += dtheta * 0.5;
        add_point();
    }
}

static void sensor_pos (float64_t lateral_mm, float64_t * p_x, float64_t * p_y)
{
    float64_t fwd = (float64_t)LINE_SENSOR_FORWARD_MM;

    *p_x = s_x + (fwd * cos(s_theta)) - (lateral_mm * sin(s_theta));
    *p_y = s_y + (fwd * sin(s_theta)) + (lateral_mm * cos(s_theta));
}

static float64_t nearest (float64_t x, float64_t y, int32_t * p_idx)
{
    float64_t best = 1e18;
    int32_t best_i = 0;
    int32_t i;

    for (i = 0; i < s_points; i++)
    {
        float64_t dx = s_px[i] - x;
        float64_t dy = s_py[i] - y;
        float64_t d2 = (dx * dx) + (dy * dy);

        if (d2 < best)
        {
            best = d2;
            best_i = i;
        }
    }

    if (0 != p_idx)
    {
        *p_idx = best_i;
    }

    return sqrt(best);
}

void track_reset (void)
{

    s_points = 0;
    /* The line starts 200 mm behind the robot, centred between the two sensors
     * (both sensors start on the line). */
    s_cx = -200.0;
    s_cy = 0.0;
    s_ch = 0.0;
    add_point();
    add_straight(400.0);
    add_arc(100.0, 180.0);      /* U-turn left, radius 10 cm */
    add_straight(150.0);
    add_arc(100.0, -180.0);     /* S-curve back to the right */
    add_straight(150.0);
    add_arc(100.0, 270.0);      /* loop section */
    add_straight(200.0);
    add_arc(150.0, -90.0);      /* gentler right bend */
    add_straight(300.0);        /* then the line ends */

    s_x = 0.0;
    s_y = 0.0;
    s_theta = 0.0;
    s_last_left = sim_true_position_cm(MOTION_SIDE_LEFT) * 10.0;
    s_last_right = sim_true_position_cm(MOTION_SIDE_RIGHT) * 10.0;
    s_progress_idx = 200;
    s_mask = 0u;
}

void track_step_1ms (void)
{
    float64_t left = sim_true_position_cm(MOTION_SIDE_LEFT) * 10.0;
    float64_t right = sim_true_position_cm(MOTION_SIDE_RIGHT) * 10.0;
    float64_t dl = left - s_last_left;
    float64_t dr = right - s_last_right;
    float64_t ds = (dl + dr) / 2.0;
    float64_t dth = (dr - dl) / (float64_t)MOTION_WHEEL_BASE_CM / 10.0;
    float64_t sx;
    float64_t sy;
    int32_t idx;

    s_last_left = left;
    s_last_right = right;
    s_x += ds * cos(s_theta + (dth / 2.0));
    s_y += ds * sin(s_theta + (dth / 2.0));
    s_theta += dth;

    s_mask = 0u;
    /* car-left sensor */
    sensor_pos((float64_t)LINE_SENSOR_SPACING_MM / 2.0, &sx, &sy);

    if (nearest(sx, sy, &idx) <= ((float64_t)LINE_WIDTH_MM / 2.0))
    {
        s_mask |= LINE_SENSOR_LEFT;
    }

    /* car-right sensor */
    sensor_pos(-(float64_t)LINE_SENSOR_SPACING_MM / 2.0, &sx, &sy);

    if (nearest(sx, sy, &idx) <= ((float64_t)LINE_WIDTH_MM / 2.0))
    {
        s_mask |= LINE_SENSOR_RIGHT;
    }

    /* progress along the track = nearest line point to the robot centre line at
     * the sensors */
    sensor_pos(0.0, &sx, &sy);
    (void)nearest(sx, sy, &idx);

    if ((idx > s_progress_idx) && (idx < (s_progress_idx + 150)))
    {
        s_progress_idx = idx;
    }
}

float64_t track_error_mm (void)
{
    float64_t sx;
    float64_t sy;

    sensor_pos(0.0, &sx, &sy);
    return nearest(sx, sy, 0);
}

float64_t track_progress (void)
{
    return (float64_t)s_progress_idx / (float64_t)s_points;
}

void track_pose (float64_t * p_x_mm, float64_t * p_y_mm,
                 float64_t * p_heading_deg)
{
    *p_x_mm = s_x;
    *p_y_mm = s_y;
    *p_heading_deg = s_theta * 180.0 / PI_D;
}

/* line_sensors.h for the host build */
void line_sensors_init (void)
{
}

uint8_t line_sensors_read (void)
{
    return s_mask;
}

/*** end of file ***/
