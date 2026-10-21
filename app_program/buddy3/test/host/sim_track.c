/**
 * @file sim_track.c
 * @brief Track = list of straights and arcs, sampled every 1 mm. Implements
 *        line_sensors.h for the host build.
 */
#include <math.h>
#include <stdint.h>

#include "sim_track.h"
#include "sim_hw.h"
#include "line_config.h"
#include "line_sensors.h"
#include "motion_config.h"

#define MAX_POINTS      (20000)
#define PI_D            (3.14159265358979)

static double s_px[MAX_POINTS];
static double s_py[MAX_POINTS];
static int s_points;

static double s_x, s_y, s_theta;          /* robot centre (mm), heading (rad, left +) */
static double s_last_left, s_last_right;  /* wheel positions in mm at last step */
static uint8_t s_mask;
static int s_progress_idx;

static double s_cx, s_cy, s_ch;           /* track builder cursor */

static void add_point(void)
{
    if (s_points < MAX_POINTS)
    {
        s_px[s_points] = s_cx;
        s_py[s_points] = s_cy;
        s_points++;
    }
}

static void add_straight(double length_mm)
{
    int i;
    int n = (int)length_mm;

    for (i = 0; i < n; i++)
    {
        s_cx += cos(s_ch);
        s_cy += sin(s_ch);
        add_point();
    }
}

/** turn_deg positive = left. */
static void add_arc(double radius_mm, double turn_deg)
{
    double arc_len = radius_mm * fabs(turn_deg) * PI_D / 180.0;
    double dtheta = ((turn_deg > 0.0) ? 1.0 : -1.0) / radius_mm;
    int i;
    int n = (int)arc_len;

    for (i = 0; i < n; i++)
    {
        s_ch += dtheta * 0.5;
        s_cx += cos(s_ch);
        s_cy += sin(s_ch);
        s_ch += dtheta * 0.5;
        add_point();
    }
}

static void sensor_pos(double lateral_mm, double *p_x, double *p_y)
{
    double fwd = (double)LINE_SENSOR_FORWARD_MM;

    *p_x = s_x + (fwd * cos(s_theta)) - (lateral_mm * sin(s_theta));
    *p_y = s_y + (fwd * sin(s_theta)) + (lateral_mm * cos(s_theta));
}

static double nearest(double x, double y, int *p_idx)
{
    double best = 1e18;
    int best_i = 0;
    int i;

    for (i = 0; i < s_points; i++)
    {
        double dx = s_px[i] - x;
        double dy = s_py[i] - y;
        double d2 = (dx * dx) + (dy * dy);

        if (d2 < best)
        {
            best = d2;
            best_i = i;
        }
    }
    if (p_idx != 0)
    {
        *p_idx = best_i;
    }
    return sqrt(best);
}

void track_reset(void)
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

void track_step_1ms(void)
{
    double left = sim_true_position_cm(MOTION_SIDE_LEFT) * 10.0;
    double right = sim_true_position_cm(MOTION_SIDE_RIGHT) * 10.0;
    double dl = left - s_last_left;
    double dr = right - s_last_right;
    double ds = (dl + dr) / 2.0;
    double dth = (dr - dl) / (double)MOTION_WHEEL_BASE_CM / 10.0;
    double sx;
    double sy;
    int idx;

    s_last_left = left;
    s_last_right = right;
    s_x += ds * cos(s_theta + (dth / 2.0));
    s_y += ds * sin(s_theta + (dth / 2.0));
    s_theta += dth;

    s_mask = 0u;
    sensor_pos((double)LINE_SENSOR_SPACING_MM / 2.0, &sx, &sy);   /* car-left sensor */
    if (nearest(sx, sy, &idx) <= ((double)LINE_WIDTH_MM / 2.0))
    {
        s_mask |= LINE_SENSOR_LEFT;
    }
    sensor_pos(-(double)LINE_SENSOR_SPACING_MM / 2.0, &sx, &sy);  /* car-right sensor */
    if (nearest(sx, sy, &idx) <= ((double)LINE_WIDTH_MM / 2.0))
    {
        s_mask |= LINE_SENSOR_RIGHT;
    }
    /* progress along the track = nearest line point to the robot centre line at the sensors */
    sensor_pos(0.0, &sx, &sy);
    (void)nearest(sx, sy, &idx);
    if ((idx > s_progress_idx) && (idx < (s_progress_idx + 150)))
    {
        s_progress_idx = idx;
    }
}

double track_error_mm(void)
{
    double sx;
    double sy;

    sensor_pos(0.0, &sx, &sy);
    return nearest(sx, sy, 0);
}

double track_progress(void)
{
    return (double)s_progress_idx / (double)s_points;
}

void track_pose(double *p_x_mm, double *p_y_mm, double *p_heading_deg)
{
    *p_x_mm = s_x;
    *p_y_mm = s_y;
    *p_heading_deg = s_theta * 180.0 / PI_D;
}

/* line_sensors.h for the host build */
void line_sensors_init(void)
{
}

uint8_t line_sensors_read(void)
{
    return s_mask;
}
