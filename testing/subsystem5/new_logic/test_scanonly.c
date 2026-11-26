/* Scan-only build: a stationary car measures boxes of different widths. Build:
 * see run_scanonly.sh */
#include "../../../app_program/barrc_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "buddy5_plan.h"
#define D2R 0.017453292519943295
static float64_t ray_box (float64_t ox, float64_t oy, float64_t dx,
                          float64_t dy, float64_t x0,
                          float64_t x1, float64_t y0, float64_t y1)
{
    float64_t t0 = 0, t1 = 1e9, o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {x0,
        y0},
    hi[2] = {x1, y1};

    for (int32_t idx = 0; idx < 2; idx++)
    {
        if (fabs(d[idx]) < 1e-12)
        {
            if ((o[idx] < lo[idx]) || (o[idx] > hi[idx]))
            {
                return 1e9;
            }
        }
        else
        {
            float64_t a = (lo[idx] - o[idx]) / d[idx],
            b = (hi[idx] - o[idx]) / d[idx];

            if (a > b)
            {
                float64_t t = a;
                a = b;
                b = t;
            }

            if (a > t0)
            {
                t0 = a;
            }

            if (b < t1)
            {
                t1 = b;
            }

            if (t0 > t1)
            {
                return 1e9;
            }
        }
    }

    return t0;
}

static uint32_t s_rs = 1;
static float64_t rnd (void)
{
    s_rs = s_rs * 1664525u + 1013904223u;
    return (s_rs >> 8) / 16777216.0;
}

static int32_t run (float64_t face_x, float64_t cy, float64_t width,
                    float64_t noise,
                    int32_t * requests, int32_t * p_wout, int32_t * p_valid_out,
                    int32_t * p_le,
                    int32_t * p_re)
{
    b5_core_t c;
    b5_core_init(&c, 0);
    b5_sampler_t sm;
    b5_sampler_reset(&sm);
    int32_t servo = 0;
    uint32_t settle = 300;
    uint32_t next = 0;
    *requests = 0;
    int32_t done = 0;
    *p_valid_out = 0;
    for (uint32_t now = 0; now < 40000; now++)
    {
        int32_t ang = b5_core_angle(&c);

        if (ang != servo)
        {
            servo = ang;
            b5_sampler_reset(&sm);
            settle = now + B5_SERVO_SETTLE_MS;
        }

        b5_range_t out;
        bool b_have = false;

        if (now >= next)
        {
            next = now + B5_PING_INTERVAL_MS;

            if (now >= settle)
            {
                float64_t sx = B5_AXLE_TO_SENSOR_MM;
                float64_t best = 1e9;

                for (float64_t off = -B5_BEAM_HALF_DEG; off <= B5_BEAM_HALF_DEG
                     + 1e-9; off += 1.5)
                {
                    float64_t a = (servo + off) * D2R,
                    d = ray_box(sx, 0, cos(a), sin(a), face_x, face_x + 150,
                                cy - width / 2, cy + width / 2);

                    if (d < best)
                    {
                        best = d;
                    }
                }

                b5_range_t s = {true, B5_NO_ECHO_MM};

                if (best < 1500)
                {
                    s.distance_mm =
                            (uint32_t)(best + (rnd() - 0.5) * 2 * noise);
                }

                if (b5_sampler_push(&sm, &s, &out))
                {
                    b_have = true;
                }
            }
        }

        b5_feedback_t f = {0};
        f.valid = true;
        f.timestamp_ms = now;
        f.stopped = true;
        b5_core_update(&c, now, &f, b_have ? &out : NULL);
        b5_request_t rq;

        if (b5_core_peek_request(&c, &rq))
        {
            ++*requests;
        }

        if (B5_ERROR == c.state)
        {
            return -1;
        }

        if ((c.profile.valid) && (B5_MONITORING == c.state))
        {
            *p_wout = c.profile.width_mm;
            *p_valid_out = c.profile.width_valid;
            *p_le = c.profile.left_edge_mm;
            *p_re = c.profile.right_edge_mm;
            done = 1;
            break;
        }
    }

    return done ? 0 : -2;
}

int main (void)
{
    float64_t widths[] = {60, 100, 140, 200, 260};
    float64_t offs[] = {0, -20, 25};
    int32_t bad = 0;

    for (int32_t w = 0; w < 5; w++)
    {
        for (int32_t o = 0; o < 3; o++)
        {
            int32_t rq;
            int32_t val;
            int32_t wm = 0;
            int32_t le = 0;
            int32_t re = 0;
            int32_t r = run(300 + B5_AXLE_TO_SENSOR_MM - 80 + 80, offs[o] * -1,
                            widths[w],
                            4, &rq, &wm, &val, &le, &re);
            float64_t err = wm - widths[w];
            printf("true width %3.0f  centre offset %+3.0f -> reported %4d mm "
                   "(L "
                   "%4d R %4d) both edges seen=%d  err %+5.0f  requests=%d "
                   "%s\n",
                   widths[w], offs[o], wm, le, re, val, err, rq,
                   0 == r ? "" : "NO RESULT");

            if ((r != 0) || (rq != 0))
            {
                bad++;
            }
        }
    }

    return bad ? 1 : 0;
}

/*** end of file ***/
