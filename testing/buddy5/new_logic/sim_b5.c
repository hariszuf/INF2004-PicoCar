/* Host simulation of the NEW Buddy 5 logic: real planner + real state machine
 * against a simple world.
 * Build: see run_tests.sh. World frame: the line is the x axis, +x forward, +y
 * left, degrees + = left (CCW). */
#include "../../../app_program/barrc_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "buddy5_plan.h"
#define D2R 0.017453292519943295
#define B5_LINE_ADVANCE_MM_SIM 40.0
typedef struct
{
    float64_t x0;
    float64_t x1;
    float64_t y0;
    float64_t y1;
} rect_t;
typedef struct
{
    const char * name;
    rect_t obs[6];
    int32_t nobs;      /* obs[0] is the box */
    float64_t fr;
    float64_t rr;
    float64_t hw;              /* TRUE car outline */
    float64_t beam;                  /* TRUE sonar cone half angle */
    float64_t noise;
    float64_t p_spike;
    float64_t p_drop;
    float64_t start_x;               /* axle x at the start (line following) */
    /* 0 don't care, +1 left, -1 right, 99 = expect a stop (no route) */
    int32_t expect_side;
    uint32_t seed;
    /* extra distance the car rolls after STOP */
    float64_t stop_overshoot;
    /* spawn an obstacle during the pass (person) */
    int32_t allow_extra_blocked_at_adv;
} scen_t;
/* ---------- tiny RNG ---------- */
static uint32_t s_rs;
static float64_t rnd (void)
{
    s_rs = s_rs * 1664525u + 1013904223u;
    return (s_rs >> 8) / 16777216.0;
}

static float64_t gauss (void)
{
    float64_t u = rnd() + 1e-9, v = rnd();
    return sqrt(-2 * log(u)) * cos(6.283185307*v);
}

/* ---------- geometry ---------- */
typedef struct
{
    float64_t x[4];
    float64_t y[4];
} quadr_t;
static int32_t sat (const quadr_t * a, const quadr_t * p_b)
{
    const quadr_t * p_q[2] = {a, p_b};

    for (int32_t k = 0; k < 2; k++)
    {
        for (int32_t i = 0; i < 4; i++)
        {
            int32_t jdx = (i + 1) & 3;
            float64_t ax = -(p_q[k]->y[jdx] - p_q[k]->y[i]),
            ay = p_q[k]->x[jdx] - p_q[k]->x[i];
            float64_t amin = 1e30;
            float64_t amax = -1e30;
            float64_t bmin = 1e30;
            float64_t bmax = -1e30;

            for (int32_t p = 0; p < 4; p++)
            {
                float64_t da = ax * a->x[p] + ay * a->y[p];
                float64_t db = ax * p_b->x[p] + ay * p_b->y[p];

                if (da < amin)
                {
                    amin = da;
                }

                if (da > amax)
                {
                    amax = da;
                }

                if (db < bmin)
                {
                    bmin = db;
                }

                if (db > bmax)
                {
                    bmax = db;
                }
            }

            if ((amax < bmin) || (bmax < amin))
            {
                return 0;
            }
        }
    }

    return 1;
}

static quadr_t car_quad (const scen_t * s, float64_t px, float64_t py,
                         float64_t ph)
{
    float64_t c = cos(ph * D2R), sn = sin(ph * D2R);
    quadr_t q;
    float64_t bx[4] = {s->fr, s->fr, -s->rr, -s->rr}, by[4] = {s->hw, -s->hw,
        -s->hw,
        s->hw};

    for (int32_t i = 0; i < 4; i++)
    {
        q.x[i] = px + bx[i] * c - by[i] * sn;
        q.y[i] = py + bx[i] * sn + by[i] * c;
    }

    return q;
}

static quadr_t rect_quad (rect_t r, float64_t m)
{
    quadr_t q = {{r.x0 - m, r.x1 + m, r.x1 + m, r.x0 - m}, {r.y0 - m, r.y0 - m,
            r.y1 + m, r.y1 + m}};
    return q;
}

/* smallest clearance (mm, 5 mm resolution, -1 = touching) between the car and
 * any obstacle */
static float64_t clearance (const scen_t * s, float64_t px, float64_t py,
                            float64_t ph)
{
    quadr_t c = car_quad(s, px, py, ph);
    float64_t best = 1e9;

    for (int32_t o = 0; o < s->nobs; o++)
    {
        float64_t m;
        int32_t hit = 0;

        for (m = 0; m <= 60; m += 5)
        {
            quadr_t r = rect_quad(s->obs[o], m);

            if (sat(&c, &r))
            {
                hit = 1;
                break;
            }
        }

        float64_t cl = hit ? (0 == m ? -1 : m - 5) : 60;

        if (cl < best)
        {
            best = cl;
        }
    }

    return best;
}

static float64_t ray_rect (float64_t ox, float64_t oy, float64_t dx,
                           float64_t dy, rect_t r)
{
    float64_t t0 = 0;
    float64_t t1 = 1e9;
    float64_t o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {r.x0, r.y0},
    hi[2] = {r.x1,
        r.y1};

    for (int32_t i = 0; i < 2; i++)
    {
        if (fabs(d[i]) < 1e-12)
        {
            if ((o[i] < lo[i]) || (o[i] > hi[i]))
            {
                return 1e9;
            }
        }
        else
        {
            float64_t a = (lo[i] - o[i]) / d[i], b = (hi[i] - o[i]) / d[i];

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

/* ---------- sonar ---------- */
static b5_range_t ping (const scen_t * s, float64_t px, float64_t py,
                        float64_t ph,
                        float64_t servo, float64_t extra_obs_x,
                        float64_t extra_obs_y)
{
    float64_t sx = px + B5_AXLE_TO_SENSOR_MM * cos(ph * D2R),
    sy = py + B5_AXLE_TO_SENSOR_MM * sin(ph * D2R);
    float64_t best = 1e9;

    for (float64_t off = -s->beam; off <= s->beam + 1e-9; off += 1.5)
    {
        float64_t a = (ph + servo + off) * D2R;

        for (int32_t o = 0; o < s->nobs; o++)
        {
            float64_t d = ray_rect(sx, sy, cos(a), sin(a), s->obs[o]);

            if (d < best)
            {
                best = d;
            }
        }

        if (extra_obs_x > 0)
        {
            rect_t p = {extra_obs_x, extra_obs_x + 80, extra_obs_y - 40,
                extra_obs_y + 40};
            float64_t d = ray_rect(sx, sy, cos(a), sin(a), p);

            if (d < best)
            {
                best = d;
            }
        }
    }

    b5_range_t r = {true, B5_NO_ECHO_MM};

    if ((best < 1500) && (best > 30))
    {
        r.distance_mm = (uint32_t)(best + gauss() * s->noise);
    }

    if (rnd() < s->p_drop)
    {
        r.distance_mm = B5_NO_ECHO_MM;
    }
    else if (rnd() < s->p_spike)
    {
        r.distance_mm = (uint32_t)(60 + rnd() * 200);
    }

    return r;
}

/* ---------- the simulated vehicle (navigation layer) ---------- */
typedef enum
{
    A_TURN = 1, A_MOVE, A_CHECK, A_SWEEP, A_REJOIN
} akind;
typedef struct
{
    akind k;
    float64_t v;
} act_t;
typedef struct
{
    /* world pose and nav origin */
    float64_t px;
    float64_t py;
    float64_t ph;
    float64_t ox;
    float64_t oy;
    float64_t oh;
    /* motion */
    int32_t mot;
    float64_t mot_target;
    float64_t mot_rem;
    float64_t mot_speed;
    int32_t mot_dir;
    int32_t still;
    /* request */
    bool active;
    bool failed;
    b5_request_t req;
    act_t act[12];
    int32_t n;
    int32_t idx;
    bool issued;
    uint32_t completed_id;
    int32_t turn_tries;
    float64_t check_since;
    int32_t look[3];
    int32_t retries;
    bool front_ok;
    float64_t recover_side;
    bool tape_in;
    float64_t tape_enter_y;
    float64_t tape_y;
    bool tape_meas;
    bool acquired;
    bool resumed;
    bool sweep_stop;
    bool lf;      /* line following */
    float64_t t_settle;
    int32_t halted;
    const char * halt_why;
    float64_t min_clear;
    float64_t worst_t;
} veh_t;
static float64_t nav_x (const veh_t * p_v)
{
    float64_t dx = p_v->px - p_v->ox;
    float64_t dy = p_v->py - p_v->oy;
    return dx * cos(p_v->oh * D2R) + dy * sin(p_v->oh * D2R);
}

static float64_t nav_y (const veh_t * p_v)
{
    float64_t dx = p_v->px - p_v->ox;
    float64_t dy = p_v->py - p_v->oy;
    return -dx * sin(p_v->oh * D2R) + dy * cos(p_v->oh * D2R);
}

static float64_t nav_h (const veh_t * p_v)
{
    return p_v->ph - p_v->oh;
}

static bool ir_black (const veh_t * p_v)
{
    float64_t yy = p_v->py + B5_AXLE_TO_SENSOR_MM * sin(p_v->ph * D2R);
    return fabs(yy) <= 10.0;
}

static void build_recover (veh_t * p_v, bool b_extend)
{
    int32_t i = 0;

    if (b_extend)
    {
        p_v->act[i++] = (act_t){A_TURN, 0};
        p_v->act[i++] = (act_t){A_MOVE, B5_EXTEND_MM};
    }

    p_v->act[i++] = (act_t){A_TURN, p_v->recover_side * 90.0};
    p_v->act[i++] = (act_t){A_CHECK, 0};
    p_v->act[i++] = (act_t){A_SWEEP, fabs(nav_y(p_v)) + B5_LINE_SWEEP_EXTRA_MM};
    p_v->act[i++] = (act_t){A_REJOIN, -p_v->recover_side};
    p_v->n = i;
    p_v->idx = 0;
    p_v->issued = false;
    p_v->tape_in = false;
    p_v->tape_meas = false;
    p_v->check_since = -1;
}

static bool nav_accept (veh_t * p_v, const b5_request_t * p_r)
{
    if (p_v->failed)
    {
        return false;
    }

    if ((p_v->active) && (p_r->kind != B5_REQ_STOP))
    {
        return false;
    }

    p_v->req = *p_r;
    p_v->active = true;
    p_v->issued = false;
    p_v->idx = p_v->n = 0;
    p_v->turn_tries = 0;
    p_v->lf = false;
    p_v->acquired = false;
    p_v->retries = 0;
    p_v->front_ok = false;

    switch (p_r->kind)
    {
        case B5_REQ_STOP:
            p_v->mot = 0;
            p_v->mot_rem = 0;
            p_v->still = 0;
        break;   /* braking handled in step */
        case B5_REQ_LOOK:
            p_v->act[0] = (act_t){A_TURN, (float64_t)p_r->lateral_mm};
            p_v->n = 1;
        break;
        case B5_REQ_BYPASS:
            {
                float64_t d = p_r->lateral_mm - nav_y(p_v);
                p_v->act[0] = (act_t){A_TURN, d >= 0 ? 90 : -90};
                p_v->act[1] = (act_t){A_MOVE, fabs(d)};
                p_v->act[2] = (act_t){A_TURN, 0};
                p_v->n = 3;
                break;
            }
        case B5_REQ_ADVANCE:
            {
                if ((p_r->forward_mm <= nav_x(p_v)) || (fabs(nav_h(p_v)) > 8))
                {
                    p_v->failed = true;
                    return false;
                }

                float64_t dx = p_r->forward_mm - nav_x(p_v),
                dy = p_r->lateral_mm - nav_y(p_v);
                float64_t b = atan2(dy, dx) / D2R;

                if (b > 8)
                {
                    b = 8;
                }

                if (b < -8)
                {
                    b = -8;
                }

                p_v->act[0] = (act_t){A_TURN, b};
                p_v->act[1] = (act_t){A_MOVE, dx / cos(b * D2R)};
                p_v->act[2] = (act_t){A_TURN, 0};
                p_v->n = 3;
                break;
            }
        case B5_REQ_RECOVER_LINE:
            p_v->recover_side = (nav_y(p_v) > 0) ? -1 : 1;
            build_recover(p_v, false);
        break;
        case B5_REQ_RESUME_LINE:
        break;
        default:
            p_v->active = false;
            return false;
    }

    return true;
}

static void begin_turn (veh_t * p_v, float64_t by_deg)
{
    p_v->mot = 1;
    p_v->mot_target = p_v->ph + by_deg + gauss() * 1.0;
    p_v->mot_rem = fabs(by_deg);
    p_v->mot_dir = by_deg >= 0 ? 1 : -1;
    p_v->mot_speed = 90.0;
}

static void begin_move (veh_t * p_v, float64_t mm, float64_t speed)
{
    p_v->mot = 2;
    p_v->mot_dir = mm >= 0 ? 1 : -1;
    p_v->mot_rem = fabs(mm) * (1.0+gauss() * 0.01);
    p_v->mot_speed = speed;
}

typedef struct
{
    float64_t ang;
    float64_t fr_mm;
    bool fresh;
} front_t;
/* one 10 ms step of the vehicle */
static void veh_step (const scen_t * s, veh_t * p_v, float64_t now,
                      float64_t dt,
                      front_t fr)
{
    if (p_v->halted)
    {
        return;
    }

    /* motion */
    if (1 == p_v->mot)
    {
        float64_t step = p_v->mot_speed * dt;
        float64_t err = p_v->mot_target - p_v->ph;

        if (fabs(err) <= step)
        {
            p_v->ph = p_v->mot_target;
            p_v->mot = 0;
            p_v->still = 0;
        }
        else
        {
            p_v->ph += err > 0 ? step : -step;
        }
    }
    else if (2 == p_v->mot)
    {
        float64_t step = p_v->mot_speed * dt;

        if (step > p_v->mot_rem)
        {
            step = p_v->mot_rem;
        }

        p_v->px += p_v->mot_dir * step * cos(p_v->ph * D2R);
        p_v->py += p_v->mot_dir * step * sin(p_v->ph * D2R);
        p_v->mot_rem -= step;

        if (p_v->mot_rem <= 1e-9)
        {
            p_v->mot = 0;
            p_v->still = 0;
        }
    }
    else
    {
        if (p_v->still < 1000)
        {
            p_v->still++;
        }
    }

    /* collision bookkeeping */
    float64_t cl = clearance(s, p_v->px, p_v->py, p_v->ph);

    if (cl < p_v->min_clear)
    {
        p_v->min_clear = cl;
        p_v->worst_t = now;
    }

    /* line following: drive along the line */
    if ((p_v->lf) && (!p_v->active))
    {
        p_v->px += 100.0*dt;
        p_v->py *= 0.9;
        p_v->ph *= 0.9;

        if (fabs(p_v->ph) < 0.05)
        {
            p_v->ph = 0;
        }

        return;
    }

    if (!p_v->active)
    {
        return;
    }

    bool b_still = ((0 == p_v->mot) && (p_v->still >= 3));
    /* STOP */
    if (B5_REQ_STOP == p_v->req.kind)
    {
        if ((!p_v->issued) && (0 == p_v->mot))
        {
            p_v->issued = true;
            begin_move(p_v, s->stop_overshoot, 150.0);
            return;
        }

        if (b_still && (p_v->issued))
        {
            p_v->ox = p_v->px;
            p_v->oy = p_v->py;
            p_v->oh = p_v->ph;
            p_v->active = false;
            p_v->completed_id = p_v->req.id;
        }

        return;
    }

    if (B5_REQ_RESUME_LINE == p_v->req.kind)
    {
        if (b_still && ((ir_black(p_v)) || (p_v->acquired)
                        || (fabs(p_v->py) < 12)))
        {
            p_v->lf = true;
            p_v->completed_id = p_v->req.id;
            p_v->active = false;
            p_v->resumed = true;
        }

        return;
    }

    /* sonar watch during the sweep to the line */
    if ((B5_REQ_RECOVER_LINE == p_v->req.kind) && (p_v->issued)
        && (p_v->idx < p_v->n)
        && (A_SWEEP == p_v->act[p_v->idx].k) && (!p_v->sweep_stop)
        && (2 == p_v->mot))
    {
        if ((!p_v->tape_in) && (fr.fresh) && (0 == fr.ang)
            && (fr.fr_mm != B5_CLEAR_FRONT_MM)
            && (fr.fr_mm >= 0))
        {
            float64_t remain =
                    fabs(nav_y(p_v) + B5_AXLE_TO_SENSOR_MM
                         * sin(nav_h(p_v) * D2R));

            if ((fr.fr_mm < remain - B5_CHECK_MARGIN_MM)
                && (fr.fr_mm <= B5_SWEEP_STOP_MM))
            {
                p_v->mot = 0;
                p_v->mot_rem = 0;
                p_v->still = 0;
                p_v->sweep_stop = true;
            }
        }

        bool black = ir_black(p_v);
        float64_t sy =
                nav_y(p_v) + B5_AXLE_TO_SENSOR_MM * sin(nav_h(p_v) * D2R);

        if (black && (!p_v->tape_in))
        {
            p_v->tape_in = true;
            p_v->tape_enter_y = sy;
        }
        else if ((p_v->tape_in)
                 && ((!black) || (fabs(sy
                                       - p_v->tape_enter_y) >= B5_TAPE_MAX_MM)))
        {
            p_v->tape_y = 0.5*(p_v->tape_enter_y + sy);
            p_v->tape_meas = true;
            p_v->mot = 0;
            p_v->mot_rem = 0;
            p_v->still = 0;
            p_v->sweep_stop = true;
            p_v->tape_in = false;
            p_v->acquired = true;
        }
    }

    if (p_v->sweep_stop)
    {
        if (!((0 == p_v->mot) && (p_v->still >= 3)))
        {
            return;
        }

        p_v->sweep_stop = false;

        if (!p_v->tape_meas) /* blocked on the way: extend */
        {
            if (p_v->retries >= B5_MAX_EXTENDS)
            {
                p_v->failed = true;
                return;
            }

            ++p_v->retries;
            p_v->front_ok = false;
            build_recover(p_v, true);
            return;
        }

        p_v->issued = false;
        ++p_v->idx;
        return;
    }

    if (p_v->issued)
    {
        if ((0 == p_v->mot) && (p_v->still >= 3))
        {
            p_v->issued = false;

            if (A_TURN == p_v->act[p_v->idx].k)
            {
                float64_t err = p_v->act[p_v->idx].v - nav_h(p_v);

                if ((fabs(err) > 2.0) && (p_v->turn_tries < 3))
                {
                    ++p_v->turn_tries;
                }
                else
                {
                    ++p_v->idx;
                    p_v->turn_tries = 0;
                }
            }
            else
            {
                ++p_v->idx;
            }
        }
        else
        {
            return;
        }
    }

    if (p_v->idx >= p_v->n)
    {
        bool b_pose = fabs(nav_h(p_v)) <= 8;

        if (B5_REQ_RECOVER_LINE == p_v->req.kind)
        {
            b_pose = fabs(nav_h(p_v)) <= 30;
        }

        if (B5_REQ_LOOK == p_v->req.kind)
        {
            b_pose = fabs(nav_h(p_v) - p_v->req.lateral_mm) <= 12;
        }

        if (B5_REQ_BYPASS == p_v->req.kind)
        {
            b_pose = b_pose && (fabs(nav_x(p_v)) <= B5_ADV_TOL_X_MM)
                    && (fabs(nav_y(p_v)
                             - p_v->req.lateral_mm) <= B5_ADV_TOL_Y_MM);
        }

        if (B5_REQ_ADVANCE == p_v->req.kind)
        {
            b_pose = b_pose && (fabs(nav_x(p_v)
                                     - p_v->req.forward_mm) <= B5_ADV_TOL_X_MM)
                    && (fabs(nav_y(p_v)
                             - p_v->req.lateral_mm) <= B5_ADV_TOL_Y_MM);
        }

        if ((B5_REQ_RECOVER_LINE == p_v->req.kind) && (p_v->acquired))
        {
            b_pose = true;
        }

        if (!b_pose)
        {
            p_v->failed = true;
            return;
        }

        p_v->completed_id = p_v->req.id;
        p_v->active = false;
        return;
    }

    if ((p_v->mot != 0) || (p_v->still < 3))
    {
        return;
    }

    act_t a = p_v->act[p_v->idx];

    if (A_TURN == a.k)
    {
        float64_t err = a.v - nav_h(p_v);

        if (fabs(err) <= 2.0)
        {
            ++p_v->idx;
            p_v->turn_tries = 0;
            return;
        }

        begin_turn(p_v, err);
        p_v->issued = true;
        return;
    }

    if (A_CHECK == a.k)
    {
        if (p_v->check_since < 0)
        {
            p_v->check_since = now;
            p_v->look[0] = p_v->look[1] = p_v->look[2] = 0;
        }

        if (now - p_v->check_since < B5_CHECK_SETTLE_MS)
        {
            return;
        }

        if (fr.fresh)
        {
            float64_t to_line =
                    fabs(nav_y(p_v)) - B5_AXLE_TO_SENSOR_MM - B5_CHECK_MARGIN_MM
                    + (fr.ang != 0 ? B5_CHECK_BEYOND_MM : 0);
            float64_t c = cos(fr.ang * D2R);
            int32_t ok = (fr.fr_mm >= to_line / (c > 0.5 ? c : 0.5)) ? 1 : 2;
            int32_t aa = (int32_t)fabs(fr.ang);
            int32_t k = 0 == aa ? 0 : (aa <= 20 ? 1 : 2);

            if (p_v->look[k] != 2)
            {
                p_v->look[k] = ok;
            }
        }

        bool b_blocked = (2 == p_v->look[0]) || (2 == p_v->look[1])
                || (2 == p_v->look[2]);
        bool b_done = (p_v->look[0]) && (p_v->look[1]) && (p_v->look[2]);

        if ((!b_blocked) && (!b_done))
        {
            if (now - p_v->check_since < B5_CHECK_SETTLE_MS + 4
                * B5_LOOK_PERIOD_MS)
            {
                return;
            }

            if (!p_v->look[0])
            {
                b_blocked = true;
            }
        }

        if (!b_blocked)
        {
            p_v->front_ok = true;
            ++p_v->idx;
            p_v->check_since = -1;
            return;
        }

        if (p_v->retries >= B5_MAX_EXTENDS)
        {
            p_v->failed = true;
            return;
        }

        ++p_v->retries;
        p_v->front_ok = false;
        build_recover(p_v, true);
        return;
    }

    if (A_REJOIN == a.k)
    {
        float64_t adv = B5_LINE_SWEEP_EXTRA_MM * 0;
        float64_t sn = sin(nav_h(p_v) * D2R);

        if ((p_v->tape_meas) && (fabs(sn) > 0.5))
        {
            adv = (p_v->tape_y - nav_y(p_v)) / sn;
        }

        if (adv > B5_LINE_ADVANCE_MM_SIM + 80)
        {
            adv = B5_LINE_ADVANCE_MM_SIM + 80;
        }

        begin_move(p_v, adv, 60.0);
        p_v->issued = true;
        /* then pivot back: modelled as an extra TURN action */
        p_v->act[p_v->idx] = (act_t){A_TURN, 0};
        p_v->act[p_v->idx + 1] = (act_t){A_TURN, 0};
        p_v->n = p_v->idx + 2;
        return;
    }

    /* MOVE / SWEEP */
    if ((A_MOVE == a.k) && (B5_REQ_BYPASS == p_v->req.kind) && (fr.fresh)
        && (0 == fr.ang)
        && (fr.fr_mm <= B5_BYPASS_GUARD_FRONT_MM))
    {
        p_v->halted = 1;
        p_v->halt_why = "front blocked during bypass";
        return;
    }

    begin_move(p_v, a.v, B5_BYPASS_SPEED_CM_S * 10.0);
    p_v->issued = true;
}

/* ======================================================================== run
 * one scenario */
typedef struct
{
    int32_t ok;
    float64_t min_clear;
    int32_t side;
    int32_t reason;
    int32_t fault;
    int32_t halted;
    const char * why;
    float64_t t;
    int32_t state;
    float64_t worst_t;
    float64_t fx;
    float64_t fy;
    float64_t fh;
    int32_t offset;
    int32_t pass_x;
    int32_t vfailed;
    int32_t phase_at_end;
    int32_t nreq;
} result_t;
static result_t run (const scen_t * s, int32_t verbose)
{
    s_rs = s->seed * 2654435761u + 12345u;
    veh_t v;
    memset(&v, 0, sizeof(v));
    v.px = s->start_x;
    v.ox = v.px;
    v.lf = true;
    v.min_clear = 1e9;
    v.completed_id = 0;
    b5_core_t core;
    b5_core_init(&core, 0);
    b5_sampler_t smp;
    b5_sampler_reset(&smp);
    int32_t servo = 0;
    uint32_t settle_until = 300;
    uint32_t next_ping = 0;
    b5_range_t last = {false, 0};
    int32_t last_ang = 0;
    uint32_t last_ms = 0;
    float64_t extra_x = 0;
    float64_t extra_y = 0;
    result_t R;
    memset(&R, 0, sizeof(R));
    const float64_t dt = 0.01;
    uint32_t prev_state = 99;
    int32_t prev_phase = 99;

    for (float64_t t = 0; t < 240.0; t += dt)
    {
        uint32_t now = (uint32_t)(t * 1000.0+0.5);
        front_t fr;
        fr.ang = last_ang;
        fr.fr_mm = b5_front_clearance(&last);
        fr.fresh = (last.valid) && ((now - last_ms) <= 400u);
        veh_step(s, &v, t * 1000.0, dt, fr);
        /* scan task */
        b5_range_t out;
        bool b_have = false;
        int32_t ang = b5_core_angle(&core);

        if (ang != servo)
        {
            servo = ang;
            b5_sampler_reset(&smp);
            settle_until = now + B5_SERVO_SETTLE_MS;
        }

        if (now >= next_ping)
        {
            next_ping = now + B5_PING_INTERVAL_MS;

            if (now >= settle_until)
            {
                b5_range_t sample =
                        ping(s, v.px, v.py, v.ph, servo, extra_x, extra_y);

                if (b5_sampler_push(&smp, &sample, &out))
                {
                    b_have = true;
                }
            }
        }

        if (b_have)
        {
            last = out;
            last_ang = servo;
            last_ms = now;
        }

        b5_feedback_t f = {0};
        f.valid = true;
        f.timestamp_ms = now;
        f.stopped = ((0 == v.mot) && (v.still >= 3));
        f.completed_id = v.completed_id;
        f.request_failed = v.failed;
        f.front_ok = v.front_ok;
        f.forward_mm = (int32_t)lround(nav_x(&v));
        f.lateral_mm = (int32_t)lround(nav_y(&v));
        f.heading_aligned = fabs(nav_h(&v)) <= 8;
        b5_core_update(&core, now, &f, b_have ? &out : NULL);
        b5_request_t rq;

        if (b5_core_peek_request(&core, &rq))
        {
            if (nav_accept(&v, &rq))
            {
                b5_core_request_sent(&core, now);
                ++R.nreq;

                if (verbose > 1)
                {
                    printf("   t=%.2f request kind=%d id=%u fwd=%d lat=%d\n", t,
                           rq.kind, rq.id, rq.forward_mm, rq.lateral_mm);
                }
            }
        }

        if ((s->allow_extra_blocked_at_adv) && (B5_AVOIDING == core.state)
            && (5 == core.phase) && (0 == extra_x))
        {
            extra_x = v.px + 330;
            extra_y = v.py;
        }

        if (verbose && ((core.state != prev_state)
                        || ((int32_t)core.phase != prev_phase)))
        {
            printf("   t=%6.2f state=%-13s phase=%d pose x=%6.0f y=%5.0f "
                   "h=%5.1f "
                   "front=%d\n", t, b5_state_name(core.state), core.phase,
                   nav_x(&v),
                   nav_y(&v), nav_h(&v),
                   B5_CLEAR_FRONT_MM == core.front_clearance_mm ? -1
                   : core.front_clearance_mm);
            prev_state = core.state;
            prev_phase = core.phase;
        }

        if (B5_ERROR == core.state)
        {
            R.fault = core.fault;
            break;
        }

        if (v.halted)
        {
            R.halted = 1;
            R.why = v.halt_why;
            break;
        }

        if (v.failed)
        {
            R.vfailed = 1;
            break;
        }

        if ((B5_MONITORING == core.state) && (v.lf) && (!v.active)
            && (v.px > s->obs[0].x1 + 700))
        {
            R.ok = 1;
            break;
        }

        R.t = t;
    }

    R.min_clear = v.min_clear;
    R.worst_t = v.worst_t;
    R.side = core.profile.side;
    R.reason = core.profile.side_reason;
    R.state = core.state;
    R.fx = v.px;
    R.fy = v.py;
    R.fh = v.ph;
    R.offset = core.offset_mm;
    R.pass_x = core.pass_x_mm;
    R.phase_at_end = core.phase;
    return R;
}

/* ========================================================================
 * scenarios */
#define BOX(y0,y1,x0,x1) {x0,x1,y0,y1}
static scen_t base (const char * name)
{
    scen_t s;
    memset(&s, 0, sizeof(s));
    s.name = name;
    s.obs[0] = (rect_t)BOX(-50, 50, 1000, 1150);
    s.nobs = 1;
    s.fr = 177;
    s.rr = 247;
    s.hw = 85;
    s.beam = 10;
    s.noise = 4;
    s.start_x = 0;
    s.expect_side = 0;
    s.seed = 1;
    s.stop_overshoot = 15;
    return s;
}

int main (int argc, char ** argv)
{
    int32_t verbose = argc > 1 ? atoi(argv[1]) : 0;
    int32_t only = argc > 2 ? atoi(argv[2]) : -1;
    scen_t S[40];
    int32_t n = 0;
    S[n] = base("centred box, open track (default side)");
    n++;
    S[n] = base("wall close on RIGHT -> must go LEFT");
    S[n].obs[1] = (rect_t)BOX(-330, -250, 850, 1500);
    S[n].nobs = 2;
    S[n].expect_side = 1;
    n++;
    S[n] = base("wall close on LEFT -> must go RIGHT");
    S[n].obs[1] = (rect_t)BOX(250, 330, 850, 1500);
    S[n].nobs = 2;
    S[n].expect_side = -1;
    n++;
    S[n] = base("box 15 mm left of line");
    S[n].obs[0] = (rect_t)BOX(-35, 65, 1000, 1150);
    n++;
    S[n] = base("box 15 mm right of line");
    S[n].obs[0] = (rect_t)BOX(-65, 35, 1000, 1150);
    n++;
    S[n] = base("box 20 mm deeper than spec");
    S[n].obs[0] = (rect_t)BOX(-50, 50, 1000, 1170);
    n++;
    S[n] = base("box wider than spec (160)");
    S[n].obs[0] = (rect_t)BOX(-80, 80, 1000, 1150);
    n++;
    S[n] = base("car rolls 60 mm after the stop");
    S[n].stop_overshoot = 75;
    n++;
    S[n] = base("car rolls 0 mm after the stop");
    S[n].stop_overshoot = 0;
    n++;
    S[n] = base("heavy noise + spikes + dropouts");
    S[n].noise = 10;
    S[n].p_spike = 0.03;
    S[n].p_drop = 0.05;
    n++;
    S[n] = base("wide sonar beam (15 deg)");
    S[n].beam = 15;
    n++;
    S[n] = base("narrow sonar beam (5 deg)");
    S[n].beam = 5;
    n++;
    S[n] = base("true car 108 front / 247 rear");
    S[n].fr = 108;
    S[n].rr = 247;
    n++;
    S[n] = base("true car 177 front / 177 rear");
    S[n].fr = 177;
    S[n].rr = 177;
    n++;
    S[n] = base("both sides walled -> must stop, no motion into walls");
    S[n].obs[1] = (rect_t)BOX(-330, -250, 850, 1500);
    S[n].obs[2] = (rect_t)BOX(250, 330, 850, 1500);
    S[n].nobs = 3;
    S[n].expect_side = 99;
    n++;
    S[n] = base("person steps in during the pass -> must stop");
    S[n].allow_extra_blocked_at_adv = 1;
    S[n].expect_side = 99;
    n++;
    S[n] = base("wall ahead on RIGHT pass (seen only later) -> fall back LEFT");
    S[n].obs[1] = (rect_t)BOX(-420, -220, 1420, 1700);
    S[n].nobs = 2;
    S[n].expect_side = 1;
    n++;
    S[n] = base("wall ahead on LEFT pass (seen only later) -> stay RIGHT");
    S[n].obs[1] = (rect_t)BOX(220, 420, 1420, 1700);
    S[n].nobs = 2;
    S[n].expect_side = -1;
    n++;
    int32_t total = 0;
    int32_t passed = 0;

    for (int32_t i = 0; i < n; i++)
    {
        if ((only >= 0) && (i != only))
        {
            continue;
        }

        int32_t nseed = 10;
        int32_t okc = 0;
        int32_t bad = 0;
        float64_t worst = 1e9;
        const char * firstfail = NULL;
        char buf[200] = "";

        for (int32_t sd = ((only >= 0)
                           && (argc > 3) ? atoi(argv[3])
                                   : 1); sd <= nseed; sd++)
        {
            scen_t sc = S[i];
            sc.seed = (uint32_t)sd;
            result_t r = run(&sc, only >= 0 ? verbose : 0);
            int32_t good;

            if (99 == sc.expect_side)
            {
                good = ((r.fault != 0) || (r.halted)) && (r.min_clear >= 0)
                        && (!r.vfailed);
            }
            else
            {
                good = (r.ok) && (r.min_clear >= 10)
                        && ((0 == sc.expect_side)
                            || (r.side == sc.expect_side));
            }

            if (r.min_clear < worst)
            {
                worst = r.min_clear;
            }

            if (good)
            {
                ++okc;
            }
            else
            {
                ++bad;

                if (!firstfail)
                {
                    firstfail = sc.name;
                    snprintf(buf, sizeof(buf),
                             "seed %d: ok=%d fault=%d halted=%d vfail=%d "
                             "side=%d(want "
                             "%d) min_clear=%.0f state=%d "
                             "pos=(%.0f,%.0f,%.0f) t=%.1f "
                             "worst_t=%.2f", sd, r.ok, r.fault, r.halted,
                             r.vfailed,
                             r.side, sc.expect_side, r.min_clear, r.state, r.fx,
                             r.fy,
                             r.fh, r.t, r.worst_t / 1000.0);
                }
            }
        }

        ++total;

        if (!bad)
        {
            ++passed;
        }

        printf("[%s] %-62s %d/%d  worst clearance %4.0f mm\n",
               bad ? "FAIL" : "PASS",
               S[i].name, okc, nseed, worst);

        if (bad)
        {
            printf("       %s\n", buf);
        }
    }

    printf("\n%d/%d scenarios fully passing\n", passed, total);
    return passed == total ? 0 : 1;
}

/*** end of file ***/
