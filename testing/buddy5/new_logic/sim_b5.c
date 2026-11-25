/* Host simulation of the NEW Buddy 5 logic: real planner + real state machine
 * against a simple world.
 * Build: see run_tests.sh. World frame: the line is the x axis, +x forward, +y
 * left, degrees + = left (CCW). */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "buddy5_plan.h"
#define D2R 0.017453292519943295
#define B5_LINE_ADVANCE_MM_SIM 40.0
typedef struct
{
    double x0;
    double x1;
    double y0;
    double y1;
} rect_t;
typedef struct
{
    const char * name;
    rect_t obs[6];
    int nobs;      /* obs[0] is the box */
    double fr;
    double rr;
    double hw;              /* TRUE car outline */
    double beam;                  /* TRUE sonar cone half angle */
    double noise;
    double p_spike;
    double p_drop;
    double start_x;               /* axle x at the start (line following) */
    /* 0 don't care, +1 left, -1 right, 99 = expect a stop (no route) */
    int expect_side;
    uint32_t seed;
    double stop_overshoot;        /* extra distance the car rolls after STOP */
    /* spawn an obstacle during the pass (person) */
    int allow_extra_blocked_at_adv;
} scen_t;
/* ---------- tiny RNG ---------- */
static uint32_t rs;
static double rnd (void)
{
    rs = rs * 1664525u + 1013904223u;
    return (rs >> 8) / 16777216.0;
}

static double gauss (void)
{
    double u = rnd() + 1e-9, v = rnd();
    return sqrt(-2 * log(u)) * cos(6.283185307*v);
}

/* ---------- geometry ---------- */
typedef struct
{
    double x[4];
    double y[4];
} quadr_t;
static int sat (const quadr_t * a, const quadr_t * b)
{
    const quadr_t * q[2] = {a, b};

    for (int k = 0; k < 2; k++)
    {
        for (int i = 0; i < 4; i++)
        {
            int jdx = (i + 1) & 3;
            double ax = -(q[k]->y[jdx] - q[k]->y[i]),
            ay = q[k]->x[jdx] - q[k]->x[i];
            double amin = 1e30;
            double amax = -1e30;
            double bmin = 1e30;
            double bmax = -1e30;

            for (int p = 0; p < 4; p++)
            {
                double da = ax * a->x[p] + ay * a->y[p];
                double db = ax * b->x[p] + ay * b->y[p];

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

static quadr_t car_quad (const scen_t * s, double px, double py, double ph)
{
    double c = cos(ph * D2R), sn = sin(ph * D2R);
    quadr_t q;
    double bx[4] = {s->fr, s->fr, -s->rr, -s->rr}, by[4] = {s->hw, -s->hw,
            -s->hw,
        s->hw};

    for (int i = 0; i < 4; i++)
    {
        q.x[i] = px + bx[i] * c - by[i] * sn;
        q.y[i] = py + bx[i] * sn + by[i] * c;
    }

    return q;
}

static quadr_t rect_quad (rect_t r, double m)
{
    quadr_t q = {{r.x0 - m, r.x1 + m, r.x1 + m, r.x0 - m}, {r.y0 - m, r.y0 - m,
            r.y1 + m, r.y1 + m}};
    return q;
}

/* smallest clearance (mm, 5 mm resolution, -1 = touching) between the car and
 * any obstacle */
static double clearance (const scen_t * s, double px, double py, double ph)
{
    quadr_t c = car_quad(s, px, py, ph);
    double best = 1e9;

    for (int o = 0; o < s->nobs; o++)
    {
        double m;
        int hit = 0;

        for (m = 0; m <= 60; m += 5)
        {
            quadr_t r = rect_quad(s->obs[o], m);

            if (sat(&c, &r))
            {
                hit = 1;
                break;
            }
        }

        double cl = hit ? (0 == m ? -1 : m - 5) : 60;

        if (cl < best)
        {
            best = cl;
        }
    }

    return best;
}

static double ray_rect (double ox, double oy, double dx, double dy, rect_t r)
{
    double t0 = 0;
    double t1 = 1e9;
    double o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {r.x0, r.y0},
            hi[2] = {r.x1,
        r.y1};

    for (int i = 0; i < 2; i++)
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
            double a = (lo[i] - o[i]) / d[i], b = (hi[i] - o[i]) / d[i];

            if (a > b)
            {
                double t = a;
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
static b5_range_t ping (const scen_t * s, double px, double py, double ph,
                        double servo, double extra_obs_x, double extra_obs_y)
{
    double sx = px + B5_AXLE_TO_SENSOR_MM * cos(ph * D2R),
    sy = py + B5_AXLE_TO_SENSOR_MM * sin(ph * D2R);
    double best = 1e9;

    for (double off = -s->beam; off <= s->beam + 1e-9; off += 1.5)
    {
        double a = (ph + servo + off) * D2R;

        for (int o = 0; o < s->nobs; o++)
        {
            double d = ray_rect(sx, sy, cos(a), sin(a), s->obs[o]);

            if (d < best)
            {
                best = d;
            }
        }

        if (extra_obs_x > 0)
        {
            rect_t p = {extra_obs_x, extra_obs_x + 80, extra_obs_y - 40,
                extra_obs_y + 40};
            double d = ray_rect(sx, sy, cos(a), sin(a), p);

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
    double v;
} act_t;
typedef struct
{
    /* world pose and nav origin */
    double px;
    double py;
    double ph;
    double ox;
    double oy;
    double oh;
    /* motion */
    int mot;
    double mot_target;
    double mot_rem;
    double mot_speed;
    int mot_dir;
    int still;
    /* request */
    bool active;
    bool failed;
    b5_request_t req;
    act_t act[12];
    int n;
    int idx;
    bool issued;
    uint32_t completed_id;
    int turn_tries;
    double check_since;
    int look[3];
    int retries;
    bool front_ok;
    double recover_side;
    bool tape_in;
    double tape_enter_y;
    double tape_y;
    bool tape_meas;
    bool acquired;
    bool resumed;
    bool sweep_stop;
    bool lf;      /* line following */
    double t_settle;
    int halted;
    const char * halt_why;
    double min_clear;
    double worst_t;
} veh_t;
static double nav_x (const veh_t * v)
{
    double dx = v->px - v->ox;
    double dy = v->py - v->oy;
    return dx * cos(v->oh * D2R) + dy * sin(v->oh * D2R);
}

static double nav_y (const veh_t * v)
{
    double dx = v->px - v->ox;
    double dy = v->py - v->oy;
    return -dx * sin(v->oh * D2R) + dy * cos(v->oh * D2R);
}

static double nav_h (const veh_t * v)
{
    return v->ph - v->oh;
}

static bool ir_black (const veh_t * v)
{
    double yy = v->py + B5_AXLE_TO_SENSOR_MM * sin(v->ph * D2R);
    return fabs(yy) <= 10.0;
}

static void build_recover (veh_t * v, bool extend)
{
    int i = 0;

    if (extend)
    {
        v->act[i++] = (act_t){A_TURN, 0};
        v->act[i++] = (act_t){A_MOVE, B5_EXTEND_MM};
    }

    v->act[i++] = (act_t){A_TURN, v->recover_side * 90.0};
    v->act[i++] = (act_t){A_CHECK, 0};
    v->act[i++] = (act_t){A_SWEEP, fabs(nav_y(v)) + B5_LINE_SWEEP_EXTRA_MM};
    v->act[i++] = (act_t){A_REJOIN, -v->recover_side};
    v->n = i;
    v->idx = 0;
    v->issued = false;
    v->tape_in = false;
    v->tape_meas = false;
    v->check_since = -1;
}

static bool nav_accept (veh_t * v, const b5_request_t * r)
{
    if (v->failed)
    {
        return false;
    }

    if ((v->active) && (r->kind != B5_REQ_STOP))
    {
        return false;
    }

    v->req = *r;
    v->active = true;
    v->issued = false;
    v->idx = v->n = 0;
    v->turn_tries = 0;
    v->lf = false;
    v->acquired = false;
    v->retries = 0;
    v->front_ok = false;

    switch (r->kind)
    {
        case B5_REQ_STOP:
        v->mot = 0;
        v->mot_rem = 0;
        v->still = 0;
        break;   /* braking handled in step */
        case B5_REQ_LOOK:
        v->act[0] = (act_t){A_TURN, (double)r->lateral_mm};
        v->n = 1;
        break;
        case B5_REQ_BYPASS:
        {
            double d = r->lateral_mm - nav_y(v);
            v->act[0] = (act_t){A_TURN, d >= 0 ? 90 : -90};
            v->act[1] = (act_t){A_MOVE, fabs(d)};
            v->act[2] = (act_t){A_TURN, 0};
            v->n = 3;
            break;
        }
        case B5_REQ_ADVANCE:
        {
            if ((r->forward_mm <= nav_x(v)) || (fabs(nav_h(v)) > 8))
            {
                v->failed = true;
                return false;
            }

            double dx = r->forward_mm - nav_x(v), dy = r->lateral_mm - nav_y(v);
            double b = atan2(dy, dx) / D2R;

            if (b > 8)
            {
                b = 8;
            }

            if (b < -8)
            {
                b = -8;
            }

            v->act[0] = (act_t){A_TURN, b};
            v->act[1] = (act_t){A_MOVE, dx / cos(b * D2R)};
            v->act[2] = (act_t){A_TURN, 0};
            v->n = 3;
            break;
        }
        case B5_REQ_RECOVER_LINE:
        v->recover_side = (nav_y(v) > 0) ? -1 : 1;
        build_recover(v, false);
        break;
        case B5_REQ_RESUME_LINE:
        break;
        default:
        v->active = false;
        return false;
    }

    return true;
}

static void begin_turn (veh_t * v, double by_deg)
{
    v->mot = 1;
    v->mot_target = v->ph + by_deg + gauss() * 1.0;
    v->mot_rem = fabs(by_deg);
    v->mot_dir = by_deg >= 0 ? 1 : -1;
    v->mot_speed = 90.0;
}

static void begin_move (veh_t * v, double mm, double speed)
{
    v->mot = 2;
    v->mot_dir = mm >= 0 ? 1 : -1;
    v->mot_rem = fabs(mm) * (1.0+gauss() * 0.01);
    v->mot_speed = speed;
}

typedef struct
{
    double ang;
    double fr_mm;
    bool fresh;
} front_t;
/* one 10 ms step of the vehicle */
static void veh_step (const scen_t * s, veh_t * v, double now, double dt,
                      front_t fr)
{
    if (v->halted)
    {
        return;
    }

    /* motion */
    if (1 == v->mot)
    {
        double step = v->mot_speed * dt;
        double err = v->mot_target - v->ph;

        if (fabs(err) <= step)
        {
            v->ph = v->mot_target;
            v->mot = 0;
            v->still = 0;
        }
        else
        {
            v->ph += err > 0 ? step : -step;
        }
    }
    else if (2 == v->mot)
    {
        double step = v->mot_speed * dt;

        if (step > v->mot_rem)
        {
            step = v->mot_rem;
        }

        v->px += v->mot_dir * step * cos(v->ph * D2R);
        v->py += v->mot_dir * step * sin(v->ph * D2R);
        v->mot_rem -= step;

        if (v->mot_rem <= 1e-9)
        {
            v->mot = 0;
            v->still = 0;
        }
    }
    else
    {
        if (v->still < 1000)
        {
            v->still++;
        }
    }

    /* collision bookkeeping */
    double cl = clearance(s, v->px, v->py, v->ph);

    if (cl < v->min_clear)
    {
        v->min_clear = cl;
        v->worst_t = now;
    }

    /* line following: drive along the line */
    if ((v->lf) && (!v->active))
    {
        v->px += 100.0*dt;
        v->py *= 0.9;
        v->ph *= 0.9;

        if (fabs(v->ph) < 0.05)
        {
            v->ph = 0;
        }

        return;
    }

    if (!v->active)
    {
        return;
    }

    bool still = ((0 == v->mot) && (v->still >= 3));
    /* STOP */
    if (B5_REQ_STOP == v->req.kind)
    {
        if ((!v->issued) && (0 == v->mot))
        {
            v->issued = true;
            begin_move(v, s->stop_overshoot, 150.0);
            return;
        }

        if (still && (v->issued))
        {
            v->ox = v->px;
            v->oy = v->py;
            v->oh = v->ph;
            v->active = false;
            v->completed_id = v->req.id;
        }

        return;
    }

    if (B5_REQ_RESUME_LINE == v->req.kind)
    {
        if (still && ((ir_black(v)) || (v->acquired) || (fabs(v->py) < 12)))
        {
            v->lf = true;
            v->completed_id = v->req.id;
            v->active = false;
            v->resumed = true;
        }

        return;
    }

    /* sonar watch during the sweep to the line */
    if ((B5_REQ_RECOVER_LINE == v->req.kind) && (v->issued) && (v->idx < v->n)
        && (A_SWEEP == v->act[v->idx].k) && (!v->sweep_stop) && (2 == v->mot))
    {
        if ((!v->tape_in) && (fr.fresh) && (0 == fr.ang)
            && (fr.fr_mm != B5_CLEAR_FRONT_MM)
            && (fr.fr_mm >= 0))
        {
            double remain =
                    fabs(nav_y(v) + B5_AXLE_TO_SENSOR_MM * sin(nav_h(v) * D2R));

            if ((fr.fr_mm < remain - B5_CHECK_MARGIN_MM)
                && (fr.fr_mm <= B5_SWEEP_STOP_MM))
            {
                v->mot = 0;
                v->mot_rem = 0;
                v->still = 0;
                v->sweep_stop = true;
            }
        }

        bool black = ir_black(v);
        double sy = nav_y(v) + B5_AXLE_TO_SENSOR_MM * sin(nav_h(v) * D2R);

        if (black && (!v->tape_in))
        {
            v->tape_in = true;
            v->tape_enter_y = sy;
        }
        else if ((v->tape_in)
                 && ((!black) || (fabs(sy
                                       - v->tape_enter_y) >= B5_TAPE_MAX_MM)))
        {
            v->tape_y = 0.5*(v->tape_enter_y + sy);
            v->tape_meas = true;
            v->mot = 0;
            v->mot_rem = 0;
            v->still = 0;
            v->sweep_stop = true;
            v->tape_in = false;
            v->acquired = true;
        }
    }

    if (v->sweep_stop)
    {
        if (!((0 == v->mot) && (v->still >= 3)))
        {
            return;
        }

        v->sweep_stop = false;

        if (!v->tape_meas) /* blocked on the way: extend */
        {
            if (v->retries >= B5_MAX_EXTENDS)
            {
                v->failed = true;
                return;
            }

            ++v->retries;
            v->front_ok = false;
            build_recover(v, true);
            return;
        }

        v->issued = false;
        ++v->idx;
        return;
    }

    if (v->issued)
    {
        if ((0 == v->mot) && (v->still >= 3))
        {
            v->issued = false;

            if (A_TURN == v->act[v->idx].k)
            {
                double err = v->act[v->idx].v - nav_h(v);

                if ((fabs(err) > 2.0) && (v->turn_tries < 3))
                {
                    ++v->turn_tries;
                }
                else
                {
                    ++v->idx;
                    v->turn_tries = 0;
                }
            }
            else
            {
                ++v->idx;
            }
        }
        else
        {
            return;
        }
    }

    if (v->idx >= v->n)
    {
        bool pose = fabs(nav_h(v)) <= 8;

        if (B5_REQ_RECOVER_LINE == v->req.kind)
        {
            pose = fabs(nav_h(v)) <= 30;
        }

        if (B5_REQ_LOOK == v->req.kind)
        {
            pose = fabs(nav_h(v) - v->req.lateral_mm) <= 12;
        }

        if (B5_REQ_BYPASS == v->req.kind)
        {
            pose = pose && (fabs(nav_x(v)) <= B5_ADV_TOL_X_MM)
                    && (fabs(nav_y(v) - v->req.lateral_mm) <= B5_ADV_TOL_Y_MM);
        }

        if (B5_REQ_ADVANCE == v->req.kind)
        {
            pose = pose && (fabs(nav_x(v)
                                 - v->req.forward_mm) <= B5_ADV_TOL_X_MM)
                    && (fabs(nav_y(v) - v->req.lateral_mm) <= B5_ADV_TOL_Y_MM);
        }

        if ((B5_REQ_RECOVER_LINE == v->req.kind) && (v->acquired))
        {
            pose = true;
        }

        if (!pose)
        {
            v->failed = true;
            return;
        }

        v->completed_id = v->req.id;
        v->active = false;
        return;
    }

    if ((v->mot != 0) || (v->still < 3))
    {
        return;
    }

    act_t a = v->act[v->idx];

    if (A_TURN == a.k)
    {
        double err = a.v - nav_h(v);

        if (fabs(err) <= 2.0)
        {
            ++v->idx;
            v->turn_tries = 0;
            return;
        }

        begin_turn(v, err);
        v->issued = true;
        return;
    }

    if (A_CHECK == a.k)
    {
        if (v->check_since < 0)
        {
            v->check_since = now;
            v->look[0] = v->look[1] = v->look[2] = 0;
        }

        if (now - v->check_since < B5_CHECK_SETTLE_MS)
        {
            return;
        }

        if (fr.fresh)
        {
            double to_line =
                    fabs(nav_y(v)) - B5_AXLE_TO_SENSOR_MM - B5_CHECK_MARGIN_MM
                    + (fr.ang != 0 ? B5_CHECK_BEYOND_MM : 0);
            double c = cos(fr.ang * D2R);
            int ok = (fr.fr_mm >= to_line / (c > 0.5 ? c : 0.5)) ? 1 : 2;
            int aa = (int)fabs(fr.ang);
            int k = 0 == aa ? 0 : (aa <= 20 ? 1 : 2);

            if (v->look[k] != 2)
            {
                v->look[k] = ok;
            }
        }

        bool blocked = (2 == v->look[0]) || (2 == v->look[1])
                || (2 == v->look[2]);
        bool done = (v->look[0]) && (v->look[1]) && (v->look[2]);

        if ((!blocked) && (!done))
        {
            if (now - v->check_since < B5_CHECK_SETTLE_MS + 4
                * B5_LOOK_PERIOD_MS)
            {
                return;
            }

            if (!v->look[0])
            {
                blocked = true;
            }
        }

        if (!blocked)
        {
            v->front_ok = true;
            ++v->idx;
            v->check_since = -1;
            return;
        }

        if (v->retries >= B5_MAX_EXTENDS)
        {
            v->failed = true;
            return;
        }

        ++v->retries;
        v->front_ok = false;
        build_recover(v, true);
        return;
    }

    if (A_REJOIN == a.k)
    {
        double adv = B5_LINE_SWEEP_EXTRA_MM * 0;
        double sn = sin(nav_h(v) * D2R);

        if ((v->tape_meas) && (fabs(sn) > 0.5))
        {
            adv = (v->tape_y - nav_y(v)) / sn;
        }

        if (adv > B5_LINE_ADVANCE_MM_SIM + 80)
        {
            adv = B5_LINE_ADVANCE_MM_SIM + 80;
        }

        begin_move(v, adv, 60.0);
        v->issued = true;
        /* then pivot back: modelled as an extra TURN action */
        v->act[v->idx] = (act_t){A_TURN, 0};
        v->act[v->idx + 1] = (act_t){A_TURN, 0};
        v->n = v->idx + 2;
        return;
    }

    /* MOVE / SWEEP */
    if ((A_MOVE == a.k) && (B5_REQ_BYPASS == v->req.kind) && (fr.fresh)
        && (0 == fr.ang)
        && (fr.fr_mm <= B5_BYPASS_GUARD_FRONT_MM))
    {
        v->halted = 1;
        v->halt_why = "front blocked during bypass";
        return;
    }

    begin_move(v, a.v, B5_BYPASS_SPEED_CM_S * 10.0);
    v->issued = true;
}

/* ======================================================================== run
 * one scenario */
typedef struct
{
    int ok;
    double min_clear;
    int side;
    int reason;
    int fault;
    int halted;
    const char * why;
    double t;
    int state;
    double worst_t;
    double fx;
    double fy;
    double fh;
    int offset;
    int pass_x;
    int vfailed;
    int phase_at_end;
    int nreq;
} result_t;
static result_t run (const scen_t * s, int verbose)
{
    rs = s->seed * 2654435761u + 12345u;
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
    int servo = 0;
    uint32_t settle_until = 300;
    uint32_t next_ping = 0;
    b5_range_t last = {false, 0};
    int last_ang = 0;
    uint32_t last_ms = 0;
    double extra_x = 0;
    double extra_y = 0;
    result_t R;
    memset(&R, 0, sizeof(R));
    const double dt = 0.01;
    uint32_t prev_state = 99;
    int prev_phase = 99;

    for (double t = 0; t < 240.0; t += dt)
    {
        uint32_t now = (uint32_t)(t * 1000.0+0.5);
        front_t fr;
        fr.ang = last_ang;
        fr.fr_mm = b5_front_clearance(&last);
        fr.fresh = (last.valid) && ((now - last_ms) <= 400u);
        veh_step(s, &v, t * 1000.0, dt, fr);
        /* scan task */
        b5_range_t out;
        bool have = false;
        int ang = b5_core_angle(&core);

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
                    have = true;
                }
            }
        }

        if (have)
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
        b5_core_update(&core, now, &f, have ? &out : NULL);
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
                        || ((int)core.phase != prev_phase)))
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
    int verbose = argc > 1 ? atoi(argv[1]) : 0;
    int only = argc > 2 ? atoi(argv[2]) : -1;
    scen_t S[40];
    int n = 0;
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
    int total = 0;
    int passed = 0;

    for (int i = 0; i < n; i++)
    {
        if ((only >= 0) && (i != only))
        {
            continue;
        }

        int nseed = 10;
        int okc = 0;
        int bad = 0;
        double worst = 1e9;
        const char * firstfail = NULL;
        char buf[200] = "";

        for (int sd = ((only >= 0)
                       && (argc > 3) ? atoi(argv[3]) : 1); sd <= nseed; sd++)
        {
            scen_t sc = S[i];
            sc.seed = (unsigned)sd;
            result_t r = run(&sc, only >= 0 ? verbose : 0);
            int good;

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
