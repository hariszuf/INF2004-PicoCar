#include "../barrc_types.h"
#include "buddy5_plan.h"
#include <math.h>
#define DEG2RAD 0.01745329252f
static int32_t i_abs (int32_t v)
{
    return v < 0 ? -v : v;
}

static int32_t i_min (int32_t a, int32_t b)
{
    return a < b ? a : b;
}

static int32_t round_f (float32_t v)
{
    return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

/* ---- swept outline test (separating axis on two convex quads) ---- */
typedef struct
{
    float32_t x[4];
    float32_t y[4];
} quad_t;
static bool quads_overlap (const quad_t * a, const quad_t * p_b)
{
    const quad_t * p_q[2] = {a, p_b};

    for (int32_t k = 0; k < 2; k++)
    {
        for (int32_t idx = 0; idx < 4; idx++)
        {
            int32_t jdx = (idx + 1) & 3;
            float32_t ax = -(p_q[k]->y[jdx] - p_q[k]->y[idx]),
            ay = p_q[k]->x[jdx] - p_q[k]->x[idx];
            float32_t amin = 1e30f;
            float32_t amax = -1e30f;
            float32_t bmin = 1e30f;
            float32_t bmax = -1e30f;

            for (int32_t p = 0; p < 4; p++)
            {
                float32_t da = ax * a->x[p] + ay * a->y[p];
                float32_t db = ax * p_b->x[p] + ay * p_b->y[p];

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
                return false;
            }
        }
    }

    return true;
}

bool b5_car_hits_box (float32_t x, float32_t u, float32_t phi_deg,
                      float32_t bx0, float32_t bx1,
                      float32_t bu0, float32_t bu1, float32_t margin)
{
    /* Body coordinates: forward bx, left by. The car is at lateral u (away from
     * the line, positive) and turns toward the
     * line (heading rotates toward -u). Mirror symmetry makes left and right
     * identical. */
    float32_t p = phi_deg * DEG2RAD, c = cosf(p), s = sinf(p);
    static const int32_t sgn[4][2] = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
    quad_t car;
    quad_t box;

    for (int32_t idx = 0; idx < 4; idx++)
    {
        float32_t bx = sgn[idx][0] > 0 ? (float32_t)B5_AXLE_TO_FRONT_MM
                : -(float32_t)B5_AXLE_TO_REAR_MM;
        float32_t by = sgn[idx][1] * (float32_t)B5_CAR_HALF_WIDTH_MM;
        car.x[idx] = x + bx * c + by * s;
        car.y[idx] = u - bx * s + by * c;
    }

    box.x[0] = bx0 - margin;
    box.y[0] = bu0 - margin;
    box.x[1] = bx1 + margin;
    box.y[1] = bu0 - margin;
    box.x[2] = bx1 + margin;
    box.y[2] = bu1 + margin;
    box.x[3] = bx0 - margin;
    box.y[3] = bu1 + margin;
    return quads_overlap(&car, &box);
}

int32_t b5_pass_x (int32_t x_face_mm, int32_t depth_mm, int32_t near_edge_mm,
                   int32_t far_edge_mm, int32_t offset_mm)
{
#if B5_PASS_SIMPLE
    /* cast: result intentionally ignored */
    (void)depth_mm;
    /* cast: result intentionally ignored */
    (void)near_edge_mm;
    /* cast: result intentionally ignored */
    (void)far_edge_mm;
    /* cast: result intentionally ignored */
    (void)offset_mm;
    /* distance to the box + box length + room past its rear face */
    return x_face_mm + B5_BOX_LENGTH_MM + B5_PASS_PAST_REAR_MM;
#else
    float32_t bx0 = (float32_t)x_face_mm,
    bx1 = (float32_t)(x_face_mm + depth_mm);
    float32_t bu0 = (float32_t)(-far_edge_mm), bu1 = (float32_t)near_edge_mm,
    m = (float32_t)B5_SWEEP_MARGIN_MM;
    int32_t limit =
            x_face_mm + depth_mm + B5_AXLE_TO_REAR_MM + B5_AXLE_TO_FRONT_MM
            + B5_CAR_HALF_WIDTH_MM + 200;

    for (int32_t x = x_face_mm; x <= limit; x += 10)
    {
        bool b_ok = true;
        /* beside the box, then the 90 degree pivot, then the drive to the line
         * facing
         * it */
        for (int32_t phi = 0; (phi <= 90) && b_ok; phi += 5)
        {
            if (b5_car_hits_box((float32_t)x, (float32_t)offset_mm,
                                (float32_t)phi, bx0,
                                bx1, bu0,
                                bu1, m))
            {
                b_ok = false;
            }
        }

        for (int32_t u = offset_mm; (u >= 0) && b_ok; u -= 20)
        {
            if (b5_car_hits_box((float32_t)x, (float32_t)u, 90.0f, bx0, bx1,
                                bu0, bu1,
                                m))
            {
                b_ok = false;
            }
        }

        /* finally the pivot back to face along the line, on the line (the rear
         * swings
         * through the box's lane) */
        for (int32_t phi = 90; (phi >= 0) && b_ok; phi -= 5)
        {
            if (b5_car_hits_box((float32_t)x, 0.0f, (float32_t)phi, bx0, bx1,
                                bu0, bu1,
                                m))
            {
                b_ok = false;
            }
        }

        if (b_ok)
        {
            return x + B5_PASS_EXTRA_MM;
        }
    }

    return limit + B5_PASS_EXTRA_MM;
#endif
}

/* ---- scan analysis ---- */
static int32_t median_i (int32_t * p_v, int32_t n)
{
    for (int32_t idx = 1; idx < n; idx++)
    {
        int32_t t = p_v[idx];
        int32_t jdx = idx - 1;

        while ((jdx >= 0) && (p_v[jdx] > t))
        {
            p_v[jdx + 1] = p_v[jdx];
            jdx--;
        }

        p_v[jdx + 1] = t;
    }

    return p_v[n / 2];
}

typedef enum
{
    RAY_NONE, RAY_BOX, RAY_NEAR, RAY_FAR, RAY_CLEAR
} ray_t;
void b5_plan_make (const b5_scan_t * p_scan, int32_t front_hint_mm,
                   uint8_t blocked_mask, b5_plan_t * out)
{
    *out = (b5_plan_t){0};
    /* 1. the box face: median of the straight-ahead cluster (-10..+10 degrees),
     * measured square-on */
    int32_t face[5];
    int32_t nf = 0;

    for (int32_t idx = 0; idx < B5_SCAN_POINTS; idx++)
    {
        int32_t a = b5_slot_angle(idx);

        if ((a < -10) || (a > 10)
            || (B5_SCAN_ECHO != b5_scan_state(p_scan, idx)))
        {
            continue;
        }

        face[nf++] =
                round_f((float32_t)b5_scan_dist(p_scan, idx)
                        * cosf((float32_t)a * DEG2RAD));
    }

    if (nf >= 2)
    {
        out->face_dist_mm = median_i(face, nf);
        out->face_valid = true;
    }
    else if (1 == nf)
    {
        out->face_dist_mm = face[0];
        out->face_valid = true;
    }
    else if ((front_hint_mm > 0) && (B5_CLEAR_FRONT_MM != front_hint_mm))
    {
        /* no echo straight ahead: only trust the stop-time reading if something
         * is
         * still that close */
        out->face_dist_mm = front_hint_mm + B5_FRONT_OVERHANG_MM;
#if B5_SCAN_ONLY
        /* measuring only: the trigger reading was a real close object, so use
         * it as
         * the face */
        out->face_valid = true;
#else
        out->face_valid = false;
#endif
    }
    else
    {
        /* no action required */
    }

    if (!out->face_valid)
    {
        return;
    }

    /* A "box" must be close: anything farther than the stop distance plus a
     * margin
     * is not what stopped the car. */
    if (out->face_dist_mm > B5_STOP_FRONT_MM + B5_FRONT_OVERHANG_MM + 250)
    {
        out->face_valid = false;
        return;
    }

    out->x_face_mm = B5_AXLE_TO_SENSOR_MM + out->face_dist_mm;
    /* 2. label every ray against the face plane */
    ray_t cls[B5_SCAN_POINTS];
    int32_t lat[B5_SCAN_POINTS];

    for (int32_t idx = 0; idx < B5_SCAN_POINTS; idx++)
    {
        float32_t a = (float32_t)b5_slot_angle(idx) * DEG2RAD;
        lat[idx] = 0;
        cls[idx] = RAY_NONE;

        if (B5_SCAN_NO_ECHO == b5_scan_state(p_scan, idx))
        {
            cls[idx] = RAY_CLEAR;
            continue;
        }

        if (B5_SCAN_ECHO != b5_scan_state(p_scan, idx))
        {
            continue;
        }

        float32_t d = (float32_t)b5_scan_dist(p_scan, idx),
        expect = (float32_t)out->face_dist_mm / cosf(a);
        lat[idx] = round_f(d * sinf(a));   /* + = left of the sonar axis */
        if (d < expect - (float32_t)B5_FACE_TOL_MM)
        {
            cls[idx] = RAY_NEAR;
        }
        else if (d > expect + (float32_t)B5_FACE_TOL_MM)
        {
            cls[idx] = RAY_FAR;
        }
        else
        {
            cls[idx] = RAY_BOX;
        }
    }

    /* 3. per side: box edge (the measured scan edge; the specified width is
     * only
     * the fallback when the edge is not seen) and the room beyond it */
    const int32_t spec_edge = B5_BOX_WIDTH_MM / 2 + B5_BOX_POS_TOL_MM;
    const int32_t depth = B5_BOX_LENGTH_MM + B5_BOX_DEPTH_TOL_MM;

    for (int32_t s = 0; s < 2; s++)
    {
        /* slot step toward the left (+) or right (-) */
        int32_t dir = 0 == s ? 1 : -1;
        int32_t centre = -B5_SCAN_MIN_DEG / 5;
        int32_t first_miss = -1;

        for (int32_t idx = centre; (idx >= 0)
             && (idx < B5_SCAN_POINTS); idx += dir)
        {
            if ((RAY_BOX == cls[idx]) || (RAY_NONE == cls[idx]))
            {
                continue;
            }

            first_miss = idx;
            break;
        }

        /* fallback only: used when the sonar never finds this side's edge */
        int32_t edge = spec_edge;
        out->seen[s] = false;
        out->meas_edge_mm[s] = 0;
        {
            /* outermost ray that still hit the box: the box reaches at least
             * that far */
            int32_t last_box = -1;

            for (int32_t idx = centre; (idx >= 0) && (idx < B5_SCAN_POINTS)
                 && (idx != first_miss); idx += dir)
            {
                if (RAY_BOX == cls[idx])
                {
                    last_box = idx;
                }
            }

            if (last_box >= 0)
            {
                float32_t a_lo =
                        (float32_t)i_abs(b5_slot_angle(last_box))
                        - (float32_t)B5_BEAM_HALF_DEG;

                if (a_lo > 0.0f)
                {
                    out->meas_edge_mm[s] =
                            round_f((float32_t)out->face_dist_mm
                                    * tanf(a_lo * DEG2RAD));
                }
            }
        }

        if (first_miss >= 0)
        {
            float32_t a_miss = (float32_t)i_abs(b5_slot_angle(first_miss));
            float32_t a_edge = a_miss - (float32_t)B5_BEAM_HALF_DEG;

            if (a_edge > 0.0f)
            {
                int32_t e = round_f((float32_t)out->face_dist_mm
                                    * tanf(a_edge * DEG2RAD));
                out->seen[s] = true;
                out->meas_edge_mm[s] = e;
                /* the sonar found where the box ends on this side: use that,
                 * not the
                 * specified width */
                edge = e;
            }
            else
            {
                out->seen[s] = true;
            }
        }

        out->edge_mm[s] = edge;
        /* room: nearest thing beyond the edge on that side (NEAR rays, or far
         * rays
         * that are not in the box shadow) */
        int32_t room = B5_ROOM_FAR_MM;

        for (int32_t idx = 0; idx < B5_SCAN_POINTS; idx++)
        {
            int32_t a = b5_slot_angle(idx);

            if (((0 == s) && (a <= 0)) || ((1 == s) && (a >= 0)))
            {
                continue;
            }

            if ((RAY_NEAR != cls[idx]) && (RAY_FAR != cls[idx]))
            {
                continue;
            }

            int32_t u = i_abs(lat[idx]);
            int32_t x_hit =
                    B5_AXLE_TO_SENSOR_MM
                    + round_f((float32_t)b5_scan_dist(p_scan, idx)
                              * cosf((float32_t)a * DEG2RAD));

            if (x_hit > out->x_face_mm + depth + B5_AXLE_TO_FRONT_MM + 250)
            {
                continue;   /* beyond the pass: not in the way */
            }

            if (u <= edge)
            {
                /* inside the box width: not "beside" */
                continue;
            }

            room = i_min(room, u - edge);
        }

        out->room_mm[s] = room;
    }

    /* 4. feasibility, cost, pass distance per side */
    for (int32_t s = 0; s < 2; s++)
    {
        int32_t off = out->edge_mm[s] + B5_CAR_HALF_WIDTH_MM + B5_SIDE_GAP_MM;

        if (off < B5_MIN_BYPASS_OFFSET_MM)
        {
            off = B5_MIN_BYPASS_OFFSET_MM;
        }

        out->offset_mm[s] = off;
        bool b_blocked = 0 != (blocked_mask & (1u << s));
        out->feasible[s] = (!b_blocked) && (out->room_mm[s] >= B5_NEED_ROOM_MM)
                && (off <= B5_MAX_OFFSET_MM);
        int32_t cost = off;

        if (!out->seen[s])
        {
            cost += B5_UNSEEN_EDGE_PENALTY_MM;
        }

        if (out->room_mm[s] < B5_NEED_ROOM_MM + B5_TIGHT_ROOM_EXTRA_MM)
        {
            cost += B5_TIGHT_PENALTY_MM;
        }

        out->cost_mm[s] = cost;
        out->pass_x_mm[s] =
                b5_pass_x(out->x_face_mm, depth, out->edge_mm[s],
                          out->edge_mm[1 - s],
                          off);
    }

    /* 5. choose */
    bool b_fl = out->feasible[0];
    bool b_fr = out->feasible[1];

    if ((!b_fl) && (!b_fr))
    {
        out->route_ok = false;
        out->reason = B5_WHY_NO_ROUTE;
        return;
    }

    out->route_ok = true;

    if (b_fl != b_fr)
    {
        out->side = b_fl ? 1 : -1;
        out->reason = B5_WHY_ONLY_FEASIBLE;
        return;
    }

    int32_t dc = out->cost_mm[0] - out->cost_mm[1];

    if (i_abs(dc) >= B5_TIE_COST_MM)
    {
        out->side = dc < 0 ? 1 : -1;
        out->reason = B5_WHY_CHEAPER;
        return;
    }

    int32_t dr = out->room_mm[0] - out->room_mm[1];

    if (i_abs(dr) >= B5_TIE_ROOM_MM)
    {
        out->side = dr > 0 ? 1 : -1;
        out->reason = B5_WHY_TIE_MORE_ROOM;
        return;
    }

    out->side = B5_DEFAULT_SIDE > 0 ? 1 : -1;
    out->reason = B5_WHY_DEFAULT;
}

/*** end of file ***/
