#include "buddy5_core.h"
#include "buddy5_plan.h"
#include <math.h>
#include <string.h>

enum
{
    PH_NONE = 0, PH_LOOK, PH_CHECK1, PH_BYPASS, PH_CHECK2, PH_ADVANCE,
    PH_RECOVER, PH_RESUME,
    PH_RETURN, PH_RESCAN_STOP, PH_UNWIND
};
typedef enum
{
    CHK_WAIT, CHK_CLEAR, CHK_BLOCKED
} chk_t;

/* ------------------------------------------------------------------ sampler */
static void sort3 (int32_t * v, int n)
{
    for (int idx = 1; idx < n; idx++)
    {
        int32_t t = v[idx];
        int jdx = idx - 1;

        while ((jdx >= 0) && (v[jdx] > t))
        {
            v[jdx + 1] = v[jdx];
            jdx--;
        }

        v[jdx + 1] = t;
    }
}

b5_range_t b5_filter (const b5_range_t samples[B5_SAMPLER_N])
{
    b5_range_t out = {false, 0};
    int32_t echo[B5_SAMPLER_N];
    int e = 0;
    int none = 0;

    for (int idx = 0; idx < B5_SAMPLER_N; idx++)
    {
        if (!samples[idx].valid)
        {
            continue;
        }

        if (B5_NO_ECHO_MM == samples[idx].distance_mm)
        {
            ++none;
        }
        else
        {
            echo[e++] = (int32_t)samples[idx].distance_mm;
        }
    }

    if (e >= (int)B5_MIN_VALID_SAMPLES)
    {
        sort3(echo, e);

        if (3 == e)
        {
            if ((echo[1] - echo[0] <= (int32_t)B5_SAMPLES_MAX_SPREAD_MM)
                || (echo[2] - echo[1] <= (int32_t)B5_SAMPLES_MAX_SPREAD_MM))
            {
                out.valid = true;
                out.distance_mm = (uint32_t)echo[1];
            }
        }
        else if (echo[1] - echo[0] <= (int32_t)B5_SAMPLES_MAX_SPREAD_MM)
        {
            out.valid = true;
            out.distance_mm = (uint32_t)((echo[0] + echo[1]) / 2);
        }
        else
        {
            /* no action required */
        }
    }
    else if (none >= (int)B5_MIN_VALID_SAMPLES)
    {
        out.valid = true;
        out.distance_mm = B5_NO_ECHO_MM;
    }
    else
    {
        /* no action required */
    }

    return out;
}

void b5_sampler_reset (b5_sampler_t * s)
{
    s->n = 0;
    s->fresh = 0;
}

bool b5_sampler_push (b5_sampler_t * s, const b5_range_t * sample,
                      b5_range_t * out)
{
    if (s->n < B5_SAMPLER_N)
    {
        s->s[s->n++] = *sample;
    }
    else
    {
        s->s[0] = s->s[1];
        s->s[1] = s->s[2];
        s->s[2] = *sample;
    }

    if (s->fresh < 255)
    {
        ++s->fresh;
    }

    if (s->fresh < B5_SAMPLER_N)
    {
        return false;
    }

    *out = b5_filter(s->s);
    return true;
}

/* ------------------------------------------------------------------ helpers */
const char * b5_state_name (b5_state_t s)
{
    static const char * const names[] = {"MONITORING", "WAIT_STOP", "SCANNING",
        "PROCESSING", "AVOIDING", "LINE_RECOVERY", "ERROR"};
    return (unsigned)s < sizeof(names) / sizeof(names[0]) ? names[s]
            : "UNKNOWN";
}

static void set_state (b5_core_t * c, b5_state_t st, uint32_t now)
{
    c->state = st;
    c->state_since_ms = now;
}

static void issue (b5_core_t * c, b5_request_kind_t kind, uint32_t now,
                   int32_t forward_mm, int32_t lateral_mm)
{
    c->request.kind = kind;
    c->request.id = ++c->next_request_id;
    c->request.forward_mm = forward_mm;
    c->request.lateral_mm = lateral_mm;
    c->request_pending = true;
    c->request_active = false;
    c->request_since_ms = now;
}

static bool request_done (const b5_core_t * c, const b5_feedback_t * f)
{
    return f && (f->valid) && (0 != c->request.id) && (!c->request_pending)
            && (c->request_active) && (f->completed_id == c->request.id);
}

void b5_core_fail (b5_core_t * c, b5_fault_t fault, uint32_t now_ms)
{
    c->fault = fault;
    c->request_pending = false;
    c->request_active = false;
    set_state(c, B5_ERROR, now_ms);
}

void b5_core_init (b5_core_t * c, uint32_t now_ms)
{
    memset(c, 0, sizeof(*c));
    c->state = B5_MONITORING;
    c->state_since_ms = now_ms;
    c->now_ms = now_ms;
    c->front_clearance_mm = -1;
}

void b5_core_reset (b5_core_t * c, uint32_t now_ms)
{
    uint32_t id = c->next_request_id;
    b5_core_init(c, now_ms);
    c->next_request_id = id;
}

bool b5_core_peek_request (const b5_core_t * c, b5_request_t * r)
{
    if (!c->request_pending)
    {
        return false;
    }

    *r = c->request;
    return true;
}

void b5_core_request_sent (b5_core_t * c, uint32_t now_ms)
{
    c->request_pending = false;
    c->request_active = true;
    c->request_since_ms = now_ms;
}

int b5_core_angle (const b5_core_t * c)
{
    if (B5_SCANNING == c->state)
    {
        return b5_slot_angle(c->slot < B5_SCAN_POINTS ? c->slot : B5_SCAN_POINTS
                             - 1);
    }

    if ((B5_LINE_RECOVERY == c->state)
        && ((PH_RECOVER == c->phase) || (PH_RETURN == c->phase))
                && (!c->fb_front_ok))
    {
        /* Looks the navigation asks for after it turns to face the line: ahead,
         * half
         * back, fully back toward the box.
         * The box is on the side the car came from, i.e. servo + when it passed
         * on
         * the right, - when on the left. */
        int far = c->side > 0 ? B5_SCAN_MIN_DEG : B5_SCAN_MAX_DEG;
        int half = c->side > 0 ? -15 : 15;
        uint32_t k =
                ((c->now_ms - c->recover_since_ms) / B5_LOOK_PERIOD_MS) % 3u;
        return 0u == k ? 0 : (1u == k ? half : far);
    }

    return 0;
}

/* Forward-clearance needed so that driving `travel_mm` with the axle still
 * leaves B5_ROUTE_MARGIN_MM at the bumper. */
static int32_t need_clearance (int32_t travel_mm)
{
    int32_t extra =
            B5_AXLE_TO_FRONT_MM - (B5_AXLE_TO_SENSOR_MM + B5_FRONT_OVERHANG_MM);
    return travel_mm + (extra > 0 ? extra : 0) + B5_ROUTE_MARGIN_MM;
}

static chk_t route_check (b5_core_t * c, uint32_t now, bool fresh, int32_t need,
                          int32_t ignore_lo, int32_t ignore_hi)
{
    if ((uint32_t)(now - c->check_since_ms) < B5_ROUTE_CHECK_MS)
    {
        return CHK_WAIT;
    }

    if (fresh)
    {
        int32_t cl = c->front_clearance_mm;
        /* the box seen by the wide cone, not an obstacle */
        bool box_echo = (ignore_hi > 0) && (cl >= ignore_lo)
                && (cl <= ignore_hi);

        if ((!box_echo) && (cl < c->chk_min))
        {
            c->chk_min = cl;
        }

        if (c->chk_n < 255)
        {
            ++c->chk_n;
        }
    }

    if (c->chk_n >= B5_ROUTE_CHECK_READINGS)
    {
        return c->chk_min >= need ? CHK_CLEAR : CHK_BLOCKED;
    }

    if ((uint32_t)(now - c->check_since_ms) > B5_ROUTE_CHECK_TIMEOUT_MS)
    {
        return CHK_BLOCKED;
    }

    return CHK_WAIT;
}

static void begin_check (b5_core_t * c, uint32_t now)
{
    c->check_since_ms = now;
    c->chk_n = 0;
    c->chk_min = INT32_MAX;
}

static void use_side (b5_core_t * c, int side)
{
    int idx = side > 0 ? 0 : 1;
    c->side = (int8_t)side;
    c->offset_mm = c->off_arr[idx];
    c->pass_x_mm = c->pass_arr[idx];
    c->edge_near_mm = c->edge_arr[idx];
}

static bool other_side_ok (const b5_core_t * c)
{
    int o = c->side > 0 ? 1 : 0;
    return (c->feas_arr[o]) && (!(c->blocked_mask & (1u << o)));
}

static void start_recover (b5_core_t * c, uint32_t now, uint8_t phase)
{
    set_state(c, B5_LINE_RECOVERY, now);
    c->phase = phase;
    c->recover_since_ms = now;
    c->fb_front_ok = false;
    issue(c, B5_REQ_RECOVER_LINE, now, 0, 0);
}

static void do_processing (b5_core_t * c, uint32_t now)
{
    b5_plan_t p;
    b5_plan_make(&c->scan, c->stop_front_mm, c->blocked_mask, &p);
    b5_profile_t * pr = &c->profile;
#if B5_SCAN_ONLY
    if (!p.face_valid)
    {
        /* Keep the last good result: a scan that saw no box must not erase it.
         */
        ++c->noface_count;
        c->cooldown_until_ms = now + 1000u;
        c->hit_count = 0;
        set_state(c, B5_MONITORING, now);
        return;
    }

    ++c->scan_count;
#endif
    *pr = (b5_profile_t){0};
    pr->front_mm = c->stop_front_mm;

    if (!p.face_valid)
    {
        /* Nothing close any more: a false alarm. Carry on along the line. */
        if (++c->false_alarms > B5_MAX_FALSE_ALARMS)
        {
            b5_core_fail(c, B5_FAULT_SENSOR, now);
            return;
        }

        set_state(c, B5_LINE_RECOVERY, now);
        c->phase = PH_RESUME;
        c->side = 0;
        issue(c, B5_REQ_RESUME_LINE, now, 0, 0);
        return;
    }

    pr->left_seen = p.seen[0];
    pr->right_seen = p.seen[1];
    pr->left_edge_mm = p.edge_mm[0];
    pr->right_edge_mm = p.edge_mm[1];
    pr->left_room_mm = p.room_mm[0];
    pr->right_room_mm = p.room_mm[1];
    pr->left_cost_mm = p.cost_mm[0];
    pr->right_cost_mm = p.cost_mm[1];
    pr->width_valid = true;
    pr->width_mm = p.edge_mm[0] + p.edge_mm[1];
    pr->depth_valid = true;
    pr->depth_low_mm = B5_BOX_LENGTH_MM;
    pr->depth_high_mm = B5_BOX_LENGTH_MM + B5_BOX_DEPTH_TOL_MM;
    pr->side_reason = p.reason;

    for (int idx = 0; idx < 2; idx++)
    {
        c->off_arr[idx] = p.offset_mm[idx];
        c->pass_arr[idx] = p.pass_x_mm[idx];
        c->edge_arr[idx] = p.edge_mm[idx];
        c->feas_arr[idx] = p.feasible[idx];
    }

    c->x_face_mm = p.x_face_mm;
#if B5_SCAN_ONLY
    /* Measurement only: report what the scan saw (no minimum box size), never
     * ask
     * the car to move. */
    pr->left_edge_mm = p.meas_edge_mm[0];
    pr->right_edge_mm = p.meas_edge_mm[1];
    pr->width_mm = p.meas_edge_mm[0] + p.meas_edge_mm[1];
    pr->width_valid = (p.seen[0]) && (p.seen[1]);
    /* the sonar only sees the front face: depth cannot be measured */
    pr->depth_valid = false;
    pr->depth_low_mm = pr->depth_high_mm = 0;
    pr->valid = true;
    pr->side = 0;
    pr->offset_mm = 0;
    pr->pass_x_mm = 0;
    c->cooldown_until_ms = now + B5_SCANONLY_HOLD_MS;
    c->hit_count = 0;
    set_state(c, B5_MONITORING, now);
    return;
#endif
    if (!p.route_ok)
    {
        b5_core_fail(c, B5_FAULT_NO_ROUTE, now);
        return;
    }   /* the car is on the line, stopped: safe */

    use_side(c, p.side);
    pr->valid = true;
    pr->side = (int8_t)p.side;
    pr->offset_mm = c->offset_mm;
    pr->pass_x_mm = c->pass_x_mm;
    set_state(c, B5_AVOIDING, now);
    c->avoid_since_ms = now;
    c->phase = PH_LOOK;
    /* pivot toward the chosen side: check the path before any translation */
    issue(c, B5_REQ_LOOK, now, 0, c->side * 90);
}

static void switch_side (b5_core_t * c, uint32_t now)
{
    use_side(c, -c->side);
    c->profile.side = c->side;
    c->profile.offset_mm = c->offset_mm;
    c->profile.pass_x_mm = c->pass_x_mm;
    c->profile.side_reason = B5_WHY_ONLY_FEASIBLE;
    c->phase = PH_LOOK;
    issue(c, B5_REQ_LOOK, now, 0, c->side * 90);
}

/* ------------------------------------------------------------------ update */
void b5_core_update (b5_core_t * c, uint32_t now, const b5_feedback_t * f,
                     const b5_range_t * range)
{
    c->now_ms = now;

    if (B5_ERROR == c->state)
    {
        return;
    }

    int angle = b5_core_angle(c);
    bool fresh_front = false;

    if (range && (0 == angle))
    {
        c->front_clearance_mm = b5_front_clearance(range);
        c->front_ms = now;
        ++c->front_seq;
        fresh_front = range->valid;
    }

    if (f && (f->valid))
    {
        c->fb_front_ok = f->front_ok;

        if ((f->request_failed)
            && ((c->request_active) || (c->request_pending)))
        {
            b5_core_fail(c, B5_FAULT_CONTROLLER, now);
            return;
        }
    }

    bool done = request_done(c, f);

    switch (c->state)
    {
        case B5_MONITORING:
        if (now < c->cooldown_until_ms)
        {
            c->hit_count = 0;
            break;
        }
#if !B5_SCAN_ONLY
        if (!(f && (f->valid)))
        {
            break;
        }
#endif
        if (fresh_front)
        {
            if (c->front_clearance_mm <= (int32_t)B5_STOP_FRONT_MM)
            {
                if (++c->hit_count >= B5_TRIGGER_COUNT)
                {
                    c->hit_count = 0;
                    c->stop_front_mm = c->front_clearance_mm;
                    c->blocked_mask = 0;
#if B5_SCAN_ONLY
                    memset(&c->scan, 0, sizeof(c->scan));
                    c->slot = 0;
                    set_state(c, B5_SCANNING, now);
#else
                    issue(c, B5_REQ_STOP, now, 0, 0);
                    set_state(c, B5_WAIT_STOP, now);
#endif
                }
            }
            else
            {
                c->hit_count = 0;
            }
        }
        break;
        case B5_WAIT_STOP:
        if (done && (f->stopped))
        {
            memset(&c->scan, 0, sizeof(c->scan));
            c->slot = 0;
            set_state(c, B5_SCANNING, now);
        }
        else if ((uint32_t)(now - c->state_since_ms) > B5_REQUEST_TIMEOUT_MS)
        {
            b5_core_fail(c, B5_FAULT_TIMEOUT, now);
        }
        else
        {
            /* no action required */
        }
        break;
        case B5_SCANNING:
        if (range && (c->slot < B5_SCAN_POINTS))
        {
            int idx = c->slot;

            if (b5_range_no_echo(range))
            {
                b5_scan_set(&c->scan, idx, B5_SCAN_NO_ECHO, 0u);
            }
            else if (range->valid)
            {
                b5_scan_set(&c->scan, idx, B5_SCAN_ECHO, range->distance_mm);
            }
            else
            {
                b5_scan_set(&c->scan, idx, 0u, 0u);
            }

            if (++c->slot >= B5_SCAN_POINTS)
            {
                set_state(c, B5_PROCESSING, now);
            }
        }
        else if ((uint32_t)(now - c->state_since_ms) > B5_SCAN_TIMEOUT_MS)
        {
            b5_core_fail(c, B5_FAULT_TIMEOUT, now);
        }
        else
        {
            /* no action required */
        }

        if (B5_PROCESSING == c->state)
        {
            do_processing(c, now);
        }
        break;
        case B5_PROCESSING:
        do_processing(c, now);
        break;
        case B5_AVOIDING:
        if ((uint32_t)(now - c->avoid_since_ms) > B5_AVOID_TIMEOUT_MS)
        {
            b5_core_fail(c, B5_FAULT_TIMEOUT, now);
            break;
        }

        switch (c->phase)
        {
            case PH_LOOK:
            if (done)
            {
                c->phase = PH_CHECK1;
                begin_check(c, now);
            }
            break;
            case PH_CHECK1:
            {
                chk_t r = route_check(c, now, fresh_front,
                                      need_clearance(c->offset_mm), 0,
                                      0);

                if (CHK_CLEAR == r)
                {
                    c->phase = PH_BYPASS;
                    issue(c, B5_REQ_BYPASS, now, 0, c->side * c->offset_mm);
                }
                else if (CHK_BLOCKED == r)
                {
                    c->blocked_mask |= (uint8_t)(1u << (c->side > 0 ? 0 : 1));

                    if (other_side_ok(c))
                    {
                        switch_side(c, now);
                    }
                    else
                    {
                        c->phase = PH_UNWIND;
                        issue(c, B5_REQ_LOOK, now, 0, 0);
                    }   /* turn back square to the line, then stop */
                }
                else
                {
                    /* no action required */
                }
                break;
            }
            case PH_UNWIND:
            if (done)
            {
                b5_core_fail(c, B5_FAULT_NO_ROUTE, now);
            }
            break;
            case PH_BYPASS:
            if (done)
            {
                c->phase = PH_CHECK2;
                begin_check(c, now);
            }
            break;
            case PH_CHECK2:
            {
                int32_t fx = f ? f->forward_mm : 0;
                int32_t xs = fx + B5_AXLE_TO_SENSOR_MM;
                int32_t lo = 0;
                int32_t hi = 0;

                if (c->x_face_mm > xs)
                {
                    /* Echoes the wide cone could return from the box itself are
                     * not obstacles
                     * in the pass. */
                    float gap = (float)(c->offset_mm - c->edge_near_mm);
                    float dlo = (float)(c->x_face_mm - xs),
                    dhi = (float)(c->x_face_mm + B5_BOX_LENGTH_MM
                                  + B5_BOX_DEPTH_TOL_MM
                                  - xs);
                    lo = (int32_t)sqrtf(dlo * dlo + gap * gap) - 40
                            - B5_FRONT_OVERHANG_MM;
                    hi = (int32_t)sqrtf(dhi * dhi + gap * gap) + 40
                            - B5_FRONT_OVERHANG_MM;
                }

                chk_t r = route_check(c, now, fresh_front,
                                      need_clearance(c->pass_x_mm - fx),
                                      lo, hi);

                if (CHK_CLEAR == r)
                {
                    c->phase = PH_ADVANCE;
                    c->adv_close = 0u;
                    issue(c, B5_REQ_ADVANCE, now, c->pass_x_mm,
                          c->side * c->offset_mm);
                }
                else if (CHK_BLOCKED == r)
                {
                    c->blocked_mask |= (uint8_t)(1u << (c->side > 0 ? 0 : 1));
                    /* go back to the line, scan again without this side */
                    start_recover(c, now, PH_RETURN);
                }
                else
                {
                    /* no action required */
                }
                break;
            }
            case PH_ADVANCE:
            /* The controller ignores its own forward guard during the pass, so
             * the stop
             * is decided here. The car is beside
             * the box and cannot recover from a standstill here: a close echo
             * ahead
             * halts the vehicle (error state). */
            if (fresh_front)
            {
                if (c->front_clearance_mm <= (int32_t)B5_ADVANCE_STOP_MM)
                {
                    if (++c->adv_close < 2u)
                    {
                    }
                    else
                    {
                        b5_core_fail(c, B5_FAULT_NO_ROUTE, now);
                        break;
                    }
                }
                else
                {
                    c->adv_close = 0u;
                }
            }

            if (done)
            {
                start_recover(c, now, PH_RECOVER);
            }
            break;
            default:
            break;
        }
        break;
        case B5_LINE_RECOVERY:
        if (((uint32_t)(now - c->avoid_since_ms) > B5_AVOID_TIMEOUT_MS)
            && (PH_RESUME != c->phase))
        {
            b5_core_fail(c, B5_FAULT_TIMEOUT, now);
            break;
        }

        switch (c->phase)
        {
            case PH_RECOVER:
            if (done)
            {
                c->phase = PH_RESUME;
                issue(c, B5_REQ_RESUME_LINE, now, 0, 0);
            }
            break;
            case PH_RETURN:
            if (done)
            {
                c->phase = PH_RESCAN_STOP;
                issue(c, B5_REQ_STOP, now, 0, 0);
            }   /* STOP re-zeroes the origin on the line */
            break;
            case PH_RESCAN_STOP:
            if (done && (f->stopped))
            {
                memset(&c->scan, 0, sizeof(c->scan));
                c->slot = 0;
                set_state(c, B5_SCANNING, now);
                c->phase = PH_NONE;
            }
            break;
            case PH_RESUME:
            if (done)
            {
                c->false_alarms = (c->profile.valid) ? 0u : c->false_alarms;
                c->blocked_mask = 0;
                c->phase = PH_NONE;
                c->hit_count = 0;
                c->cooldown_until_ms = now + B5_RESUME_COOLDOWN_MS;
                set_state(c, B5_MONITORING, now);
            }
            break;
            default:
            break;
        }
        break;
        default:
        break;
    }

    if ((c->request_active) && (B5_ERROR != c->state)
        && ((uint32_t)(now - c->request_since_ms) > B5_REQUEST_TIMEOUT_MS))
    {
        b5_core_fail(c, B5_FAULT_TIMEOUT, now);
    }
}

/*** end of file ***/
