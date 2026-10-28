/**
 * @file motion_selftest_mtk.c
 * @brief Buddy 2 micro T-Kernel trial: scripted motion self-test.
 *
 * After BUDDY2_SELFTEST_DELAY_S seconds it runs, once:
 *   1. forward 30 cm     2. backward 30 cm
 *   3. turn left 90 deg  4. turn right 90 deg
 *   5. drive 3 cm/s for 3 s (line-following speed), then stop
 * and prints one status line every 500 ms plus a result line per step.
 * Encoder figures are what Buddy 2 measured; check the real distance / angle
 * with a ruler / protractor. Put the car on a stand (wheels free) for the first
 * run. Power off to abort.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "motion_config.h"
#include "motion_control.h"
#include "motion_mtk_hw.h"
#include "motion_task_mtk.h"
#include "motion_selftest_mtk.h"

#define PRINT_MS        (500)
#define PAUSE_MS        (1000)

static ID s_task_id = 0;

/* tm_printf has no %f: signed value with one decimal as text. */
static const char *tenths(char *buf, float v)
{
    int32_t t = (int32_t)((v >= 0.0f) ? (v * 10.0f + 0.5f) : (v * 10.0f - 0.5f));
    uint32_t a = (uint32_t)((t < 0) ? -t : t);

    (void)tm_sprintf((UB *)buf, (UB *)"%s%u.%u", (t < 0) ? "-" : "", (UINT)(a / 10u), (UINT)(a % 10u));
    return buf;
}

static const char *state_name(motion_state_t s)
{
    switch (s)
    {
        case MOTION_STATE_IDLE:      return "IDLE";
        case MOTION_STATE_EXECUTING: return "EXECUTING";
        case MOTION_STATE_STOPPING:  return "STOPPING";
        case MOTION_STATE_ERROR:     return "ERROR";
        default:                     return "?";
    }
}

static const char *fault_name(motion_fault_t f)
{
    switch (f)
    {
        case MOTION_FAULT_NONE:  return "none";
        case MOTION_FAULT_ESTOP: return "E-STOP";
        case MOTION_FAULT_STALL: return "STALL";
        default:                 return "?";
    }
}

static void print_status(uint32_t elapsed_ms)
{
    motion_status_t s;
    char l[16];
    char r[16];
    char d[16];
    char h[16];

    motion_get_status(&s);
    tm_printf((UB *)"B2 | t=%5u ms | %-9s | count L %6d R %6d | speed L %5s R %5s cm/s | dist %6s cm | heading %6s deg | fault %s\n",
              (UINT)elapsed_ms, (UB *)state_name(s.state), (INT)s.left_count, (INT)s.right_count,
              (UB *)tenths(l, s.left_speed_cm_s), (UB *)tenths(r, s.right_speed_cm_s),
              (UB *)tenths(d, s.distance_cm), (UB *)tenths(h, s.heading_deg), (UB *)fault_name(s.fault));
}

/* Wait until the command finishes (idle), faults, or the timeout passes. */
static bool wait_idle(uint32_t timeout_ms)
{
    motion_status_t s;
    uint32_t t;

    for (t = 0u; t < timeout_ms; t += 100u)
    {
        (void)tk_dly_tsk(100);
        motion_get_status(&s);
        if (s.state == MOTION_STATE_ERROR)
        {
            print_status(t);
            return false;
        }
        if (motion_is_idle())
        {
            print_status(t);
            return true;
        }
        if ((t % PRINT_MS) == 0u)
        {
            print_status(t);
        }
    }
    return false;
}

typedef enum { STEP_FWD = 0, STEP_BACK, STEP_LEFT, STEP_RIGHT, STEP_COUNT } step_t;

static bool run_step(step_t step, int *p_pass)
{
    static const char *const names[STEP_COUNT] = {
        "forward 30 cm", "backward 30 cm", "turn left 90 deg", "turn right 90 deg" };
    motion_status_t before;
    motion_status_t after;
    motion_result_t r = MOTION_ERR_INVALID_ARG;
    char a[16];
    char b[16];
    bool ok;

    (void)motion_reset_odometry();
    motion_get_status(&before);
    tm_printf((UB *)"\n[B2 TEST] Step %d: %s\n", (INT)step + 1, (UB *)names[step]);
    switch (step)
    {
        case STEP_FWD:   r = motion_move_forward(30.0f); break;
        case STEP_BACK:  r = motion_move_backward(30.0f); break;
        case STEP_LEFT:  r = motion_turn_left(90.0f); break;
        case STEP_RIGHT: r = motion_turn_right(90.0f); break;
        default: break;
    }
    ok = (r == MOTION_OK) && wait_idle(10000u);
    motion_get_status(&after);
    tm_printf((UB *)"[B2 TEST] Step %d %s: encoder distance %s cm, heading %s deg (%s)\n",
              (INT)step + 1, (UB *)(ok ? "DONE" : "FAILED"),
              (UB *)tenths(a, after.distance_cm - before.distance_cm),
              (UB *)tenths(b, after.heading_deg - before.heading_deg),
              (UB *)(ok ? "measure the real value" : fault_name(after.fault)));
    if (ok)
    {
        (*p_pass)++;
    }
    (void)tk_dly_tsk(PAUSE_MS);
    return ok;
}

static bool run_slow_drive(int *p_pass)
{
    motion_status_t s;
    float sum_l = 0.0f;
    float sum_r = 0.0f;
    int n = 0;
    uint32_t t;
    char l[16];
    char r[16];
    bool ok = true;

    tm_printf((UB *)"\n[B2 TEST] Step 5: drive 3 cm/s for 3 s (line-following speed)\n");
    if (motion_drive(3.0f, 3.0f) != MOTION_OK)
    {
        ok = false;
    }
    for (t = 0u; ok && (t < 3000u); t += 100u)
    {
        (void)tk_dly_tsk(100);
        motion_get_status(&s);
        if (s.state == MOTION_STATE_ERROR)
        {
            ok = false;
        }
        if (t >= 1000u)   /* skip the speed-up */
        {
            sum_l += s.left_speed_cm_s;
            sum_r += s.right_speed_cm_s;
            n++;
        }
        if ((t % PRINT_MS) == 0u)
        {
            print_status(t);
        }
    }
    motion_stop();
    ok = wait_idle(3000u) && ok;
    tm_printf((UB *)"[B2 TEST] Step 5 %s: mean speed L %s R %s cm/s (target 3.0)\n",
              (UB *)(ok ? "DONE" : "FAILED"),
              (UB *)tenths(l, (n > 0) ? (sum_l / (float)n) : 0.0f),
              (UB *)tenths(r, (n > 0) ? (sum_r / (float)n) : 0.0f));
    if (ok)
    {
        (*p_pass)++;
    }
    return ok;
}

static void selftest_task(INT stacd, void *exinf)
{
    int pass = 0;
    int s;
    UW updates;
    UW max_gap;

    (void)stacd;
    (void)exinf;
    for (s = BUDDY2_SELFTEST_DELAY_S; s > 0; s--)
    {
        tm_printf((UB *)"[B2 TEST] Motion self-test starts in %2d s - wheels off the ground (or 1 m clear floor)\n", (INT)s);
        (void)tk_dly_tsk(1000);
    }
    print_status(0u);

    for (s = 0; s < (int)STEP_COUNT; s++)
    {
        if (!run_step((step_t)s, &pass))
        {
            break;       /* a fault latches Buddy 2: stop the sequence */
        }
    }
    if (pass == (int)STEP_COUNT)
    {
        (void)run_slow_drive(&pass);
    }

    buddy2_task_stats(&updates, &max_gap);
    tm_printf((UB *)"\n[B2 TEST] Finished: %d of 5 steps done. Motion task ran %u updates, longest gap %u us (target %u ms)\n",
              (INT)pass, (UINT)updates, (UINT)max_gap, (UINT)MOTION_UPDATE_PERIOD_MS);
    tm_printf((UB *)"[B2 TEST] Car stays braked. Press reset / power-cycle to run again.\n");

    for (;;)
    {
        (void)tk_dly_tsk(5000);
        print_status(0u);
    }
}

static T_CTSK s_task_config = {
    .itskpri = BUDDY2_SELFTEST_PRIORITY,
    .stksz = 4096,
    .task = selftest_task,
    .tskatr = TA_HLNG | TA_RNG3
};

ER buddy2_selftest_start(void)
{
    if (s_task_id > 0)
    {
        return E_OBJ;
    }
    s_task_id = tk_cre_tsk(&s_task_config);
    if (s_task_id < E_OK)
    {
        return s_task_id;
    }
    return tk_sta_tsk(s_task_id, 0);
}
