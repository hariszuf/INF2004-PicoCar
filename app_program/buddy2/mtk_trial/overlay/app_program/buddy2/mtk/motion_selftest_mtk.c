/**
 * @file motion_selftest_mtk.c
 * @brief Buddy 2 micro T-Kernel trial: scripted motion self-test.
 *
 * Counts down BUDDY2_SELFTEST_DELAY_S seconds from power-up (it does NOT wait
 * for the serial monitor), then runs once:
 *   1. forward 30 cm     2. backward 30 cm
 *   3. turn left 90 deg  4. turn right 90 deg
 *   5. drive 3 cm/s for 3 s (line-following speed), then stop
 * Each step prints a small table (every 0.5 s) and a result line; a summary
 * table follows at the end and is repeated every 15 s, so a serial monitor
 * opened late still shows the results. Encoder figures are what Buddy 2 measured: check
 * the real distance / angle with a ruler / protractor. First run with the
 * wheels off the ground. Power off to abort.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "motion_config.h"
#include "motion_control.h"
#include "motion_mtk_hw.h"
#include "motion_task_mtk.h"
#include "motion_selftest_mtk.h"

#define PRINT_MS            (500u)
#define PAUSE_MS            (1500)
#define STEP_TIMEOUT_MS     (10000u)
#define SUMMARY_REPEAT_MS   (15000)
#define STEP_COUNT          (5)

static const char *const RULE  = "==================================================================================\n";
static const char *const LINE  = "----------------------------------------------------------------------------------\n";

typedef struct
{
    const char *name;
    bool        done;
    float       distance_cm;
    float       heading_deg;
    float       left_cm_s;      /* step 5 only */
    float       right_cm_s;
    motion_fault_t fault;
} step_result_t;

static ID s_task_id = 0;
static step_result_t s_result[STEP_COUNT] = {
    { "Forward 30 cm",       false, 0.0f, 0.0f, 0.0f, 0.0f, MOTION_FAULT_NONE },
    { "Backward 30 cm",      false, 0.0f, 0.0f, 0.0f, 0.0f, MOTION_FAULT_NONE },
    { "Turn left 90 deg",    false, 0.0f, 0.0f, 0.0f, 0.0f, MOTION_FAULT_NONE },
    { "Turn right 90 deg",   false, 0.0f, 0.0f, 0.0f, 0.0f, MOTION_FAULT_NONE },
    { "Drive 3 cm/s for 3 s", false, 0.0f, 0.0f, 0.0f, 0.0f, MOTION_FAULT_NONE },
};

/* tm_printf has no %f: one decimal as text (own buffer per call site). */
static const char *num(char *buf, float v)
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
        case MOTION_STATE_EXECUTING: return "MOVING";
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
        case MOTION_FAULT_STALL: return "STALL (wheel not turning)";
        default:                 return "?";
    }
}

static void table_header(void)
{
    tm_printf((UB *)"   time  state       enc L    enc R   speed L  speed R  distance  heading\n");
    tm_printf((UB *)"   (ms)              (count)  (count) (cm/s)   (cm/s)   (cm)      (deg)\n");
}

static void table_row(uint32_t elapsed_ms)
{
    motion_status_t s;
    char l[16];
    char r[16];
    char d[16];
    char h[16];

    motion_get_status(&s);
    tm_printf((UB *)"  %5u  %-9s %8d %8d  %7s  %7s  %8s  %7s\n",
              (UINT)elapsed_ms, (UB *)state_name(s.state), (INT)s.left_count, (INT)s.right_count,
              (UB *)num(l, s.left_speed_cm_s), (UB *)num(r, s.right_speed_cm_s),
              (UB *)num(d, s.distance_cm), (UB *)num(h, s.heading_deg));
}

static void step_header(int index)
{
    tm_printf((UB *)"\n%s", (UB *)LINE);
    tm_printf((UB *)" STEP %d/%d   %s\n", (INT)(index + 1), (INT)STEP_COUNT, (UB *)s_result[index].name);
    tm_printf((UB *)"%s", (UB *)LINE);
    table_header();
}

/* Poll until the move finishes (idle), faults, or times out. Prints rows. */
static bool wait_idle(uint32_t timeout_ms, uint32_t start_ms)
{
    motion_status_t s;
    uint32_t t;

    for (t = 100u; t <= timeout_ms; t += 100u)
    {
        (void)tk_dly_tsk(100);
        motion_get_status(&s);
        if ((s.state == MOTION_STATE_ERROR) || motion_is_idle())
        {
            table_row(start_ms + t);
            return s.state != MOTION_STATE_ERROR;
        }
        if (((start_ms + t) % PRINT_MS) == 0u)
        {
            table_row(start_ms + t);
        }
    }
    return false;
}

static void result_line(const step_result_t *r, const char *detail)
{
    tm_printf((UB *)" RESULT: %s   %s\n", (UB *)(r->done ? "DONE  " : "FAILED"), (UB *)detail);
}

static bool run_move_step(int index)
{
    step_result_t *r = &s_result[index];
    motion_status_t s;
    motion_result_t cmd = MOTION_ERR_INVALID_ARG;
    char a[16];
    char b[16];
    char detail[96];

    (void)motion_reset_odometry();
    step_header(index);
    table_row(0u);
    switch (index)
    {
        case 0: cmd = motion_move_forward(30.0f); break;
        case 1: cmd = motion_move_backward(30.0f); break;
        case 2: cmd = motion_turn_left(90.0f); break;
        case 3: cmd = motion_turn_right(90.0f); break;
        default: break;
    }
    r->done = (cmd == MOTION_OK) && wait_idle(STEP_TIMEOUT_MS, 0u);
    motion_get_status(&s);
    r->distance_cm = s.distance_cm;
    r->heading_deg = s.heading_deg;
    r->fault = s.fault;

    if (r->done)
    {
        (void)tm_sprintf((UB *)detail, (UB *)"encoder says %s cm, %s deg  ->  now measure the real %s",
                         (UB *)num(a, r->distance_cm), (UB *)num(b, r->heading_deg),
                         (UB *)((index < 2) ? "distance" : "angle"));
    }
    else
    {
        (void)tm_sprintf((UB *)detail, (UB *)"fault: %s", (UB *)fault_name(s.fault));
    }
    result_line(r, detail);
    (void)tk_dly_tsk(PAUSE_MS);
    return r->done;
}

static bool run_slow_drive(void)
{
    step_result_t *r = &s_result[4];
    motion_status_t s;
    float sum_l = 0.0f;
    float sum_r = 0.0f;
    int n = 0;
    uint32_t t;
    char a[16];
    char b[16];
    char detail[96];
    bool ok = true;

    (void)motion_reset_odometry();
    step_header(4);
    table_row(0u);
    ok = (motion_drive(3.0f, 3.0f) == MOTION_OK);
    for (t = 100u; ok && (t <= 3000u); t += 100u)
    {
        (void)tk_dly_tsk(100);
        motion_get_status(&s);
        if (s.state == MOTION_STATE_ERROR)
        {
            ok = false;
        }
        if (t > 1000u)               /* skip the first second (speeding up) */
        {
            sum_l += s.left_speed_cm_s;
            sum_r += s.right_speed_cm_s;
            n++;
        }
        if ((t % PRINT_MS) == 0u)
        {
            table_row(t);
        }
    }
    motion_stop();
    ok = wait_idle(3000u, 3000u) && ok;
    motion_get_status(&s);
    r->done = ok;
    r->fault = s.fault;
    r->distance_cm = s.distance_cm;
    r->left_cm_s = (n > 0) ? (sum_l / (float)n) : 0.0f;
    r->right_cm_s = (n > 0) ? (sum_r / (float)n) : 0.0f;
    if (ok)
    {
        (void)tm_sprintf((UB *)detail, (UB *)"mean speed L %s  R %s cm/s  (target 3.0)",
                         (UB *)num(a, r->left_cm_s), (UB *)num(b, r->right_cm_s));
    }
    else
    {
        (void)tm_sprintf((UB *)detail, (UB *)"fault: %s", (UB *)fault_name(s.fault));
    }
    result_line(r, detail);
    return ok;
}

static void print_summary(int passed, int attempted)
{
    UW updates;
    UW max_gap_us;
    int i;
    char a[16];
    char b[16];
    char gap[16];

    buddy2_task_stats(&updates, &max_gap_us);
    tm_printf((UB *)"\n%s", (UB *)RULE);
    tm_printf((UB *)" SUMMARY - BUDDY 2 MOTION SELF-TEST\n");
    tm_printf((UB *)"%s", (UB *)RULE);
    tm_printf((UB *)"  #  step                    result   encoder reading            your measurement\n");
    for (i = 0; i < STEP_COUNT; i++)
    {
        const step_result_t *r = &s_result[i];

        if (i >= attempted)
        {
            tm_printf((UB *)"  %d  %-22s  SKIPPED\n", (INT)(i + 1), (UB *)r->name);
        }
        else if (!r->done)
        {
            tm_printf((UB *)"  %d  %-22s  FAILED   fault: %s\n", (INT)(i + 1), (UB *)r->name,
                      (UB *)fault_name(r->fault));
        }
        else if (i < 2)
        {
            tm_printf((UB *)"  %d  %-22s  DONE     %7s cm              ______ cm\n",
                      (INT)(i + 1), (UB *)r->name, (UB *)num(a, r->distance_cm));
        }
        else if (i < 4)
        {
            tm_printf((UB *)"  %d  %-22s  DONE     %7s deg             ______ deg\n",
                      (INT)(i + 1), (UB *)r->name, (UB *)num(a, r->heading_deg));
        }
        else
        {
            tm_printf((UB *)"  %d  %-22s  DONE     L %s / R %s cm/s\n",
                      (INT)(i + 1), (UB *)r->name, (UB *)num(a, r->left_cm_s), (UB *)num(b, r->right_cm_s));
        }
    }
    tm_printf((UB *)"  Note: for turns the encoder shows about %u%% of the angle on purpose\n",
              (UINT)(MOTION_TURN_SCALE * 100.0f + 0.5f));
    tm_printf((UB *)"        (MOTION_TURN_SCALE corrects for wheel slip). Judge turns by the real angle.\n");
    tm_printf((UB *)"%s", (UB *)LINE);
    tm_printf((UB *)"  Steps passed      : %d of %d\n", (INT)passed, (INT)STEP_COUNT);
    tm_printf((UB *)"  Motion task       : %u updates, longest gap %s ms (target %u ms)\n",
              (UINT)updates, (UB *)num(gap, (float)max_gap_us / 1000.0f), (UINT)MOTION_UPDATE_PERIOD_MS);
    tm_printf((UB *)"%s", (UB *)RULE);
    tm_printf((UB *)" Test finished. Motors stay braked. This summary repeats every 15 s.\n Press reset / power-cycle to run the test again.\n");
}

static void selftest_task(INT stacd, void *exinf)
{
    int passed = 0;
    int attempted = 0;
    int s;

    (void)stacd;
    (void)exinf;

    /* Countdown from power-up. The header is repeated every 5 s so a serial
       monitor opened part-way through still shows what is happening. */
    for (s = BUDDY2_SELFTEST_DELAY_S; s > 0; s--)
    {
        if (((s % 5) == 0) || (s == BUDDY2_SELFTEST_DELAY_S))
        {
            tm_printf((UB *)"\n%s", (UB *)RULE);
            tm_printf((UB *)" BUDDY 2 MOTION SELF-TEST  (micro T-Kernel trial, Buddy 2 only)\n");
            tm_printf((UB *)"%s", (UB *)RULE);
            tm_printf((UB *)"  Motors   : left GP8/GP9, right GP10/GP11, 15 kHz PWM\n");
            tm_printf((UB *)"  Encoders : left GP6, right GP4 (one phase each)\n");
            tm_printf((UB *)"  Steps    : forward 30 cm, backward 30 cm, left 90, right 90, drive 3 cm/s\n");
            tm_printf((UB *)"  SAFETY   : first run with the wheels OFF the ground. Power off to abort.\n");
            tm_printf((UB *)"%s", (UB *)RULE);
        }
        tm_printf((UB *)"  Starting in %2d s ...\n", (INT)s);
        (void)tk_dly_tsk(1000);
    }

    for (s = 0; s < 4; s++)
    {
        attempted++;
        if (!run_move_step(s))
        {
            break;          /* a fault latches Buddy 2: stop the sequence */
        }
        passed++;
    }
    if (passed == 4)
    {
        attempted++;
        if (run_slow_drive())
        {
            passed++;
        }
    }
    for (;;)
    {
        print_summary(passed, attempted);    /* repeated for late monitors */
        (void)tk_dly_tsk(SUMMARY_REPEAT_MS);
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
