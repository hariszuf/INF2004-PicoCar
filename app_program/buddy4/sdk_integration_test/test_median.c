/* Reuse calibrated fixtures and run their complete regression suite as well. */
#define main legacy_regression_main
#include "../../barrc_types.h"
#include "../tests/test_imu.c"
#undef main
/* Include implementation to exercise exact filtered-roll window boundaries. */
#include "../buddy4_imu.c"
static void feed (side_event_t * p_e, uint64_t t, float32_t roll)
{
    update_median(p_e, t, fabsf(roll), 11.3f);
}

static void test_windows (void)
{
    side_event_t e = {0};

    for (int32_t idx = 0; idx < 10; idx++)
    {
        feed(&e, 20000 * idx, 6);
    }

    assert(!e.median_candidate_valid);
    feed(&e, 200000, 60); /* One isolated spike cannot dominate a median. */
    assert((e.median_candidate_valid) && (6 == e.median_candidate_roll_deg));
    assert(fabsf(e.median_candidate_height_cm
                 - 11.3f*sinf(6 * DEG_TO_RAD)) < 1e-6f);

    for (int32_t idx = 11; idx < 22; idx++)
    {
        feed(&e, 20000 * idx, 8);
    }

    assert(8 == e.median_candidate_roll_deg);

    for (int32_t idx = 22; idx < 40; idx++)
    {
        feed(&e, 20000 * idx, 5);
    }

    /* Retain highest accepted median. */
    assert(8 == e.median_candidate_roll_deg);
    e = (side_event_t){0};

    for (int32_t idx = 0; idx < 11; idx++)
    {
        feed(&e, 20000 * idx, 4 + idx);
    }

    assert((e.median_candidate_valid) && (9 == e.median_candidate_roll_deg));

    for (int32_t span = 189999; span <= 210001; span++)
    {
        if ((189999 != span) && (190000 != span) && (210000 != span)
            && (210001 != span))
        {
            continue;
        }

        e = (side_event_t){0};

        for (int32_t idx = 0; idx < 11; idx++)
        {
            feed(&e, (uint64_t)span * idx / 10, 6);
        }

        assert(e.median_candidate_valid == ((span >= 190000)
                                            && (span <= 210000)));
    }

    e = (side_event_t){0};
    feed(&e, 0, 6);
    feed(&e, 40001, 6);
    assert(1 == e.median_count);
    feed(&e, 40001, 6);
    assert(1 == e.median_count);
    feed(&e, 39999, 6);
    assert(1 == e.median_count);
    e = (side_event_t){0};

    for (int32_t idx = 0; idx < 11; idx++)
    {
        feed(&e, 40000 * idx, 6);
    }

    /* Adjacent gaps legal, total span too long. */
    assert(!e.median_candidate_valid);
    e = (side_event_t){0};

    for (int32_t idx = 0; idx < 11; idx++)
    {
        feed(&e, (idx < 5 ? 20000 * idx : 20000 * idx + 20000), 6);
    }

    /* Missing one scheduled sample: span 220ms. */
    assert(!e.median_candidate_valid);
}

static void test_comparison (void)
{
    side_event_t stable = {0};
    side_event_t changing = {0};

    for (int32_t idx = 0; idx < 11; idx++)
    {
        uint64_t t = 20000 * idx;
        update_side_event(&stable, 1, t, 6, 11.3f*sinf(6 * DEG_TO_RAD));
        update_median(&stable, t, 6, 11.3f);
        float32_t r = 4 + idx;
        update_side_event(&changing, 1, t, r, 11.3f*sinf(r * DEG_TO_RAD));
        update_median(&changing, t, r, 11.3f);
    }

    assert((stable.peak_confirmed) && (stable.median_candidate_valid));
    assert((6 == stable.peak_roll_deg)
           && (6 == stable.median_candidate_roll_deg));
    assert((!changing.peak_confirmed) && (changing.median_candidate_valid));
    assert(9 == changing.median_candidate_roll_deg);
    puts("Synthetic comparison: sustained 6deg stability=6 median=6; ramp "
         "4..14deg stability=unconfirmed median=9");
}

static void test_lifecycle (void)
{
    buddy4_imu_t s;
    fake_t f;
    setup(&s, &f);
    uint64_t t = 0;
    samples(&s, &f, &t, 0, 8, 11);
    assert(s.output.event.median_candidate_valid);
    float32_t saved = s.output.event.median_candidate_roll_deg;
    /* Stale, repeated and reversed polls restart window, preserve event
     * maximum.
     */
    f.regs[0x27] = 0;
    assert(BUDDY4_STALE == buddy4_poll(&s, t + 20000));
    assert(0 == s.output.event.median_count);
    f.regs[0x27] = 8;
    assert(BUDDY4_STALE == buddy4_poll(&s, t));
    assert(BUDDY4_STALE == buddy4_poll(&s, t - 1));
    assert(s.output.event.median_candidate_roll_deg == saved);
    samples(&s, &f, &t, 0, 8, 10);
    assert(10 == s.output.event.median_count);
    /* A >40ms gap primes a new window, without discarding an accepted maximum.
     */
    t += 60000;
    angle(&f, 0, 8);
    assert(BUDDY4_FRESH == buddy4_poll(&s, t));
    assert(1 == s.output.event.median_count);
    /* Force a direct side reversal using existing filter gap reset semantics.
     */
    t += 60000;
    angle(&f, 0, -9);
    assert(BUDDY4_FRESH == buddy4_poll(&s, t));
    assert((s.output.event_ended)
           && (s.output.ended_event.median_candidate_valid));
    assert(s.output.ended_event.median_candidate_roll_deg == saved);
    assert((-1 == s.output.event.side) && (1 == s.output.event.median_count));
    assert(!s.output.event.median_candidate_valid);
    t += 60000;
    angle(&f, 0, 0);
    assert(BUDDY4_FRESH == buddy4_poll(&s, t));
    assert((s.output.event_ended)
           && (!s.output.ended_event.median_candidate_valid));
    assert((0 == s.output.event.side) && (0 == s.output.event.median_count));
    samples(&s, &f, &t, 0, 8, 40);
    assert(s.output.event.median_candidate_valid);
    t += 60000;
    angle(&f, 0, 0);
    assert(BUDDY4_FRESH == buddy4_poll(&s, t));
    assert(s.output.ended_event.median_candidate_valid);
    saved = s.output.ended_event.median_candidate_roll_deg;
    samples(&s, &f, &t, 0, 8, 40);
    f.fail_at = f.calls + 1;
    t += 20000;
    assert(BUDDY4_IO_ERROR == buddy4_poll(&s, t));
    assert((!s.output.event.median_candidate_valid)
           && (0 == s.output.event.median_count));
    assert(s.output.ended_event.median_candidate_roll_deg == saved);
    f.fail_at = 0;
    assert(buddy4_configure(&s));
    assert((!s.output.event.median_candidate_valid)
           && (0 == s.output.event.median_count));
    assert(s.output.ended_event.median_candidate_valid);
    assert(s.output.ended_event.median_candidate_roll_deg == saved);
    samples(&s, &f, &t, 0, 8, 10);
    assert(!s.output.event.median_candidate_valid);
    samples(&s, &f, &t, 0, 8, 1);
    assert(s.output.event.median_candidate_valid);
    assert(buddy4_configure(&s));
    assert(!s.output.event.median_candidate_valid);
    buddy4_init(&s, (buddy4_bus_t){&f, wr, rd}, buddy4_default_config());
    assert(!s.output.ended_event.median_candidate_valid);
}

static void test_interruption_and_event_isolation (void)
{
    buddy4_imu_t s;
    fake_t f;
    setup(&s, &f);
    uint64_t t = 0;
    /* Missing data interrupts both estimators. Ending afterward may describe
     * the
       side-tilt lifecycle, but neither estimator may claim a successful result.
       */
    samples(&s, &f, &t, 0, 8, 5);
    assert((!s.output.event.peak_confirmed)
           && (!s.output.event.median_candidate_valid));
    f.regs[0x27] = 0;
    assert(BUDDY4_STALE == buddy4_poll(&s, t + 20000));
    assert((!s.output.valid) && (!s.output.event_ended));
    assert((!s.output.event.window_active)
           && (0 == s.output.event.median_count));
    f.regs[0x27] = 8;
    t += 60000;
    angle(&f, 0, 0);
    assert((BUDDY4_FRESH == buddy4_poll(&s, t)) && (s.output.event_ended));
    assert(!s.output.ended_event.peak_confirmed);
    assert(!s.output.ended_event.median_candidate_valid);

    /* Duplicate and reversed times restart evidence; samples observed before
     * the
       time fault cannot complete a confirmation window afterward. */
    t += 60000;
    angle(&f, 0, 8);
    assert(BUDDY4_FRESH == buddy4_poll(&s, t));
    samples(&s, &f, &t, 0, 8, 4);
    assert(BUDDY4_STALE == buddy4_poll(&s, t));
    assert(BUDDY4_STALE == buddy4_poll(&s, t - 1));
    samples(&s, &f, &t, 0, 8, 6);
    assert((!s.output.event.peak_confirmed)
           && (!s.output.event.median_candidate_valid));
    samples(&s, &f, &t, 0, 8, 5);
    assert((s.output.event.peak_confirmed)
           && (s.output.event.median_candidate_valid));

    /* A later lower event must publish its own result, never the older maximum.
     */
    t += 60000;
    angle(&f, 0, 0);
    assert(BUDDY4_FRESH == buddy4_poll(&s, t));
    assert((s.output.event_ended)
           && (s.output.ended_event.peak_roll_deg > 7.9f));
    t += 60000;
    angle(&f, 0, 6);
    assert(BUDDY4_FRESH == buddy4_poll(&s, t));
    samples(&s, &f, &t, 0, 6, 10);
    assert((s.output.event.peak_confirmed)
           && (s.output.event.median_candidate_valid));
    t += 60000;
    angle(&f, 0, 0);
    assert(BUDDY4_FRESH == buddy4_poll(&s, t));
    assert((s.output.event_ended) && (s.output.ended_event.peak_confirmed));
    assert(s.output.ended_event.median_candidate_valid);
    assert(fabsf(s.output.ended_event.peak_roll_deg - 6) < 0.01f);
    assert(fabsf(s.output.ended_event.median_candidate_roll_deg - 6) < 0.01f);
    assert(fabsf(s.output.ended_event.peak_height_cm -
                 11.3f*sinf(6 * DEG_TO_RAD)) < 0.01f);
}

int main (void)
{
    assert(0 == legacy_regression_main());
    test_windows();
    test_comparison();
    test_lifecycle();
    test_interruption_and_event_isolation();
    puts("Experimental rolling-median regression tests passed");
    return 0;
}

/*** end of file ***/
