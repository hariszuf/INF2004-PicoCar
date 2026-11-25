#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "buddy4_obstruction.h"

/* Independent synthetic fixture. Only 114.86, 114.96 and 115.06 s are
 * operator-reported observations. Intermediate 20 ms values below are invented
 * for continuous replay, NOT recovered hardware samples or contact timestamps.
 */
typedef struct
{
    uint64_t us;
    float left;
    float right;
} trace_sample_t;
static const trace_sample_t foam[] = {
    {114860000, 3.18f, 3.09f}, {114880000, 1.59f, 1.545f},
    {114900000, 0.795f, 0.7725f}, {114920000, 0.42f, 0.41f},
    {114940000, 0.23f, 0.22f}, {114960000, 0.12f, 0.12f},
    {114980000, 0.06f, 0.06f}, {115000000, 0.03f, 0.03f},
    {115020000, 0.015f, 0.015f}, {115040000, 0.008f, 0.008f},
    {115060000, 0.004f, 0.004f}, {115080000, 0.002f, 0.002f}
};
static const uint64_t stop_us = 115090000;
static void reproduce_input (void)
{
    assert((foam[0].left > 3) && (foam[0].right > 3));
    assert((114960000 == foam[5].us) && (0.12f == foam[5].left));
    assert((115060000 == foam[10].us) && (0.004f == foam[10].right));
    assert((foam[10].us < stop_us) && (130000 == stop_us - foam[5].us));

    for (uint32_t idx = 1; idx < sizeof(foam) / sizeof(foam[0]); idx++)
    {
        assert(20000 == foam[idx].us - foam[idx - 1].us);
        assert((foam[idx].left < foam[idx - 1].left)
               && (foam[idx].right < foam[idx - 1].right));
        assert(foam[idx].us < stop_us);
    }

    puts("Independent synthetic replay: both wheels 3.18/3.09 -> 0.12/0.12 -> "
         "0.004/0.004 cm/s before S; intermediate observations assumed");
}

static buddy4_obstruction_t d;
static buddy4_obstruction_input_t in;
static uint64_t t;
static void reset (void)
{
    buddy4_obstruction_init(&d);
    t = 0;
    in = (buddy4_obstruction_input_t){.valid = true, .eligible = true,
        .demand_id = 1,
        .command_left_cm_s = 3, .command_right_cm_s = 3};
}

static buddy4_obstruction_output_t tick (float left, float right)
{
    t += 20000;
    in.observation_us = t;
    in.left_cm_s = left;
    in.right_cm_s = right;
    buddy4_obstruction_update(&d, &in, t);
    return buddy4_obstruction_get(&d, t);
}

static void establish (void)
{
    for (int idx = 0; idx < 26; idx++)
    {
        assert(!tick(3, 3).possible_obstruction);
    }

    assert((d.output.valid) && (d.output.established));
}

static void blocked (void)
{
    for (int idx = 0; idx < 5; idx++)
    {
        assert(!tick(0.12f, 0.12f).possible_obstruction);
    }

    assert(tick(0.004f, 0.004f).possible_obstruction);
}

static void replay (void)
{
    reset();

    for (t = 114300000; t <= 114840000; t += 20000)
    {
        in.observation_us = t;
        in.left_cm_s = 3.18f;
        in.right_cm_s = 3.09f;
        buddy4_obstruction_update(&d, &in, t);
    }

    assert(d.output.established);
    uint64_t detected = 0;

    for (uint32_t idx = 0; idx < sizeof(foam) / sizeof(foam[0]); idx++)
    {
        in.observation_us = foam[idx].us;
        in.left_cm_s = foam[idx].left;
        in.right_cm_s = foam[idx].right;
        buddy4_obstruction_update(&d, &in, foam[idx].us);

        if (d.output.possible_obstruction)
        {
            detected = d.output.event_us;
        }
    }

    assert((115020000 == detected) && (detected < stop_us)
           && (1 == d.output.event_count));
    buddy4_obstruction_cancel(&d, stop_us);
    assert((!d.output.valid) && (!d.output.possible_obstruction)
           && (1 == d.output.event_count));
    assert(d.output.event_us == detected);
    printf("Synthetic continuous foam replay: POSSIBLE_OBSTRUCTION at %.2f s "
           "before S at 115.09 s (not hardware validation)\n", detected / 1e6);
    /* The actual sparse anchors alone cannot bridge 100 ms gaps as fresh input.
     */
    reset();
    establish();
    tick(3.18f, 3.09f);
    t += 100000;
    in.observation_us = t;
    in.left_cm_s = in.right_cm_s = 0.12f;
    buddy4_obstruction_update(&d, &in, t);
    t += 100000;
    in.observation_us = t;
    in.left_cm_s = in.right_cm_s = 0.004f;
    buddy4_obstruction_update(&d, &in, t);
    assert((0 == d.output.event_count) && (!d.output.established));
}

static void driving_and_stops (void)
{
    reset();

    for (int idx = 0; idx < 200; idx++)
    {
        tick(0, 0); /* Cannot detect startup/non-moving demand. */
    }

    assert((!d.output.established) && (0 == d.output.event_count));
    establish();

    for (int idx = 0; idx < 500; idx++)
    {
        assert(!tick(3 + 0.15f*sinf(idx),
                     3 - 0.1f*sinf(idx)).possible_obstruction);
    }

    /* Same known forward-speed history, then commanded stop before wheel decay.
     */
    buddy4_obstruction_cancel(&d, t + 1);
    in.eligible = false;

    for (uint32_t idx = 1; idx < sizeof(foam) / sizeof(foam[0]); idx++)
    {
        assert(!tick(foam[idx].left, foam[idx].right).possible_obstruction);
    }

    assert((!d.output.valid) && (0 == d.output.event_count));
    reset();
    establish();

    for (int idx = 0; idx < 40; idx++)
    {
        assert(!tick(1.5f, 1.5f).possible_obstruction); /* Partial reduction. */
    }

    assert(0 == d.output.event_count);
    reset();
    establish();
    tick(0.12f, 0.12f);
    tick(0.12f, 0.12f);
    buddy4_obstruction_cancel(&d, t + 1);
    in.eligible = false;

    for (int idx = 0; idx < 20; idx++)
    {
        tick(0, 0);
    }

    assert(0 == d.output.event_count); /* Pending evidence cannot survive S. */
    /* A reduced/new forward target cannot inherit earlier establishment. */
    reset();
    establish();
    in.command_left_cm_s = in.command_right_cm_s = 2;

    for (int idx = 0; idx < 20; idx++)
    {
        tick(0.1f, 0.1f);
    }

    assert((!d.output.established) && (0 == d.output.event_count));
    reset();
    in.command_left_cm_s = in.command_right_cm_s = 1;

    for (int idx = 0; idx < 100; idx++)
    {
        tick(1, 1);
    }

    for (int idx = 0; idx < 100; idx++)
    {
        tick(0, 0);
    }

    /* Below qualified speed envelope. */
    assert((!d.output.valid) && (0 == d.output.event_count));
}

static void exclusions (void)
{
    for (int kind = 0; kind < 6; kind++)
    {
        reset();
        establish();
        tick(0.12f, 0.12f);

        if (0 == kind)
        {
            for (int idx = 0; idx < 30; idx++)
            {
                tick(0, 3);
            }
        }

        if (1 == kind)
        {
            for (int idx = 0; idx < 30; idx++)
            {
                tick(3, 0);
            }
        }

        if (2 == kind)
        {
            in.command_left_cm_s = 2;

            for (int idx = 0; idx < 30; idx++)
            {
                tick(0, 0);
            }
        }

        if (3 == kind)
        {
            for (int idx = 0; idx < 30; idx++)
            {
                tick(-0.1f, -0.1f);
            }
        }

        if (4 == kind)
        {
            in.eligible = false;

            for (int idx = 0; idx < 30; idx++)
            {
                tick(0, 0);
            }
        } /* Ramp/fault. */

        if (5 == kind)
        {
            ++in.demand_id;

            for (int idx = 0; idx < 30; idx++)
            {
                tick(0, 0);
            }
        } /* Mode/session. */

        assert((!d.output.established) && (0 == d.output.event_count));
    }

    reset();
    establish();

    for (int idx = 0; idx < 50; idx++)
    {
        tick(0.12f, 0.12f);
        tick(3, 3);
    }

    /* Isolated low samples are not sustained. */
    assert(0 == d.output.event_count);
}

static void faults_and_recovery (void)
{
    for (int history = 0; history < 2; history++)
    {
        for (int kind = 0; kind < 9; kind++)
        {
            reset();
            establish();

            if (history)
            {
                blocked();
            }

            uint64_t event = d.output.event_us;
            tick(0.1f, 0.1f);
            in.observation_us = t + 20000;
            in.left_cm_s = in.right_cm_s = 0.1f;
            uint64_t now = t + 20000;

            if (0 == kind)
            {
                in.valid = false;
            }

            if (1 == kind)
            {
                in.left_cm_s = NAN;
            }

            if (2 == kind)
            {
                in.right_cm_s = INFINITY;
            }

            if (3 == kind)
            {
                in.observation_us = t;
            }

            if (4 == kind)
            {
                in.observation_us = t - 1;
            }

            if (5 == kind)
            {
                in.observation_us = now + 1;
            }

            if (6 == kind)
            {
                in.observation_us = now - 40001;
            }

            if (7 == kind)
            {
                now = t - 1;
            }

            if (8 == kind)
            {
                buddy4_obstruction_get(&d, t + 40001);
                now = t + 60000;
                in.valid = false;
            }

            buddy4_obstruction_update(&d, &in, now);
            assert((!d.output.valid) && (!d.output.possible_obstruction)
                   && (!d.output.established));
            assert((d.output.event_count == (unsigned)history)
                   && (d.output.event_us == event));
            t += 2000000;
            in.valid = true;

            for (int idx = 0; idx < 20; idx++)
            {
                tick(0, 0);
            }

            /* Recovery cannot prime while stuck. */
            assert(d.output.event_count == (unsigned)history);
            establish();
            blocked();
            assert(d.output.event_count == (unsigned)history + 1);
        }
    }
}

static void cooldown_and_boundaries (void)
{
    reset();

    for (int idx = 0; idx < 25; idx++)
    {
        tick(3, 3);
    }

    assert(!d.output.established);
    tick(3, 3);
    assert(d.output.established);
    blocked();
    uint64_t event = t;

    for (int idx = 0; idx < 100; idx++)
    {
        assert(!tick(0, 0).possible_obstruction);
    }

    assert(1 == d.output.event_count); /* Holding blockage cannot rearm. */
    establish();
    blocked();
    assert(2 == d.output.event_count);
    reset();
    establish();
    blocked();
    event = t;

    for (int idx = 0; idx < 26; idx++)
    {
        tick(3, 3);
    }

    /* Full moving window, but cooldown not complete. */
    assert(!d.output.established);

    while (t < event + 980000)
    {
        tick(3, 3);
    }

    assert(!d.output.established);
    tick(3, 3);
    assert(d.output.established);
    blocked();
    assert(2 == d.output.event_count);
    assert(buddy4_obstruction_get(&d, t + 40000).valid);
    assert(!buddy4_obstruction_get(&d, t + 40001).valid);
    assert(2 == d.output.event_count);
    t += 1200000;
    establish();
    d.output.event_count = UINT32_MAX;
    blocked();
    assert((UINT32_MAX == d.output.event_count) && (d.output.event_us == t));
    reset();
    establish();
    tick(0.5f, 0.5f);

    for (int idx = 0; idx < 4; idx++)
    {
        tick(0.5f, 0.5f);
    }

    in.observation_us = t + 19999;
    buddy4_obstruction_update(&d, &in, t + 19999);
    assert(!d.output.possible_obstruction);
    in.observation_us = t + 20000;
    buddy4_obstruction_update(&d, &in, t + 20000);
    /* Inclusive speed and 100 ms edges. */
    assert(d.output.possible_obstruction);
}

static void asymmetric_transition (void)
{
    reset();
    establish();

    for (int idx = 0; idx < 6; idx++)
    {
        buddy4_obstruction_output_t o = tick(2.10f, 3.15f);
        assert((o.valid) && (o.established) && (!o.possible_obstruction)
               && (0 == o.slow_ms));
    }

    tick(0.58f, 0.99f);
    assert((d.output.established) && (!d.pending));
    blocked();
    /* Both-low confirmation is still 100 ms. */
    assert(1 == d.output.event_count);
    /* Symmetry never establishes motion by itself, and one-wheel slip alone
     * never triggers. A persistent transition expires rather than being
     * latched. */
    reset();

    for (int idx = 0; idx < 20; idx++)
    {
        tick(1, 3);
    }

    for (int idx = 0; idx < 20; idx++)
    {
        tick(0, 0);
    }

    assert((!d.output.established) && (0 == d.output.event_count));

    for (int slip = 0; slip < 2; slip++)
    {
        reset();
        establish();

        for (int idx = 0; idx < 11; idx++)
        {
            assert(!tick(slip ? 3 : 0, slip ? 6 : 3).possible_obstruction);
        }

        /* Exactly 200 ms since first asymmetry. */
        assert(d.output.established);
        tick(slip ? 3 : 0, slip ? 6 : 3);
        assert(!d.output.established); /* 220 ms: expired. */
        for (int idx = 0; idx < 20; idx++)
        {
            tick(0, 0);
        }

        assert(0 == d.output.event_count);
        establish();
        blocked();
        assert(1 == d.output.event_count);
    }

    /* Becoming roughly balanced but neither moving nor stopped cannot extend
     * the transition indefinitely. Only recovery or a true both-low candidate
     * resolves it. */
    reset();
    establish();
    tick(1, 3);

    for (int idx = 0; idx < 11; idx++)
    {
        tick(1, 1);
    }

    assert((!d.output.established) && (0 == d.output.event_count));
    /* A low sample before asymmetry must not contribute to later confirmation.
     */
    reset();
    establish();
    tick(0.1f, 0.1f);
    tick(1, 3);

    for (int idx = 0; idx < 5; idx++)
    {
        assert(!tick(0.1f, 0.1f).possible_obstruction);
    }

    assert(tick(0.1f, 0.1f).possible_obstruction);
    /* Turns, changed demand/mode, invalid input and gaps always cancel the
     * hold. */
    for (int kind = 0; kind < 6; kind++)
    {
        reset();
        establish();
        tick(1, 3);

        if (0 == kind)
        {
            in.eligible = false;
        }

        if (1 == kind)
        {
            in.command_right_cm_s = 2;
        }

        if (2 == kind)
        {
            ++in.demand_id;
        }

        if (3 == kind)
        {
            in.valid = false;
        }

        if (4 == kind)
        {
            in.left_cm_s = NAN;
        }

        if (5 == kind)
        {
            t += 40001;
            in.observation_us = t;
            in.left_cm_s = in.right_cm_s = 0;
        }
        else
        {
            t += 20000;
            in.observation_us = t;
        }

        buddy4_obstruction_update(&d, &in, t);
        assert((!d.output.established) && (!d.output.possible_obstruction)
               && (0 == d.output.event_count));
    }
}

static void pitch_pause_boundaries (void)
{
    reset();
    establish();
    in.pause_motion = true;
    tick(0, 0);
    uint64_t paused = t;
    tick(0, 0);
    assert((d.paused) && (!d.output.valid) && (!d.output.established));
    in.pause_motion = false;
    tick(0, 0);
    assert((t == paused + 40000) && (d.output.established)
           && (0 == d.output.slow_ms));

    for (int idx = 0; idx < 4; idx++)
    {
        assert(!tick(0, 0).possible_obstruction);
    }

    assert(tick(0, 0).possible_obstruction);
    uint64_t event = d.output.event_us;
    in.pause_motion = true;
    tick(0, 0); /* No established history after an event. */
    assert((!d.paused) && (1 == d.output.event_count)
           && (d.output.event_us == event));
    in.pause_motion = false;

    for (int idx = 0; idx < 27; idx++)
    {
        tick(3, 3);
    }

    assert(!d.output.established); /* Pause cannot bypass event cooldown. */

    for (int kind = 0; kind < 4; kind++)
    {
        reset();
        establish();
        in.pause_motion = true;
        tick(0, 0);
        assert(d.paused);

        if (0 == kind) /* A persistent excursion expires at exactly 40 ms. */
        {
            tick(0, 0);
            tick(0, 0);
        }
        else if (1 == kind) /* Duplicate is not new paused evidence. */
        {
            buddy4_obstruction_update(&d, &in, t);
        }

        /* No fresh update: normal age limit still applies. */
        else if (2 == kind)
        {
            buddy4_obstruction_get(&d, t + 40001);
            t += 40001;
        }
        else /* New input carrying stale observation time cannot revive it. */
        {
            buddy4_obstruction_update(&d, &in, t + 40001);
            t += 40001;
        }

        assert((!d.paused) && (!d.output.established)
               && (!d.output.possible_obstruction));
        in.pause_motion = false;

        for (int idx = 0; idx < 15; idx++)
        {
            assert(!tick(0, 0).possible_obstruction);
        }

        assert(!d.output.event_count);
    }
}

int main (void)
{
    pitch_pause_boundaries();
    reproduce_input();
    replay();
    driving_and_stops();
    exclusions();
    faults_and_recovery();
    cooldown_and_boundaries();
    asymmetric_transition();
    puts("Buddy 4 possible-obstruction core tests passed");
    return 0;
}

/*** end of file ***/
