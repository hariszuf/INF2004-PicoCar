#include "../../barrc_types.h"
#include "buddy4_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static buddy4_motion_t c;
static uint64_t t;
static buddy4_motion_output_t tick (float32_t l, float32_t r)
{
    t += 20000;
    buddy4_motion_update(&c, l, r, true, t, t);
    return buddy4_motion_get(&c, t);
}

static void reset (void)
{
    assert(buddy4_motion_init(&c, buddy4_motion_default_config()));
    t = 0;
    buddy4_motion_output_t o = buddy4_motion_get(&c, 0);
    assert((!o.valid) && (BUDDY4_MOTION_UNKNOWN == o.state));
}

static void confirmed (float32_t l, float32_t r, buddy4_motion_state_t expected)
{
    for (int32_t idx = 0; idx < 6; idx++)
    {
        tick(l, r);
    }

    buddy4_motion_output_t o = buddy4_motion_get(&c, t);
    assert((o.valid) && (o.state == expected));
}

static void cases (void)
{
    struct
    {
        float32_t l;
        float32_t r;
        buddy4_motion_state_t s;
    }
    cases[] = {
        {0, 0, BUDDY4_MOTION_STOPPED}, {3, 3, BUDDY4_MOTION_FORWARD},
        {-3, -3, BUDDY4_MOTION_REVERSE}, {1, 3, BUDDY4_MOTION_TURNING_LEFT},
        {3, 1, BUDDY4_MOTION_TURNING_RIGHT}, {-3, -1,
            BUDDY4_MOTION_TURNING_LEFT},
        {-1, -3, BUDDY4_MOTION_TURNING_RIGHT}, {-3, 3,
            BUDDY4_MOTION_TURNING_LEFT},
        {3, -3, BUDDY4_MOTION_TURNING_RIGHT}, {0, 3,
            BUDDY4_MOTION_TURNING_LEFT}};

    for (uint32_t idx = 0; idx < sizeof(cases) / sizeof(cases[0]); idx++)
    {
        reset();
        confirmed(cases[idx].l, cases[idx].r, cases[idx].s);
    }
}

static void jitter (void)
{
    reset();
    confirmed(0, 0, BUDDY4_MOTION_STOPPED);

    for (int32_t idx = 0; idx < 20; idx++)
    {
        assert(BUDDY4_MOTION_STOPPED == tick(idx % 2 ? 0.7f : 0.4f,
                                             0.6f).state);
    }

    for (int32_t idx = 0; idx < 20; idx++)
    {
        assert(BUDDY4_MOTION_STOPPED == tick(idx % 2 ? 0.9f : 0.7f,
                                             idx % 2 ? 0.9f : 0.7f).state);
    }

    confirmed(1, 1, BUDDY4_MOTION_FORWARD);
    confirmed(1, 3, BUDDY4_MOTION_TURNING_LEFT);

    for (int32_t idx = 0; idx < 20; idx++)
    {
        assert(BUDDY4_MOTION_TURNING_LEFT == tick(1,
                idx % 2 ? 2.2f : 2.6f).state);
    }

    confirmed(3, 3, BUDDY4_MOTION_FORWARD);
    /* At turn entry jitter cannot satisfy confirmation continuously. */
    for (int32_t idx = 0; idx < 20; idx++)
    {
        assert(BUDDY4_MOTION_FORWARD == tick(3, idx % 2 ? 4.4f : 4.6f).state);
    }

    reset();

    for (int32_t idx = 0; idx < 5; idx++)
    {
        buddy4_motion_output_t o = tick(3, 3);
        assert(!o.valid);
    }

    buddy4_motion_update(&c, 3, 3, true, 119999, 119999);
    assert(!buddy4_motion_get(&c, 119999).valid);
    buddy4_motion_update(&c, 3, 3, true, 120000, 120000);
    assert(BUDDY4_MOTION_FORWARD == buddy4_motion_get(&c, 120000).state);
}

static void invalid (void)
{
    const float32_t bad[] = {NAN, INFINITY, -INFINITY};

    for (int32_t side = 0; side < 2; side++)
    {
        for (int32_t idx = 0; idx < 3; idx++)
        {
            reset();
            confirmed(3, 3, BUDDY4_MOTION_FORWARD);
            buddy4_motion_output_t o =
                    tick(side ? 3 : bad[idx], side ? bad[idx] : 3);
            assert((!o.valid) && (BUDDY4_MOTION_UNKNOWN == o.state));

            for (int32_t jdx = 0; jdx < 5; jdx++)
            {
                assert(!tick(3, 3).valid);
            }

            assert(tick(3, 3).valid);
        }
    }

    reset();
    confirmed(0, 0, BUDDY4_MOTION_STOPPED);
    t += 20000;
    buddy4_motion_update(&c, 0, 0, false, t, t);
    assert(BUDDY4_MOTION_UNKNOWN == buddy4_motion_get(&c, t).state);
    reset();
    confirmed(3, 3, BUDDY4_MOTION_FORWARD);
    assert(buddy4_motion_get(&c, t + 99999).valid);
    assert(!buddy4_motion_get(&c, t + 100000).valid);
    t += 100000;
    confirmed(-3, -3, BUDDY4_MOTION_REVERSE);
    reset();
    confirmed(3, 3, BUDDY4_MOTION_FORWARD);
    buddy4_motion_update(&c, 3, 3, true, t, t); /* Duplicate sample. */
    assert(!buddy4_motion_get(&c, t).valid);
    confirmed(3, 3, BUDDY4_MOTION_FORWARD);
    buddy4_motion_update(&c, 3, 3, true, t - 1, t); /* Reversed source time. */
    assert(!buddy4_motion_get(&c, t).valid);
    confirmed(3, 3, BUDDY4_MOTION_FORWARD);
    assert(!buddy4_motion_get(&c, t - 1).valid); /* Reversed consumer clock. */
    confirmed(3, 3, BUDDY4_MOTION_FORWARD);
    t += 20000;
    buddy4_motion_update(&c, 3, 3, true, t + 1, t);
    assert(!buddy4_motion_get(&c, t).valid); /* Future sample. */
    reset();
    buddy4_motion_update(&c, 0, 0, true, 0, 100000);
    /* Already stale at arrival. */
    assert(!buddy4_motion_get(&c, 100000).valid);
    reset();
    tick(3, 3);
    tick(3, 3);
    t += 100000;
    buddy4_motion_update(&c, 3, 3, true, t, t);
    /* Gap cannot complete pending label. */
    assert(!buddy4_motion_get(&c, t).valid);

}

int main (void)
{
    cases();
    jitter();
    /* UNKNOWN cancellation is tested separately (valid must remain false). */
    reset();

    for (int32_t idx = 0; idx < 20; idx++)
    {
        assert(!tick(-0.7f, 0.7f).valid);
    }

    invalid();
    buddy4_motion_config_t cfg = buddy4_motion_default_config();
    cfg.stop_enter_cm_s = NAN;
    assert(!buddy4_motion_init(&c, cfg));
    assert(!tick(0, 0).valid);
    cfg = buddy4_motion_default_config();
    cfg.confirm_us = 40000;
    assert(buddy4_motion_init(&c, cfg));
    t = 0;
    tick(0, 0);
    tick(0, 0);
    assert(BUDDY4_MOTION_STOPPED == tick(0, 0).state);
    assert(!strcmp(buddy4_motion_name(BUDDY4_MOTION_UNKNOWN), "UNKNOWN"));
    puts("Buddy 4 encoder motion classification tests passed");
    return 0;
}

/*** end of file ***/
