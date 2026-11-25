/* Host test: gcc -I.. test_lift.c ../buddy4_lift.c -lm && ./a.out */
#include "buddy4_lift.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static buddy4_lift_t d;
static uint64_t t;
static void tick (float x, float y, float z, int n)
{
    for (int idx = 0; idx < n; idx++)
    {
        t += 20000u;
        buddy4_lift_update(&d, x, y, z, true, t, t);
    }
}

static void tilt (float deg, int n)
{
    float r = deg * 0.0174532925f;
    tick(sinf(r), 0, cosf(r), n);
}

static void reset (void)
{
    buddy4_lift_config_t c = buddy4_lift_default_config();
    /* tests below use +Z up; the car itself is mounted with +Z down (default
     * -1) */
    c.up_sign = 1.0f;
    assert(buddy4_lift_init(&d, c));
    t = 1000000u;
}

int main (void)
{
    /* Level and still: never lifted. */
    reset();
    tick(0, 0, 1, 500);
    assert((!d.output.lifted) && (0 == d.output.count));
    /* Ramp-like tilts and vibration do not trigger. */
    reset();
    tilt(15, 200);
    assert(!d.output.lifted);
    reset();

    for (int idx = 0; idx < 200; idx++)
    {
        tick(0.03f * sinf(idx), 0, 1.0f + 0.05f * sinf(idx * 0.9f), 1);
    }

    assert(!d.output.lifted);
    /* A 3-sample bump (60 ms) at 60 degrees or 0.4 g is too short. */
    reset();
    tilt(60, 3);
    tick(0, 0, 1, 20);
    assert(!d.output.lifted);
    reset();
    tick(0, 0, 1.4f, 3);
    tick(0, 0, 1, 20);
    assert(!d.output.lifted);
    /* Picked up and tilted: lifted after >=100 ms. */
    reset();
    tilt(60, 4);
    assert(!d.output.lifted);
    tilt(60, 3);
    assert((d.output.lifted) && (BUDDY4_LIFT_TILTED == d.output.reason)
           && (1 == d.output.count));
    /* Latches while still tilted and after levelling for less than 1 s. */
    tilt(60, 100);
    assert((d.output.lifted) && (1 == d.output.count));
    tick(0, 0, 1, 40);
    assert(d.output.lifted);                 /* 0.8 s level */
    tilt(60, 2);
    tick(0, 0, 1, 40);
    assert(d.output.lifted);    /* shaken again: restart wait */
    tick(0, 0, 1, 20);
    assert(!d.output.lifted);                /* 1 s level and still */
    /* Overturned. */
    reset();
    tick(0, 0, -1, 10);
    assert((d.output.lifted) && (BUDDY4_LIFT_OVERTURNED == d.output.reason));
    reset();
    tilt(90, 10);
    assert((d.output.lifted) && (BUDDY4_LIFT_TILTED == d.output.reason));
    /* Level lift: sustained 0.3 g push for 200 ms. */
    reset();
    tick(0, 0, 1.3f, 4);
    assert(!d.output.lifted);
    tick(0, 0, 1.3f, 4);
    assert((d.output.lifted) && (BUDDY4_LIFT_MOTION == d.output.reason));
    /* Invalid input keeps the latch, then releases after 2 s of failure. */
    reset();
    tilt(60, 10);
    assert(d.output.lifted);

    for (int idx = 0; idx < 50; idx++)
    {
        t += 20000u;
        buddy4_lift_update(&d, 0, 0, 0, false, t, t);
    }

    assert((d.output.lifted) && (!d.output.valid));

    for (int idx = 0; idx < 60; idx++)
    {
        t += 20000u;
        buddy4_lift_update(&d, 0, 0, 0, false, t, t);
    }

    assert(!d.output.lifted);
    /* Stale samples are not trusted. */
    reset();
    t += 20000u;
    buddy4_lift_update(&d, 1, 0, 0, true, t - 500000u, t);
    assert(!d.output.valid);
    /* Default config (car mounting, Z down): level reads z = -1 and must NOT
     * count as lifted. */
    assert(buddy4_lift_init(&d, buddy4_lift_default_config()));
    t = 1000000u;
    tick(0, 0, -1, 500);
    assert((!d.output.lifted) && (d.output.tilt_deg < 1.0f));
    tick(0.26f, 0.05f, -0.96f, 200);
    assert(!d.output.lifted); /* ~15 deg ramp */
    tick(0, 0, 1, 10);
    /* flipped over */
    assert((d.output.lifted) && (BUDDY4_LIFT_OVERTURNED == d.output.reason));
    puts("buddy4_lift host test OK");
    return 0;
}

/*** end of file ***/
