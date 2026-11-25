#include "../../barrc_types.h"
#include "buddy4_encoder_yaw.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
static buddy4_encoder_yaw_t d;
static const float64_t deg_per_rad = 57.29577951308232;
static void reset (float32_t spacing)
{
    buddy4_encoder_yaw_config_t cfg = buddy4_encoder_yaw_default_config();
    cfg.centre_spacing_cm = spacing;
    assert(buddy4_encoder_yaw_init(&d, cfg));
    buddy4_encoder_yaw_output_t o = buddy4_encoder_yaw_get(&d, 0);
    /* Zero initialized numbers do not mean a valid zero rate. */
    assert(!o.valid);
}

static buddy4_encoder_yaw_output_t sample (float32_t l, float32_t r, uint64_t t)
{
    buddy4_encoder_yaw_update(&d, l, r, true, t, t);
    return buddy4_encoder_yaw_get(&d, t);
}

static void rate (float32_t l, float32_t r, float32_t spacing,
                  float64_t expected)
{
    reset(spacing);
    buddy4_encoder_yaw_output_t o = sample(l, r, 20000);
    assert((o.valid) && (20000 == o.sample_us));
    assert(fabs(o.encoder_yaw_rate_estimate_rad_s - expected) < 1e-6);
    assert(fabs(o.encoder_yaw_rate_estimate_deg_s - expected
                * deg_per_rad) < 1e-4);
}

static void signs_units (void)
{
    rate(0, 0, 11.3f, 0);
    rate(3, 3, 11.3f, 0);
    rate(-3, -3, 11.3f, 0);
    rate(1, 4, 11.3f, 3.0/11.3);
    rate(4, 1, 11.3f, -3.0/11.3);
    rate(-4, -1, 11.3f, 3.0/11.3);
    rate(-1, -4, 11.3f, -3.0/11.3);
    rate(-3, 3, 11.3f, 6.0/11.3);
    rate(3, -3, 11.3f, -6.0/11.3);
    rate(0, 10, 10, 1);
    rate(0, -10, 10, -1);
    /* Configurable geometry, no Buddy 2 setting dependency. */
    rate(0, 10, 20, 0.5);
    /* No filter delay: a new opposite spin changes sign immediately. */
    reset(10);
    sample(-5, 5, 20000);
    assert(-1 == sample(5, -5, 40000).encoder_yaw_rate_estimate_rad_s);
}

static void invalid_values (void)
{
    float32_t bad[] = {NAN, INFINITY, -INFINITY};

    for (int32_t k = 0; k < 3; k++)
    {
        for (int32_t side = 0; side < 2; side++)
        {
            reset(10);
            sample(0, 10, 20000);
            buddy4_encoder_yaw_output_t o =
                    sample(side ? 0 : bad[k], side ? bad[k] : 10, 40000);
            assert((!o.valid) && (20000 == o.sample_us)
                   && (1 == o.encoder_yaw_rate_estimate_rad_s));
            assert(sample(0, 0, 60000).valid);
        }
    }

    float32_t bad_spacing[] = {0, -1, NAN, INFINITY, -INFINITY};

    for (int32_t idx = 0; idx < 5; idx++)
    {
        buddy4_encoder_yaw_config_t cfg = {bad_spacing[idx], 100000};
        assert(!buddy4_encoder_yaw_init(&d, cfg));
        assert(!sample(0, 0, 20000).valid);
    }

    buddy4_encoder_yaw_config_t cfg = {11.3f, 0};
    assert(!buddy4_encoder_yaw_init(&d, cfg));
    reset(10);
    sample(0, 10, 20000);
    buddy4_encoder_yaw_update(&d, 0, 0, false, 40000, 40000);
    assert(!buddy4_encoder_yaw_get(&d, 40000).valid);
    assert(sample(0, 0, 60000).valid);
    reset(1e-30f);
    assert(!sample(-FLT_MAX, FLT_MAX, 20000).valid);
    /* Finite large inputs whose intermediate float difference would overflow.
     */
    reset(FLT_MAX);
    assert(sample(-FLT_MAX, FLT_MAX, 20000).valid);
}

static void time_faults (void)
{
    reset(10);
    sample(0, 10, 20000);
    assert(buddy4_encoder_yaw_get(&d, 119999).valid);
    buddy4_encoder_yaw_output_t o = buddy4_encoder_yaw_get(&d, 120000);
    assert((!o.valid) && (1 == o.encoder_yaw_rate_estimate_rad_s)
           && (20000 == o.sample_us));
    /* Missing observations, then immediate recovery. */
    assert(sample(0, 0, 140000).valid);
    /* Duplicate input cannot renew validity. */
    assert(!sample(0, 0, 140000).valid);
    assert(sample(0, 10, 160000).valid);
    buddy4_encoder_yaw_update(&d, 0, 0, true, 159999, 180000);
    assert(!buddy4_encoder_yaw_get(&d, 180000).valid);
    assert(sample(0, 0, 200000).valid);
    buddy4_encoder_yaw_update(&d, 0, 0, true, 220001, 220000);
    assert(!buddy4_encoder_yaw_get(&d, 220000).valid);
    assert(sample(0, 10, 240000).valid);
    /* Consumer clock reversal. */
    assert(!buddy4_encoder_yaw_get(&d, 239999).valid);
    assert(sample(0, 0, 260000).valid);
    /* Stale on arrival. */
    buddy4_encoder_yaw_update(&d, 0, 10, true, 280000, 380000);
    assert(!buddy4_encoder_yaw_get(&d, 380000).valid);
    assert(sample(0, 0, 400000).valid);
    reset(10);
    assert(sample(0, 0, 0).valid); /* Timestamp zero is allowed initially. */
    buddy4_encoder_yaw_config_t cfg = {10, 50000};
    assert(buddy4_encoder_yaw_init(&d, cfg));
    sample(0, 10, 0);
    assert(!buddy4_encoder_yaw_get(&d, 50000).valid);
}

int main (void)
{
    assert(11.3f == buddy4_encoder_yaw_default_config().centre_spacing_cm);
    signs_units();
    invalid_values();
    time_faults();
    puts("Encoder yaw-rate estimate tests passed (synthetic, no hardware "
         "validation)");
    return 0;
}

/*** end of file ***/
