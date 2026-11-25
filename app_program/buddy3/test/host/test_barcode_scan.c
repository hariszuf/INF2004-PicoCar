/* Host test: barcode_scan fed with a sampled sensor signal. gcc only. */
#include "../../../barrc_types.h"
#include <stdio.h>
#include <string.h>

#include "barcode_scan.h"

static int32_t g_run;
static int32_t g_fail;
#define CHECK(c, ...) do { g_run++; if (!(c)) { g_fail++; \
    printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } \
    } while (0)

/* Measured widths in px (paper photo = F; deck renders A, Z, W, X, Y). */
static const uint32_t PHOTO_F[29] = {15, 47, 16, 16, 48, 16, 48, 16, 16, 16, 16,
    16, 49, 15, 50, 48, 17, 16, 17, 16, 18, 50, 18, 16, 52, 16, 52, 17, 18};
static const uint32_t DECK_A[29]  = {15, 46, 15, 15, 46, 15, 46, 15, 15, 15, 46,
    15, 15, 15, 16, 45, 15, 16, 45, 15, 16, 45, 15, 16, 45, 15, 46, 15, 15};
static const uint32_t DECK_Z[29]  = {15, 46, 15, 15, 46, 15, 46, 15, 15, 15, 15,
    46, 46, 15, 45, 16, 15, 15, 15, 15, 15, 46, 15, 16, 45, 15, 46, 15, 15};
static const uint32_t DECK_W[29]  = {29, 86, 29, 28, 86, 29, 86, 29, 29, 28, 86,
    87, 86, 28, 29, 29, 29, 28, 29, 29, 28, 86, 29, 29, 86, 29, 86, 29, 28};
static const uint32_t DECK_Y[29]  = {16, 46, 16, 15, 47, 15, 47, 15, 16, 15, 47,
    46, 16, 15, 47, 15, 16, 15, 16, 15, 16, 46, 16, 15, 47, 15, 47, 15, 16};

static uint32_t s_now_us;
static char s_hook_letter;
static int32_t s_hook_calls;

void barcode_scan_on_decoded (const barcode39_result_t * r)
{
    s_hook_letter = r->text[0];
    s_hook_calls++;
}

/* Sample the sensor every 5 ms for `us` microseconds at the given level. */
static void hold (bool black, uint32_t us)
{
    uint32_t end = s_now_us + us;

    while (((uint32_t)(end - s_now_us) > 0) && (s_now_us < end))
    {
        barcode_scan_update(black, s_now_us);
        s_now_us += 5000u;
    }
}

/* Drive over a barcode: white lead-in, the bars (black first), white lead-out.
 * us_per_px sets the speed. */
static void drive_over (const uint32_t * w, int32_t n, uint32_t us_per_px,
                        bool backwards, uint32_t glitch_gap)
{
    hold(false, 600000u);

    for (int32_t idx = 0; idx < n; idx++)
    {
        uint32_t px = backwards ? w[n - 1 - idx] : w[idx];
        bool black = 0 == (idx % 2);
        uint32_t us = px * us_per_px;

        if (glitch_gap && (!black) && (us > 40000u))
        {
            /* a one-sample black glitch in the middle of a white gap */
            hold(false, us / 2);
            barcode_scan_update(true, s_now_us);
            s_now_us += 5000u;
            hold(false, us / 2);
        }
        else
        {
            hold(black, us);
        }
    }

    hold(false, 600000u);
}

static void expect (const char * name, const uint32_t * w, char letter,
                    uint32_t us_per_px,
                    bool backwards, uint32_t glitch)
{
    barcode39_result_t r;
    s_hook_calls = 0;
    memset(&r, 0, sizeof r);
    barcode_scan_init();
    s_now_us = 1000u;
    drive_over(w, 29, us_per_px, backwards, glitch);
    bool got = barcode_scan_take(&r);
    CHECK(got, "%s: no barcode decoded", name);

    if (got)
    {
        CHECK((r.text[0] == letter) && (1 == r.length),
              "%s: got '%s' want '%c'",
              name, r.text, letter);
        CHECK(r.reversed == backwards, "%s: reversed flag %d", name,
              r.reversed);
        CHECK((1 == s_hook_calls) && (s_hook_letter == letter),
              "%s: hook calls=%d",
              name, s_hook_calls);
    }

    CHECK(!barcode_scan_take(&r), "%s: result should be taken only once", name);
}

int main (void)
{
    /* speed: 1 px = 0.7 ms .. 6 ms. At 1.5 ms/px a narrow bar (15 px) is ~22
     * ms. */
    expect("F  fast", PHOTO_F, 'F', 1500u, false, 0u);
    expect("F  slow", PHOTO_F, 'F', 5000u, false, 0u);
    expect("A", DECK_A, 'A', 2000u, false, 0u);
    expect("Z", DECK_Z, 'Z', 2000u, false, 0u);
    expect("W", DECK_W, 'W', 1000u, false, 0u);
    expect("Y", DECK_Y, 'Y', 2000u, false, 0u);
    expect("F backwards", PHOTO_F, 'F', 2000u, true, 0u);
    expect("Z backwards", DECK_Z, 'Z', 2000u, true, 0u);
    expect("F with glitch", PHOTO_F, 'F', 3000u, false, 1u);

    /* Hand push: starts slow and speeds up over the first few bars (as seen on
     * the car:
       first bar 38 ms, later narrow bars 18 ms). Per-element scale in percent.
       */
    {
        static const uint32_t ramp[29] = {210, 170, 140, 125, 115, 108, 104,
            102, 100, 100, 100, 100, 100, 100, 100,
            100, 100, 100, 100, 100, 100, 100,
            100, 100, 100, 100, 100, 100,
            100};
        uint32_t w[29];
        barcode39_result_t r;

        for (int32_t idx = 0; idx < 29; idx++)
        {
            w[idx] = PHOTO_F[idx] * ramp[idx] / 100u;
        }

        barcode_scan_init();
        s_now_us = 1000u;
        drive_over(w, 29, 1200u, false, 0u);
        bool got = barcode_scan_take(&r);
        CHECK(got && ('F' == r.text[0]), "hand-push ramp: got %d '%s' fail=%u",
              got, got ? r.text : "", (uint32_t)barcode_scan_last_fail());
    }

    /* First character seen on the car (ms): 38 75 18 18 71 22 65 19 26 must
     * read as '*'. */
    {
        uint32_t b[29] = {38, 75, 18, 18, 71, 22, 65, 19, 26, 20, 20, 20, 65,
            20, 65, 65, 20, 20, 20, 20, 20, 70, 20, 20, 70, 20, 70, 20, 20};
        barcode39_result_t r;
        barcode39_cfg_t cfg = { 14u, true };
        bool ok = barcode39_decode_elements(b, 29, &cfg, &r);
        CHECK(ok && ('F' == r.text[0]), "car data decode: ok=%d fail=%u", ok,
              (uint32_t)barcode39_last_fail());
    }

    /* half-read barcode, long pause, then a good one: stale data must not
     * poison it */
    {
        barcode39_result_t r;
        barcode_scan_init();
        s_now_us = 1000u;
        hold(false, 600000u);

        for (int32_t idx = 0; idx < 14; idx++)
        {
            hold(0 == (idx % 2), PHOTO_F[idx] * 2000u);
        }

        hold(false, 3000000u);
        drive_over(DECK_A, 29, 2000u, false, 0u);
        bool got = barcode_scan_take(&r);
        CHECK(got && ('A' == r.text[0]), "after stale partial: got %d '%s'",
              got,
              got ? r.text : "");
    }

    /* line edges / dirt: random short blobs must not decode anything */
    {
        barcode39_result_t r;
        barcode_scan_init();
        s_now_us = 1000u;

        for (int32_t idx = 0; idx < 20; idx++)
        {
            hold(false, 300000u);
            hold(true, 60000u + (idx % 3) * 20000u);
        }

        hold(false, 600000u);
        CHECK(!barcode_scan_take(&r), "noise decoded as '%s'", r.text);
    }

    /* two codes in a row */
    {
        barcode39_result_t r1;
        barcode39_result_t r2;
        barcode_scan_init();
        s_now_us = 1000u;
        drive_over(DECK_W, 29, 1500u, false, 0u);
        bool a = barcode_scan_take(&r1);
        drive_over(DECK_Y, 29, 1500u, false, 0u);
        bool b = barcode_scan_take(&r2);
        CHECK(a && b && ('W' == r1.text[0]) && ('Y' == r2.text[0]),
              "two codes: %d %d", a, b);
    }

    printf("%d checks, %d failed\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}

/*** end of file ***/
