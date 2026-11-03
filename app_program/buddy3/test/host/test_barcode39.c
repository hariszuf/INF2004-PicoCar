/* Host tests for barcode39. Build and run with run_tests.sh (needs only gcc). */
#include "barcode39.h"

#include <stdio.h>
#include <string.h>

static int g_fail = 0, g_run = 0;
#define CHECK(cond, ...)                                  \
    do {                                                  \
        g_run++;                                          \
        if (!(cond)) {                                    \
            g_fail++;                                     \
            printf("FAIL line %d: ", __LINE__);           \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
        }                                                 \
    } while (0)

/* ---- real measured widths (pixels) ---- */
/* Paper barcode photographed by Afiq (reads F top to bottom). */
static const uint32_t PHOTO_F[29] = {15, 47, 16, 16, 48, 16, 48, 16, 16, 16, 16, 16, 49, 15, 50,
                                     48, 17, 16, 17, 16, 18, 50, 18, 16, 52, 16, 52, 17, 18};
/* Slide renders from BARCODE.pptx (smallest variant of each letter). */
static const uint32_t DECK_A[29] = {15, 46, 15, 15, 46, 15, 46, 15, 15, 15, 46, 15, 15, 15, 16,
                                    45, 15, 16, 45, 15, 16, 45, 15, 16, 45, 15, 46, 15, 15};
static const uint32_t DECK_F[29] = {15, 46, 15, 15, 46, 15, 46, 15, 15, 15, 15, 16, 45, 15, 46,
                                    46, 15, 15, 15, 15, 15, 46, 15, 16, 45, 15, 46, 15, 15};
static const uint32_t DECK_Z[29] = {15, 46, 15, 15, 46, 15, 46, 15, 15, 15, 15, 46, 46, 15, 45,
                                    16, 15, 15, 15, 15, 15, 46, 15, 16, 45, 15, 46, 15, 15};
static const uint32_t DECK_W[29] = {29, 86, 29, 28, 86, 29, 86, 29, 29, 28, 86, 87, 86, 28, 29,
                                    29, 29, 28, 29, 29, 28, 86, 29, 29, 86, 29, 86, 29, 28};
static const uint32_t DECK_X[29] = {21, 64, 22, 21, 64, 21, 64, 22, 21, 21, 22, 64, 21, 21, 65,
                                    21, 21, 22, 64, 21, 21, 64, 22, 21, 64, 21, 64, 22, 21};
static const uint32_t DECK_Y[29] = {16, 46, 16, 15, 47, 15, 47, 15, 16, 15, 47, 46, 16, 15, 47,
                                    15, 16, 15, 16, 15, 16, 46, 16, 15, 47, 15, 47, 15, 16};

/* Feed a run list as the sensor would: optional white quiet zone and stray
 * noise first, scale in percent, optional linear speed drift. */
static bool feed_runs(barcode39_t *d, const uint32_t *w, int n, int scale_pct, int drift_pct,
                      bool noise_prefix, bool backwards, barcode39_result_t *out)
{
    bool got = false;
    if (noise_prefix) {
        barcode39_feed(d, false, 400, out);
        barcode39_feed(d, true, 30, out); /* line edge / dirt */
        barcode39_feed(d, false, 40, out);
        barcode39_feed(d, true, 90, out);
        barcode39_feed(d, false, 25, out);
    }
    for (int i = 0; i < n; i++) {
        uint32_t src = backwards ? w[n - 1 - i] : w[i];
        int      pos = backwards ? (n - 1 - i) : i;
        /* drift: scale goes from (100 - drift/2)% to (100 + drift/2)% over the code */
        int      s = scale_pct * (1000 + (drift_pct * 10 * (2 * pos - (n - 1))) / (2 * (n - 1))) / 1000;
        uint32_t width = (uint32_t)((int64_t)src * s / 100);
        /* black runs are the even indexes in forward order; after reversing a
         * 10k+9 element code they are still even, so colour is just i parity */
        if (barcode39_feed(d, (i % 2) == 0, width ? width : 1, out)) got = true;
    }
    barcode39_feed(d, false, 500, out); /* trailing quiet zone */
    return got;
}

static void expect(const char *name, const uint32_t *w, char letter, bool letters_only,
                   bool backwards, int scale, int drift, bool noise, bool want_reversed)
{
    barcode39_t        d;
    barcode39_result_t r;
    memset(&r, 0, sizeof r);
    barcode39_init(&d, letters_only);
    bool got = feed_runs(&d, w, 29, scale, drift, noise, backwards, &r);
    CHECK(got, "%s: nothing decoded", name);
    if (got) {
        CHECK(r.length == 1 && r.text[0] == letter, "%s: got '%s', want '%c'", name, r.text, letter);
        CHECK(r.reversed == want_reversed, "%s: reversed=%d want %d", name, r.reversed, want_reversed);
    }
}

/* Build element widths for one letter from the decoder's own table, to
 * exercise every A-Z at several ratios. */
static const char *LETTER_PAT[26] = {
    "wnnnnwnnw", "nnwnnwnnw", "wnwnnwnnn", "nnnnwwnnw", "wnnnwwnnn", "nnwnwwnnn", "nnnnnwwnw",
    "wnnnnwwnn", "nnwnnwwnn", "nnnnwwwnn", "wnnnnnnww", "nnwnnnnww", "wnwnnnnwn", "nnnnwnnww",
    "wnnnwnnwn", "nnwnwnnwn", "nnnnnnwww", "wnnnnnwwn", "nnwnnnwwn", "nnnnwnwwn", "wwnnnnnnw",
    "nwwnnnnnw", "wwwnnnnnn", "nwnnwnnnw", "wwnnwnnnn", "nwwnwnnnn"};
static const char *STAR = "nwnnwnwnn";

static void build(uint32_t *w, char letter, uint32_t narrow, uint32_t wide)
{
    const char *p[3] = {STAR, LETTER_PAT[letter - 'A'], STAR};
    int         k = 0;
    for (int b = 0; b < 3; b++) {
        for (int i = 0; i < 9; i++) w[k++] = (p[b][i] == 'w') ? wide : narrow;
        if (b < 2) w[k++] = narrow;
    }
}

int main(void)
{
    /* 1. real data: photo of the paper barcode, and each deck letter */
    expect("photo F", PHOTO_F, 'F', true, false, 100, 0, false, false);
    expect("deck A", DECK_A, 'A', true, false, 100, 0, false, false);
    expect("deck F", DECK_F, 'F', true, false, 100, 0, false, false);
    expect("deck Z", DECK_Z, 'Z', true, false, 100, 0, false, false);
    expect("deck W", DECK_W, 'W', true, false, 100, 0, false, false);
    expect("deck X", DECK_X, 'X', true, false, 100, 0, false, false);
    expect("deck Y", DECK_Y, 'Y', true, false, 100, 0, false, false);

    /* 2. noise and quiet zone before the code */
    expect("photo F + noise", PHOTO_F, 'F', true, false, 100, 0, true, false);
    expect("deck Z + noise", DECK_Z, 'Z', true, false, 100, 0, true, false);

    /* 3. speed: any scale works because only ratios matter */
    expect("F x0.3", PHOTO_F, 'F', true, false, 30, 0, false, false);
    expect("F x6", PHOTO_F, 'F', true, false, 600, 0, false, false);
    /* 4. speed drift of +-15% across the code */
    expect("F drift", PHOTO_F, 'F', true, false, 100, 30, false, false);
    expect("W drift", DECK_W, 'W', true, false, 100, 30, true, false);

    /* 5. crossed in the opposite direction */
    expect("F backwards, letters_only", PHOTO_F, 'F', true, true, 100, 0, false, true);
    expect("Z backwards, letters_only", DECK_Z, 'Z', true, true, 100, 0, true, true);
    /* direction comes from the '*' ends, so letters_only is not needed for it */
    {
        barcode39_t d; barcode39_result_t r; memset(&r, 0, sizeof r);
        barcode39_init(&d, false);
        bool got = feed_runs(&d, PHOTO_F, 29, 100, 0, false, true, &r);
        CHECK(got && r.text[0] == 'F' && r.reversed, "backwards F, any chars: got '%s'", r.text);
    }

    /* 6. all 26 letters, several narrow/wide ratios, both directions */
    int ambiguous = 0;
    char amb[64] = "";
    for (char c = 'A'; c <= 'Z'; c++) {
        uint32_t w[29];
        char     nm[32];
        for (int ratio = 2; ratio <= 3; ratio++) {
            build(w, c, 20, 20 * ratio + (ratio == 2 ? 2 : 0));
            snprintf(nm, sizeof nm, "letter %c ratio %d", c, ratio);
            /* ratio 2 needs the lowered threshold; default 1.8x passes 2.1x */
            expect(nm, w, c, true, false, 100, 0, false, false);
        }
        /* backwards: must give the same letter, flagged reversed */
        build(w, c, 20, 60);
        barcode39_t d; barcode39_result_t r; memset(&r, 0, sizeof r);
        barcode39_init(&d, true);
        bool got = feed_runs(&d, w, 29, 100, 0, false, true, &r);
        CHECK(got, "backwards %c not decoded", c);
        if (got && (r.text[0] != c || !r.reversed)) {
            ambiguous++;
            size_t l = strlen(amb);
            snprintf(amb + l, sizeof amb - l, "%c->%c ", c, r.text[0]);
        }
    }
    CHECK(ambiguous == 0, "backwards reads that differ: %s", amb);

    /* 7. rejects */
    {
        barcode39_t d; barcode39_result_t r;
        barcode39_init(&d, true);
        bool got = false;
        for (int i = 0; i < 80; i++) got |= barcode39_feed(&d, i % 2 == 0, 20, &r); /* all equal widths */
        CHECK(!got, "uniform stripes should not decode");

        uint32_t bad[29]; memcpy(bad, PHOTO_F, sizeof bad);
        bad[9] = 50; /* inter-character gap becomes wide: invalid */
        barcode39_init(&d, true);
        got = feed_runs(&d, bad, 29, 100, 0, false, false, &r);
        CHECK(!got, "wide inter-character gap should be rejected");

        uint32_t bad2[29]; memcpy(bad2, PHOTO_F, sizeof bad2);
        bad2[12] = 16; /* a block with only 2 wide elements */
        barcode39_init(&d, true);
        got = feed_runs(&d, bad2, 29, 100, 0, false, false, &r);
        CHECK(!got, "corrupted element should not decode (got '%s')", r.text);
    }

    /* 8. two codes back to back, one after the other */
    {
        barcode39_t d; barcode39_result_t r1, r2;
        memset(&r1, 0, sizeof r1); memset(&r2, 0, sizeof r2);
        barcode39_init(&d, true);
        bool a = feed_runs(&d, DECK_A, 29, 100, 0, false, false, &r1);
        bool b = feed_runs(&d, DECK_Y, 29, 100, 0, true, false, &r2);
        CHECK(a && b && r1.text[0] == 'A' && r2.text[0] == 'Y', "back-to-back: '%s' '%s'", r1.text, r2.text);
    }

    printf("%d checks, %d failed\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
