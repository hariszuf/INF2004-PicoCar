/*
 * barcode39 - Code 39 decoder for the PicoCar single-IR-sensor barcode.
 *
 * Added to Buddy 3 by Afiq (with Claude). Host-tested; not yet run on the car.
 *
 * Hardware independent: no Pico SDK, no RTOS. The caller measures how long the
 * IR sensor stays black or white and feeds those "runs" in, in any unit
 * (microseconds, timer ticks, or encoder counts). Only RATIOS matter, so the
 * decoder tolerates different speeds and slow speed drift.
 *
 * Barcode format expected: Code 39, '*' + text + '*', 3:1-ish wide/narrow.
 * Decoded text for the track barcodes is a single letter A-Z.
 */
#ifndef BARCODE39_H
#define BARCODE39_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define B39_MAX_CHARS    8                          /* data characters, excluding the two '*' */
#define B39_MAX_ELEMENTS (10 * (B39_MAX_CHARS + 2) - 1)

/* Why the last complete-looking barcode was rejected (see barcode39_last_fail()). */
#define B39_FAIL_NONE    0u
#define B39_FAIL_LENGTH  1u  /* wrong number of bars/gaps */
#define B39_FAIL_RATIO   2u  /* a character's wide bars were not clearly wider than its narrow ones */
#define B39_FAIL_PATTERN 3u  /* 3 wide bars, but not a valid Code 39 character */
#define B39_FAIL_ENDS    4u  /* start/stop '*' missing */
#define B39_FAIL_GAP     5u  /* gap between characters too wide */
#define B39_FAIL_NONLETTER 6u /* valid Code 39, but not a letter A-Z */

typedef struct {
    uint8_t min_wide_ratio_x10; /* wide must be >= this/10 x narrow (default 14 = 1.4x) */
    bool    letters_only;       /* accept only A-Z; enables wrong-direction recovery   */
} barcode39_cfg_t;

typedef struct {
    char    text[B39_MAX_CHARS + 1]; /* decoded characters, NUL terminated                */
    uint8_t length;
    bool    reversed;                /* true if the code was crossed in the opposite way */
} barcode39_result_t;

typedef struct {
    barcode39_cfg_t cfg;
    uint32_t        w[B39_MAX_ELEMENTS]; /* element widths, index 0 is always black */
    uint8_t         n;
} barcode39_t;

/* letters_only = true: only A-Z results are returned (anything else is rejected).
 * Direction is detected automatically from the start/stop '*', which looks
 * different when read backwards, so a code crossed either way gives the same
 * text; result.reversed tells you which way the car crossed it. */
void barcode39_init(barcode39_t *d, bool letters_only);
void barcode39_reset(barcode39_t *d);

/* Feed one run: is_black = sensor saw black for `width` units, then changed.
 * Returns true and fills *out when a complete, valid barcode was decoded.
 * Runs must alternate black/white; a repeated colour is merged into the last run.
 * Zero-width runs are ignored. Safe to call from a task, not from an ISR that
 * could race with another feeder. */
bool barcode39_feed(barcode39_t *d, bool is_black, uint32_t width, barcode39_result_t *out);

/* Decode a complete element array (black first, black last, 10k+9 entries).
 * Used internally and handy for tests. */
bool barcode39_decode_elements(const uint32_t *w, uint8_t count, const barcode39_cfg_t *cfg,
                               barcode39_result_t *out);

/* Diagnostics: reason and count of decode attempts that looked like a barcode but failed. */
uint8_t barcode39_last_fail(void);
uint32_t barcode39_fail_count(void);

#endif
