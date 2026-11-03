/* barcode39 - Code 39 decoder (draft). See barcode39.h. */
#include "barcode39.h"

#include <string.h>

/* Code 39 patterns: 9 elements bar,space,bar,space,... 'w' = wide, 'n' = narrow.
 * Verified against the six track barcodes in BARCODE.pptx (A, F, Z, W, X, Y). */
typedef struct {
    const char *pat;
    char        ch;
} b39_entry_t;

static const b39_entry_t TABLE[] = {
    {"nwnnwnwnn", '*'},
    {"wnnnnwnnw", 'A'}, {"nnwnnwnnw", 'B'}, {"wnwnnwnnn", 'C'}, {"nnnnwwnnw", 'D'},
    {"wnnnwwnnn", 'E'}, {"nnwnwwnnn", 'F'}, {"nnnnnwwnw", 'G'}, {"wnnnnwwnn", 'H'},
    {"nnwnnwwnn", 'I'}, {"nnnnwwwnn", 'J'}, {"wnnnnnnww", 'K'}, {"nnwnnnnww", 'L'},
    {"wnwnnnnwn", 'M'}, {"nnnnwnnww", 'N'}, {"wnnnwnnwn", 'O'}, {"nnwnwnnwn", 'P'},
    {"nnnnnnwww", 'Q'}, {"wnnnnnwwn", 'R'}, {"nnwnnnwwn", 'S'}, {"nnnnwnwwn", 'T'},
    {"wwnnnnnnw", 'U'}, {"nwwnnnnnw", 'V'}, {"wwwnnnnnn", 'W'}, {"nwnnwnnnw", 'X'},
    {"wwnnwnnnn", 'Y'}, {"nwwnwnnnn", 'Z'},
    /* digits and symbols: only needed to recognise a letter read backwards */
    {"nnnwwnwnn", '0'}, {"wnnwnnnnw", '1'}, {"nnwwnnnnw", '2'}, {"wnwwnnnnn", '3'},
    {"nnnwwnnnw", '4'}, {"wnnwwnnnn", '5'}, {"nnwwwnnnn", '6'}, {"nnnwnnwnw", '7'},
    {"wnnwnnwnn", '8'}, {"nnwwnnwnn", '9'},
    {"nwnnnnwnw", '-'}, {"wwnnnnwnn", '.'}, {"nwwnnnwnn", ' '},
};
#define TABLE_LEN (sizeof(TABLE) / sizeof(TABLE[0]))

static uint8_t s_last_fail;     /* B39_FAIL_* of the most recent failed decode */
static uint32_t s_fail_count;

/* Classify 9 widths: the 3 widest are 'wide', the rest 'narrow'. Needs no fixed
 * timing; fails if wide is not clearly wider than narrow. Returns the character
 * or 0 if the pattern is invalid. *thr = narrow/wide decision threshold. */
static char classify_block(const uint32_t *b, uint8_t ratio_x10, uint32_t *thr)
{
    bool     wide[9] = {false};
    uint32_t min_wide = UINT32_MAX;
    uint32_t max_narrow = 0;

    for (int k = 0; k < 3; k++) {
        int best = -1;
        for (int i = 0; i < 9; i++) {
            if (!wide[i] && (best < 0 || b[i] > b[best])) {
                best = i;
            }
        }
        wide[best] = true;
    }
    for (int i = 0; i < 9; i++) {
        if (wide[i]) {
            if (b[i] < min_wide) min_wide = b[i];
        } else {
            if (b[i] > max_narrow) max_narrow = b[i];
        }
    }
    if (max_narrow == 0 || (uint64_t)min_wide * 10u < (uint64_t)max_narrow * ratio_x10) {
        return 0;
    }

    char pat[10];
    for (int i = 0; i < 9; i++) pat[i] = wide[i] ? 'w' : 'n';
    pat[9] = '\0';
    if (thr) *thr = (min_wide + max_narrow) / 2u;

    for (size_t t = 0; t < TABLE_LEN; t++) {
        if (strcmp(TABLE[t].pat, pat) == 0) return TABLE[t].ch;
    }
    return '?';   /* 3 wide, but not a Code 39 pattern */
}

static bool decode_dir(const uint32_t *w, uint8_t n, uint8_t ratio_x10, char *text, uint8_t *len)
{
    if (n % 10 != 9) { s_last_fail = B39_FAIL_LENGTH; return false; }
    uint8_t blocks = (uint8_t)((n + 1) / 10);
    if (blocks < 3 || blocks - 2 > B39_MAX_CHARS) { s_last_fail = B39_FAIL_LENGTH; return false; }

    char     ch[B39_MAX_CHARS + 2];
    uint32_t thr[B39_MAX_CHARS + 2];

    for (uint8_t k = 0; k < blocks; k++) {
        ch[k] = classify_block(&w[10 * k], ratio_x10, &thr[k]);
        if (ch[k] == 0) { s_last_fail = B39_FAIL_RATIO; return false; }
        if (ch[k] == '?') { s_last_fail = B39_FAIL_PATTERN; return false; }
    }
    if (ch[0] != '*' || ch[blocks - 1] != '*') { s_last_fail = B39_FAIL_ENDS; return false; }
    for (uint8_t k = 1; k < blocks - 1; k++) {
        if (ch[k] == '*') { s_last_fail = B39_FAIL_ENDS; return false; }
    }
    /* inter-character gaps must be narrow */
    for (uint8_t k = 0; k + 1 < blocks; k++) {
        uint32_t g = w[10 * k + 9];
        if (g >= thr[k] || g >= thr[k + 1]) { s_last_fail = B39_FAIL_GAP; return false; }
    }
    for (uint8_t k = 1; k < blocks - 1; k++) text[k - 1] = ch[k];
    *len = (uint8_t)(blocks - 2);
    text[*len] = '\0';
    return true;
}

static bool all_letters(const char *t, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        if (t[i] < 'A' || t[i] > 'Z') return false;
    }
    return len > 0;
}

bool barcode39_decode_elements(const uint32_t *w, uint8_t count, const barcode39_cfg_t *cfg,
                               barcode39_result_t *out)
{
    char    text[B39_MAX_CHARS + 1];
    uint8_t len = 0;
    uint8_t fwd_fail;
    bool    fwd_ok;
    bool    rev_ok;

    fwd_ok = decode_dir(w, count, cfg->min_wide_ratio_x10, text, &len);
    fwd_fail = s_last_fail;
    if (fwd_ok && cfg->letters_only && !all_letters(text, len)) {
        fwd_ok = false;
        fwd_fail = B39_FAIL_NONLETTER;
    }
    if (fwd_ok) {
        if (out) {
            memcpy(out->text, text, (size_t)len + 1);
            out->length = len;
            out->reversed = false;
        }
        return true;
    }

    uint32_t rev[B39_MAX_ELEMENTS];
    for (uint8_t i = 0; i < count; i++) rev[i] = w[count - 1 - i];
    rev_ok = decode_dir(rev, count, cfg->min_wide_ratio_x10, text, &len);
    if (rev_ok && cfg->letters_only && !all_letters(text, len)) {
        rev_ok = false;
        s_last_fail = B39_FAIL_NONLETTER;
    }
    if (rev_ok) {
        if (out) {
            memcpy(out->text, text, (size_t)len + 1);
            out->length = len;
            out->reversed = true;
        }
        return true;
    }
    /* Report the more informative reason: when the forward read only failed on the
       '*' ends, the code is probably backwards and the reverse reason is the real one. */
    if (fwd_fail != B39_FAIL_ENDS) {
        s_last_fail = fwd_fail;
    }
    s_fail_count++;
    return false;
}

uint8_t barcode39_last_fail(void) { return s_last_fail; }
uint32_t barcode39_fail_count(void) { return s_fail_count; }

void barcode39_reset(barcode39_t *d)
{
    d->n = 0;
}

void barcode39_init(barcode39_t *d, bool letters_only)
{
    d->cfg.min_wide_ratio_x10 = 14;
    d->cfg.letters_only = letters_only;
    d->n = 0;
}

static void drop2(barcode39_t *d)
{
    uint8_t keep = (d->n >= 2) ? (uint8_t)(d->n - 2) : 0;
    memmove(d->w, d->w + 2, (size_t)keep * sizeof(d->w[0]));
    d->n = keep;
}

/* A barcode starts and ends with '*'. Read backwards, each '*' pattern shows up
 * mirrored, which happens to be the pattern of 'P'. So an end block that is '*'
 * (forward) or 'P' (backwards) marks a possible barcode, and also tells us which
 * way the car crossed it. */
static bool block_is_end(const barcode39_t *d, const uint32_t *b)
{
    char c = classify_block(b, d->cfg.min_wide_ratio_x10, NULL);
    return c == '*' || c == 'P';
}

bool barcode39_feed(barcode39_t *d, bool is_black, uint32_t width, barcode39_result_t *out)
{
    if (width == 0) return false;

    if (d->n == 0 && !is_black) return false; /* window always starts on a black bar */

    bool expect_black = (d->n % 2) == 0;
    if (d->n > 0 && is_black != expect_black) {
        d->w[d->n - 1] += width; /* same colour twice: merge */
        return false;
    }
    d->w[d->n++] = width;

    for (;;) {
        /* re-sync: the window must start with a '*' (or mirrored '*') block */
        while (d->n >= 9 && !block_is_end(d, d->w)) drop2(d);

        if (d->n >= 29 && (d->n % 10) == 9 && block_is_end(d, &d->w[d->n - 9])) {
            if (barcode39_decode_elements(d->w, d->n, &d->cfg, out)) {
                d->n = 0;
                return true;
            }
            drop2(d);
            continue;
        }
        if (d->n >= B39_MAX_ELEMENTS) {
            drop2(d);
            continue;
        }
        break;
    }
    return false;
}
