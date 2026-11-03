/**
 * @file barcode_scan.c
 * @brief Barcode sensor to Code 39 letters. See barcode_scan.h.
 */
#include "barcode_scan.h"

static barcode39_t s_decoder;
static bool s_started;
static bool s_level;            /* accepted level, true = black */
static uint32_t s_level_since;  /* when the accepted level began */
static bool s_pending;          /* a different level is being tested */
static uint32_t s_pending_since;
static uint32_t s_last_edge_us;
static barcode39_result_t s_result;
static bool s_have_result;
static uint32_t s_count;

__attribute__((weak)) void barcode_scan_on_decoded(const barcode39_result_t *p_result)
{
    (void)p_result;
}

__attribute__((weak)) void barcode_scan_on_run(bool black, uint32_t width_us)
{
    (void)black;
    (void)width_us;
}

void barcode_scan_init(void)
{
    barcode39_init(&s_decoder, true);   /* the track barcodes are letters A-Z */
    s_started = false;
    s_pending = false;
    s_have_result = false;
    s_count = 0u;
}

void barcode_scan_update(bool black, uint32_t now_us)
{
    barcode39_result_t decoded;

    if (!s_started)
    {
        s_started = true;
        s_level = black;
        s_level_since = now_us;
        s_last_edge_us = now_us;
        return;
    }

    if (black == s_level)
    {
        s_pending = false;      /* the other level was only a glitch */
    }
    else if (!s_pending)
    {
        s_pending = true;
        s_pending_since = now_us;
    }
    else if ((uint32_t)(now_us - s_pending_since) >= BARCODE_DEBOUNCE_US)
    {
        /* Real edge. It happened when the new level first appeared. */
        uint32_t width = s_pending_since - s_level_since;

        barcode_scan_on_run(s_level, width);
        if (barcode39_feed(&s_decoder, s_level, width, &decoded))
        {
            s_result = decoded;
            s_have_result = true;
            s_count++;
            barcode_scan_on_decoded(&s_result);
        }
        s_level = black;
        s_level_since = s_pending_since;
        s_pending = false;
        s_last_edge_us = now_us;
    }

    if ((s_decoder.n > 0u) && ((uint32_t)(now_us - s_last_edge_us) > BARCODE_IDLE_RESET_US))
    {
        barcode39_reset(&s_decoder);    /* stale partial read */
    }
}

bool barcode_scan_take(barcode39_result_t *out)
{
    if (!s_have_result)
    {
        return false;
    }
    if (out != (barcode39_result_t *)0)
    {
        *out = s_result;
    }
    s_have_result = false;
    return true;
}

uint32_t barcode_scan_count(void)
{
    return s_count;
}

uint32_t barcode_scan_fail_count(void)
{
    return barcode39_fail_count();
}

uint8_t barcode_scan_last_fail(void)
{
    return barcode39_last_fail();
}
