/**
 * @file barcode_scan.c
 * @brief Barcode sensor to Code 39 letters. See barcode_scan.h.
 */
#include "barcode_scan.h"

/* Packed yes/no flags. On the host (no T-Kernel headers) the lock does nothing.
 */
#if !defined(__has_include)
#define ROBOT_FLAGS_IRQ_T unsigned
#define ROBOT_FLAGS_LOCK(s) ((void)((s) = 0u))
#define ROBOT_FLAGS_UNLOCK(s) ((void)(s))
#elif !__has_include(<tk/tkernel.h>)
#define ROBOT_FLAGS_IRQ_T unsigned
#define ROBOT_FLAGS_LOCK(s) ((void)((s) = 0u))
#define ROBOT_FLAGS_UNLOCK(s) ((void)(s))
#endif
#include "../robot_flags.h"

static barcode39_t s_decoder;
/* s_have_result is set by the barcode task and cleared by whoever calls
 * barcode_scan_take(), so all four flags
 * share one word that is only changed with locked updates. */
#define BS_STARTED (1u << 0)
#define BS_LEVEL (1u << 1)          /* accepted level, set = black */
#define BS_PENDING (1u << 2)        /* a different level is being tested */
#define BS_HAVE_RESULT (1u << 3)
static robot_flags_t s_flags;
#define BS(mask) robot_flags_test(&s_flags, (mask))
static uint32_t s_level_since;  /* when the accepted level began */
static uint32_t s_pending_since;
static uint32_t s_last_edge_us;
static barcode39_result_t s_result;
static uint32_t s_count;

__attribute__((weak)) void barcode_scan_on_decoded(const barcode39_result_t
        * p_result)
{
    (void)p_result;
}

__attribute__((weak)) void barcode_scan_on_run(bool black, uint32_t width_us)
{
    (void)black;
    (void)width_us;
}

void barcode_scan_init (void)
{
    barcode39_init(&s_decoder, true);   /* the track barcodes are letters A-Z */
    robot_flags_update(&s_flags, BS_STARTED | BS_PENDING | BS_HAVE_RESULT, 0u);
    s_count = 0u;
}

void barcode_scan_update (bool black, uint32_t now_us)
{
    barcode39_result_t decoded;

    if (!BS(BS_STARTED))
    {
        robot_flags_update(&s_flags, BS_STARTED | BS_LEVEL,
                           BS_STARTED | (black ? BS_LEVEL : 0u));
        s_level_since = now_us;
        s_last_edge_us = now_us;
        return;
    }

    if (black == BS(BS_LEVEL))
    {
        /* the other level was only a glitch */
        robot_flags_clear(&s_flags, BS_PENDING);
    }
    else if (!BS(BS_PENDING))
    {
        robot_flags_set(&s_flags, BS_PENDING);
        s_pending_since = now_us;
    }
    else if ((uint32_t)(now_us - s_pending_since) >= BARCODE_DEBOUNCE_US)
    {
        /* Real edge. It happened when the new level first appeared. */
        uint32_t width = s_pending_since - s_level_since;

        barcode_scan_on_run(BS(BS_LEVEL), width);

        if (barcode39_feed(&s_decoder, BS(BS_LEVEL), width, &decoded))
        {
            s_result = decoded;
            robot_flags_set(&s_flags, BS_HAVE_RESULT);
            s_count++;
            barcode_scan_on_decoded(&s_result);
        }

        robot_flags_update(&s_flags, BS_LEVEL | BS_PENDING,
                           black ? BS_LEVEL : 0u);
        s_level_since = s_pending_since;
        s_last_edge_us = now_us;
    }

    if ((s_decoder.n > 0u)
        && ((uint32_t)(now_us - s_last_edge_us) > BARCODE_IDLE_RESET_US))
    {
        barcode39_reset(&s_decoder);    /* stale partial read */
    }
}

bool barcode_scan_take (barcode39_result_t * out)
{
    if (!BS(BS_HAVE_RESULT))
    {
        return false;
    }

    if (out != (barcode39_result_t *)0)
    {
        *out = s_result;
    }

    robot_flags_clear(&s_flags, BS_HAVE_RESULT);
    return true;
}

uint32_t barcode_scan_count (void)
{
    return s_count;
}

uint32_t barcode_scan_fail_count (void)
{
    return barcode39_fail_count();
}

uint8_t barcode_scan_last_fail (void)
{
    return barcode39_last_fail();
}

/*** end of file ***/
