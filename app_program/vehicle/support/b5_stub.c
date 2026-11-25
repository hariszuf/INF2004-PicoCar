/* Buddy 5 (obstacle detection) has been removed and will be rewritten.
 * This file keeps the vehicle controller building and line following: it
 * reports "no obstacle" and does nothing.
 * Replace it with the new Buddy 5 implementation of the scan_link.h interface.
 */
#ifdef B5_FULL_TEST
#include "../../barrc_types.h"
#include <tk/tkernel.h>
#include <stdint.h>
#include <string.h>
#include "scan_link.h"

#define B5_STUB_REG(a) (*(volatile uint32_t *)(uintptr_t)(a))

uint32_t buddy5_clock_ms (void)
{
    uint32_t hi;
    uint32_t lo;
    uint32_t again;

    do
    {
        hi = B5_STUB_REG(0x40054024u);
        lo = B5_STUB_REG(0x40054028u);
        again = B5_STUB_REG(0x40054024u);
    } while (hi != again);
    return (uint32_t)((((uint64_t)hi << 32) | lo) / 1000u);
}

const char * b5_state_name (b5_state_t s)
{
    static const char * const names[] = {"MONITORING", "WAIT_STOP", "SCANNING",
        "PROCESSING", "AVOIDING", "LINE_RECOVERY", "ERROR"};
    return (uint32_t)s < sizeof(names) / sizeof(names[0]) ? names[s]
            : "UNKNOWN";
}

int32_t line_scan_start (void)
{
    return E_OK;
}

void line_scan_reset (void)
{
}

void line_scan_abort (void)
{
}

void line_scan_get_status (line_scan_status_t * status)
{
    uint32_t now = buddy5_clock_ms();
    memset(status, 0, sizeof(*status));
    status->state = B5_MONITORING;
    status->fault = B5_FAULT_NONE;
    status->front_clearance_mm = B5_CLEAR_FRONT_MM;
    status->front_valid = true;
    status->front_timestamp_ms = now;
    status->heartbeat_ms = now;
}
#endif

/*** end of file ***/
