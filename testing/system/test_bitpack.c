/* Host test for bitpack.h and the packed Buddy 5 scan (13 slots x (13-bit distance + 2-bit state)).
 *   gcc -std=gnu11 -Wall -Wextra -Iapp_program -Iapp_program/buddy5 testing/system/test_bitpack.c -o /tmp/bp && /tmp/bp */
#include <stdio.h>
#include <string.h>
#include "bitpack.h"
#include "buddy5_core.h"

static int s_failures;
#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

int main(void)
{
    uint8_t buf[16];
    int ok = 1;

    /* Every width 1..32 at every offset 0..7 round-trips and leaves the neighbouring bits alone. */
    for (unsigned width = 1u; width <= 32u && ok; width++) {
        for (unsigned off = 0u; off < 8u && ok; off++) {
            uint32_t mask = width == 32u ? 0xFFFFFFFFu : ((1u << width) - 1u);
            uint32_t values[3] = { 0u, mask, 0xA5A5A5A5u & mask };
            for (int v = 0; v < 3 && ok; v++) {
                memset(buf, 0x5A, sizeof buf);
                uint8_t before[16]; memcpy(before, buf, sizeof buf);
                bitpack_put(buf, off, width, values[v]);
                if (bitpack_get(buf, off, width) != values[v]) ok = 0;
                for (unsigned b = 0u; b < 16u * 8u; b++) {
                    if (b >= off && b < off + width) continue;
                    if (bitpack_get(buf, b, 1u) != bitpack_get(before, b, 1u)) ok = 0;
                }
            }
        }
    }
    CHECK(ok, "widths 1..32 at offsets 0..7: round trip, neighbours untouched");

    CHECK(BITPACK_BYTES(1u) == 1u && BITPACK_BYTES(8u) == 1u && BITPACK_BYTES(9u) == 2u, "BITPACK_BYTES rounds up");

    /* Packed scan: 13 slots, edge values, no slot disturbs another. */
    b5_scan_t scan;
    memset(&scan, 0, sizeof scan);
    const uint32_t dist[B5_SCAN_POINTS] = { 0u, 1u, 80u, 300u, 4116u, 8191u, 4095u, 4096u, 150u, 2047u, 2048u, 7u, 8190u };
    const uint8_t state[B5_SCAN_POINTS] = { 0u, 1u, 2u, 1u, 2u, 1u, 0u, 2u, 1u, 2u, 0u, 1u, 2u };
    for (int i = 0; i < B5_SCAN_POINTS; i++) b5_scan_set(&scan, i, state[i], dist[i]);
    ok = 1;
    for (int i = 0; i < B5_SCAN_POINTS; i++)
        if (b5_scan_dist(&scan, i) != dist[i] || b5_scan_state(&scan, i) != state[i]) ok = 0;
    CHECK(ok, "13 scan slots keep their own distance and state");

    b5_scan_set(&scan, 6, B5_SCAN_ECHO, 123u);
    ok = b5_scan_dist(&scan, 6) == 123u && b5_scan_state(&scan, 6) == B5_SCAN_ECHO;
    for (int i = 0; i < B5_SCAN_POINTS; i++)
        if (i != 6 && (b5_scan_dist(&scan, i) != dist[i] || b5_scan_state(&scan, i) != state[i])) ok = 0;
    CHECK(ok, "rewriting one slot leaves the other 12 unchanged");

    b5_scan_set(&scan, 0, B5_SCAN_ECHO, 100000u);
    CHECK(b5_scan_dist(&scan, 0) == B5_SCAN_DIST_MAX, "a distance above 8191 mm is clamped, not wrapped");

    CHECK(sizeof(b5_scan_t) == 25u, "packed scan is 25 bytes (was 39)");
    CHECK(sizeof(b5_feedback_t) <= 20u, "feedback struct is 20 bytes or less (was 28)");
    CHECK(sizeof(b5_profile_t) <= 52u, "profile struct is 52 bytes or less (was 56)");

    printf("%s\n", s_failures == 0 ? "ALL PASSED" : "SOME FAILED");
    return s_failures == 0 ? 0 : 1;
}
