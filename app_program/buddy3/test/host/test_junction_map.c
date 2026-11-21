/* Host test for junction_map (editable barcode-letter -> junction-action table).
 *   gcc -std=c11 -Wall -Wextra -I. junction_map.c test/host/test_junction_map.c -o /tmp/jm && /tmp/jm */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "junction_map.h"

static int s_failures;
#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

int main(void)
{
    /* Built-in defaults: A = left, F = U-turn, W = right, everything else no instruction. */
    junction_map_reset();
    CHECK(junction_action_for_letter('A') == JUNCTION_LEFT, "default A = LEFT");
    CHECK(junction_action_for_letter('F') == JUNCTION_UTURN, "default F = U-TURN");
    CHECK(junction_action_for_letter('W') == JUNCTION_RIGHT, "default W = RIGHT");
    CHECK(junction_action_for_letter('B') == JUNCTION_NONE, "default B = NONE");
    CHECK(junction_action_for_letter('a') == JUNCTION_LEFT, "lower case a = LEFT");

    /* Characters that are not letters. */
    CHECK(junction_action_for_letter('1') == JUNCTION_NONE, "digit = NONE");
    CHECK(junction_action_for_letter('*') == JUNCTION_NONE, "symbol = NONE");
    CHECK(junction_action_for_letter('\0') == JUNCTION_NONE, "NUL = NONE");

    /* Replace the whole table: A = L, B = S, C = R, D = U, the other 22 letters = N. */
    CHECK(junction_map_apply("LSRUNNNNNNNNNNNNNNNNNNNNNN", 26) == 0, "apply valid map");
    CHECK(junction_action_for_letter('A') == JUNCTION_LEFT, "map A = LEFT");
    CHECK(junction_action_for_letter('B') == JUNCTION_STRAIGHT, "map B = STRAIGHT");
    CHECK(junction_action_for_letter('C') == JUNCTION_RIGHT, "map C = RIGHT");
    CHECK(junction_action_for_letter('D') == JUNCTION_UTURN, "map D = U-TURN");
    CHECK(junction_action_for_letter('W') == JUNCTION_NONE, "map W = NONE");

    /* Bad input is rejected and leaves the table unchanged. */
    CHECK(junction_map_apply("LSR", 3) == -1, "reject wrong length");
    CHECK(junction_map_apply("LSRUNNNNNNNNNNNNNNNNNNNNNX", 26) == -1, "reject bad character");
    CHECK(junction_action_for_letter('B') == JUNCTION_STRAIGHT, "table unchanged after rejects");

    /* Reset restores the defaults. */
    junction_map_reset();
    CHECK(junction_action_for_letter('A') == JUNCTION_LEFT && junction_action_for_letter('B') == JUNCTION_NONE,
          "reset restores defaults");

    CHECK(strcmp(junction_action_name(JUNCTION_RIGHT), "RIGHT") == 0, "name RIGHT");
    CHECK(strcmp(junction_action_name(JUNCTION_NONE), "NONE") == 0, "name NONE");

    printf("%s\n", s_failures == 0 ? "ALL PASSED" : "SOME FAILED");
    return s_failures == 0 ? 0 : 1;
}
