/* Host test for junction_map (editable barcode-letter -> junction-action
 * table).
 * gcc -std=c11 -Wall -Wextra -I. junction_map.c test/host/test_junction_map.c
 * -o /tmp/jm && /tmp/jm */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "junction_map.h"

static int s_failures;
#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

int main (void)
{
    /* Built-in defaults: A = left, F = U-turn, W = right, everything else no
     * instruction. */
    junction_map_reset();
    CHECK(JUNCTION_LEFT == junction_action_for_letter('A'), "default A = LEFT");
    CHECK(JUNCTION_UTURN == junction_action_for_letter('F'),
          "default F = U-TURN");
    CHECK(JUNCTION_RIGHT == junction_action_for_letter('W'),
          "default W = RIGHT");
    CHECK(JUNCTION_NONE == junction_action_for_letter('B'), "default B = NONE");
    CHECK(JUNCTION_LEFT == junction_action_for_letter('a'),
          "lower case a = LEFT");

    /* Characters that are not letters. */
    CHECK(JUNCTION_NONE == junction_action_for_letter('1'), "digit = NONE");
    CHECK(JUNCTION_NONE == junction_action_for_letter('*'), "symbol = NONE");
    CHECK(JUNCTION_NONE == junction_action_for_letter('\0'), "NUL = NONE");

    /* Replace the whole table: A = L, B = S, C = R, D = U, the other 22 letters
     * = N. */
    CHECK(junction_map_apply("LSRUNNNNNNNNNNNNNNNNNNNNNN", 26) == 0,
          "apply valid map");
    CHECK(JUNCTION_LEFT == junction_action_for_letter('A'), "map A = LEFT");
    CHECK(JUNCTION_STRAIGHT == junction_action_for_letter('B'),
          "map B = STRAIGHT");
    CHECK(JUNCTION_RIGHT == junction_action_for_letter('C'), "map C = RIGHT");
    CHECK(JUNCTION_UTURN == junction_action_for_letter('D'), "map D = U-TURN");
    CHECK(JUNCTION_NONE == junction_action_for_letter('W'), "map W = NONE");

    /* Bad input is rejected and leaves the table unchanged. */
    CHECK(junction_map_apply("LSR", 3) == -1, "reject wrong length");
    CHECK(junction_map_apply("LSRUNNNNNNNNNNNNNNNNNNNNNX", 26) == -1,
          "reject bad character");
    CHECK(JUNCTION_STRAIGHT == junction_action_for_letter('B'),
          "table unchanged after rejects");

    /* Reset restores the defaults. */
    junction_map_reset();
    CHECK((JUNCTION_LEFT == junction_action_for_letter('A'))
          && (JUNCTION_NONE == junction_action_for_letter('B')),
          "reset restores defaults");

    CHECK(strcmp(junction_action_name(JUNCTION_RIGHT), "RIGHT") == 0,
          "name RIGHT");
    CHECK(strcmp(junction_action_name(JUNCTION_NONE), "NONE") == 0,
          "name NONE");

    printf("%s\n", 0 == s_failures ? "ALL PASSED" : "SOME FAILED");
    return 0 == s_failures ? 0 : 1;
}

/*** end of file ***/
