/* Host test for junction_map.
 *   gcc -std=c11 -Wall -Wextra -I. junction_map.c test/host/test_junction_map.c -o /tmp/jm && /tmp/jm */
#include <stdio.h>
#include <string.h>
#include "junction_map.h"

static int s_failures;
#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

int main(void)
{
    const char *left = "ADGJMPSVY";
    const char *straight = "BEHKNQTWZ";
    const char *right = "CFILORUX";
    int ok = 1;

    for (const char *p = left; *p; p++)
        if (junction_action_for_letter(*p) != JUNCTION_LEFT) ok = 0;
    CHECK(ok, "A D G J M P S V Y = LEFT");
    ok = 1;
    for (const char *p = straight; *p; p++)
        if (junction_action_for_letter(*p) != JUNCTION_STRAIGHT) ok = 0;
    CHECK(ok, "B E H K N Q T W Z = STRAIGHT");
    ok = 1;
    for (const char *p = right; *p; p++)
        if (junction_action_for_letter(*p) != JUNCTION_RIGHT) ok = 0;
    CHECK(ok, "C F I L O R U X = RIGHT");

    CHECK(junction_action_for_letter('A') == JUNCTION_LEFT, "A = LEFT");
    CHECK(junction_action_for_letter('B') == JUNCTION_STRAIGHT, "B = STRAIGHT");
    CHECK(junction_action_for_letter('C') == JUNCTION_RIGHT, "C = RIGHT");
    CHECK(junction_action_for_letter('D') == JUNCTION_LEFT, "D = LEFT");
    CHECK(junction_action_for_letter('Z') == JUNCTION_STRAIGHT, "Z = STRAIGHT");
    CHECK(junction_action_for_letter('y') == JUNCTION_LEFT, "lower case y = LEFT");
    CHECK(junction_action_for_letter('1') == JUNCTION_NONE, "digit = NONE");
    CHECK(junction_action_for_letter('*') == JUNCTION_NONE, "symbol = NONE");
    CHECK(junction_action_for_letter('\0') == JUNCTION_NONE, "NUL = NONE");
    CHECK(strcmp(junction_action_name(JUNCTION_RIGHT), "RIGHT") == 0, "name RIGHT");
    CHECK(strcmp(junction_action_name(JUNCTION_NONE), "NONE") == 0, "name NONE");

    printf("%s\n", s_failures == 0 ? "ALL PASSED" : "SOME FAILED");
    return s_failures == 0 ? 0 : 1;
}
