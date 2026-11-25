/* Host test of the start/stop policy (auto_start.h). gcc -std=c11 -Wall -Wextra
 * -I../../vehicle test_auto_start.c */
#include <stdio.h>
#include "auto_start.h"
static int fails;
#define CHECK(c) do{ if(!(c)){ printf("FAIL line %d: %s\n",__LINE__,#c); \
    fails++; } }while(0)
static uint32_t now;
/* Run the controller policy for ms milliseconds; returns the last non-trivial
 * action seen. */
static auto_action_t run (auto_start_t * s, uint32_t ms, bool ready,
                          int * resets,
                          int * starts)
{
    auto_action_t last = AUTO_NOTHING;

    for (uint32_t idx = 0; idx < ms; idx += 20)
    {
        now += 20;
        auto_action_t a = auto_start_update(s, now, true, ready);

        if (AUTO_RESET == a)
        {
            ++*resets;
        }

        if (AUTO_START == a)
        {
            ++*starts;
        }

        if (a)
        {
            last = a;
        }
    }

    return last;
}

int main (void)
{
    auto_start_t s;
    int rs = 0;
    int st = 0;
    /* power-on starts automatically after the gates */
    now = 1000;
    auto_start_init(&s, now);
    CHECK(EXEC_STARTUP_WAIT == auto_start_exec_state(&s));
    run(&s, B5_BOOT_DELAY_MS + 40, false, &rs, &st);
    CHECK((1 == rs) && (0 == st)
          && (EXEC_WAIT_READY == auto_start_exec_state(&s)));
    run(&s, 2000, false, &rs, &st);
    /* MQTT/readiness gate not passed: stationary */
    CHECK(0 == st);
    run(&s, 200, true, &rs, &st);
    CHECK((1 == st) && (EXEC_RUNNING == auto_start_exec_state(&s)));
    /* START while running: no reset, no duplicate */
    CHECK(START_ALREADY_ACTIVE == auto_start_request(&s, now));
    run(&s, 400, true, &rs, &st);
    CHECK((1 == rs) && (1 == st) && (AUTO_RUNNING == s.phase));
    /* STOP holds the car stopped */
    auto_start_stop(&s);
    run(&s, 3000, true, &rs, &st);
    CHECK((1 == st) && (EXEC_STOPPED == auto_start_exec_state(&s)));
    /* START after STOP: reset, wait for readiness, then run again */
    CHECK((START_ACCEPTED == auto_start_request(&s, now))
          && (EXEC_RESTART_PENDING == auto_start_exec_state(&s)));
    run(&s, 100, false, &rs, &st);
    CHECK((2 == rs) && (1 == st));
    run(&s, 1000, false, &rs, &st);
    CHECK((1 == st) && (EXEC_RESTART_PENDING == auto_start_exec_state(&s)));
    run(&s, 400, true, &rs, &st);
    CHECK((2 == st) && (EXEC_RUNNING == auto_start_exec_state(&s))
          && (!s.restarting));
    /* recoverable uncommanded stop is recorded and retryable */
    auto_start_halt(&s, 9, false);
    CHECK((AUTO_LATCHED == s.phase) && (9 == s.reason));
    run(&s, 3000, true, &rs, &st);
    CHECK(2 == st);
    CHECK(START_ACCEPTED == auto_start_request(&s, now));
    run(&s, 2000, true, &rs, &st);
    CHECK(3 == st);
    /* STOP during a pending restart wins */
    auto_start_halt(&s, 4, false);
    CHECK(START_ACCEPTED == auto_start_request(&s, now));
    auto_start_stop(&s);
    run(&s, 3000, true, &rs, &st);
    CHECK((3 == st) && (AUTO_LATCHED == s.phase));
    /* STOP during startup prevents automatic movement */
    now = 50000;
    auto_start_init(&s, now);
    rs = st = 0;
    run(&s, 100, true, &rs, &st);
    auto_start_stop(&s);
    run(&s, 5000, true, &rs, &st);
    CHECK(0 == st);
    /* persistent fault (motor stall): STOP does not downgrade it, START
     * rejected,
     * stays stopped */
    now = 90000;
    auto_start_init(&s, now);
    run(&s, B5_BOOT_DELAY_MS + B5_READY_DELAY_MS + 100, true, &rs, &st);
    auto_start_halt(&s, 8, true);
    CHECK((AUTO_FAULTED == s.phase)
          && (EXEC_FAULTED == auto_start_exec_state(&s)));
    auto_start_stop(&s);
    CHECK(AUTO_FAULTED == s.phase);
    auto_start_halt(&s, 4, false);
    CHECK((AUTO_FAULTED == s.phase) && (8 == s.reason));
    CHECK(START_REJECTED_FAULT == auto_start_request(&s, now));
    {
        int st0 = st;
        run(&s, 5000, true, &rs, &st);
        CHECK(st == st0);
    }

    CHECK((START_REJECTED_FAULT == s.last_start) && (s.start_count >= 1));
    printf(fails ? "FAILED (%d)\n" : "auto_start: all tests passed\n", fails);
    return 0 != fails;
}

/*** end of file ***/
