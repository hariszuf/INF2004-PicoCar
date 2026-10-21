#!/bin/sh
# Host simulation test for line following. Run from the "Buddy 3" folder:
#   sh test/host/run_tests.sh        (add -v argument for a trace:  sh test/host/run_tests.sh v)
set -e
B2="../Buddy 2"
gcc -std=c11 -Wall -Wextra -DMOTION_HOST_TEST -DMOTION_MIN_DUTY=0.10f -DMOTION_BREAKAWAY_DUTY=0.22f -DMOTION_STALL_DUTY=0.6f -DMOTION_STALL_TIME_MS=1500u -DMOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f -I. -I"$B2" -I"$B2/test/host" -Itest/host \
    line_follow.c test/host/sim_track.c test/host/test_line.c \
    "$B2/motion_control.c" "$B2/motion_pid.c" "$B2/test/host/sim_hw.c" \
    -o /tmp/line_host_test -lm
/tmp/line_host_test "$@"
