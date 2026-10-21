#!/bin/sh
# Build and run the host simulation tests for Motion Control.
# Run from the "Buddy 2" folder:  sh test/host/run_tests.sh
set -e
gcc -std=c11 -Wall -Wextra -DMOTION_HOST_TEST -I. -Itest/host \
    motion_control.c motion_pid.c test/host/sim_hw.c test/host/test_motion.c \
    -o /tmp/motion_host_test -lm
/tmp/motion_host_test
