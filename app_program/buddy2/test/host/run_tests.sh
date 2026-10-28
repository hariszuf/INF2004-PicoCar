#!/bin/sh
# Build and run the host simulation tests for Motion Control.
# Run from the "Buddy 2" folder:  sh test/host/run_tests.sh
set -e
gcc -std=c11 -Wall -Wextra -DMOTION_HOST_TEST -I. -Itest/host \
    motion_control.c motion_pid.c test/host/sim_hw.c test/host/test_motion.c \
    -o /tmp/motion_host_test -lm
/tmp/motion_host_test
# Terrain (slope / hump) speed control from the Buddy 4 IMU report
gcc -std=c11 -Wall -Wextra -DMOTION_HOST_TEST -I. -Itest/host \
    motion_control.c motion_pid.c test/host/sim_hw.c test/host/test_terrain.c \
    -o /tmp/motion_terrain_test -lm
/tmp/motion_terrain_test
# Same motion tests with the slow line-following tuning used by EmbeddedProject
gcc -std=c11 -Wall -Wextra -DMOTION_HOST_TEST -I. -Itest/host \
    -DMOTION_MIN_DUTY=0.10f -DMOTION_BREAKAWAY_DUTY=0.22f \
    -DMOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f -DMOTION_STALL_DUTY=0.6f -DMOTION_STALL_TIME_MS=1500u \
    motion_control.c motion_pid.c test/host/sim_hw.c test/host/test_motion.c \
    -o /tmp/motion_host_test_line -lm
/tmp/motion_host_test_line
