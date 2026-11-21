#!/bin/sh
# Host simulation of the new Buddy 5 logic. Usage: sh run_tests.sh [verbose 0-2] [scenario index]
cd "$(dirname "$0")" || exit 1
R=../../../app_program/vehicle/rtos
gcc -std=gnu11 -O1 -Wall -Wextra -Wno-unused-function -I$R -I../../../app_program/vehicle/support -o /tmp/sim_b5 sim_b5.c $R/buddy5_plan.c $R/buddy5_core.c -lm || exit 1
/tmp/sim_b5 "$@"
