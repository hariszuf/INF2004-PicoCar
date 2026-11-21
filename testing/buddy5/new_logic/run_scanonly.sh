#!/bin/sh
cd "$(dirname "$0")" || exit 1
R=../../../app_program/buddy5
gcc -std=gnu11 -O1 -Wall -Wextra -DB5_SCAN_ONLY=1 -I$R -I../../../app_program/vehicle/support -o /tmp/test_scanonly test_scanonly.c $R/buddy5_plan.c $R/buddy5_core.c -lm && /tmp/test_scanonly
