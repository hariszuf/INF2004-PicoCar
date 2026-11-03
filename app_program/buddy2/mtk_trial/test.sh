#!/bin/sh
# Same as test.ps1 for Linux / Git Bash.
set -e
cd "$(dirname "$0")"
python3 prepare_source.py
APP=build/source/app_program; B2=$APP/buddy2; MTK=$B2/mtk; OUT=build/host
mkdir -p $OUT
gcc -std=c11 -Wall -Wextra -Werror -DMOTION_MTK_HOST_TEST -DMOTION_USE_UTKERNEL -I$B2 -I$MTK \
    $B2/motion_control.c $B2/motion_pid.c $MTK/motion_mtk_hw.c $MTK/motion_motor_mtk.c \
    $MTK/motion_encoder_mtk.c $MTK/tests/test_mtk_drivers.c -o $OUT/mtk_drivers -lm
$OUT/mtk_drivers
(cd $B2 && sh test/host/run_tests.sh)
echo 'Buddy 2 micro T-Kernel trial host tests passed (logic only, no hardware).'
