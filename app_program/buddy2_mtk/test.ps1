# Host tests for the Buddy 2 micro T-Kernel trial (no hardware).
$ErrorActionPreference = 'Stop'
& python (Join-Path $PSScriptRoot 'prepare_source.py')
if ($LASTEXITCODE) { throw 'Trial source preparation failed' }
$app = Join-Path $PSScriptRoot 'build/source/app_program'
$b2 = Join-Path $app 'buddy2'
$mtk = Join-Path $b2 'mtk'
$out = Join-Path $PSScriptRoot 'build/host'
New-Item -ItemType Directory -Force $out | Out-Null

# 1. Drivers + motion_control.c on a fake RP2040 register map.
& gcc -std=c11 -Wall -Wextra -Werror -DMOTION_MTK_HOST_TEST -DMOTION_USE_UTKERNEL "-I$b2" "-I$mtk" `
    (Join-Path $b2 'motion_control.c') (Join-Path $b2 'motion_pid.c') `
    (Join-Path $mtk 'motion_mtk_hw.c') (Join-Path $mtk 'motion_motor_mtk.c') (Join-Path $mtk 'motion_encoder_mtk.c') `
    (Join-Path $mtk 'tests/test_mtk_drivers.c') -o (Join-Path $out 'mtk_drivers.exe') -lm
if ($LASTEXITCODE) { throw 'Driver tests failed to compile' }
& (Join-Path $out 'mtk_drivers.exe')
if ($LASTEXITCODE) { throw 'Driver tests failed' }

# 2. The unchanged Buddy 2 control-logic suites (motion + terrain).
$common = @('-std=c11', '-Wall', '-Wextra', '-DMOTION_HOST_TEST', "-I$b2", "-I$(Join-Path $b2 'test/host')")
$motion = @((Join-Path $b2 'motion_control.c'), (Join-Path $b2 'motion_pid.c'), (Join-Path $b2 'test/host/sim_hw.c'))
& gcc @common @motion (Join-Path $b2 'test/host/test_motion.c') -o (Join-Path $out 'motion.exe') -lm
if ($LASTEXITCODE) { throw 'Motion tests failed to compile' }
& (Join-Path $out 'motion.exe')
if ($LASTEXITCODE) { throw 'Motion tests failed' }
& gcc @common @motion (Join-Path $b2 'test/host/test_terrain.c') -o (Join-Path $out 'terrain.exe') -lm
if ($LASTEXITCODE) { throw 'Terrain tests failed to compile' }
& (Join-Path $out 'terrain.exe')
if ($LASTEXITCODE) { throw 'Terrain tests failed' }
Write-Host 'Buddy 2 micro T-Kernel trial host tests passed (logic only, no hardware).'
