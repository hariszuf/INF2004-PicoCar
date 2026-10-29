$ErrorActionPreference = 'Stop'
$b4 = Split-Path $PSScriptRoot -Parent
$out = Join-Path $b4 'build-host/mtk'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$core = @('buddy4_imu.c','buddy4_impact.c','buddy4_motion.c','buddy4_encoder_yaw.c') | ForEach-Object { Join-Path $b4 $_ }
& gcc -std=c11 -Wall -Wextra -Werror -UNDEBUG -DBUDDY4_MTK_HOST_TEST "-I$b4" @core (Join-Path $b4 'buddy4_device.c') (Join-Path $PSScriptRoot 'test_mtk_task.c') -o (Join-Path $out 'task.exe') -lm
if ($LASTEXITCODE) { throw 'Native task/bridge tests failed to compile' }
& (Join-Path $out 'task.exe')
if ($LASTEXITCODE) { throw 'Native task/bridge tests failed' }
# Compile the real professor-derived driver through a register/IRQ test shim.
# The upstream driver has unused parameters and legacy declaration ordering.
& gcc -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-old-style-declaration -UNDEBUG (Join-Path $PSScriptRoot 'test_native_i2c.c') -o (Join-Path $out 'i2c.exe')
if ($LASTEXITCODE) { throw 'Native I2C regressions failed to compile' }
& (Join-Path $out 'i2c.exe')
if ($LASTEXITCODE) { throw 'Native I2C regressions failed' }
& gcc -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-old-style-declaration -UNDEBUG "-I$PSScriptRoot/mtk_stubs" (Join-Path $PSScriptRoot 'test_wait_order.c') -o (Join-Path $out 'wait.exe')
if ($LASTEXITCODE) { throw 'Kernel wait ordering tests failed to compile' }
& (Join-Path $out 'wait.exe')
if ($LASTEXITCODE) { throw 'Kernel wait ordering tests failed' }
