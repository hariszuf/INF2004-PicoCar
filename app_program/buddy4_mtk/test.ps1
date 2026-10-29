$ErrorActionPreference = 'Stop'
& python (Join-Path $PSScriptRoot 'test_startup.py')
if ($LASTEXITCODE) { throw 'Startup pin checks failed' }
& python (Join-Path $PSScriptRoot 'prepare_source.py')
if ($LASTEXITCODE) { throw 'Trial source preparation failed' }
& python (Join-Path $PSScriptRoot 'test_console.py')
if ($LASTEXITCODE) { throw 'Console IRQ latency checks failed' }
& python (Join-Path $PSScriptRoot 'test_peak_console.py')
if ($LASTEXITCODE) { throw 'Tilt/peak console checks failed' }
$source = Join-Path $PSScriptRoot 'build/source'
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $source 'app_program/buddy4/tests/test_mtk.ps1')
if ($LASTEXITCODE) { throw 'Native task/device regressions failed' }
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $source 'app_program/integration_test/test.ps1')
if ($LASTEXITCODE) { throw 'Existing regression suites failed' }
