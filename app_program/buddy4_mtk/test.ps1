$ErrorActionPreference = 'Stop'
& python (Join-Path $PSScriptRoot 'prepare_source.py')
if ($LASTEXITCODE) { throw 'Trial source preparation failed' }
$source = Join-Path $PSScriptRoot 'build/source'
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $source 'app_program/buddy4/tests/test_mtk.ps1')
if ($LASTEXITCODE) { throw 'Native task/device regressions failed' }
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $source 'app_program/integration_test/test.ps1')
if ($LASTEXITCODE) { throw 'Existing regression suites failed' }
