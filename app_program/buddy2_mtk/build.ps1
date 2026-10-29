# Build the Buddy 2 micro T-Kernel trial image (SMP=0, USB CDC console).
# Paths match the team's documented Windows layout; adjust on another computer.
$ErrorActionPreference = 'Stop'
$env:PATH = "$env:USERPROFILE/.pico-sdk/toolchain/15_2_Rel1/bin;C:/msys64/ucrt64/bin;C:/msys64/usr/bin;" + $env:PATH
& python (Join-Path $PSScriptRoot 'prepare_source.py')
if ($LASTEXITCODE) { throw 'Trial source preparation failed' }
$source = Join-Path $PSScriptRoot 'build/source'
Push-Location (Join-Path $source 'build_make')
try {
    & make SMP=0 CONSOLE=usb_cdc "PICO_SDK_PATH=$env:USERPROFILE/.pico-sdk/sdk/2.3.1" -j4
    if ($LASTEXITCODE) { throw 'Buddy 2 micro T-Kernel trial build failed' }
    foreach ($ext in @('elf','map','uf2')) {
        Copy-Item -LiteralPath "mtk3pico_smp0_usb_cdc.$ext" -Destination (Join-Path $PSScriptRoot "build/mtk3pico_smp0_usb_cdc.$ext")
    }
} finally { Pop-Location }
Write-Host "Trial firmware: $(Join-Path $PSScriptRoot 'build/mtk3pico_smp0_usb_cdc.uf2')"
Write-Host 'No flashing, motor operation or Git operations performed.'
