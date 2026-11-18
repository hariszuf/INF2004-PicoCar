# Builds the PicoCar firmware and writes the full output to build_make\build-claude.log
Set-Location $PSScriptRoot
$log = Join-Path $PSScriptRoot 'build_make\build-claude.log'
"=== build started $(Get-Date) ===" | Out-File -Encoding utf8 $log
function Say($t) { Write-Host $t; $t | Out-File -Encoding utf8 -Append $log }

$gcc = Get-ChildItem "$HOME\.pico-sdk\toolchain" -Recurse -Filter arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $gcc) { Say "ERROR: arm-none-eabi-gcc.exe not found under $HOME\.pico-sdk\toolchain"; Read-Host "Press Enter"; exit 1 }
$env:PATH = "$($gcc.DirectoryName);C:\Program Files\Git\usr\bin;$env:PATH"
Say "compiler: $($gcc.FullName)"
Say "make: $((Get-Command make -ErrorAction SilentlyContinue).Source)"
Say "xargs: $((Get-Command xargs -ErrorAction SilentlyContinue).Source)"
if (-not (Test-Path 'C:/pico-sdk/lib/cyw43-driver/src/cyw43_ll.c')) { Say "ERROR: C:/pico-sdk is missing the CYW43 driver (submodules)"; Read-Host "Press Enter"; exit 1 }

git restore build_make/mtkernel_3 2>&1 | ForEach-Object { Say "$_" }
make -C build_make -f picocar_integration.mk TELEMETRY_MQTT=1 HEADLESS=1 "PICO_SDK_PATH=C:/pico-sdk" -j8 2>&1 | ForEach-Object { Say "$_" }
Say "=== make exit code: $LASTEXITCODE ==="
$uf2 = Join-Path $PSScriptRoot 'build_make\picocar_integration_headless_mqtt.uf2'
if (Test-Path $uf2) { Say "UF2: $uf2 ($((Get-Item $uf2).LastWriteTime))" } else { Say "UF2 NOT FOUND" }
Say "=== build finished $(Get-Date) ==="
