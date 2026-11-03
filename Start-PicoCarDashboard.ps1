[CmdletBinding()]
param(
    [ipaddress]$VehicleBrokerAddress = '127.0.0.1',
    [ValidateRange(1, 65535)][int]$MqttPort = 1883,
    [ValidateRange(1, 65535)][int]$DashboardPort = 8000
)

$ErrorActionPreference = 'Stop'
if ($VehicleBrokerAddress.AddressFamily -ne 'InterNetwork' -or
    $VehicleBrokerAddress.Equals([ipaddress]::Any)) {
    throw 'Use the laptop Wi-Fi IPv4 address or 127.0.0.1 for a local test.'
}
if ($MqttPort -eq $DashboardPort) {
    throw 'Choose different MQTT and dashboard ports.'
}

function Test-Listener([string]$Address, [int]$Port) {
    $client = [Net.Sockets.TcpClient]::new()
    try {
        return $client.ConnectAsync($Address, $Port).Wait(500) -and $client.Connected
    } catch {
        return $false
    } finally {
        $client.Dispose()
    }
}

if (Test-Listener '127.0.0.1' $DashboardPort) {
    throw "Port $DashboardPort already has a listener. Open http://127.0.0.1:$DashboardPort/ if the dashboard is already running."
}

$dashboard = Join-Path $PSScriptRoot 'dashboard'
$environment = Join-Path $dashboard '.venv'
$python = Join-Path $environment 'Scripts/python.exe'
$requirements = Join-Path $dashboard 'requirements.txt'
if (!(Test-Path -LiteralPath $python)) {
    if (!(Get-Command python -ErrorAction SilentlyContinue)) {
        throw 'Install Python 3.10 or newer and reopen PowerShell.'
    }
    python -m venv $environment
    if ($LASTEXITCODE) { throw 'Could not create the dashboard Python environment.' }
}

$stamp = Join-Path $environment 'requirements.sha256'
$hash = (Get-FileHash -LiteralPath $requirements).Hash
if (!(Test-Path -LiteralPath $stamp) -or (Get-Content -LiteralPath $stamp -Raw).Trim() -ne $hash) {
    & $python -m pip install -r $requirements
    if ($LASTEXITCODE) { throw 'Could not install the dashboard dependencies.' }
    Set-Content -LiteralPath $stamp -Value $hash
}

$vehicleAddress = $VehicleBrokerAddress.IPAddressToString
$broker = $null
$previousHost = $env:MQTT_BROKER_HOST
$previousPort = $env:MQTT_BROKER_PORT
try {
    if (Test-Listener '127.0.0.1' $MqttPort) {
        if (!(Test-Listener $vehicleAddress $MqttPort)) {
            throw "Port $MqttPort is already in use locally. Configure the existing broker to listen on $vehicleAddress or stop it before retrying."
        }
        Write-Host "Using existing MQTT listener on port $MqttPort."
    } else {
        $mosquitto = (Get-Command mosquitto.exe -ErrorAction SilentlyContinue).Source
        if (!$mosquitto) { $mosquitto = Join-Path $env:ProgramFiles 'mosquitto/mosquitto.exe' }
        if (!(Test-Path -LiteralPath $mosquitto)) {
            throw 'Install Eclipse Mosquitto for Windows or add mosquitto.exe to PATH.'
        }
        $config = Join-Path $environment 'picocar-mosquitto.conf'
        $listeners = @("listener $MqttPort 127.0.0.1", 'allow_anonymous true')
        if ($vehicleAddress -ne '127.0.0.1') { $listeners += "listener $MqttPort $vehicleAddress" }
        Set-Content -LiteralPath $config -Value $listeners -Encoding ascii
        $brokerLog = Join-Path $environment 'picocar-mosquitto.log'
        $broker = Start-Process $mosquitto -ArgumentList "-c `"$config`"" -WindowStyle Hidden -RedirectStandardError $brokerLog -PassThru
        $ready = $false
        for ($attempt = 0; $attempt -lt 10 -and !$broker.HasExited; $attempt++) {
            if ((Test-Listener '127.0.0.1' $MqttPort) -and (Test-Listener $vehicleAddress $MqttPort)) {
                $ready = $true
                break
            }
            Start-Sleep -Milliseconds 200
        }
        if (!$ready) {
            $detail = (Get-Content -LiteralPath $brokerLog -Tail 5) -join [Environment]::NewLine
            throw "Mosquitto could not listen on port $MqttPort.`n$detail"
        }
    }

    $env:MQTT_BROKER_HOST = '127.0.0.1'
    $env:MQTT_BROKER_PORT = [string]$MqttPort
    Write-Host "Dashboard  http://127.0.0.1:$DashboardPort/"
    Write-Host "PicoCar MQTT address  ${vehicleAddress}:$MqttPort"
    & $python -m uvicorn app:app --app-dir $dashboard --host 127.0.0.1 --port $DashboardPort
    if ($LASTEXITCODE) { throw 'The dashboard server exited with an error.' }
} finally {
    $env:MQTT_BROKER_HOST = $previousHost
    $env:MQTT_BROKER_PORT = $previousPort
    if ($broker -and !$broker.HasExited) { Stop-Process -InputObject $broker }
}
