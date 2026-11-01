# PicoCar MQTT dashboard

Local FastAPI dashboard for vehicle1 status, telemetry and START/STOP publishing.
Use Python 3.10 or newer and a running Mosquitto broker reachable by both the
dashboard and PicoCar. The broker is separate from this application.

From the repository root in PowerShell:

```powershell
python -m venv dashboard/.venv
./dashboard/.venv/Scripts/python.exe -m pip install -r dashboard/requirements.txt
$env:MQTT_BROKER_HOST = '127.0.0.1'
$env:MQTT_BROKER_PORT = '1883'
./dashboard/.venv/Scripts/python.exe -m uvicorn app:app --app-dir dashboard --host 127.0.0.1 --port 8000
```

Open http://127.0.0.1:8000. Override the broker host when Mosquitto runs elsewhere.
Run one server worker and one dashboard instance per broker: the MQTT client ID
is `picocar-dashboard`, and cached messages are held in memory.
The dependency versions match the existing local environment.

## Meaning of the indicators

- MQTT Broker reports this dashboard's broker connection.
- PicoCar Heartbeat is fresh for less than 15 seconds after receiving any message
  on `picocar/vehicle1/status`. It does not validate the payload or reject retained
  messages; a retained or manually published status can therefore appear fresh.
- Telemetry displays the latest raw `picocar/vehicle1/telemetry` payload and its
  `seq=` field. Last Telemetry measures browser-observed payload changes, not
  device sample time. Reloading the page can make a cached payload appear new.
- START/STOP posts to `/api/command/{command}` and publishes to
  `picocar/vehicle1/command` using QoS 0 without retain. Success means publishing
  was queued; it does not acknowledge delivery or controller execution.

This is a local lab tool without authentication or TLS configuration. Keep the
HTTP server on loopback; do not expose its command endpoints publicly.
Keep motors disabled during communication testing and use the existing power
bank, never a USB wall charger. This change does not wire commands to motors.

## Checks

```powershell
./dashboard/.venv/Scripts/python.exe -m unittest discover -s dashboard -p "test_*.py" -v
```

Checks mock the MQTT client: no car commands or network connection are made.
They cover the heartbeat expiry boundary, cached telemetry, command validation,
disconnection, publish failure and the page path from another working directory.
New physical command, reconnection and browser tests remain manual verification
steps; earlier firmware cold-boot evidence does not verify this dashboard PR.
