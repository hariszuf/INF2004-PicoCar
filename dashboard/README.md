# PicoCar MQTT dashboard

Local FastAPI dashboard for vehicle1 status, telemetry and START/STOP publishing.
Use Python 3.10 or newer and a running Mosquitto broker reachable by both the
dashboard and PicoCar. The broker is separate from this application.

On Windows, the [PowerShell launcher](../README.md#first-run-on-windows)
can set up the virtual environment and start both Mosquitto and the dashboard.
For manual startup, start Mosquitto first. On a trusted test network, use a
broker configuration with these lines.

```text
listener 1883 0.0.0.0
allow_anonymous true
```

Save it as `mosquitto-dev.conf` and start it from the repository root.

```powershell
& 'C:\Program Files\mosquitto\mosquitto.exe' -c .\mosquitto-dev.conf -v
```

In a second PowerShell terminal at the repository root, start the dashboard.

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
On Linux or macOS, use `dashboard/.venv/bin/python` for the same commands.

## Local design preview

Run the server normally, then open `http://127.0.0.1:8000/?preview=1`.
This opt-in mode checks the three frontend files for changes every second.
Saving CSS replaces the stylesheet without clearing telemetry observations.
Saving HTML or JavaScript refreshes the page and resets browser-held observations;
refresh waits until command requests have settled. No commands are replayed.
The normal dashboard URL does not run this file watcher. No extra packages,
commits or pushes are needed to preview local file changes.

Changes made in a separate cloud checkout must first be copied or applied as a
patch to this local checkout. The preview watches files on the machine running
FastAPI; it does not synchronize files between machines.

## Dashboard views

The sidebar opens seven views: Overview, Motion control (Buddy 2), Line & barcode
(Buddy 3), IMU & terrain (Buddy 4), Obstacle detection (Buddy 5), Controller &
safety, and MQTT diagnostics. Views use URL fragments, so a view can be bookmarked
and browser Back/Forward works. The sidebar folds into a Menu button on tablets
and phones. Escape closes the menu and returns focus to its button.

The connection area and START/STOP remain accessible across views. Advanced
integration and transport details are expandable. IMU measurements and saved
peak-height/possible-impact results are separate, with explicit placeholders
until their MQTT integration is available.

## Buddy 4 telemetry integration status

The IMU view mirrors fields supported by the native Buddy 4 snapshot in
`app_program/buddy4_mtk/overlay/app_program/buddy4/buddy4_task.h`:

| Dashboard fields | Native snapshot fields |
| --- | --- |
| Quality, freshness, reading age | `measurement_quality`, `valid`, `tilt_valid`, `sample_age_ms` |
| I²C / configuration errors | `io_errors` (from the IMU configuration/transfer error count) |
| Clipped and unsuitable tilt counts | `clipped_samples`, `unsuitable_samples` |
| Current pitch, roll, wheel-height difference | `pitch_deg`, `roll_deg`, `height_cm` |
| Terrain, side, estimated motion, turn rate | `terrain`, `side`, `motion_state`, `encoder_yaw_deg_s`, with their validity flags |
| Saved stable/median peaks and detected side | `last_event.stable_cm`, `median_cm`, `side`, with separate `stable_valid` / `median_valid` flags |
| Saved ramp event timestamp | `last_event.end_us` (device uptime) |
| Possible-impact history and count | `impact_time_us`, `impact_count`; `possible_impact` is a current one-sample pulse, not persistent history |

These are **native firmware fields, not an MQTT schema**. The publisher in
`lib/libnet/lwip/mqtt_utk.c` currently sends only `src`, `type`, `seq`, `value1`
and `value2`. Buddy 4 does not submit its snapshot to the Buddy 1 MQTT outbox.
Source 4 / IMU type 7 / hump type 8 declarations do not establish a field mapping.
All Buddy 4 measurement, diagnostic, event and count values therefore remain
**N/A**, even if a source-4 payload is observed or the heartbeat is fresh.

The quality names in `app_program/buddy4/buddy4_imu.c` are `GRAVITY_LIKE`,
`NON_GRAVITY`, `CLIPPED`, `NONFINITE` and `UNAVAILABLE`; there is no single
`INVALID` quality enum. Clipped samples are also unsuitable for tilt, so the two
counters overlap and must not be summed. An unavailable age is represented in
the native snapshot by `UINT32_MAX`, not zero. The native accessor ages current
IMU validity separately from retained event history.

Ramp-height estimates are preliminary. Side labels describe chassis orientation,
not proof that a particular wheel rose rather than the opposite wheel dropped.
Saved peaks require their estimator validity flags. Possible-impact observations
are not confirmed collisions; a false current pulse must not erase event history.
The current Buddy 4 snapshot has no possible-obstruction event/timestamp, so that
optional feature is omitted. No firmware or telemetry protocol is changed here.

## Meaning of the indicators

### Buddy 2 distance and Buddy 5 profile

Distance travelled is the signed `motion_status_t.distance_cm` odometry value
since reset, not a cumulative path-length counter. Buddy 2 does not publish an
agreed distance field to MQTT, so the dashboard shows N/A rather than estimating
distance from incomplete speed/encoder messages.

The Buddy 5 view interprets these source-5 messages. The integrated car emits
them from `app_program/buddy5/rtos/scan_task.c`:

| Type | Value 1 | Value 2 |
| --- | --- | --- |
| 50 | State enum | Fault enum |
| 51 | Width in mm, or -1 | Depth upper bound in mm, or -1 |
| 52 | Bypass side: +1 left, -1 right | Signed sideways offset in mm |
| 53 | Depth lower bound in mm, or -1 | Depth upper bound in mm, or -1 |
| 54 | Derived front clearance in mm | Commanded servo angle in degrees |
| 55 | Obstacle ID | Report uptime in ms |
| 56 | Validity/profile bit mask | Reserved (integrated car) |
| 58 | Last raw HC-SR04+ range in mm, or -1 | Raw status: 0 valid, 1 echo stuck high, 2 rise timeout, 3 fall timeout, 4 out of range; -1 stale/unavailable |

Type 56 carries width/depth validity in value1 bits 0/1. Bit 2 marks the
stationary bench profile; bit 4 marks the integrated driving profile. The MQTT
envelope also transmits its `flags` field. Unknown mask bits are not used to
approve estimates.

Width and depth are displayed only when their packet and type-56 validity packet
share the latest observed source-5 sequence. Depth bounds must also be ordered.
Missing or invalid estimates show **Unknown**, never `-1 mm`; conflicting fields
show an inconsistent-validity label. A sequence decrease clears held Buddy 5
packets. Matching sequences cannot establish device boot identity or an atomic
snapshot, and browser polling can miss whole bursts. All values retain explicit
last-observation labels, including after stale heartbeat or connection loss.

The current publishers send about once per second. State transitions are printed
to the console, but immediate MQTT updates on state change are not implemented.
The backend still holds only one payload per topic, so a coherent status snapshot
requires future publisher/receiver integration rather than a frontend assumption.

Type 58 reports the most recent raw ping only while it is at most two seconds
old. The dashboard displays range only for status 0; a timeout does not establish
free space. Type 54 remains the filtered, derived *front clearance* and servo
command. Its `-1` sentinel is Unknown. During a scan, the latest raw ping may
have been taken at a different angle from the current servo command. The
integrated car also reports line reacquisition count on source 6/type 57.

### General status semantics

- MQTT Broker reports this dashboard's broker connection.
- PicoCar Heartbeat is fresh for less than 15 seconds after receiving any message
  on `picocar/vehicle1/status`. It does not validate the payload or reject retained
  messages; a retained or manually published status can therefore appear fresh.
- Readiness remains **Unconfirmed**: neither broker connection nor a fresh
  heartbeat proves controller, motor or safety-monitor readiness.
- Telemetry displays the latest raw `picocar/vehicle1/telemetry` payload and its
  `seq=` field. Initial payloads are labelled cached with unknown age. Later ages
  measure browser-observed payload changes, not device sample or broker receipt
  time. Identical repeated payloads cannot be distinguished from a cached value.
- Missing measurements display **N/A**, not zero. Sensor fields without a verified
  MQTT mapping are labelled as not integrated. A payload observation alone does
  not establish a valid measurement or a healthy subsystem.
- Buddy 5 source 5/type 50 displays the state and fault enums in
  `app_program/buddy5/rtos/buddy5_core.h`. Source 5/type 54 displays derived front
  clearance (mm) and commanded servo angle (degrees), as published in
  `app_program/buddy5/rtos/scan_task.c`. Clearance `-1` is unknown; zero and other
  signed clearance values are preserved. The servo angle is a command, not
  measured servo position. Type 58 supplies the last raw range and status.
- Interpreted values are held only in this browser session, with observation
  labels. They remain last observations when the link fails or the heartbeat
  goes stale. They are not a synchronized vehicle snapshot.
- MQTT diagnostics contains a capped log of 100 changed payloads observed by
  this browser. Polling can miss messages, and the API stores only the latest
  payload per topic. This is not a complete broker log. Internal telemetry
  validity flags are not transmitted by the current firmware.
- START/STOP posts to `/api/command/{command}` and publishes to
  `picocar/vehicle1/command` using QoS 0 without retain. Success means publishing
  was queued; it does not acknowledge delivery or controller execution.
- START requires a confirmed dashboard-to-broker connection. STOP remains
  independently available while START is pending and can be attempted when
  connection evidence is unknown. Commands are never automatically retried.
  A timeout leaves publication outcome unknown; STOP is not a hardware
  emergency stop. A command response never changes reported operating mode.

## Frontend structure

`static/index.html` defines the shell and persistent controls.
`static/dashboard.css` owns the shared colour, typography and spacing tokens.
`static/dashboard.js` defines view metadata, measurement placeholders, payload
interpretation, bounded observations and coordinated polling. FastAPI serves
these assets under `/static`; existing API responses and MQTT behaviour remain
unchanged. No frontend build step or new runtime dependency is required.

White surfaces and a light neutral canvas carry dark text. Blue marks navigation
and START, green marks connection evidence, red marks STOP/faults, and amber
marks stale or warning states. Labels carry the meaning independently of colour.
The text palette exceeds 4.5:1 on its intended surfaces. Controls have visible
keyboard focus, mobile targets are at least 44px, and no decorative motion is used.

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

Optional browser regression checks require Playwright in a development Python
environment and its Chromium browser:

```sh
python -m pip install playwright
python -m playwright install chromium
python dashboard/check_browser.py
```

For an existing Chromium installation, pass `--chromium /path/to/chromium`.
Use `--output /path/to/screenshots` to choose a screenshot directory (by default,
the system temporary directory). The script serves static files on loopback and
intercepts every API request with explicit test fixtures. It never imports the
dashboard backend, connects to MQTT or sends real vehicle commands. It covers
all seven views at 1440, 768, 390 and 320 pixels, keyboard navigation, cached and
malformed payloads, N/A versus zero, stale/disconnected states, failed API calls,
and command results, including STOP during an in-flight START. Its screenshots
contain simulated test data, not physical-hardware evidence.

Live broker reconnection, physical command execution and sensor accuracy still
require independent lab verification. Earlier firmware cold-boot evidence does
not verify this frontend redesign.
