# Buddy 1 hardware acceptance checklist

Date/firmware/runtime commit/profile must accompany each result. Current
baseline is `0f07d6d`, single core, UART-only, Buddy 1 + Buddy 5, UF2 SHA256
`ed1ceeaeb06e8bca56ce19031d9796f7db68eada4899388c2552d95f47f3b893`.

**All communication testing keeps motors disabled, uses the existing power
bank/cable, and excludes USB wall chargers.** Do not run autonomous demo images.
Confirm the tested executable contains no motor task/control entry points before
publishing START. Do not flash the built USB variant as if it had the UART
image's hardware verification. Preserve the saved working image for rollback.

The timing tolerances below are proposed bench acceptance criteria, not quoted
assignment requirements. Record failures and actual values instead of changing
criteria to fit a run. No pending row is a hardware PASS.

## Capture setup

- Laptop, car and Mosquitto must use the same reachable hotspot/broker. Verify
  the laptop IP and dashboard broker environment; localhost may be another broker.
- Start broker verbose capture and a uniquely named independent subscriber
  before power-on. Save timestamped `picocar/vehicle1/status` and `/telemetry`;
  broker capture should include car CONNECT, CONNACK, PUBLISH and disconnect.
- Never publish manual freshness tests on those topics. Record publisher identity
  from broker output; status/telemetry topic names alone are not authentication.
- Record physical power-on time, firmware name/hash, LED pattern, client ID,
  source port, first telemetry, first status and dashboard heartbeat age.
- A subscriber timeout is an intentional capture stop, not a car disconnect.

| ID | Procedure / acceptance | Current evidence status |
|---|---|---|
| B1-H01 | Three complete cold boots: power absent for ten seconds, reconnect directly to bank without BOOTSEL; require new car connection, advancing telemetry and status. Log startup duration; initially investigate any run exceeding 90 s, without calling 90 s a guaranteed lease deadline. | PASS for connection/status recovery in three user-performed boots. Exact power-on latency NOT MEASURED. |
| B1-H02 | After CONNACK, first status about 5 s later; capture at least ten status messages. On healthy bench network target interval 5 ±1 s, no persistent gap >=15 s, advancing sequences. | Five-second cadence observed; eight consecutive messages in saved normal capture. Full ten-message controlled acceptance run PENDING. |
| B1-H03 | Dashboard MQTT-connected and freshness agree with independent traffic; age resets on status, becomes stale >=15 s without it; reconnect must not show cached status as proof of new traffic. | Normal fresh state and pre-fix stale state observed. Controlled stale/recovery validation PENDING. |
| B1-H04 | Capture ten minutes of telemetry, plot/inspect sequence gaps, startup backlog and drop counters. Source/type/unit/validity must match producer; no sustained backlog. Kernel heartbeat is about 1 Hz. | Live synthetic Buddy 1 heartbeat telemetry observed. Ten-minute/drop-counter and other-buddy telemetry checks PENDING. |
| B1-H05 | START then STOP on command topic, non-retained. Require broker receipt, MCU parsing, command-buffer consumption and application acknowledgement or trustworthy consumer log. Unsupported payload and wrong topic must not create a command. Motors remain disabled. | Host parser/IPC tests PASS. Earlier historical USB command observations do not certify the current UART-only image. Current-image hardware consumer proof PENDING. |
| B1-H06 | Send repeated START/STOP and a small burst exceeding queue capacity in an isolated dry-run build. Require bounded errors/drops, no task lockup, and STOP priority/response agreed before controller integration. | Non-blocking source inspected. Real queue saturation and controller STOP policy PENDING. |
| B1-H07 | Start board while hotspot unavailable, enable it later; require bounded initial join retries and new MQTT/status without USB or board reset. | Initial-join retries pass mocked tests. Physical late-hotspot run PENDING. |
| B1-H08 | With car connected, restart only an agreed isolated broker instance, keep hotspot alive. Require fresh car session/subscription, five-second status resumes, STOP communication still works in dry run. | Host connect errors/refusal/timeouts tested. Physical broker restart PENDING. |
| B1-H09 | With car connected, disable hotspot temporarily then restore it. Require Wi-Fi association, DHCP and MQTT recovery without reboot; record IP changes and delays. No replay of old motion commands. | PENDING; known absence of application post-success rejoin, SDK auto-rejoin unverified. |
| B1-H10 | Repeated outage/restore cycles and delayed DHCP, preserving capture. Confirm stale data never becomes a fresh sensor decision; invalid/timeout messages remain explicit. | PENDING; failure paths documented, not validated hardware recovery. |
| B1-H11 | Communication-only combined load: other buddies use simulated inputs or disconnected actuators; check task periods, owner-poll latency, stack headroom, queue drops and STOP latency against agreed budget. | PENDING. Current firmware starts only Buddy 1 and Buddy 5. |
| B1-H12 | Recheck ultrasonic timeouts and measured distance after the global TIMER fix, with the sensor owner. | Formula/state host tests PASS; physical timing/accuracy PENDING. |
| B1-H13 | Corrected USB CDC image: cold bank boot without a host, then laptop data connection/terminal open and close; require uninterrupted owner progress and correct timing. | Build/USB boundary checks PASS; corrected USB image hardware PENDING. |

## START/STOP observation constraint

The working UART image deliberately has no COM5 USB CDC endpoint. Broker
`Sending PUBLISH` only proves broker forwarding, not MCU command consumption.
Do not mark B1-H05 complete from that alone. Use a verified, electrically
appropriate existing UART capture on uncontested pins, or a separately named,
reviewed dry-run diagnostic image providing application acknowledgements. Do
not change the saved working image merely to obtain a green status indicator.
No UART hardware was attached and no such diagnostic change is implemented here.

Once a Vehicle Controller exists, distinguish command RECEIVED, ACCEPTED or
REJECTED, and COMPLETED. A broker acknowledgement does not replace these states.
STOP must not wait behind stale START requests. Agree limits and loss behavior
before enabling any actuator in a later, separate motor-validation activity.

## Result record template

For each row retain: test ID, date/time/timezone, operator, exact source commit,
UF2 hash, build profile, motor-disabled state, power source, broker endpoint,
steps, expected result, observed timestamps/sequences, PASS/FAIL/NOT RUN,
evidence filenames and limitations. Record network recovery separately from
physical power cycling. The existing three boots do not establish Wi-Fi
reconnection after a live link failure.
