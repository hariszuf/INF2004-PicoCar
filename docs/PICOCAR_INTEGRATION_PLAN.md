# Prioritized INF2004 PicoCar integration plan

## Starting point

Buddy 1 has a reproducible, physically observed single-core networking baseline.
Its current command task logs messages; the current application starts Buddy 1
and Buddy 5 only. `robot_event_t` exists in shared headers but is not a running
Vehicle Controller, and differs from Buddy 1's command/telemetry structures.
Do not describe the final robot as integrated on the strength of MQTT alone.

This is a plan, not implementation of the stages below. No motor activation,
new driver port, controller state machine, wire-format redesign or minimum-bit
tribunal implementation is included in this review branch. The lecturer's
clarification removes the minimum-bit exercise from the implementation backlog.

## P0 — Freeze the baseline and resolve ownership before combining firmware

Deliverables: review the three troubleshooting runtime commits, preserve the
known-good UF2/hash, agree one final application target, and name owners for
controller, motion, sensors and communications. Keep the single-core profile
initially; do not introduce SMP as an integration prerequisite.

Resolve the pin/driver matrix explicitly:

| Resource | Current users / required decision |
|---|---|
| GP0/GP1 | BSP UART console uses both; Buddy 3 requires GP0 line input. Choose/remap/disable console with team agreement before line-sensor integration. Building USB CDC alone may still retain a UART mirror. |
| GP16/GP17 | Buddy 4 I2C SDA/SCL; BSP external liveness LED defaults to GP16. Remove or relocate only the conflicting diagnostic ownership in an integration-specific change. |
| GP8–GP11 | Buddy 2 motor PWM ownership only; no network callback writes. |
| GP4/GP6 | Buddy 2 encoder inputs; native µT-Kernel IRQ dispatch must work. SDK-compatible IRQ registration stubs in the networking port are not a working encoder interrupt service. |
| GP2/GP26 | Buddy 3 line/barcode input; avoid sample ADC-driver pin setup overriding digital sensor ownership. |
| GP28/GP7 | Buddy 5 trigger/echo; ensure sample ADC driver does not also claim GP28. |
| GP23/24/25/29 | Pico W radio-owned pins; never repurpose as ordinary LEDs or sensors. |
| Clock/IRQ/I2C | One BSP initializes clocks; retain corrected tick. Review IRQ and I2C ownership before importing SDK libraries or test overlays. |

Existing local history includes Buddy 2 native trial commits ending at
`ad7f867` and Buddy 4 trial checkpoint `ac3c2ab`. Review these with their owners
for reusable adapters and hardware evidence. They are not integrated into this
branch. Do not blindly merge their alternative app_main/build/BSP/kernel
overlays or reimplement work already present there.

Exit gate: agreed pin/resource map, one boot/clock owner, no duplicated main or
tasks, unchanged teammate modules except separately reviewed adapter/config work.

## P1 — Build the Vehicle Controller in dry-run mode

Give one controller task ownership of vehicle state and decision arbitration.
Start with a small explicit STOPPED/RUNNING/FAULT policy or the team's agreed
statechart. Map START/STOP and obstacle/fault events into it. Enforce STOP and
fault handling ahead of motion requests; no automatic restart from an old START
after reboot or reconnection. Decide the network-loss policy explicitly.

Use one command queue consumer: replace the logging-only sink with a handoff or
let the controller be its consumer. Do not add a competing consumer to the same
message buffer. Preserve the radio owner and non-blocking producer interfaces.
Map between `command_message_t`, `telemetry_message_t` and `robot_event_t` only
at defined boundaries; agree units, validity, acquisition timestamp and sequence
semantics. Their layouts and enums are not interchangeable.

Dry-run outputs record intended actions only, with no linked motor driver.
Add command acceptance/rejection and resulting controller state to telemetry
so physical START/STOP consumption can be verified without activating motors.
Keep the current text MQTT payload until a real interoperability requirement
requires a change; no bit-minimization work is required.

Exit gate: host tests for START/STOP, invalid command, duplicate/stale input,
queue pressure and fault precedence; communication-only hardware proof of
controller reception/state, with motors disabled.

## P1 — Qualify recovery and queue behavior before unattended use

Run hardware checklist B1-H07–H10: late hotspot, broker restart, live Wi-Fi
loss, DHCP delay and repeated restoration. Measure rather than assume driver
auto-rejoin. Correct only reproduced gaps: distinguish association recovery,
DHCP start error, MQTT retry and owner-task progress. Radio initialization
retry needs safe partial-resource cleanup; never add blind reset loops.

Define bounded queues and observability for drops, publish rejection, task
periods, data age and reconnect count. Keep non-blocking producers. A failed
telemetry publish currently loses the dequeued sample; decide latest-value versus
event preservation per producer. Critical STOP/fault delivery must not share an
unbounded telemetry backlog. Do not equate local sequence assignment with a
remote command identifier or proof of execution.

Exit gate: repeatable recovery without USB/reset, clear stale/fault state,
documented command-loss policy and no unexpected action replay.

## P2 — Add Buddy 5 and Buddy 4 as read-only controller inputs

Buddy 5 already runs in this profile but logs locally; route obstacle/clear/
sensor-fault observations into controller events and rate-limited telemetry.
Revalidate ultrasonic timing after the shared TIMER correction and quantify
busy-wait/preemption effects. Sensor fault must not silently mean CLEAR.

Reuse the existing Buddy 4 native I2C/task trial after reviewing its BSP/kernel
changes. Resolve GP16 LED/I2C conflict first. Port the adapter around preserved
processing logic; verify LSM303D identity 0x49 at address 0x1D, calibration,
timeouts and sample age without actuators. Preserve uncertainty on height and
possible-impact detection; do not promote observational classification into an
unvalidated safety guarantee.

Exit gate: both sources reach controller/dashboard with correct units and
validity under Wi-Fi load; stale/fault injection produces the agreed safe state.

## P3 — Integrate Buddy 2's native motion service with outputs inhibited

Review the existing Buddy 2 µT-Kernel trial instead of using the old task
skeleton as if it were validated. Keep motor and encoder ownership in Buddy 2;
use its public API through an agreed task/queue boundary. Its header's broad
cross-task claims must be checked against the chosen single-core scheduler
and native adapter; do not assume every getter/queued command is atomic.

Verify the 20 ms update budget, GPIO IRQ dispatch, pin mapping, encoder polarity,
counts-per-revolution edge mode, stack use and fault latching. Reconcile Buddy
2's 12.2 cm geometry with Buddy 4's measured 11.3 cm centre spacing through
the owners, not a silent cross-module edit. Keep communications tests dry-run;
physical motor tuning is a later separately controlled activity.

Exit gate: controller requests and feedback are correct with inhibited outputs,
STOP/fault priority works, sensor faults override intent and network load does
not violate task deadlines. Motor performance remains unclaimed until tested.

## P4 — Add Buddy 3 line following and barcode behavior

Resolve GP0 console conflict and use the controller to enable/disable line
following. Preserve its steering algorithm and Buddy 2 API, but ensure a STOP
or obstacle/fault cannot be undone by the next periodic steering update.
Agree line-lost behavior, barcode speed/format and event delivery. Current
Buddy 3 README states barcode decoding is not implemented in this snapshot;
check newer teammate work before planning or writing a duplicate decoder.

Exit gate: deterministic simulated routes, line loss, sensor inversion,
barcode event scenarios and STOP/fault precedence pass; then perform separate
controlled driving validation with teammates.

## P5 — Mission and submission validation

Combine agreed route/controller states, obstacle response, barcode actions and
terrain observations. First run with simulated inputs and motors inhibited;
then execute the team's controlled physical mission tests. Cover boot with
missing sensors/network, command bursts, faults, reconnect, changing battery
conditions, repeated missions and recovery. Measure CPU/stack/queue margins,
motion accuracy and timing against the assignment's actual criteria.

Update test IDs/evidence against the exact integration commit and UF2 hash.
Complete reports, architecture/pin/state diagrams and limitations. Do not copy
prototype success into an assignment-wide PASS or count the minimum-bit
reasoning exercise as outstanding implementation work.
