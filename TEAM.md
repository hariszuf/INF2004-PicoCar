# INF2004 PicoCar — Team Architecture and Interfaces

**Hardware:** Cytron Robo-Pico / Raspberry Pi Pico W  
**RTOS:** µT-Kernel 3.0  
**Current tested profile:** Single-core (`SMP=0`), USB CDC, Wi-Fi, DHCP, DNS  
**Team:** Five developers

## 1. Members and subsystem ownership

| Buddy | Developer | Subsystem | Directory |
|---|---|---|---|
| 1 | Haris (`hariszuf`) | Wi-Fi, commands, telemetry and networking | `app_program/buddy1/` |
| 2 | Afiq | Motion, PWM and motor control | `app_program/buddy2/` |
| 3 | Wayne | Barcode recognition | `app_program/buddy3/` |
| 4 | Ignatius | IMU and hump measurement | `app_program/buddy4/` |
| 5 | Vijay | Obstacle detection | `app_program/buddy5/` |

Each developer owns their subsystem implementation and tests.

Shared files, kernel configuration, hardware allocation and cross-subsystem interfaces require team agreement.

All subsystems must provide `buddyN_init()` and `buddyN_start()`. All initialisation functions execute before any Buddy startup functions.

## 2. GPIO allocation

Pin ownership must be confirmed against the Robo-Pico schematic and actual wiring.

| Resource | Owner | GPIO |
|---|---|---|
| Wi-Fi radio | Kernel networking service | Pico W reserved pins |
| Motor PWM and driver signals | Afiq | TBD |
| Barcode sensors | Wayne | TBD |
| IMU / I2C | Ignatius | TBD |
| Obstacle ranging | Vijay | TBD |
| Line-following sensors | TBD | TBD |

GP23, GP24, GP25 and GP29 are reserved for Pico W radio-related hardware. GP0 and GP1 may be required for UART crash reporting.

Pin claims must be documented individually before wiring. Hardware connections on the Robo-Pico board take precedence over generic Pico GPIO availability.

## 3. Peripheral ownership

Each peripheral has one owning task. Other tasks request information or actions using agreed interfaces rather than accessing that peripheral independently.

Buddy 1 reads Wi-Fi state through `cyw43_utk_get_status()`. The network service owns CYW43 and lwIP operations.

## 4. Task priorities and kernel resources

Lower numeric values mean higher µT-Kernel priority.

Current Buddy 1 priorities:

| Task | Priority |
|---|---:|
| Command consumer | 6 |
| Telemetry consumer | 7 |
| Heartbeat producer | 8 |
| Network monitor | 9 |

These are current implementation values, **not the final team-wide scheduling plan**.

Final priority allocation must account for motor-control timing, obstacle detection, line tracking, IMU sampling and networking. The team must agree on timing requirements before modifying task priorities.

Current `config/config.h` limits include:

| Resource | Limit | Buddy 1 allocation |
|---|---:|---:|
| Tasks | 32 | 4 |
| Message buffers | 4 | 2 |
| Semaphores | 16 | 0 |
| Event flags | 16 | 0 |
| Mailboxes | 4 | 0 |
| Mutexes | 4 | 0 |

Other buddies must declare their required resources before integration. Kernel and networking tasks also consume system resources.

## 5. Cross-subsystem interfaces

| ID | Producer | Consumer | Data |
|---|---|---|---|
| IF-01 | Wayne | Mission controller | Line readings and decoded barcode events |
| IF-02 | Ignatius | Mission controller | IMU readings and hump events |
| IF-03 | Vijay | Motion / mission controller | Obstacle distance and emergency stop requests |
| IF-04 | Haris | Mission controller | Remote START, STOP, PAUSE and RESUME |
| IF-05 | All buddies | Haris | Telemetry and diagnostics |
| IF-06 | Motion controller | Afiq's motor task | Motion targets and stop commands |

**Mission-controller owner: pending team confirmation.** Afiq is a proposed owner, but motor control and mission planning may be separate tasks.

### Shared communication rules

- Discrete events use the agreed `robot_event_t` format.
- Continuous sensor readings may require a latest-value mechanism instead of an event queue.
- Emergency stop requests must use a reliable, high-priority control path rather than the telemetry queue.
- Motor control must handle stale sensor data safely.
- Message timestamps, units, validity and sequence numbers must be documented.
- The receiving subsystem owns its communication object.

Buddy 1 already provides `buddy1_telemetry_submit()`, `buddy1_command_submit()`, `buddy1_init()` and `buddy1_start()`.

An incoming remote command is not yet automatically forwarded to motor control. That routing requires an agreed receiving API.

## 6. Assessment coverage

| Criterion | Principal contributors |
|---|---|
| Mission completion | Entire team |
| Navigation accuracy | Motion, barcode, IMU and mission controller |
| Obstacle avoidance | Vijay, Afiq and mission controller |
| Barcode recognition reliability | Wayne |
| Hump measurement | Ignatius |
| Robustness and resource efficiency | Entire team |
| Software quality and integration | Entire team |
| Demonstration performance | Entire team |

## 7. Integration and handover rules

Each subsystem must provide:

- Public header and implementation
- Two-phase initialisation interface
- Documented task priorities and GPIO ownership
- Defined IPC producer/consumer interfaces
- Unit test evidence
- Handling of invalid or unavailable sensor data
- A pull request for integration

Testing records will be organised under `testing/buddy1/` through `testing/buddy5/`, plus `testing/integration/` and `testing/system/`.

The team will initially integrate using the working single-core profile. SMP support will be validated separately.
