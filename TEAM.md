# Team and subsystems

Team 08, INF2004 PicoCar. This file is the single place that says **who owns
what**, so that five people can work in one firmware image without clashing
over pins, priorities, or kernel objects. The values below were taken from the
code on `main` (`app_program/`, `config/config.h`) and from
`CarComponentsandConnections.md`.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Buddy 1: Wi-Fi, MQTT, remote commands, telemetry and heartbeat | _add student ID_ | `hariszuf` | `app_program/buddy1/` | `testing/buddy1/` |
| 2 | Buddy 2: motion control (motor PWM, wheel encoders, PID) | _add student ID_ | `AfiqDanial2003` | `app_program/buddy2/` | `testing/buddy2/` |
| 3 | Buddy 3: line following, junction map and Code 39 barcode reading | _add student ID_ | `waayyne` | `app_program/buddy3/` | `testing/buddy3/` |
| 4 | Buddy 4: IMU, tilt, hump and terrain measurement | _add student ID_ | `Ignatius-Pang` | `app_program/buddy4/` | `testing/buddy4/` |
| 5 | Buddy 5: obstacle detection (ultrasonic + servo scan) and bypass | _add student ID_ | `CBKia222` | `app_program/buddy5/` | `testing/buddy5/` |

Shared files (`app_program/app_main.c`, `app_program/robot_*.h`,
`app_program/telemetry_frame.h`, `app_program/vehicle/` and this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S3 | Right IR line sensor DO (`LINE_PIN_GP0_IR`). Conflicts with `CRASH_REPORT=1`, so build without it |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | S3 | Centre IR line sensor DO (`LINE_PIN_GP2_IR`) |
| GP3 | | |
| GP4 | S2 | Right wheel encoder, phase A only (`MOTION_PIN_RIGHT_ENCODER`) |
| GP5 | | |
| GP6 | S2 | Left wheel encoder, phase A only (`MOTION_PIN_LEFT_ENCODER`) |
| GP7 | S5 | Ultrasonic echo (`B5_ECHO_PIN`) |
| GP8 | S2 | Left motor M1A (PWM) |
| GP9 | S2 | Left motor M1B (PWM) |
| GP10 | S2 | Right motor M2A (PWM) |
| GP11 | S2 | Right motor M2B (PWM) |
| GP12 | S5 | Servo signal, rotates the ultrasonic sensor (`B5_SERVO_PIN`, PWM) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S4 | IMU I2C0 SDA |
| GP17 | S4 | IMU I2C0 SCL |
| GP18 | | |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | S3 | Barcode IR sensor DO (`LINE_PIN_BARCODE_IR`). Reads the barcode only, never steers the car |
| GP27 | | (ADC1 capable) |
| GP28 | S5 | Ultrasonic trigger (`B5_TRIG_PIN`), used as a digital output, not as ADC2 |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | not used |
| I2C0 | S4 | Buddy 4 task (`buddy4_task.c`) | GP16, GP17 |
| I2C1 | | | not used |
| SPI0 | | | not used |
| SPI1 | | | not used |
| ADC | | | not used (GP26 and GP28 are used as digital pins) |
| PWM slices (list) | S2: slice 4 (GP8, GP9) and slice 5 (GP10, GP11). S5: slice 6 (GP12) | S2: motion update in the vehicle `controller_task`. S5: `scan_task` | GP8–GP12 |
| GPIO interrupts (wheel encoders) | S2 | `motion_encoder_irq_handler` (counts only, no blocking calls) | GP4, GP6 |
| Digital inputs (IR sensors) | S3 | Barcode task for GP26 (`barcode_task.c`). `controller_task` reads GP0 and GP2 through `line_sensors` | GP0, GP2, GP26 |
| Ultrasonic trigger and echo | S5 | `scan_task` | GP28, GP7 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
  Buddy 1 does this in its network monitor task.
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. Buddy 1 owns the
  application side: other subsystems hand telemetry to Buddy 1 with
  `buddy1_telemetry_submit()` (section 5) and never call MQTT themselves.
  Changes to shared library code must be **agreed with the team and
  the instructor first**, and recorded as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each
subsystem gets its own band by default; change it only by team agreement.
Write down every task you create. The bands below are the values used in the
code today.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 6–9 | command consumer – 6 – any; telemetry consumer – 7 – any; heartbeat producer – 8 – any; network monitor – 9 – any |
| 2 | none | no task of its own. The motion update runs inside the vehicle `controller_task` (`motion_task.c` is a skeleton that the integrated build does not create) |
| 3 | 6 | barcode task – 6 – any |
| 4 | 10 | Buddy 4 sampling task – 10 – any (plus one cyclic handler, section 6) |
| 5 | 3 | `scan_task` – 3 – any |
| Integration (`app_program/vehicle/`) | 2 | `controller_task` – 2 – any |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any". The tested profile is
single core (`SMP=0`), so no task is pinned to a core.

Known clashes to settle as a team:

- Priority 6 is used by both Buddy 1's command consumer and Buddy 3's barcode task.
- `app_program/buddy5/buddy5_config.h` defines `B5_TASK_PRIORITY` as 11, but
  `buddy5_task.c` creates `scan_task` at priority 3, and the vehicle
  `controller_task` is at 2. Both are inside the kernel's reserved 1–4 range.

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own header; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2, S3, S4, S5 and `controller_task` → S1 | message buffer (owned by S1) | `telemetry_buffer_id` (static in `telemetry.c`) | frames from `telemetry_frame.h`, sent with `buddy1_telemetry_submit()` | Team 08 (as implemented) |
| IF-02 | remote dashboard → S1 → `controller_task` | message buffer (owned by S1) | `command_buffer_id` (static in `command.c`) | START / STOP / PAUSE / RESUME commands, sent with `buddy1_command_submit()` | Team 08 (as implemented) |
| IF-03 | S4 → `controller_task` | shared data under a mutex + event flag + cyclic handler (owned by S4) | `data_mutex`, `release_flag`, `release_cyclic` | IMU tilt, hump and impact results from `buddy4_task.h`, read only while locked | Team 08 (as implemented) |
| IF-04 | `controller_task` ↔ S5 | shared request and feedback structs (owned by S5) | `pending_request`, `feedback_shared` | `b5_request_t` (scan or bypass request) and `b5_feedback_t` (front clear, scan result) | Team 08 (as implemented) |
| IF-05 | `controller_task` → S2 | direct calls that only queue (owned by S2) | none | `motion_*` calls in `motion_control.h`: move, spin, drive, stop | Team 08 (as implemented) |
| IF-06 | S3 → `controller_task` | direct function calls (owned by S3) | none | line sensor state (`line_sensors.h`), line-follow output (`line_follow.h`), decoded barcode letters and the junction map (`junction_map.h`) | Team 08 (as implemented) |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

Shared communication rules:

- The receiving subsystem owns its communication object.
- Emergency stop requests use a high-priority control path, not the telemetry queue.
- Motor control must stop safely when sensor data is stale or invalid.
- Message timestamps, units, validity and sequence numbers must be documented
  in the owning subsystem's header.

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below count the objects created in the code today.
The kernel's USB and WiFi service tasks also use a few of these.

| Object | Limit | S1 | S2 | S3 | S4 | S5 | Integration |
|---|---|---|---|---|---|---|---|
| Task | 32 | 4 | 0 | 1 | 1 | 1 | 1 |
| Semaphore | 16 | | | | | | |
| Event flag | 16 | | | | 1 | | |
| Mailbox | 4 | | | | | | |
| Mutex | 4 | | | | 1 | | |
| Message buffer | 4 | 3 | | | | | |
| Fixed memory pool | 4 | | | | | | |
| Variable memory pool | 4 | | | | | | |
| Cyclic handler | 4 | | | | 1 | | |
| Alarm handler | 4 | | | | | | |

Buddy 1's three message buffers are the command buffer, the telemetry buffer
and the MQTT outbox. The outbox is only created in the Wi-Fi/MQTT build.
