# Subsystem 1 — test log

**Subsystem:** Buddy 1: Wi-Fi, MQTT, commands, telemetry  ·  **Student ID:** _add student ID_  ·  **Code:** `app_program/buddy1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | Two-phase startup: tasks start |  | manual | Flash image; watch console | Telemetry and networking tasks run |
| S1-U02 | Heartbeat telemetry |  | manual | Watch telemetry | Sequence increases continuously |
| S1-U03 | Wi-Fi association |  | manual | Watch console | Connection completes |
| S1-U04 | DHCP |  | manual | Watch console | Valid IPv4 address obtained |
| S1-U05 | DNS (`example.com`) |  | manual | Watch console | Hostname resolves |
| S1-U06 | Command IPC | IF-01 | manual | Submit a command | Command received and decoded |
| S1-U07 | Telemetry queue saturation |  | manual | Flood the queue | Producer does not block; drops recorded |

*Method:* **auto** = `TEST_CHECK` or host program, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Notes

Serial-monitor runs during development showed S1-U01 to S1-U05 working. They are
not formal evidence until repeated against a recorded commit. S1-U06 and S1-U07
still need explicit tests.

## Finalization checkpoint — 29 October 2026

Runtime commit `0f07d6d`; review branch `review/buddy1-finalization`.
Three user-performed power-bank cold boots and live MQTT heartbeat/telemetry
were observed. START/STOP application consumption on this exact UART-only
image, live Wi-Fi/broker reconnection and combined-controller operation remain
hardware NOT RUN. Host parser/IPC and recovery-characterization tests are
software evidence, not physical passes.

See [the final review](../../docs/BUDDY1_FINAL_REVIEW.md) and `testing/subsystem1/finalization/2026-10-29/`
for firmware hash, build/test commands, raw evidence and remaining gates.
The minimum-bit tribunal is not an implementation requirement under the
lecturer clarification relayed by the user.

Earlier IDs `B1-U01`..`B1-U07` are now `S1-U01`..`S1-U07`. Hardware checks `B1-H01`..`B1-H11`
stay in `docs/BUDDY1_HARDWARE_CHECKLIST.md`.
