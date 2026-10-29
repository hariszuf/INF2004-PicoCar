# Buddy 1 — Test Log

## Test cases

| ID | Test | Expected result |
|---|---|---|
| B1-U01 | Two-phase startup | Telemetry and networking tasks run |
| B1-U02 | Heartbeat telemetry | Sequence increases continuously |
| B1-U03 | Wi-Fi association | Connection completes successfully |
| B1-U04 | DHCP | Valid IPv4 address obtained |
| B1-U05 | DNS | `example.com` resolves successfully |
| B1-U06 | Command IPC | Submitted command is received and decoded |
| B1-U07 | Telemetry queue saturation | Producer remains non-blocking; dropped count recorded |

## Results

Record the test date, tested commit SHA, PASS/FAIL status and saved console log for each test.

The serial captures obtained during development show successful operation of B1-U01 through B1-U05. These should be repeated or recorded against a specific commit before treating them as formal test evidence.

B1-U06 and B1-U07 still require explicit testing.

## Integration tests

Future integration tests should verify:

- Remote command reception and delivery to the motion controller
- Telemetry reception from other buddies
- Network reconnection and failure recovery
- Operation while motor and sensor tasks are active


## Finalization checkpoint — 29 October 2026

Runtime commit `0f07d6d`; review branch `review/buddy1-finalization`.
Three user-performed power-bank cold boots and live MQTT heartbeat/telemetry
were observed. START/STOP application consumption on this exact UART-only
image, live Wi-Fi/broker reconnection and combined-controller operation remain
hardware NOT RUN. Host parser/IPC and recovery-characterization tests are
software evidence, not physical passes.

See [the final review](../../docs/BUDDY1_FINAL_REVIEW.md) and `testing/buddy1/finalization/2026-10-29/`
for firmware hash, build/test commands, raw evidence and remaining gates.
The minimum-bit tribunal is not an implementation requirement under the
lecturer clarification relayed by the user.
