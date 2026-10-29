# Buddy 1 - Wi-Fi, Communication and Telemetry

Owner: Haris

## Test Cases

| ID | Test | Expected Result |
|---|---|---|
| B1-U01 | Two-phase startup | Tasks start successfully |
| B1-U02 | Heartbeat telemetry | Sequence increases continuously |
| B1-U03 | Wi-Fi association | Connection completes successfully |
| B1-U04 | DHCP | Valid IPv4 address obtained |
| B1-U05 | DNS | Hostname resolves successfully |
| B1-U06 | Command IPC | Command received and decoded |
| B1-U07 | Telemetry queue saturation | Producer does not block; drops recorded |

## Test Results

| Date | Test ID | Commit | Tester | Result | Evidence |
|---|---|---|---|---|---|
| | | | | | |

## Notes

Previous serial-monitor runs demonstrated
telemetry, heartbeat, Wi-Fi, DHCP and DNS.

Formal results should reference a specific
commit and saved test evidence.

Command IPC and queue saturation remain
to be explicitly tested.


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
