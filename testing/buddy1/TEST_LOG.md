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
