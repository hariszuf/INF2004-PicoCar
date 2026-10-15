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
