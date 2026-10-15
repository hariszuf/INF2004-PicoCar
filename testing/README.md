# INF2004 PicoCar - Testing

This directory contains unit, integration and
system testing evidence for the PicoCar project.

## Testing levels

| Level | Location | Purpose |
|---|---|---|
| Unit | buddy1/ to buddy5/ | Test individual buddy components |
| Integration | integration/ | Test interactions between buddies |
| System | system/ | Validate complete robotic car missions |

## Rules

1. Define test cases before executing them.
2. Record the expected results.
3. Record PASS or FAIL only after execution.
4. Save serial logs, screenshots or other evidence.
5. Record the Git commit used for each test.
6. Preserve old results when retesting.
7. Re-run affected tests before merging changes.

## Test identifiers

- B1-U01: Buddy 1 unit test
- B2-U01: Buddy 2 unit test
- B3-U01: Buddy 3 unit test
- B4-U01: Buddy 4 unit test
- B5-U01: Buddy 5 unit test
- INT-01: Integration test
- SYS-01: System test

## Evidence naming

Use YYYY-MM-DD_description.txt for serial logs.

Record the date, commit SHA, tester, result
and evidence path in the relevant TEST_LOG.md.
