# Buddy 5 - Obstacle Detection

Owner: Vijay

## Test Cases

Keep the car stationary for these tests. Thresholds are provisional, not validated driving stop distances.

| ID | Test | Procedure | Expected Result |
|---|---|---|---|
| B5-U01 | Basic sensor response | Move a flat object closer and farther away. | Distance decreases/increases respectively. |
| B5-U02 | State logic (host) | Compile and run test_obstacle_state.c. | All assertions pass, including exact boundaries and every sensor error code. |
| B5-H01 | Obstacle entry | Begin above 250 mm; move object to 200 mm or less as reported by sensor. | OBSTACLE at <= 200 mm. |
| B5-H02 | Hysteresis | From OBSTACLE move to 225 mm, then >= 250 mm; from CLEAR move to 225 mm, then <= 200 mm. | Retain OBSTACLE at 225, clear at >= 250; retain CLEAR at 225, obstacle at <= 200. |
| B5-H03 | Fault and recovery | With power off disconnect sensor; power on and observe. Power off, reconnect, then retest. | Invalid readings report SENSOR FAULT; valid <= 200 reports OBSTACLE; valid >= 250 reports CLEAR. A valid reading between thresholds retains the previous state, including SENSOR FAULT. |

## Host test command (PowerShell, repository root)

```powershell
g++ -std=c++11 -Wall -Wextra -Werror testing/buddy5/test_obstacle_state.c -o testing/buddy5/test_obstacle_state.exe
.\testing\buddy5\test_obstacle_state.exe
```

The header contains the actual state logic used by the firmware. The older test_distance.c tests a separate copy of the conversion formula, not the driver.

## Test Results

| Date | Test ID | Commit | Tester | Result | Evidence |
|---|---|---|---|---|---|
| 2026-10-15 | B5-U01 | Not recorded | Vijay | User reported expected direction of distance changes; accuracy against ruler not measured | Serial screenshot in chat; not yet saved under logs/ |

B5-U02 execution is reported in the implementation chat. B5-H01 through B5-H03 require testing after flashing the new firmware; no hardware PASS is claimed here.

## Behaviour and limits

- Initial state is SENSOR FAULT until a valid reading crosses either threshold.
- Every measurement prints its distance and state, or a sensor fault and error code.
- Sampling retains the existing 100 ms delay after each measurement.
- State reporting does not issue motor commands or detect a stalled sensor task. Motor integration and stale-data handling remain future work.
- Do not unplug sensor wiring while powered. An unobstructed sensor may report a distant surface or a timeout; absence of an object is not guaranteed to produce a fault.
- Save serial evidence under logs/ and record the firmware commit and actual hardware results before merging.
