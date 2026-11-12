# Buddy 3 - Line following and barcode scanning

Buddy 3 reads the line sensors, decodes Code 39 barcodes, and chooses movements at junctions. Wheel control uses the Buddy 2 motion library; Buddy 3 does not maintain a separate motor PID or encoder interrupt handler.

## Integrated firmware

The integrated RTOS build selects Buddy 3 sources in `build_make/picocar_integration.mk`. The vehicle coordinator starts and stops line following and the barcode task. The former standalone line-following demo and motor diagnostic targets in `app_program/CMakeLists.txt` have been retired.

Barcode decoding is included when the integration profile has USB CDC console or MQTT support. `NO_BARCODE=1` disables starting the barcode task; `BARCODE_ONLY=1` provides a scanning-only mode without driving.

## Sensors and line following

- GP0 and GP2 are the line sensors; GP26 is the barcode sensor.
- `LINE_BLACK_LEVEL` defines the input level that represents black.
- `LINE_GP0_IS_CAR_LEFT` assigns the line sensors to the car's left and right sides.
- The current configuration uses straddle mode: the line runs between the sensors, so both white means centred. One sensor seeing black causes a correction toward that side.
- With junction handling enabled, both line sensors seeing black triggers the junction sequence.
- Buddy 4 tilt observations can temporarily hold the previous wheel commands while the car crosses a ramp or hump.

Pins, speeds, terrain hold, and junction timing are configured in `line_config.h`. Build definitions can override settings guarded by `#ifndef`.

## Barcode scanning

1. `barcode_task.c` samples the GP26 sensor and calls `barcode_scan_update()`.
2. `barcode_scan.c` debounces the signal and measures black-bar and white-gap durations.
3. `barcode39.c` decodes those durations as Code 39 text, including reverse traversal.
4. `barcode_scan_on_decoded()` saves a valid single uppercase A-Z letter and its read timestamp.
5. At the next junction, `line_follow.c` uses the latest unused scan and asks `junction_action_for_letter()` for its action.

The current mapping in `junction_map.c` is:

| Letter | Action |
|---|---|
| A | Left |
| W | Right |
| F | Clockwise U-turn |
| Any other letter | No instruction; stop at the junction with the current default |

The mapping function also accepts lowercase letters. No letter currently maps to straight ahead, although `JUNCTION_STRAIGHT` remains supported.

Scanning stores an instruction for the next junction. It does not immediately turn the car with the current junction configuration. Each scan is consumed once. Without a new mapped letter, the default `JUNCTION_NONE` keeps the car stopped at the junction.

The current junction sequence pauses for 3 seconds, advances 8 cm before a turn, then turns and searches for the outgoing line. Left/right turns target 90 degrees and the U-turn targets 180 degrees. The initial encoder-controlled turn ends 20 degrees before the target; sensor search completes line reacquisition. These distances and angles require verification on the physical track.

## Files

| File | Purpose |
|---|---|
| `line_config.h` | Sensor pins, speeds, terrain and junction settings |
| `line_sensors.c/.h` | Reads the line and barcode sensors |
| `line_follow.c/.h` | Steering, junction sequence and line reacquisition |
| `junction_map.c/.h` | Letter-to-action mapping and action names |
| `barcode_task.c/.h` | RTOS sampling task and latest decoded letter |
| `barcode_scan.c/.h` | Debouncing, run timing and decoder interface |
| `barcode39.c/.h` | Code 39 decoding |
| `test/barcode_test/` | Standalone barcode reader test |
| `test/host/` | Host tests for decoding, mapping and line-following logic |

## Verification

Barcode host tests can be run from `app_program/buddy3/` with a shell and GCC:

```sh
sh test/host/run_barcode_tests.sh
```

Some older host tests and instructions need updating: `test_junction_map.c` still expects the former repeating letter mapping, and `run_tests.sh` uses the old `../Buddy 2` directory name. Review these before relying on their results with the current configuration.

Host tests exercise software logic and simulated inputs; they do not establish physical barcode-reading or turn accuracy. This README update did not rerun tests or verify the firmware on the car.
