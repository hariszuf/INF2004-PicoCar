# Barcode (Code 39) decoder - DRAFT for Buddy 3

Written by Afiq (Buddy 2) with Claude as a starting point. Wayne (Buddy 3) owns the real
barcode code, so this lives outside `app_program/buddy3/`. Nothing here touches other buddies' files.

## What it does

- Decodes Code 39 `*` + letter + `*` from a stream of black/white run lengths.
- Hardware independent C (no Pico SDK, no RTOS). Feed it run lengths in any unit
  (microseconds, ticks, or encoder counts). Only ratios matter, so speed does not need to be fixed.
- Classifies each 9-element character as "3 widest = wide". No fixed timing threshold.
- Rejects bad reads: wide/narrow not clearly different, wide inter-character gap, bad start/stop, non-letters.
- Detects direction from the start/stop `*` (mirrored `*` has the pattern of `P`), so a code crossed
  backwards still decodes to the same letter; `result.reversed` says which way.
- Re-syncs through noise (line edges, dirt) before the code.

## Verified

`sh run_tests.sh` (gcc only): 233 checks pass, using

- the measured widths of the paper barcode photo (reads **F**),
- the six deck barcodes A, F, Z, W, X, Y (rendered from `BARCODE.pptx`; the table was also checked against
  all 12 barcode slides, 3 sizes each of A/F/Z),
- all 26 letters at 2x and 3x ratio, both directions,
- speed scale 0.3x to 6x, +-15% drift, noise prefix, back-to-back codes, rejects.

Not verified: real IR sensor timing on the car.

## Using it on the car (sketch for Wayne)

```c
barcode39_t dec;
barcode39_init(&dec, true);            /* letters A-Z only */

/* GPIO edge IRQ on the barcode IR pin (GP26): store (level, timestamp) in a queue.
 * In the Buddy 3 task, turn consecutive timestamps into runs and feed them: */
bool is_black = /* depends on sensor polarity: check line_sensors.c */;
uint32_t width_us = t_now - t_prev;

barcode39_result_t r;
if (barcode39_feed(&dec, is_black, width_us, &r)) {
    /* r.text[0] is the letter; send it as a decoded-barcode event to the mission controller (IF-01) */
}
```

Things to settle with the team:

1. **What each letter does** is not in the brief. Ask the lecturer (tribunal).
2. **Sensor polarity** (does DO go high on black or white) and debounce on edges.
3. **Speed over the barcode** with Buddy 2 (constant, slow). Narrow bar width on the real printout in mm
   decides the minimum timer resolution: width_us = narrow_mm / speed.
4. If the print has a larger narrow:wide ratio than about 2:1 (it is about 3:1 in the deck),
   `cfg.min_wide_ratio_x10` (default 18) can be tuned.
5. The barcode GPIO is also read by `line_sensors_read()`. Do not have two owners of GP26.

## Files

| File | Purpose |
|---|---|
| `barcode39.h/.c` | decoder |
| `test_barcode39.c`, `run_tests.sh` | host tests |
