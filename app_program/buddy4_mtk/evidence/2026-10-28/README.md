# B4-DIAG-E user hardware capture

Operator-provided serial recording, 28 October 2026, COM5.

- 40 displayed status blocks, all B4-DIAG-E USB-only.
- Cumulative I2C errors and skipped sampling slots remained zero throughout.
- Every displayed poll is FRESH, with 20 ms poll-start interval.
- Displayed reading ages range from 10 to 20 ms.
- Angles change substantially later in the capture; it includes handling/tilting,
  so it is not a purely stationary baseline or a height-accuracy validation.
- The displayed zero transfer duration is from a coarse 10 ms clock, not a
  physical zero-duration transfer. 1 Hz snapshots do not expose every sample.
- This run supports improved communication/cadence in Buddy 4-only mode. It
  does not validate longer operation, Buddy 1/5 coexistence, motor operation,
  collision discrimination, height accuracy or final course integration.

Raw capture SHA-256: 57f945dc4c05448411dfc8096344893b63230bac49d5f500a832040c3f671146
