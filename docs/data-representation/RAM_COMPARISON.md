# RAM usage comparison

Static RAM = `.data` + `.bss` from `arm-none-eabi-size` on `picocar_integration_headless_mqtt.elf`.
Raw outputs are kept in `evidence/` and `local-artifacts/`.

| Build | text (flash) | data | bss | Static RAM | Change vs baseline |
|---|---|---|---|---|---|
| Baseline (`baseline-before-bitpacking`) | 393796 | 2608 | 53368 | 55976 | |
| Stage 1: controller flags + fault mask | see `evidence/size_stage1.txt` | 2608 | 53368 (map: 53176, unchanged) | 55976 | 0 B (flash +224 B) |
| Stage 2: Buddy 5 structs | 394904 | 2608 | 53336 | 55944 | -32 B (flash +1108 B vs baseline) |
| Stage 3: binary MQTT telemetry and commands | 394060 | 2608 | 53336 | 55944 | -32 B (flash +264 B vs baseline, -844 B vs Stage 2) |

Stage 3 changes the wire format, not static RAM: its gain is message size (see `MESSAGE_SIZES.md`), and flash even drops
because the text formatting code is gone.

Stage 1 saves no RAM on its own: seven one-byte flags become two 32-bit words. Its purpose is safe shared access.

## Struct sizes (host build, bytes)
| Struct | Before | After Stage 2 | How |
|---|---|---|---|
| `b5_scan_t` | 40 | 25 | 13 slots x (13-bit distance + 2-bit state) packed in a byte array |
| `b5_feedback_t` | 28 | 20 | 7 flags as 1-bit fields, three counters share one word |
| `b5_profile_t` | 56 | 52 | 5 flags + side (2 bits) + reason (3 bits) in 2 bytes |
| `b5_core_t` | 248 | 224 | state (3 bits), fault (4 bits), request flags, slot, phase, side, blocked mask as bit-fields |

Fields deliberately left at 32 bits: millimetre values that carry sentinels (`INT32_MAX` for "nothing ahead") and
millisecond timestamps that must not wrap.
