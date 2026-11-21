# Minimum-bit data representation: evidence index

Branch: `Tribunal-1-Changes`. Baseline code: git tag `baseline-before-bitpacking`.

| Required evidence | File | Status |
|---|---|---|
| Data representation table | [AUDIT.md](AUDIT.md) (becomes the final table) | draft, fields being converted stage by stage |
| RAM usage before and after | [RAM_COMPARISON.md](RAM_COMPARISON.md) | baseline + Stage 1 recorded |
| Message size before and after | MESSAGE_SIZES.md | done (Stage 3, binary MQTT; 87% smaller telemetry payloads) |
| Correctness and performance | CORRECTNESS.md | host tests recorded per stage, on-car results to add |

## Stages
1. Controller flags in one lock-protected word, shared fault bitmask (`robot_flags.h`, `robot_faults.h`).
2. Buddy 5 structs: bit-fields and a packed scan (`bitpack.h`, `buddy5_core.h`).
3. Binary MQTT telemetry and commands (firmware, dashboard backend, dashboard page).
4. Buddy 2, 3 and 4 data, then the final table.

## Host tests (run from the repository root)
```
gcc -std=c11 -Wall -Wextra -Iapp_program testing/system/test_robot_flags.c -o rf
gcc -std=gnu11 -Wall -Wextra -Iapp_program -Iapp_program/buddy5 testing/system/test_bitpack.c -o bp
```
