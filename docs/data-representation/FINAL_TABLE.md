# Final data representation table

Each field uses the fewest bits that cover its valid range. "Shared" means more than one task or an interrupt can
write it, so it is packed only into a lock-protected word (`robot_flags.h`: writes run with interrupts masked,
reads are one 32-bit load).

| Subsystem | Field | Valid range | Bits used | Was | How | Shared / safe access |
|---|---|---|---|---|---|---|
| Controller | run enabled, Buddy 4 ready, user stop, start pending, start rejected, fault latched, request pending | 0..1 each | 1 each, 1 word | 7 bool/volatile | `robot_flags_t ctl_flags` | locked read-modify-write |
| All | fault mask (motor stall, e-stop, lift, navigation, Buddy 5 faults, MQTT lost, ...) | 12 conditions | 12 bits, 1 word | separate fields | `robot_faults.h`, `fault_mask` | locked update, one reader |
| Buddy 2 | initialised, e-stop request, pending command, terrain input present | 0..1 each | 1 each, 1 word | 4 bool | `s_flags` in `motion_control.c` | locked; set by any task |
| Buddy 2 | state, fault, terrain | 4, 3, 5 values | 8 each (enums stored in 1 byte) | 32-bit enums | `uint8_t` | single writer |
| Buddy 2 | wheel direction | -1 / +1 | 8 | int32 | `int8_t` | single writer |
| Buddy 3 | scanner started, level, pending, result ready | 0..1 each | 1 each, 1 word | 4 bool | `s_flags` in `barcode_scan.c` | locked; result flag crosses tasks |
| Buddy 4 | valid, acceleration valid, tilt valid, transient, height confirmed, event ended | 0..1 each | 1 each | 6 bool | bit-fields in `buddy4_output_t` | single writer (IMU task) |
| Buddy 4 | quality | 5 values | 3 | enum | bit-field | single writer |
| Buddy 4 | terrain, side | -1..4, -1..1 | 8 each | int | `int8_t` | single writer |
| Buddy 4 | peak confirmed, median candidate valid, window active | 0..1 each | 1 each | 3 bool | bit-fields in `side_event_t` | single writer |
| Buddy 4 | ready, has sample, filter valid, tilt filter valid, bump armed, had bump | 0..1 each | 1 each | 6 bool | bit-fields in `buddy4_imu_t` | single writer |
| Buddy 5 | scan distance / state, 13 slots | 0..4116 mm, 0..2 | 13 + 2 bits per slot, 25 bytes | 40 bytes | `bitpack.h` | single writer |
| Buddy 5 | state, fault, request kind, side, slot, phase, blocked mask, 7 flags, counters | small | 1 to 4 bits each | int / bool | bit-fields in `b5_*` structs | feedback/request guarded by critical section |
| MQTT telemetry | whole frame | per message | 5 to 15 bytes | 53 to 94 bytes of text | `telemetry_frame.h` | one message buffer per message |
| MQTT commands | START, STOP, MAP | 1 opcode | 1 to 11 bytes | 4 to 30 bytes of text | `telemetry_frame.h` | command path |

Left at full width on purpose: millimetre values that use `INT32_MAX` as "nothing ahead", and millisecond timestamps that
must not wrap. Packing them would add code and risk for almost no RAM.

Evidence: [RAM_COMPARISON.md](RAM_COMPARISON.md), [MESSAGE_SIZES.md](MESSAGE_SIZES.md), [CORRECTNESS.md](CORRECTNESS.md).
