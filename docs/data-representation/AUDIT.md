# Data representation audit (draft)

Status: first pass, read from the code on branch `Tribunal-1-Changes`. Fields marked (verify) have not been read
line by line yet. "Min bits" is the smallest width that holds the valid range. "Now" is the current type.

## 1. Baseline (before any change)

| Item | Value | Source |
|---|---|---|
| Telemetry message (`telemetry_message_t`) | 20 bytes | `buddy1/telemetry.h` |
| Inter-buddy event (`robot_event_t`) | 20 bytes | `robot_messages.h` |
| Telemetry input buffer | 48 slots, (5 + 4) words each = 1728 bytes | `buddy1/telemetry.c` |
| MQTT outbox buffer | 48 slots, 1728 bytes | `buddy1/telemetry.c` |
| MQTT telemetry payload | text, `src=%u type=%u seq=%u <name>=%d <name>=%d flags=%u`, buffer 192 bytes | `lib/libnet/lwip/mqtt_utk.c` |
| MQTT status payload | text, buffer 48 bytes | `lib/libnet/lwip/mqtt_utk.c` |
| MQTT commands | text: `START`, `STOP`, `MAP <26 letters>`, `MAPRESET` | `buddy1/command.c` |
| Static RAM (.data + .bss) | measure with `arm-none-eabi-size` on the tagged build | Step 1 |

## 2. Fields by area

### Telemetry and messaging (Buddy 1)
| Field | Valid range | Min bits | Now |
|---|---|---|---|
| `source` | 1..5 | 3 | uint8_t |
| `type` | 1..60 | 6 | uint8_t |
| `flags` | per type, up to 16 flags | 1 per flag | uint16_t |
| `sequence` | wraps | 16 (needs wrap-aware compare in the dashboard) | uint32_t |
| `value_1`, `value_2` | per type; many are 1 to 16 bit | per type | int32_t |
| `queue depth` | 48 | n/a | 48 |

### Motion control and encoders (Buddy 2)
| Field | Valid range | Min bits | Now |
|---|---|---|---|
| `motion_state_t` | 4 states | 2 | enum |
| `motion_result_t` | 5 results | 3 | enum |
| `motion_fault_t` | none / e-stop / stall | 2 as bits in the shared fault mask | enum |
| `motion_terrain_t` | 5 values | 3 | enum |
| `motion_side_t` | 0..1 | 1 | enum |
| `pitch_state`, `roll_state` | -1, 0, +1 | 2 | int8_t |
| `valid` (terrain input) | 0..1 | 1 | bool |
| encoder counts | signed, run length (verify) | 24 likely | int32_t |
| speeds `left/right/speed` | about -50..+50 cm/s, 0.1 resolution | 11 (fixed point) | float |
| `distance_cm`, `heading_deg` | verify | verify | float |

### Line and barcode (Buddy 3)
| Field | Valid range | Min bits | Now |
|---|---|---|---|
| `line_state_t` | 9 states | 4 | enum |
| `sensor_mask` | 3 sensors | 3 | uint8_t |
| barcode letter | A..Z (+ none) | 5 | char |
| `junction_action_t` | 5 values | 3 | enum |
| junction map | 26 letters x 3 bits | 78 (already packed into 3 words for telemetry) | table |
| barcode `fail_count` / `last_fail` | verify | verify | uint32_t / uint8_t |

### IMU, terrain and hump (Buddy 4)
| Field | Valid range | Min bits | Now |
|---|---|---|---|
| `valid`, `acceleration_valid`, `tilt_valid`, `transient_event`, `event_ended`, `height_geometry_confirmed`, `peak_confirmed`, `median_candidate_valid`, `window_active`, ... | 0..1 each | 1 each, grouped into one flags word | about 12 separate bool |
| `clipped_axes` | XYZ | 3 | uint8_t |
| `terrain`, `side` | small enums | 3 and 2 | int |
| `quality` | verify | verify | enum |
| `error_count`, `clipped_samples`, `unsuitable_samples` | saturating counters | 8 to 16 | uint32_t |
| `wheel_height_cm`, `pitch/roll` | about 0..10 cm, -90..90 deg, 0.1 resolution | 7 to 12 | float |
| median buffers | 11 samples | n/a | float + uint64_t |

### Obstacle detection (Buddy 5)
| Field | Valid range | Min bits | Now |
|---|---|---|---|
| `b5_state_t` | 7 states | 3 | enum |
| `b5_fault_t` | 9 values | 4, or one bit each in the shared fault mask | enum |
| `b5_request_kind_t` | 10 values | 4 | enum |
| `side` | -1 / +1 | 1 | int8_t |
| `side_reason` | 0..5 | 3 | int8_t |
| `phase` | small | 3 (verify) | uint8_t |
| `slot` | 0..12 | 4 | uint8_t |
| scan `distance[13]` | 0..4000 mm | 12 | uint16_t |
| scan `state[13]` | 0..2 | 2 | uint8_t |
| `blocked_mask` | 2 bits | 2 | uint8_t |
| `hit_count`, `false_alarms`, `adv_close`, `chk_n`, `extends`, `turn_fixes`, `block_fixes` | small counters | 2 to 4 | uint8_t |
| bools in `b5_feedback_t`, `b5_profile_t`, `b5_core_t` (about 15) | 0..1 | 1 each, grouped | bool |
| profile/plan distances (`width_mm`, `offset_mm`, `pass_x_mm`, edges, rooms, costs) | about 0..2000 mm, signed | 12 to 13 | int32_t |

### Vehicle controller
| Field | Valid range | Min bits | Now |
|---|---|---|---|
| `execution_state` | 6 states | 3 | int32_t |
| stop reason | 15 reasons | 4 | int32_t |
| start result | 5 values | 3 | packed already in 5/60 |
| automatic restarts | small counter | 8 | packed already in 5/60 |
| `run_enabled`, `buddy4_ready`, `user_stop_pending`, `remote_start_pending`, `remote_start_rejected`, `fault_latched`, `request_pending` | 0..1 each | 1 each, into one atomic flag word | 7 separate bool/volatile |
| `commands[8]` ring | small command codes | 3 | unsigned char |

## 3. Shared data that needs safe access
Read-modify-write on these must be protected once they are packed into one word.
- Controller flags above (`run_enabled`, `user_stop_pending`, `remote_start_pending`, `fault_latched`): written by the
  controller task, the MQTT command path and USB input. They are `volatile` but not atomic.
- `feedback_shared` and `pending_request` (Buddy 5 to controller): guarded by a critical section today (verify).
- Encoder ISR counters (Buddy 2) and ultrasonic ISR data (Buddy 5): verify.
- Telemetry and MQTT outbox message buffers: kernel message buffers, already serialised.

## 4. Proposed targets (not implemented)
1. One shared fault bitmask for: motor stall, e-stop, encoder fault, IMU fault, no echo, sonar/servo fault, Wi-Fi lost,
   MQTT lost, line lost, lifted/overturned. Replaces the separate fault enums and bools.
2. Group the many `bool` fields per module into one flags word (bit-fields or masks).
3. Narrow enums and small counters to the widths in the tables above.
4. Compact binary MQTT telemetry frame and command frame (fixed layout, one type byte), replacing the text lines.
5. Atomic flag word for the controller flags.

## 5. Open items
- Verify every "(verify)" entry and read the remaining counters and sensor structs.
- Confirm valid ranges against the hardware (encoder counts per run, speeds, distances).
- Decide the binary frame layout, and the dashboard (`app.py`, `dashboard.js`) changes that go with it.
