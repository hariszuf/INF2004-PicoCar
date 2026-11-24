# Message sizes: text vs binary MQTT (Week 13 evidence)

Before this change every telemetry message was an ASCII line such as
`src=5 type=51 seq=42 width_mm=189 depth_high_mm=300 flags=0`.
Telemetry, the status heartbeat and the dashboard commands are now compact binary frames
(`app_program/telemetry_frame.h` on the car, `dashboard/telemetry_frame.py` on the PC).

## How this was measured

- Script: `python testing/system/measure_message_sizes.py` (run from the repo root).
- It builds one typical message of each type in the old text format and in the new binary format,
  checks that the binary frame decodes back to the same values, and prints the byte counts below.
- The binary bytes are the same ones the firmware codec produces: `testing/system/test_telemetry_frame.c`
  prints test vectors and `dashboard/test_app.py` decodes those exact bytes.
- Values are typical, not worst case. Packet size adds the MQTT fixed header and the topic name
  (`picocar/vehicle1/telemetry`).

## Telemetry payloads

| Message (source/type) | Meaning | Text payload (bytes) | Binary payload (bytes) | Saved |
|---|---|---|---|---|
| 1/1 | heartbeat_marker | 60 | 5 | 91% |
| 2/3 | left_speed_mm_s | 72 | 7 | 90% |
| 2/4 | left_encoder_ticks | 81 | 9 | 88% |
| 2/10 | odometry_mm | 62 | 7 | 88% |
| 3/5 | ir_mask | 53 | 7 | 86% |
| 3/6 | barcode_ascii | 67 | 8 | 88% |
| 3/21 | map_a_to_j | 69 | 13 | 81% |
| 3/22 | map_u_to_z | 74 | 13 | 82% |
| 4/7 | pitch_cdeg | 59 | 9 | 84% |
| 4/11 | sample_age_ms | 61 | 8 | 86% |
| 4/15 | wheel_height_mm | 72 | 9 | 87% |
| 4/19 | highest_hump_mm | 59 | 9 | 84% |
| 4/20 | lift_reason | 71 | 12 | 83% |
| 5/50 | obstacle_state | 65 | 6 | 90% |
| 5/51 | width_mm | 62 | 9 | 85% |
| 5/52 | bypass_side | 66 | 8 | 87% |
| 5/54 | front_clearance_mm | 76 | 8 | 89% |
| 5/55 | obstacle_id | 69 | 10 | 85% |
| 5/56 | obstacle_validity_mask | 75 | 7 | 90% |
| 5/58 | raw_range_mm | 68 | 9 | 86% |
| 5/59 | box_edges_packed | 78 | 13 | 83% |
| 5/60 | exec_state_and_start_result | 94 | 8 | 91% |
| 5/61 | scan_m35_to_m20 | 81 | 13 | 83% |
| **Total (23 message types)** | | **1594** | **207** | **87%** |

Whole MQTT PUBLISH packet (fixed header + topic 'picocar/vehicle1/telemetry' + payload), same 23 messages: 2284 bytes as text, 897 bytes as binary (60% smaller).

| Other messages | Text (bytes) | Binary (bytes) |
|---|---|---|
| Status heartbeat (`heartbeat seq=12345`) | 19 | 5 |
| Command START | 5 | 1 |
| Command STOP | 4 | 1 |
| Command MAP (26 letters) | 30 | 11 |
| Command MAPRESET | 8 | 1 |

## Design notes

- Frame: 1 byte header (marker + size class), source/type, a variable-length sequence number and only the
  value bytes that are needed (1, 2 or 4 bytes each, chosen per message), so small values cost one byte.
- The 5-byte heartbeat is the minimum frame: it carries source, type and sequence only.
- Commands are one opcode byte. `MAP` carries the 26 barcode letters as 2 bits each in 11 bytes in total.
- The connection handshake (`startup_probe=`, `dashboard_ready=`) stays as text on purpose. It is sent once
  per connection, is human readable when debugging with `mosquitto_sub`, and is not in the periodic traffic.
- Old text-only dashboards cannot decode the binary frames; the updated `dashboard/app.py` must be used.

## Correctness

- C codec round trip including extreme values (`INT32_MIN`, `INT32_MAX`, 24-bit sequence): `test_telemetry_frame.c`.
- Python codec matches the C vectors byte for byte: `dashboard/test_app.py` (13 tests pass).
- Malformed frames (truncated, wrong length, reserved bits) are ignored by the dashboard, not mis-decoded.
