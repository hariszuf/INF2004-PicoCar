"""Compare the old text MQTT payloads with the new binary frames for typical values.

Run from the repository root:  python testing/system/measure_message_sizes.py
The old format is the one the firmware used before this change:
    src=%u type=%u seq=%u <name1>=%d <name2>=%d flags=%u
Names come from the old firmware field table. Values are typical ones, not worst case.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "dashboard"))
import telemetry_frame as tf  # noqa: E402

TOPIC = "picocar/vehicle1/telemetry"
# (source, type, name1, name2, typical value1, typical value2, flags)
SAMPLES = [
    (1, 1, "heartbeat_marker", "reserved", 0, 0, 0),
    (2, 3, "left_speed_mm_s", "right_speed_mm_s", 100, -100, 0),
    (2, 4, "left_encoder_ticks", "right_encoder_ticks", 12345, 12340, 0),
    (2, 10, "odometry_mm", "motor_fault", 1500, 0, 0),
    (3, 5, "ir_mask", "line_state", 5, 1, 0),
    (3, 6, "barcode_ascii", "barcode_age_ms", 65, 1500, 0),
    (3, 21, "map_a_to_j", "map_k_to_t", 1234567, 2345678, 0),
    (3, 22, "map_u_to_z", "map_version_result", 123456, 65537, 0),
    (4, 7, "pitch_cdeg", "roll_cdeg", -123, 45, 1),
    (4, 11, "sample_age_ms", "imu_errors", 12, 0, 1),
    (4, 15, "wheel_height_mm", "measurement_quality", 14, 2, 1),
    (4, 19, "highest_hump_mm", "run_id", 14, 1, 1),
    (4, 20, "lift_reason", "lift_count_and_tilt", 1, 65537, 1),
    (5, 50, "obstacle_state", "obstacle_fault", 3, 0, 0),
    (5, 51, "width_mm", "depth_high_mm", 189, 300, 0),
    (5, 52, "bypass_side", "bypass_offset_mm", 1, 350, 0),
    (5, 54, "front_clearance_mm", "servo_command_deg", 250, -20, 0),
    (5, 55, "obstacle_id", "report_uptime_ms", 3, 123456, 0),
    (5, 56, "obstacle_validity_mask", "telemetry_drops", 15, 2, 0),
    (5, 58, "raw_range_mm", "raw_range_status", 187, 0, 16),
    (5, 59, "box_edges_packed", "decision_info", 12345678, 654321, 0),
    (5, 60, "exec_state_and_start_result", "stop_reason_and_auto_restarts", 2, 0, 16),
    (5, 61, "scan_m35_to_m20", "scan_m15_to_0", 987654321, 123456789, 0),
]
SEQ = 12345


def mqtt_packet(payload_len):
    remaining = 2 + len(TOPIC) + payload_len
    return 1 + (1 if remaining < 128 else 2) + remaining


rows, old_total, new_total, old_pkt, new_pkt = [], 0, 0, 0, 0
for src, typ, n1, n2, v1, v2, flags in SAMPLES:
    text = f"src={src} type={typ} seq={SEQ} {n1}={v1} {n2}={v2} flags={flags}"
    frame = tf.encode_telemetry(src, typ, flags, v1, v2, SEQ)
    assert tf.decode_telemetry(frame) == (src, typ, flags, v1, v2, SEQ)
    rows.append((f"{src}/{typ}", n1, len(text), len(frame)))
    old_total += len(text); new_total += len(frame)
    old_pkt += mqtt_packet(len(text)); new_pkt += mqtt_packet(len(frame))

print("| Message (source/type) | Meaning | Text payload (bytes) | Binary payload (bytes) | Saved |")
print("|---|---|---|---|---|")
for key, name, old, new in rows:
    print(f"| {key} | {name} | {old} | {new} | {100 * (old - new) // old}% |")
print(f"| **Total ({len(rows)} message types)** | | **{old_total}** | **{new_total}** | **{100 * (old_total - new_total) // old_total}%** |")
print()
print(f"Whole MQTT PUBLISH packet (fixed header + topic '{TOPIC}' + payload), same {len(rows)} messages: "
      f"{old_pkt} bytes as text, {new_pkt} bytes as binary ({100 * (old_pkt - new_pkt) // old_pkt}% smaller).")
print()
print("| Other messages | Text (bytes) | Binary (bytes) |")
print("|---|---|---|")
print(f"| Status heartbeat (`heartbeat seq=12345`) | {len('heartbeat seq=12345')} | {len(tf.encode_telemetry(1, 1, 0, 0, 0, 12345))} |")
print(f"| Command START | {len('START')} | {len(tf.encode_command(tf.CMD_START))} |")
print(f"| Command STOP | {len('STOP')} | {len(tf.encode_command(tf.CMD_STOP))} |")
print(f"| Command MAP (26 letters) | {len('MAP ' + 'N' * 26)} | {len(tf.encode_map('N' * 26))} |")
print(f"| Command MAPRESET | {len('MAPRESET')} | {len(tf.encode_command(tf.CMD_MAPRESET))} |")
