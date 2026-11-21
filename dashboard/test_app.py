"""Offline endpoint checks; never start the MQTT network loop."""
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import app as dashboard
from fastapi import HTTPException


class DashboardTests(unittest.TestCase):
    def setUp(self):
        dashboard.latest_messages.clear()
        dashboard.last_message_times.clear()
        dashboard.sensor_frames.clear()
        self.client_patch = patch.object(dashboard, "mqtt_client")
        self.client = self.client_patch.start()
        self.addCleanup(self.client_patch.stop)

    def test_page_path_is_independent_of_working_directory(self):
        original = os.getcwd()
        self.addCleanup(os.chdir, original)
        os.chdir(tempfile.gettempdir())
        response = dashboard.home()
        self.assertTrue(Path(response.path).is_file())
        self.assertIn("PicoCar Control Center", Path(response.path).read_text(encoding="utf-8"))

    def test_heartbeat_expiry_and_telemetry_cache(self):
        self.client.is_connected.return_value = False
        self.assertFalse(dashboard.get_status()["vehicle_heartbeat_fresh"])
        self.assertIsNone(dashboard.get_status()["heartbeat_age_seconds"])
        with patch.object(dashboard.time, "monotonic", return_value=100):
            dashboard.on_message(None, None, SimpleNamespace(
                topic=dashboard.TOPIC_STATUS, payload=b"heartbeat seq=1"))
            dashboard.on_message(None, None, SimpleNamespace(
                topic=dashboard.TOPIC_TELEMETRY, payload=b"seq=2"))
        with patch.object(dashboard.time, "monotonic", return_value=114.9):
            self.assertTrue(dashboard.get_status()["vehicle_heartbeat_fresh"])
            self.assertFalse(dashboard.get_status()["mqtt_connected"])
        with patch.object(dashboard.time, "monotonic", return_value=115):
            self.assertFalse(dashboard.get_status()["vehicle_heartbeat_fresh"])
        self.assertEqual(dashboard.get_telemetry()[dashboard.TOPIC_TELEMETRY], "seq=2")

    def test_subscribe_only_after_successful_connection(self):
        client = Mock()
        dashboard.on_connect(client, None, None, 1, None)
        client.subscribe.assert_not_called()
        dashboard.on_connect(client, None, None, 0, None)
        client.subscribe.assert_called_once_with([
            (dashboard.TOPIC_STATUS, 0), (dashboard.TOPIC_TELEMETRY, 0)])

    def test_startup_probe_acknowledges_only_live_valid_telemetry(self):
        packet = SimpleNamespace(topic=dashboard.TOPIC_TELEMETRY,
                                 payload=b"startup_probe=123", retain=True)
        dashboard.on_message(self.client, None, packet)
        packet.retain = False
        packet.payload = b"startup_probe=4294967296"
        dashboard.on_message(self.client, None, packet)
        self.client.publish.assert_not_called()
        packet.payload = b"startup_probe=123"
        dashboard.on_message(self.client, None, packet)
        self.client.publish.assert_called_once_with(
            dashboard.TOPIC_COMMAND, "dashboard_ready=123", qos=0, retain=False)
        self.assertEqual(dashboard.get_telemetry()["sensor_frames"], [])

    def test_buddy5_raw_range_packet_is_retained_for_dashboard(self):
        payload = b"src=5 type=58 seq=12 raw_range_mm=187 raw_range_status=0 flags=16"
        dashboard.on_message(None, None, SimpleNamespace(
            topic=dashboard.TOPIC_TELEMETRY, payload=payload))
        frames = dashboard.get_telemetry()["sensor_frames"]
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["payload"], payload.decode())

    def test_execution_status_requires_live_telemetry_after_reconnect(self):
        message = SimpleNamespace(
            topic=dashboard.TOPIC_TELEMETRY,
            payload=b"src=5 type=60 seq=10 value1=2 value2=0 flags=16",
            retain=False,
        )
        dashboard.on_message(self.client, None, message)
        self.assertEqual(len(dashboard.get_telemetry()["sensor_frames"]), 1)
        dashboard.on_connect(self.client, None, None, 0, None)
        self.assertEqual(dashboard.get_telemetry()["sensor_frames"], [])
        message.retain = True
        dashboard.on_message(self.client, None, message)
        self.assertEqual(dashboard.get_telemetry()["sensor_frames"], [])
        message.retain = False
        dashboard.on_message(self.client, None, message)
        self.assertEqual(len(dashboard.get_telemetry()["sensor_frames"]), 1)

    def test_buddy4_frames_survive_refresh_and_reconnect_with_real_ages(self):
        def receive(payload, clock):
            with patch.object(dashboard.time, "monotonic", return_value=clock):
                dashboard.on_message(None, None, SimpleNamespace(
                    topic=dashboard.TOPIC_TELEMETRY, payload=payload.encode()))

        receive("src=4 type=7 seq=20 pitch_cdeg=-123 roll_cdeg=0 flags=1", 100)
        receive("src=4 type=16 seq=20 stable_peak_mm=14 median_peak_mm=9 flags=3", 100)
        receive("src=4 type=19 seq=20 highest_hump_mm=14 run_id=1 flags=1", 100)
        with patch.object(dashboard.time, "monotonic", return_value=105):
            first = dashboard.get_telemetry()["sensor_frames"]
            second = dashboard.get_telemetry()["sensor_frames"]
        self.assertEqual(first, second)
        self.assertEqual(len(first), 3)
        self.assertTrue(all(frame["age_seconds"] == 5 for frame in first))
        self.assertIn("highest_hump_mm=14", first[-1]["payload"])
        dashboard.on_connect(Mock(), None, None, 0, None)
        with patch.object(dashboard.time, "monotonic", return_value=110):
            self.assertEqual(dashboard.get_telemetry()["sensor_frames"][-1]["age_seconds"], 10)
        receive("src=4 type=19 seq=21 highest_hump_mm=0 run_id=2 flags=0", 111)
        with patch.object(dashboard.time, "monotonic", return_value=112):
            self.assertIn("run_id=2 flags=0", dashboard.get_telemetry()["sensor_frames"][-1]["payload"])


    def test_commands_are_validated_and_publish_is_not_acknowledgement(self):
        with self.assertRaises(HTTPException) as error:
            dashboard.send_command("TURN")
        self.assertEqual(error.exception.status_code, 400)
        self.client.publish.assert_not_called()
        self.client.is_connected.return_value = False
        with self.assertRaises(HTTPException) as error:
            dashboard.send_command("START")
        self.assertEqual(error.exception.status_code, 503)
        self.client.publish.assert_not_called()
        self.client.is_connected.return_value = True
        self.client.publish.return_value.rc = dashboard.mqtt.MQTT_ERR_SUCCESS
        for command, frame in (("start", b"\x81"), ("stop", b"\x82")):
            result = dashboard.send_command(command)
            self.assertEqual(result["status"], "publish queued")
            self.client.publish.assert_called_with(
                dashboard.TOPIC_COMMAND, payload=frame, qos=0, retain=False)
        self.client.publish.return_value.rc = dashboard.mqtt.MQTT_ERR_NO_CONN
        with self.assertRaises(HTTPException) as error:
            dashboard.send_command("STOP")
        self.assertEqual(error.exception.status_code, 503)


    # Binary frames. The hex strings are the bytes printed by testing/system/test_telemetry_frame.c (the firmware codec).
    FIRMWARE_FRAMES = [
        ("a503d20400649c", (2, 3, 0, 100, -100, 1234)),
        ("da332a0000bd002c01", (5, 51, 0, 189, 300, 42)),
        ("df7bffffff0100ffffff7f00000080", (5, 59, 1, 2147483647, -2147483648, 16777215)),
    ]

    def test_binary_telemetry_matches_the_firmware_codec(self):
        for hex_bytes, (source, kind, flags, first, second, sequence) in self.FIRMWARE_FRAMES:
            raw = bytes.fromhex(hex_bytes)
            self.assertEqual(
                dashboard.frames_codec.encode_telemetry(source, kind, flags, first, second, sequence), raw)
            frame = dashboard.frames_codec.decode_telemetry(raw)
            self.assertEqual(tuple(frame), (source, kind, flags, first, second, sequence))

    def test_binary_telemetry_reaches_the_page_as_the_usual_text_fields(self):
        dashboard.on_message(None, None, SimpleNamespace(
            topic=dashboard.TOPIC_TELEMETRY, payload=bytes.fromhex("da332a0000bd002c01")))
        frames = dashboard.get_telemetry()["sensor_frames"]
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["payload"], "src=5 type=51 seq=42 value1=189 value2=300 flags=0")

    def test_binary_heartbeat_on_the_status_topic_keeps_the_heartbeat_fresh(self):
        raw = dashboard.frames_codec.encode_telemetry(1, 1, 0, 0, 0, 7)
        self.assertEqual(len(raw), 5)
        with patch.object(dashboard.time, "monotonic", return_value=100):
            dashboard.on_message(None, None, SimpleNamespace(topic=dashboard.TOPIC_STATUS, payload=raw))
        with patch.object(dashboard.time, "monotonic", return_value=101):
            self.assertTrue(dashboard.get_status()["vehicle_heartbeat_fresh"])
        self.assertEqual(dashboard.get_telemetry()[dashboard.TOPIC_STATUS], "heartbeat seq=7")

    def test_malformed_binary_frames_are_ignored(self):
        for raw in (b"\xa5", b"\xa5\x03\x00\x00", bytes.fromhex("a503d20400649c00"), bytes.fromhex("a583d20400649c")):
            dashboard.on_message(None, None, SimpleNamespace(topic=dashboard.TOPIC_TELEMETRY, payload=raw))
        self.assertNotIn(dashboard.TOPIC_TELEMETRY, dashboard.get_telemetry())
        self.assertEqual(dashboard.get_telemetry()["sensor_frames"], [])

    def test_barcode_map_is_sent_as_a_compact_binary_command(self):
        self.client.is_connected.return_value = True
        self.client.publish.return_value.rc = dashboard.mqtt.MQTT_ERR_SUCCESS
        default = "LNNNNUNNNNNNNNNNNNNNNNRNNN"
        result = dashboard.set_barcode_map(default)
        self.assertEqual(result["command"], "MAP " + default)
        self.client.publish.assert_called_with(
            dashboard.TOPIC_COMMAND, payload=bytes.fromhex("8501000200000000000c00"), qos=0, retain=False)
        self.assertEqual(len(bytes.fromhex("8501000200000000000c00")), 11)
        dashboard.set_barcode_map("reset")
        self.client.publish.assert_called_with(dashboard.TOPIC_COMMAND, payload=b"\x86", qos=0, retain=False)
        with self.assertRaises(HTTPException) as error:
            dashboard.set_barcode_map("LSR")
        self.assertEqual(error.exception.status_code, 400)


if __name__ == "__main__":
    unittest.main()
