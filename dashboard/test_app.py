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

    def test_buddy5_raw_range_packet_is_retained_for_dashboard(self):
        payload = b"src=5 type=58 seq=12 raw_range_mm=187 raw_range_status=0 flags=16"
        dashboard.on_message(None, None, SimpleNamespace(
            topic=dashboard.TOPIC_TELEMETRY, payload=payload))
        frames = dashboard.get_telemetry()["sensor_frames"]
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["payload"], payload.decode())

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
        for command in ("start", "stop"):
            result = dashboard.send_command(command)
            self.assertEqual(result["status"], "publish queued")
            self.client.publish.assert_called_with(
                dashboard.TOPIC_COMMAND, payload=command.upper(), qos=0, retain=False)
        self.client.publish.return_value.rc = dashboard.mqtt.MQTT_ERR_NO_CONN
        with self.assertRaises(HTTPException) as error:
            dashboard.send_command("STOP")
        self.assertEqual(error.exception.status_code, 503)


if __name__ == "__main__":
    unittest.main()
