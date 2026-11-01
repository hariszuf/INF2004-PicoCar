
import os
from pathlib import Path
from contextlib import asynccontextmanager

import paho.mqtt.client as mqtt
from fastapi import FastAPI
from fastapi.responses import FileResponse
import time
from fastapi import HTTPException

BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "127.0.0.1")
BROKER_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))

TOPIC_STATUS = "picocar/vehicle1/status"
TOPIC_TELEMETRY = "picocar/vehicle1/telemetry"
TOPIC_COMMAND = "picocar/vehicle1/command"

latest_messages = {}
last_message_times = {}


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("[MQTT] Connected to Mosquitto")
        client.subscribe([
            (TOPIC_STATUS, 0),
            (TOPIC_TELEMETRY, 0),
        ])
    else:
        print(f"[MQTT] Connection failed: {reason_code}")


def on_message(client, userdata, message):
    payload = message.payload.decode("utf-8", errors="replace")

    latest_messages[message.topic] = payload
    last_message_times[message.topic] = time.monotonic()

    print(f"[MQTT] {message.topic}: {payload}")


mqtt_client = mqtt.Client(
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
    client_id="picocar-dashboard",
)

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message


@asynccontextmanager
async def lifespan(app: FastAPI):
    mqtt_client.connect_async(BROKER_HOST, BROKER_PORT)
    mqtt_client.loop_start()

    try:
        yield
    finally:
        mqtt_client.disconnect()
        mqtt_client.loop_stop()


app = FastAPI(
    title="PicoCar Dashboard",
    lifespan=lifespan,
)


@app.get("/")
def home():
    return FileResponse(Path(__file__).parent / "static" / "index.html")


@app.get("/api/telemetry")
def get_telemetry():
    return latest_messages

@app.get("/api/status")
def get_status():
    now = time.monotonic()
    heartbeat_age = None

    if TOPIC_STATUS in last_message_times:
        heartbeat_age = round(
            now - last_message_times[TOPIC_STATUS], 1
        )

    return {
        "mqtt_connected": mqtt_client.is_connected(),
        "vehicle_heartbeat_fresh": (
            heartbeat_age is not None and heartbeat_age < 15
        ),
        "heartbeat_age_seconds": heartbeat_age,
    }


@app.post("/api/command/{command}")
def send_command(command: str):
    command = command.upper()

    if command not in ("START", "STOP"):
        raise HTTPException(
            status_code=400,
            detail="Unsupported command",
        )

    if not mqtt_client.is_connected():
        raise HTTPException(
            status_code=503,
            detail="MQTT broker disconnected",
        )

    result = mqtt_client.publish(
        TOPIC_COMMAND,
        payload=command,
        qos=0,
        retain=False,
    )

    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        raise HTTPException(
            status_code=503,
            detail="MQTT publish failed",
        )

    return {
        "command": command,
        "status": "publish queued",
    }