"""Trigger OTA via MQTT; read serial without reset."""
import json
import sys
import time

import paho.mqtt.publish as publish
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM40"
URL = sys.argv[2] if len(sys.argv) > 2 else "http://192.168.1.103:8765/mqtt_tcp_custom_outbox.bin"
SECONDS = int(sys.argv[3]) if len(sys.argv) > 3 else 120

s = serial.Serial(PORT, 115200, timeout=0.3)
time.sleep(0.3)
payload = json.dumps(
    {
        "cmd": "ota_start",
        "args": {"url": URL, "version": "0.39.0", "force": True},
    }
)
print("MQTT publish...", flush=True)
publish.single(
    topic="hexnet/v1/2/14/command",
    payload=payload,
    hostname="185.33.234.10",
    port=1883,
)
print("Listening (no reset)...", flush=True)
end = time.time() + SECONDS
while time.time() < end:
    chunk = s.read(8192)
    if chunk:
        sys.stdout.buffer.write(chunk)
        sys.stdout.flush()
