"""Monitor COM40, trigger OTA via MQTT, capture logs."""
import json
import sys
import time

import paho.mqtt.publish as publish
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM40"
URL = sys.argv[2] if len(sys.argv) > 2 else "http://192.168.1.103:8765/mqtt_tcp_custom_outbox.bin"
BROKER = "185.33.234.10"
COMMAND_TOPIC = "hexnet/v1/2/14/command"

s = serial.Serial(PORT, 115200, timeout=0.3)
time.sleep(0.5)
buf = b""
end = time.time() + 100

payload = {
    "cmd": "ota_start",
    "args": {"url": URL, "version": "0.39.0", "force": True},
}
msg = json.dumps(payload)
print(f"=== MQTT publish {COMMAND_TOPIC} ===", flush=True)
print(msg, flush=True)
publish.single(topic=COMMAND_TOPIC, payload=msg, hostname=BROKER, port=1883)
print("=== Serial log (100s) ===", flush=True)

while time.time() < end:
    chunk = s.read(8192)
    if chunk:
        buf += chunk
        sys.stdout.write(chunk.decode("utf-8", errors="replace"))
        sys.stdout.flush()
