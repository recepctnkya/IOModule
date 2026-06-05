"""Publish ota_start to Hexnet IO device command topic."""
import json
import sys

import paho.mqtt.publish as publish

BROKER = "185.33.234.10"
PORT = 1883
TOPIC = "hexnet/v1/2/14/command"
URL = sys.argv[1] if len(sys.argv) > 1 else "http://192.168.1.103:8765/mqtt_tcp_custom_outbox.bin"

payload = {
    "cmd": "ota_start",
    "args": {
        "url": URL,
        "version": "0.39.0",
        "force": True,
    },
}
msg = json.dumps(payload)
print(f"Publishing to {TOPIC}: {msg}")
publish.single(topic=TOPIC, payload=msg, hostname=BROKER, port=PORT)
print("Done.")
