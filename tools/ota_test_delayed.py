"""Wait for boot/WiFi, trigger OTA, then capture serial tail."""
import json
import sys
import time

import paho.mqtt.publish as publish
import serial

WAIT_BOOT_S = int(sys.argv[1]) if len(sys.argv) > 1 else 25
OTA_WAIT_S = int(sys.argv[2]) if len(sys.argv) > 2 else 90
LOG_S = int(sys.argv[3]) if len(sys.argv) > 3 else 40
URL = "http://192.168.1.103:8765/mqtt_tcp_custom_outbox.bin"

print(f"Waiting {WAIT_BOOT_S}s for WiFi/MQTT...", flush=True)
time.sleep(WAIT_BOOT_S)

payload = json.dumps(
    {
        "cmd": "ota_start",
        "args": {"url": URL, "version": "0.39.0", "force": True},
    }
)
print("MQTT ota_start publish", flush=True)
publish.single(
    topic="hexnet/v1/2/14/command",
    payload=payload,
    hostname="185.33.234.10",
    port=1883,
)

print(f"Waiting {OTA_WAIT_S}s for OTA (no serial)...", flush=True)
time.sleep(OTA_WAIT_S)

print(f"Serial capture {LOG_S}s...", flush=True)
s = serial.Serial("COM40", 115200, timeout=0.3)
end = time.time() + LOG_S
while time.time() < end:
    chunk = s.read(8192)
    if chunk:
        sys.stdout.buffer.write(chunk)
        sys.stdout.flush()
