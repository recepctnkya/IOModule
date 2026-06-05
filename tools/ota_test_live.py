import json
import threading
import time

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

TELEMETRY = "hexnet/v1/2/14/telemetry"
COMMAND = "hexnet/v1/2/14/command"
URL = "http://192.168.1.103:8765/mqtt_tcp_custom_outbox.bin"
done = threading.Event()


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        ota = data.get("ota", {})
        st = ota.get("state", "?")
        prog = ota.get("progress", 0)
        err = ota.get("last_error", "")
        if st != "idle" or prog or err:
            print(f"  >> ota state={st} progress={prog}% err={err}", flush=True)
        if st in ("success", "failed", "rebooting"):
            done.set()
    except Exception:
        pass


client = mqtt.Client()
client.on_message = on_message
client.connect("185.33.234.10", 1883, 60)
client.subscribe(TELEMETRY)
client.loop_start()

print("Waiting 8s (MQTT up)...", flush=True)
time.sleep(8)

payload = json.dumps(
    {"cmd": "ota_start", "args": {"url": URL, "version": "0.39.0", "force": True}}
)
print(f"Publish ota_start -> {COMMAND}", flush=True)
publish.single(topic=COMMAND, payload=payload, hostname="185.33.234.10", port=1883)

print("Watching telemetry 120s...", flush=True)
done.wait(timeout=120)
time.sleep(3)
client.loop_stop()
print("Done.", flush=True)
