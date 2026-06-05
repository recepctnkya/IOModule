import json
import sys
import time

import paho.mqtt.client as mqtt

TOPIC = "hexnet/v1/2/14/telemetry"
msgs = []


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        ota = data.get("ota", {})
        status = data.get("status", {})
        dev = data.get("device", {})
        print(
            f"ota state={ota.get('state')} progress={ota.get('progress')} "
            f"err={ota.get('last_error')} online={status.get('online')} fw={dev.get('fw')}",
            flush=True,
        )
        msgs.append(data)
    except Exception as e:
        print("parse err", e, msg.payload[:200], flush=True)


client = mqtt.Client()
client.on_message = on_message
client.connect("185.33.234.10", 1883, 60)
client.subscribe(TOPIC)
client.loop_start()
time.sleep(int(sys.argv[1]) if len(sys.argv) > 1 else 30)
client.loop_stop()
