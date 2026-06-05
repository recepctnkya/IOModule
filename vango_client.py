#!/usr/bin/env python3
"""
VANGO MQTT Client  –  HexNet Technology
========================================
pip install paho-mqtt

Usage:
    python vango_client.py                          # listen to all VANGO devices
    python vango_client.py --device AABBCCDDEEFF    # target one device by MAC
    python vango_client.py --broker 192.168.1.10    # custom / local broker
    python vango_client.py --quiet                  # disable auto dashboard
    python vango_client.py --log                    # start CSV logging at boot

  The device MAC is printed at ESP32 boot:
    I (xxx) MQTT: Device   : VANGO-DDEEFF
    I (xxx) MQTT: Telemetry: hexnet/vango/AABBCCDDEEFF/telemetry

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  DIGITAL OUTPUTS  (16 independent outputs)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  out <0-15>              Toggle single output
  out <0-15>  <0|1>       Set single output  (0=OFF  1=ON)
  out all     <0|1>       All 16 outputs ON or OFF at once
  out mask    <decimal>   Set all 16 from decimal bitmask  e.g. out mask 255
  out mask    0x<hex>     Set all 16 from hex bitmask      e.g. out mask 0xFF00
  out show                Print output state grid

  Examples:
    out 3                 → toggle output bit 3
    out 7 1               → turn output 7 ON
    out all 0             → turn everything OFF
    out mask 0x000F       → outputs 0-3 ON, rest OFF

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  DIMMABLE / PWM  (4 channels, 0–100 %)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  dim <0-3>   <0-100>     Set one channel  (0=off  100=full)
  dim all     <0-100>     Set all four channels to same level
  dim up      <0-3>       Increase channel by 10 %
  dim down    <0-3>       Decrease channel by 10 %
  dim show                Print all channel levels

  Examples:
    dim 0 75              → channel 0 at 75%
    dim all 50            → all channels at 50%
    dim up 1              → channel 1 + 10%
    dim down 2            → channel 2 - 10%

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  RGB LED STRIP
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  rgb <r> <g> <b>         Set colour (0-255 each), auto-enables strip
  rgb #RRGGBB             Hex colour shorthand
  rgb on                  Enable strip (keeps last colour)
  rgb off                 Disable strip
  rgb dim <0-100>         Scale current colour brightness (0=off 100=full)
  rgb show                Print current colour and state

  Colour presets:
  rgb warm                Warm white    (255, 200, 120)
  rgb cool                Cool white    (200, 220, 255)
  rgb white               Full white    (255, 255, 255)
  rgb red                 Red           (255,   0,   0)
  rgb green               Green         (  0, 255,   0)
  rgb blue                Blue          (  0,   0, 255)
  rgb orange              Orange        (255, 140,   0)
  rgb purple              Purple        (180,   0, 255)
  rgb cyan                Cyan          (  0, 220, 220)
  rgb off                 Off / black

  Examples:
    rgb 255 128 0         → custom orange
    rgb #1E90FF           → dodger blue
    rgb dim 30            → dim current colour to 30%

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  MOTOR
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  motor stop   (or  motor 0)   Stop motor
  motor fwd    (or  motor 1)   Run forward
  motor rev    (or  motor 2)   Run reverse

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  GENERAL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  status                  Show last received telemetry dashboard
  watch on / watch off    Auto-print dashboard on every telemetry packet
  log on / log off        Toggle CSV logging  →  vango_telemetry.csv
  help                    Show this reference
  quit                    Exit

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  RAW JSON  (any MQTT client / Node-RED / Home Assistant)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Publish to:  hexnet/vango/<MAC>/control
  Payload (all fields optional):
    {
      "out_0": 1,          "out_5": 0,        (individual bits)
      "outputs": 255,                          (full 16-bit mask)
      "pwm_0": 75,         "pwm_1": 50,       (0-100 %)
      "pwm_2": 100,        "pwm_3": 0,
      "rgb_r": 255,        "rgb_g": 100,
      "rgb_b": 0,          "rgb_en": 1,
      "motor": 1
    }

"""

import argparse
import json
import sys
import threading
import time
import csv
import os
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt not found.  Install with:  pip install paho-mqtt")

# ─────────────────────────────────────────────────────────────────────────────
#  Configuration
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_BROKER  = "broker.hivemq.com"
DEFAULT_PORT    = 1883
TOPIC_PREFIX    = "hexnet/vango"
LOG_FILE        = "vango_telemetry.csv"

CSV_FIELDS = [
    "timestamp", "device", "uptime_s",
    "temp", "humidity",
    "water_1", "water_2", "water_3", "water_4",
    "battery_v",
    "outputs",
    "pwm_0", "pwm_1", "pwm_2", "pwm_3",
    "rgb_r", "rgb_g", "rgb_b", "rgb_en",
    "motor",
]

RGB_PRESETS = {
    "warm":   (255, 200, 120),
    "cool":   (200, 220, 255),
    "white":  (255, 255, 255),
    "red":    (255,   0,   0),
    "green":  (  0, 255,   0),
    "blue":   (  0,   0, 255),
    "orange": (255, 140,   0),
    "purple": (180,   0, 255),
    "cyan":   (  0, 220, 220),
    "off":    (  0,   0,   0),
}

# ─────────────────────────────────────────────────────────────────────────────
#  Shared state
# ─────────────────────────────────────────────────────────────────────────────

_state: dict      = {}          # last telemetry received
_lock             = threading.Lock()
_client           = None
_target_mac: str  = None        # None = first device seen
_active_mac: str  = None        # MAC of first/selected device
log_enabled       = False
watch_enabled     = True        # auto-print dashboard
_csv_writer       = None
_csv_file         = None

# ─────────────────────────────────────────────────────────────────────────────
#  Topic helpers
# ─────────────────────────────────────────────────────────────────────────────

def _ctrl_topic(mac: str) -> str:
    return f"{TOPIC_PREFIX}/{mac}/control"

def _tel_sub() -> str:
    mac = _target_mac or "+"
    return f"{TOPIC_PREFIX}/{mac}/telemetry"

# ─────────────────────────────────────────────────────────────────────────────
#  Display helpers
# ─────────────────────────────────────────────────────────────────────────────

_RESET  = "\033[0m"
_BOLD   = "\033[1m"
_CYAN   = "\033[96m"
_BLUE   = "\033[94m"
_GREEN  = "\033[92m"
_YELLOW = "\033[93m"
_RED    = "\033[91m"
_GRAY   = "\033[90m"

def _bar(pct: int, width: int = 20) -> str:
    pct = max(0, min(100, int(pct or 0)))
    filled = round(pct / 100 * width)
    bar = "█" * filled + "░" * (width - filled)
    colour = _GREEN if pct > 60 else (_YELLOW if pct > 30 else _RED)
    return f"{colour}{bar}{_RESET} {pct:3d}%"

def _rgb_swatch(r: int, g: int, b: int, en: int) -> str:
    if not en:
        return f"{_GRAY}■ OFF{_RESET}"
    return f"\033[38;2;{r};{g};{b}m■{_RESET} #{r:02X}{g:02X}{b:02X}  R:{r} G:{g} B:{b}"

def _out_grid(mask: int) -> str:
    rows = []
    for row_start in range(0, 16, 8):
        labels = "  ".join(f"{_GRAY}{i:2d}{_RESET}" for i in range(row_start, row_start + 8))
        bits   = "  ".join(
            f"{_GREEN}ON{_RESET} " if (mask >> i) & 1 else f"{_RED}--{_RESET} "
            for i in range(row_start, row_start + 8)
        )
        rows.append(f"  {labels}\n  {bits}")
    return "\n".join(rows)

MOTOR_LABEL = {0: f"{_GRAY}STOP{_RESET}", 1: f"{_GREEN}FWD {_RESET}", 2: f"{_RED}REV {_RESET}"}

def print_dashboard(data: dict):
    ts  = datetime.now().strftime("%H:%M:%S")
    dev = data.get("device", "VANGO-??????")
    upt = int(data.get("uptime_s", 0))
    h, r = divmod(upt, 3600); m, s = divmod(r, 60)

    bat  = data.get("battery_v", 0)
    temp = data.get("temp", 0)
    hum  = data.get("humidity", 0)
    w    = [data.get(f"water_{i}", 0) for i in range(1, 5)]
    outs = int(data.get("outputs", 0))
    p    = [data.get(f"pwm_{i}", 0) for i in range(4)]
    rr   = int(data.get("rgb_r", 0))
    gg   = int(data.get("rgb_g", 0))
    bb   = int(data.get("rgb_b", 0))
    en   = int(data.get("rgb_en", 0))
    mot  = MOTOR_LABEL.get(int(data.get("motor", 0)), "?")

    sep = f"{_BLUE}{'─' * 52}{_RESET}"
    print(f"""
{_BOLD}{_CYAN}╔{'═'*52}╗{_RESET}
{_BOLD}{_CYAN}║  {dev}   {ts}   up {h:02d}:{m:02d}:{s:02d}  ║{_RESET}
{_BOLD}{_CYAN}╚{'═'*52}╝{_RESET}
{_BOLD}  ENVIRONMENT{_RESET}
  Temp     {temp:6.1f} °C       Humidity  {hum:5.1f} %
  Battery  {bat:6.2f}  V
{sep}
{_BOLD}  WATER LEVELS{_RESET}
  Tank 1  {_bar(w[0])}
  Tank 2  {_bar(w[1])}
  Tank 3  {_bar(w[2])}
  Tank 4  {_bar(w[3])}
{sep}
{_BOLD}  DIGITAL OUTPUTS  (mask 0x{outs:04X} / {outs}){_RESET}
{_out_grid(outs)}
{sep}
{_BOLD}  DIMMABLE CHANNELS{_RESET}
  CH 0    {_bar(p[0])}
  CH 1    {_bar(p[1])}
  CH 2    {_bar(p[2])}
  CH 3    {_bar(p[3])}
{sep}
{_BOLD}  RGB STRIP{_RESET}
  {_rgb_swatch(rr, gg, bb, en)}
{sep}
{_BOLD}  MOTOR{_RESET}   {mot}
""")

# ─────────────────────────────────────────────────────────────────────────────
#  CSV logging
# ─────────────────────────────────────────────────────────────────────────────

def _start_log():
    global _csv_writer, _csv_file, log_enabled
    _csv_file   = open(LOG_FILE, "a", newline="")
    _csv_writer = csv.DictWriter(_csv_file, fieldnames=CSV_FIELDS, extrasaction="ignore")
    if os.path.getsize(LOG_FILE) == 0:
        _csv_writer.writeheader()
    log_enabled = True
    print(f"[log] Logging → {LOG_FILE}")

def _stop_log():
    global _csv_writer, _csv_file, log_enabled
    log_enabled = False
    if _csv_file:
        _csv_file.close()
        _csv_file = None
    print("[log] Stopped")

def _log(data: dict):
    if _csv_writer and log_enabled:
        row = {f: data.get(f, "") for f in CSV_FIELDS}
        row["timestamp"] = datetime.now().isoformat()
        _csv_writer.writerow(row)
        _csv_file.flush()

# ─────────────────────────────────────────────────────────────────────────────
#  Publish helpers
# ─────────────────────────────────────────────────────────────────────────────

def _pub(payload: dict):
    global _active_mac
    mac = _target_mac or _active_mac
    if not mac:
        print("[error] No device seen yet. Wait for telemetry or use --device <MAC>")
        return
    topic = _ctrl_topic(mac)
    msg   = json.dumps(payload)
    _client.publish(topic, msg, qos=1)
    print(f"{_CYAN}[→ ctrl]{_RESET} {msg}")

# ─────────────────────────────────────────────────────────────────────────────
#  Control command handlers
# ─────────────────────────────────────────────────────────────────────────────

def _handle_out(args: list):
    if not args:
        _usage_out(); return

    sub = args[0].lower()

    # out show
    if sub == "show":
        with _lock: mask = int(_state.get("outputs", 0))
        print(f"\n{_BOLD}  DIGITAL OUTPUTS  (0x{mask:04X}){_RESET}")
        print(_out_grid(mask))
        return

    # out all <0|1>
    if sub == "all":
        if len(args) < 2: _usage_out(); return
        val = int(args[1])
        _pub({"outputs": 0xFFFF if val else 0})
        return

    # out mask <value>
    if sub == "mask":
        if len(args) < 2: _usage_out(); return
        raw = args[1]
        mask = int(raw, 16) if raw.startswith("0x") or raw.startswith("0X") else int(raw)
        _pub({"outputs": mask & 0xFFFF})
        return

    # out <bit> [value]  OR  out <bit>  (toggle)
    try:
        bit = int(sub)
        assert 0 <= bit <= 15
    except (ValueError, AssertionError):
        _usage_out(); return

    if len(args) >= 2:
        val = int(args[1])
        _pub({f"out_{bit}": val})
    else:
        # toggle
        with _lock: cur = int(_state.get("outputs", 0))
        cur ^= (1 << bit)
        _pub({"outputs": cur})


def _handle_dim(args: list):
    if not args:
        _usage_dim(); return

    sub = args[0].lower()

    # dim show
    if sub == "show":
        with _lock:
            vals = [int(_state.get(f"pwm_{i}", 0)) for i in range(4)]
        print(f"\n{_BOLD}  DIMMABLE CHANNELS{_RESET}")
        for i, v in enumerate(vals):
            print(f"  CH {i}  {_bar(v)}")
        return

    # dim all <0-100>
    if sub == "all":
        if len(args) < 2: _usage_dim(); return
        v = max(0, min(100, int(args[1])))
        _pub({"pwm_0": v, "pwm_1": v, "pwm_2": v, "pwm_3": v})
        return

    # dim up/down <ch>
    if sub in ("up", "down"):
        if len(args) < 2: _usage_dim(); return
        ch = int(args[1])
        assert 0 <= ch <= 3
        with _lock: cur = int(_state.get(f"pwm_{ch}", 0))
        cur = min(100, cur + 10) if sub == "up" else max(0, cur - 10)
        _pub({f"pwm_{ch}": cur})
        return

    # dim <ch> <val>
    try:
        ch = int(sub)
        assert 0 <= ch <= 3
        val = max(0, min(100, int(args[1])))
        _pub({f"pwm_{ch}": val})
    except (ValueError, AssertionError, IndexError):
        _usage_dim()


def _handle_rgb(args: list):
    if not args:
        _usage_rgb(); return

    sub = args[0].lower()

    # rgb show
    if sub == "show":
        with _lock:
            rr = int(_state.get("rgb_r", 0))
            gg = int(_state.get("rgb_g", 0))
            bb = int(_state.get("rgb_b", 0))
            en = int(_state.get("rgb_en", 0))
        print(f"\n  RGB  {_rgb_swatch(rr, gg, bb, en)}")
        return

    # rgb on / off
    if sub == "on":
        _pub({"rgb_en": 1}); return
    if sub == "off":
        _pub({"rgb_en": 0}); return

    # rgb preset name
    if sub in RGB_PRESETS:
        r, g, b = RGB_PRESETS[sub]
        en = 0 if sub == "off" else 1
        _pub({"rgb_r": r, "rgb_g": g, "rgb_b": b, "rgb_en": en})
        return

    # rgb dim <0-100>  – scale current colour
    if sub == "dim":
        if len(args) < 2: _usage_rgb(); return
        scale = max(0, min(100, int(args[1]))) / 100.0
        with _lock:
            rr = int(_state.get("rgb_r", 255))
            gg = int(_state.get("rgb_g", 255))
            bb = int(_state.get("rgb_b", 255))
        _pub({
            "rgb_r": int(rr * scale),
            "rgb_g": int(gg * scale),
            "rgb_b": int(bb * scale),
            "rgb_en": 1 if scale > 0 else 0,
        })
        return

    # rgb #RRGGBB
    if sub.startswith("#"):
        try:
            h = sub.lstrip("#")
            r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
            _pub({"rgb_r": r, "rgb_g": g, "rgb_b": b, "rgb_en": 1})
        except Exception:
            print("[error] Use  rgb #RRGGBB  e.g.  rgb #FF8800")
        return

    # rgb <r> <g> <b>
    try:
        r, g, b = int(args[0]), int(args[1]), int(args[2])
        _pub({"rgb_r": r, "rgb_g": g, "rgb_b": b, "rgb_en": 1})
    except (ValueError, IndexError):
        _usage_rgb()


def _handle_motor(args: list):
    MAP = {"stop": 0, "0": 0, "fwd": 1, "forward": 1, "1": 1,
           "rev": 2, "reverse": 2, "backward": 2, "2": 2}
    if not args or args[0].lower() not in MAP:
        print("  motor stop | fwd | rev")
        return
    _pub({"motor": MAP[args[0].lower()]})

# ─────────────────────────────────────────────────────────────────────────────
#  Usage strings
# ─────────────────────────────────────────────────────────────────────────────

def _usage_out():
    print(
        "  out <0-15>              Toggle bit\n"
        "  out <0-15>  <0|1>       Set bit\n"
        "  out all     <0|1>       All bits on/off\n"
        "  out mask    <dec|0xHH>  Full 16-bit mask\n"
        "  out show                State grid"
    )

def _usage_dim():
    print(
        "  dim <0-3>   <0-100>     Set channel %\n"
        "  dim all     <0-100>     All channels\n"
        "  dim up|down <0-3>       ±10 %\n"
        "  dim show                Channel levels"
    )

def _usage_rgb():
    presets = " | ".join(RGB_PRESETS.keys())
    print(
        f"  rgb <r> <g> <b>         Set colour (0-255)\n"
        f"  rgb #RRGGBB             Hex colour\n"
        f"  rgb on / off            Enable / disable\n"
        f"  rgb dim <0-100>         Scale brightness\n"
        f"  rgb show                Current state\n"
        f"  rgb {presets}"
    )

# ─────────────────────────────────────────────────────────────────────────────
#  Top-level command dispatcher
# ─────────────────────────────────────────────────────────────────────────────

def parse_command(line: str):
    global watch_enabled
    parts = line.strip().split()
    if not parts:
        return
    cmd = parts[0].lower()

    if   cmd == "out":    _handle_out(parts[1:])
    elif cmd == "dim":    _handle_dim(parts[1:])
    elif cmd == "rgb":    _handle_rgb(parts[1:])
    elif cmd == "motor":  _handle_motor(parts[1:])

    elif cmd == "status":
        with _lock: d = dict(_state)
        if d: print_dashboard(d)
        else: print("[info] No telemetry yet")

    elif cmd == "log":
        if len(parts) > 1 and parts[1] == "on": _start_log()
        else: _stop_log()

    elif cmd == "watch":
        watch_enabled = not (len(parts) > 1 and parts[1] == "off")
        print(f"[watch] {'ON – dashboard auto-prints on telemetry' if watch_enabled else 'OFF'}")

    elif cmd in ("help", "?"):
        print(__doc__)

    elif cmd in ("quit", "exit", "q"):
        print("Bye."); sys.exit(0)

    else:
        print(f"Unknown command '{cmd}'.  Type 'help' for reference.")

# ─────────────────────────────────────────────────────────────────────────────
#  MQTT callbacks
# ─────────────────────────────────────────────────────────────────────────────

def on_connect(client, _userdata, _flags, rc, _properties=None):
    if rc == 0:
        sub = _tel_sub()
        client.subscribe(sub, qos=1)
        print(f"\n{_GREEN}[mqtt] Connected  →  {sub}{_RESET}")
    else:
        print(f"{_RED}[mqtt] Connect failed rc={rc}{_RESET}")

def on_disconnect(_client, _userdata, _rc, _properties=None):
    print(f"{_YELLOW}[mqtt] Disconnected – reconnecting…{_RESET}")

def on_message(_client, _userdata, msg):
    global _active_mac
    try:
        data = json.loads(msg.payload.decode())
    except Exception:
        return

    # Extract device MAC from topic   hexnet/vango/<MAC>/telemetry
    parts = msg.topic.split("/")
    if len(parts) >= 3:
        mac = parts[2]
        data["_mac"] = mac
        if _active_mac is None:
            _active_mac = mac
            print(f"{_CYAN}[info] Device detected: {data.get('device','?')}  MAC={mac}{_RESET}")

    with _lock:
        _state.clear()
        _state.update(data)

    _log(data)
    if watch_enabled:
        print_dashboard(data)

# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global _client, _target_mac, watch_enabled

    ap = argparse.ArgumentParser(description="VANGO MQTT Client – HexNet Technology")
    ap.add_argument("--broker",  default=DEFAULT_BROKER)
    ap.add_argument("--port",    type=int, default=DEFAULT_PORT)
    ap.add_argument("--device",  default=None,
                    help="Target device MAC hex (12 chars).  Omit = first device seen.")
    ap.add_argument("--log",     action="store_true", help="Start CSV logging immediately")
    ap.add_argument("--quiet",   action="store_true", help="Disable auto dashboard (watch off)")
    args = ap.parse_args()

    broker       = args.broker.replace("mqtt://", "").replace("mqtts://", "")
    _target_mac  = args.device.upper() if args.device else None
    watch_enabled = not args.quiet

    banner = f"""
{_BOLD}{_BLUE}  ██╗   ██╗ █████╗ ███╗   ██╗ ██████╗  ██████╗
  ██║   ██║██╔══██╗████╗  ██║██╔════╝ ██╔═══██╗
  ██║   ██║███████║██╔██╗ ██║██║  ███╗██║   ██║
  ╚██╗ ██╔╝██╔══██║██║╚██╗██║██║   ██║██║   ██║
   ╚████╔╝ ██║  ██║██║ ╚████║╚██████╔╝╚██████╔╝
    ╚═══╝  ╚═╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝  ╚═════╝ {_RESET}
{_CYAN}  MQTT Client  –  HexNet Technology{_RESET}
  Broker  : {broker}:{args.port}
  Device  : {_target_mac or '(first seen)'}
  Type {_BOLD}help{_RESET} for commands,  {_BOLD}quit{_RESET} to exit.
"""
    print(banner)

    if args.log:
        _start_log()

    try:
        _client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        _client = mqtt.Client()

    _client.on_connect    = on_connect
    _client.on_disconnect = on_disconnect
    _client.on_message    = on_message

    _client.connect_async(broker, args.port, keepalive=60)
    _client.loop_start()

    try:
        while True:
            try:
                line = input(f"\n{_BOLD}vango>{_RESET} ")
            except EOFError:
                break
            parse_command(line)
    except KeyboardInterrupt:
        pass
    finally:
        _stop_log()
        _client.loop_stop()
        _client.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
