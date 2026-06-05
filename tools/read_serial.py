import serial
import time
import sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM40"
s = serial.Serial(port, 115200, timeout=0.3)
s.setDTR(False)
s.setRTS(True)
time.sleep(0.05)
s.setRTS(False)
time.sleep(0.05)
buf = b""
end = time.time() + 15
while time.time() < end:
    buf += s.read(8192)
sys.stdout.write(buf.decode("utf-8", errors="replace"))
