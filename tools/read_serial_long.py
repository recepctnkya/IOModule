import serial
import sys
import time

port = sys.argv[1] if len(sys.argv) > 1 else "COM40"
seconds = int(sys.argv[2]) if len(sys.argv) > 2 else 60

s = serial.Serial(port, 115200, timeout=0.3)
s.setDTR(False)
s.setRTS(True)
time.sleep(0.1)
s.setRTS(False)
time.sleep(0.1)
buf = b""
end = time.time() + seconds
while time.time() < end:
    buf += s.read(8192)
sys.stdout.write(buf.decode("utf-8", errors="replace"))
