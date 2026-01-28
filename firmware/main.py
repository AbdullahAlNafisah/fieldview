import sys
import time
import select
from sampler import Sampler
from protocol import pack_frame, SIZE

sampler = Sampler()
streaming = False
period_ms = 1000
last_t = 0

def send_bin(frame):
    sys.stdout.buffer.write(b'BIN ')
    sys.stdout.buffer.write(frame)
    sys.stdout.buffer.write(b'\n')

def handle(cmd):
    global streaming, period_ms
    parts = cmd.strip().split()

    if not parts:
        return

    if parts[0] == 'PING':
        print("OK")

    elif parts[0] == 'INFO':
        print("INFO sensors=6 frame_bytes=%d" % SIZE)

    elif parts[0] == 'READ':
        vals = sampler.read_all()
        frame = pack_frame(vals)
        send_bin(frame)

    elif parts[0] == 'START':
        hz = int(parts[1])
        period_ms = int(1000 / hz)
        streaming = True
        print("OK")

    elif parts[0] == 'STOP':
        streaming = False
        print("OK")

    else:
        print("ERR unknown")

print("READY")

while True:
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline()
        if line:
            handle(line)

    if streaming:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_t) >= period_ms:
            last_t = now
            vals = sampler.read_all()
            frame = pack_frame(vals)
            send_bin(frame)
