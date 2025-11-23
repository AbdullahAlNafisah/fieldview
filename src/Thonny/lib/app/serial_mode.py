# app/serial_mode.py
import uasyncio as asyncio
import json, time
from app import config as C
from app.i2c_nodes import make_i2c_nodes

async def _serial_loop():
    nodes = make_i2c_nodes()
    # fallback values in case a sensor fails one cycle
    last = [(0.0, 0.0, 0.0, 25.0) for _ in range(len(nodes))]
    period_ms = max(1, int(1000 / C.HZ))

    while True:
        t0 = time.ticks_ms()
        arr = []

        for idx, node in enumerate(nodes):
            out = node.read()
            if out is not None:
                last[idx] = out
            x, y, z, tc = last[idx]

            # keep same “flip even indices” logic as server.py
            if (idx % 2) == 0:
                arr.append({"idx": idx, "x": x,  "y": y, "z": z,  "temp": tc})
            else:
                arr.append({"idx": idx, "x": -x, "y": y, "z": -z, "temp": tc})

        # one JSON array per line -> super easy to read on PC
        try:
            print(json.dumps(arr))
        except Exception as e:
            # optional: if C.DEBUG: print("JSON error", e)
            pass

        dt = time.ticks_diff(time.ticks_ms(), t0)
        sleep_ms = period_ms - dt
        await asyncio.sleep_ms(sleep_ms if sleep_ms > 0 else 1)


def run_serial():
    asyncio.run(_serial_loop())
    asyncio.new_event_loop()
