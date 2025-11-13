# main.py — ESP32-S2 + MicroPython — Bosch BMM350 over SoftI2C
# Robust streaming with bus-unstick + re-init and bad-frame filtering.
# Works with sensors that share the same I2C address by assigning each its own SDA/SCL pair.

from machine import Pin, SoftI2C
import time
import uasyncio as asyncio
import json
import math
import time

# ========= USER CONFIG =========
# List of (SDA, SCL) pin pairs — add more if you have multiple sensors
SENSORS_PINS = [
    (0, 1),
    (2, 3),
    (4, 5),
    (6, 7),
    (8, 9),
    (10, 11),
]

ADDR = 0x14  # 0x15 if ADSEL is pulled HIGH on your board
I2C_FREQ = 100_000  # stay conservative; raise after it’s rock solid
HZ = 50  # read rate (matches AGGR ODR below)
SAMPLES = 100  # how many CSV lines to print
DEBUG = False
FORCED_PER_SAMPLE = (
    False  # True = per-sample forced mode (bulletproof, slightly slower)
)
MAX_FAIL_BEFORE_RECOVER = 3  # consecutive read failures before unstick + re-init

# ========= REGISTERS / COMMANDS =========
REG_CHIP_ID = 0x00
CHIP_ID = 0x33
REG_AGGR = 0x04
REG_AXIS_EN = 0x05
REG_PMU_CMD = 0x06
REG_PMU_ST0 = 0x07
REG_INT_STATUS = 0x30
REG_MAG = 0x31
REG_OTP_CMD = 0x50
REG_CMD = 0x7E

CMD_SOFTRESET = 0xB6
CMD_SUS = 0x00
CMD_NM = 0x01
CMD_UPD = 0x02
CMD_FM = 0x03
CMD_FM_FAST = 0x04
CMD_FGR = 0x05
CMD_BR = 0x07
OTP_PWR_OFF = 0x80  # power off OTP after reset (Bosch driver does this)

# ========= AGGR (ODR/AVG) / AXIS =========
# ODR=50Hz (0x5), AVG=8 (0x3<<4) -> 0x35
AGGR_SET = 0x35
AXIS_EN_XYZ = 0x07


# ========= SCALE FACTORS (µT / °C) =========
def _lsb_scales():
    # These mirror Bosch defaults; you can adjust later if you have factory calibration
    bxy_sens = 14.55
    bz_sens = 9.0
    temp_sens = 0.00204
    ina_xy_gain_trgt = 19.46
    ina_z_gain_trgt = 31.0
    adc_gain = 1 / 1.5
    lut_gain = 0.714607238769531
    power = 1000000.0 / 1048576.0
    sx = power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain)
    sy = sx
    sz = power / (bz_sens * ina_z_gain_trgt * adc_gain * lut_gain)
    st = 1 / (temp_sens * adc_gain * lut_gain * 1048576.0)
    return sx, sy, sz, st


SX, SY, SZ, ST = _lsb_scales()


# ========= UTILITIES =========
def _sx24(lo, mi, hi):
    v = lo | (mi << 8) | (hi << 16)
    return v - 0x1000000 if (v & 0x800000) else v


def _bus_unstick(sda_pin, scl_pin, pulses=9):
    """If SDA is stuck low, clock SCL to flush the slave, then generate a STOP."""
    sda = Pin(sda_pin, Pin.OPEN_DRAIN, value=1)
    scl = Pin(scl_pin, Pin.OPEN_DRAIN, value=1)
    time.sleep_us(5)
    for _ in range(pulses):
        if sda.value():  # SDA released already
            break
        scl.value(0)
        time.sleep_us(5)
        scl.value(1)
        time.sleep_us(5)
    # STOP: SDA low -> SCL high -> SDA high
    sda.value(0)
    time.sleep_us(5)
    scl.value(1)
    time.sleep_us(5)
    sda.value(1)
    time.sleep_us(5)


def _rdn(i2c, reg, n):
    # BMM350 quirk: must use repeated start + drop first two dummy bytes
    i2c.writeto(ADDR, bytes([reg]), False)  # repeated start (stop=False)
    b = i2c.readfrom(ADDR, n + 2)
    if not b or len(b) < (n + 2):
        return None
    b = b[2:]
    if len(b) != n:
        return None
    return b


def _read_block12(i2c):
    # Read 12 bytes X[3],Y[3],Z[3],Temp[3], with all sanity checks
    b = _rdn(i2c, REG_MAG, 12)
    if not b:
        return None
    if b == b"\x7f" * 12:  # known bogus pattern when TMR not kicked / stale / glitch
        return None
    return b


# ========= DEVICE WRAPPERS =========
def _wr1(i2c, reg, val):
    i2c.writeto(ADDR, bytes([reg, val]))


def _rd1(i2c, reg):
    b = _rdn(i2c, reg, 1)
    return b[0] if b else None


def _fm_one(i2c):
    # One forced conversion (use when FORCED_PER_SAMPLE=True)
    _wr1(i2c, REG_PMU_CMD, CMD_FM)
    time.sleep_ms(16)  # safe wait; shorten if you optimize later
    return _read_block12(i2c)


def _read_xyz_t(i2c, forced=False):
    b = _fm_one(i2c) if forced else _read_block12(i2c)
    if not b:
        return None
    x = _sx24(b[0], b[1], b[2]) * SX
    y = _sx24(b[3], b[4], b[5]) * SY
    z = _sx24(b[6], b[7], b[8]) * SZ
    t = _sx24(b[9], b[10], b[11]) * ST
    # Range gates to throw away corrupted frames (keeps logs clean)
    if not (-2000 <= x <= 2000 and -2000 <= y <= 2000 and -2000 <= z <= 2000):
        return None
    if not (-40 <= t <= 125):
        return None
    return (x, y, z, t)


def _init_bmm350(i2c):
    # Soft reset
    _wr1(i2c, REG_CMD, CMD_SOFTRESET)
    time.sleep_ms(30)
    # Chip ID
    cid = _rd1(i2c, REG_CHIP_ID)
    if cid != CHIP_ID:
        raise RuntimeError(
            "BMM350 chip_id mismatch (got 0x%02X, want 0x%02X)"
            % (cid if cid is not None else 0xFF, CHIP_ID)
        )
    # Power off OTP (Bosch driver recommends)
    _wr1(i2c, REG_OTP_CMD, OTP_PWR_OFF)
    time.sleep_ms(2)
    # Suspend → BR → FGR (magnetic reset sequence)
    _wr1(i2c, REG_PMU_CMD, CMD_SUS)
    time.sleep_ms(40)
    _wr1(i2c, REG_PMU_CMD, CMD_BR)
    time.sleep_ms(14)
    _wr1(i2c, REG_PMU_CMD, CMD_FGR)
    time.sleep_ms(18)
    time.sleep_ms(40)
    # Normal + configure ODR/AVG + enable axes
    _wr1(i2c, REG_PMU_CMD, CMD_NM)
    time.sleep_ms(40)
    _wr1(i2c, REG_AGGR, AGGR_SET)
    _wr1(i2c, REG_PMU_CMD, CMD_UPD)
    time.sleep_ms(2)
    _wr1(i2c, REG_AXIS_EN, AXIS_EN_XYZ)
    if DEBUG:
        aggr = _rd1(i2c, REG_AGGR)
        axis = _rd1(i2c, REG_AXIS_EN)
        print(
            "AGGR_SET=",
            hex(aggr if aggr is not None else 0),
            "AXIS_EN=",
            hex(axis if axis is not None else 0),
        )


class BMM350Node:
    def __init__(self, sda_pin, scl_pin):
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.i2c = None
        self.fail = 0
        self._create_bus()
        self._init_sensor()

    def _create_bus(self, freq=I2C_FREQ):
        self.i2c = SoftI2C(sda=Pin(self.sda_pin), scl=Pin(self.scl_pin), freq=int(freq))
        if DEBUG:
            print(
                f"[bus sda={self.sda_pin} scl={self.scl_pin}] freq={freq} scan={[hex(x) for x in self.i2c.scan()]}"
            )

    def _init_sensor(self):
        _init_bmm350(self.i2c)
        # throw away a couple of frames after init to settle
        for _ in range(3):
            _ = _read_xyz_t(self.i2c, forced=FORCED_PER_SAMPLE)

    def _recover(self):
        if DEBUG:
            print(f"[info sda={self.sda_pin} scl={self.scl_pin}] bus unstick + re-init")
        # Unstick bus, then recreate bus, then init sensor
        _bus_unstick(self.sda_pin, self.scl_pin)
        # Try a couple of frequencies in case lower speed helps
        for freq in (50_000, 80_000, 100_000):
            try:
                self._create_bus(freq=freq)
                # Verify presence
                if ADDR not in self.i2c.scan():
                    continue
                self._init_sensor()
                return True
            except Exception as e:
                if DEBUG:
                    print(f"[warn] re-init attempt @ {freq} Hz failed: {e}")
        return False

    def read(self):
        try:
            out = _read_xyz_t(self.i2c, forced=FORCED_PER_SAMPLE)
            if out is None:
                self.fail += 1
            else:
                self.fail = 0
                return out
        except OSError as e:
            self.fail += 1
            if DEBUG:
                print(
                    f"[warn sda={self.sda_pin} scl={self.scl_pin}] read error: {e}; fail={self.fail}"
                )

        if self.fail >= MAX_FAIL_BEFORE_RECOVER:
            ok = self._recover()
            self.fail = 0
            if not ok and DEBUG:
                print("[error] re-init failed")
        return None


# ========= APP =========
def _make_nodes():
    nodes = []
    for scl, sda in SENSORS_PINS:
        # sanity: scan before creating node (also detects stuck SDA)
        try:
            i2c_tmp = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=I2C_FREQ)
            seen = i2c_tmp.scan()
            if ADDR not in seen and 0x15 not in seen:
                # try unstick once if nothing is seen
                _bus_unstick(sda, scl)
                seen = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=I2C_FREQ).scan()
            if DEBUG:
                print(f"scan[{scl},{sda}]: {[hex(x) for x in seen]}")
        except Exception as e:
            if DEBUG:
                print(f"[warn] pre-scan failed on SCL={scl} SDA={sda}: {e}")
        # create node regardless; it will recover if needed
        nodes.append(BMM350Node(sda, scl))
    return nodes


# =========================
# HTTP + SSE server
# =========================
INDEX_HTML = r"""
HTTP/1.1 200 OK
Content-Type: text/html; charset=utf-8
Cache-Control: no-store
Connection: close

<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32-S2 Live Plots</title>
  <style>
    :root { --fg:#0f172a; --muted:#64748b; --bg:#f8fafc; }
    html,body{margin:0;padding:0;background:var(--bg);color:var(--fg);font-family:ui-sans-serif,system-ui,Segoe UI,Roboto,Arial}
    header{padding:16px 20px;border-bottom:1px solid #e2e8f0;background:white;position:sticky;top:0}
    h1{font-size:18px;margin:0}
    main{padding:16px;max-width:1100px;margin:0 auto;}
    .card{background:white;border:1px solid #e2e8f0;border-radius:14px;padding:14px;margin:12px 0;box-shadow:0 1px 2px rgba(0,0,0,.04)}
    .row{display:grid;grid-template-columns:1fr 1fr;gap:12px}
    @media(max-width:900px){.row{grid-template-columns:1fr}}
    .muted{color:var(--muted)}
    canvas{width:100%;height:300px}
    .pill{display:inline-block;padding:2px 8px;border-radius:999px;border:1px solid #e2e8f0;background:#f1f5f9;color:#334155;font-size:12px}
  </style>
  <!-- Chart.js from CDN -->
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
  <header>
    <h1>ESP32-S2 live plots <span id="status" class="pill">connecting…</span></h1>
  </header>
  <main>
    <div class="card">
      <div class="muted">Raw (latest): <code id="latest"></code></div>
    </div>

    <!-- Six per-sensor charts (x,y,z only) -->
    <div id="charts" class="row"></div>
  </main>

  <script>
    const MAX_POINTS = 200; // keep charts snappy

    // Common chart options
    const sharedOpts = {
      responsive: true,
      animation: false,
      scales: {
        x: { type: 'linear', ticks: { display:false }, grid: { display:false } },
        y: { beginAtZero: false }
      },
      plugins: {
        legend: { display: true, position: 'bottom' },
        decimation: { enabled: true, algorithm: 'lttb', samples: 200 }
      },
      elements: { point: { radius: 0 } }
    };

    // Datasets for x,y,z
    function makeXYZDatasets() {
      return [
        { label: 'x', data: [], fill:false, tension:0.15, borderWidth:2 },
        { label: 'y', data: [], fill:false, tension:0.15, borderWidth:2 },
        { label: 'z', data: [], fill:false, tension:0.15, borderWidth:2 },
      ];
    }

    // Build 6 charts programmatically
    const charts = [];
    const chartsWrap = document.getElementById('charts');
    for (let i = 0; i < 6; i++) {
      const card = document.createElement('div');
      card.className = 'card';
      card.innerHTML = `
        <div class="muted">Sensor ${i} — axes x,y,z</div>
        <canvas id="c${i}"></canvas>
      `;
      chartsWrap.appendChild(card);
      const ctx = document.getElementById(`c${i}`);
      charts[i] = new Chart(ctx, {
        type: 'line',
        data: { datasets: makeXYZDatasets() },
        options: JSON.parse(JSON.stringify(sharedOpts)) // clone to avoid shared mutation
      });
    }

    // Append and trim helper for one sensor's chart
    function pushXYZ(chart, t, x, y, z) {
      const ds = chart.data.datasets;
      ds[0].data.push({ x: t, y: x });
      ds[1].data.push({ x: t, y: y });
      ds[2].data.push({ x: t, y: z });
      // Trim to MAX_POINTS
      for (let k = 0; k < 3; k++) {
        const d = ds[k].data;
        if (d.length > MAX_POINTS) d.shift();
      }
      chart.update();
    }

    let t = 0; // sample index (x-axis)
    const latestEl = document.getElementById('latest');
    const status = document.getElementById('status');

    // Connect to SSE stream
    const es = new EventSource('/sse');
    es.onopen  = () => status.textContent = 'live';
    es.onerror = () => status.textContent = 'reconnecting…';
    es.onmessage = (ev) => {
      // ev.data is a JSON array of 6 objects: {idx,x,y,z,temp}
      try {
        const arr = JSON.parse(ev.data);

        // Show latest raw values (still includes T for reference)
        latestEl.textContent = arr
          .map(o => `${o.idx}:{x:${o.x.toFixed(2)}, y:${o.y.toFixed(2)}, z:${o.z.toFixed(2)}, T:${o.temp.toFixed(2)}}`)
          .join('  ');

        // Update each sensor's own chart with x,y,z only
        for (let i = 0; i < 6; i++) {
          const o = arr[i];
          pushXYZ(charts[i], t, o.x, o.y, o.z);
        }
        t += 1;
      } catch (e) {
        console.error('bad JSON', e);
      }
    };
  </script>
</body>
</html>
"""


async def handle_http(reader, writer):
    try:
        # Read request line+headers (very small)
        request_line = await reader.readline()
        if not request_line:
            await writer.aclose()
            return
        req = request_line.decode()
        # Minimal header drain
        while True:
            line = await reader.readline()
            if not line or line == b"\r\n":
                break
        if req.startswith("GET /sse"):
            # SSE endpoint
            await writer.awrite("HTTP/1.1 200 OK\r\n")
            await writer.awrite("Content-Type: text/event-stream\r\n")
            await writer.awrite("Cache-Control: no-store\r\n")
            await writer.awrite("Connection: keep-alive\r\n\r\n")

            # Create nodes once per connection and keep last good values
            nodes = _make_nodes()
            last_good = [(0.0, 0.0, 0.0, 25.0) for _ in range(len(nodes))]

            period_ms = max(1, int(1000 / HZ))

            while True:
                t0 = time.ticks_ms()
                arr = []

                for idx, node in enumerate(nodes):
                    out = node.read()
                    if out is not None:
                        last_good[idx] = out
                    x, y, z, tc = last_good[idx]
                    if idx % 2 == 0:
                        arr.append({"idx": idx, "x": x, "y": y, "z": z, "temp": tc})
                    else:
                        arr.append({"idx": idx, "x": -x, "y": y, "z": -z, "temp": tc})

                # (Optional) also print to serial console when DEBUG is on
                if DEBUG:
                    # compact single-line log for quick inspection
                    line = "  ".join(
                        f"{o['idx']}:{{x:{o['x']:.2f},y:{o['y']:.2f},z:{o['z']:.2f},T:{o['temp']:.2f}}}"
                        for o in arr
                    )
                    print(line)

                # Send one SSE frame: JSON array of 6 objects
                payload = json.dumps(arr)
                try:
                    await writer.awrite("data: ")
                    await writer.awrite(payload)
                    await writer.awrite("\n\n")
                    await writer.drain()
                except Exception:
                    break  # client likely disconnected

                # pace to HZ
                dt = time.ticks_diff(time.ticks_ms(), t0)
                sleep_ms = period_ms - dt
                if sleep_ms > 0:
                    await asyncio.sleep_ms(sleep_ms)
                else:
                    # if we overran the period, yield anyway
                    await asyncio.sleep_ms(1)

            try:
                await writer.aclose()
            except Exception:
                pass
            return

        # Otherwise serve the single-page app
        await writer.awrite(INDEX_HTML)
        try:
            await writer.aclose()
        except Exception:
            pass
    except Exception as e:
        try:
            await writer.aclose()
        except Exception:
            pass


async def main():
    srv = await asyncio.start_server(handle_http, "0.0.0.0", 80)
    print("HTTP server running on 0.0.0.0:80")

    # µPy doesn't have serve_forever everywhere
    # keep main alive instead:
    while True:
        await asyncio.sleep(1)


try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()

