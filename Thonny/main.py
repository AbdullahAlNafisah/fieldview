# bmm350_simple.py — MicroPython (ESP32-S2) — 6× BMM350 via Soft I2C
from machine import Pin, SoftI2C
import time

# ======== I2C ADDRESSES ========
BMM350_ADDR_LOW  = 0x14   # most boards
BMM350_ADDR_HIGH = 0x15   # if ADSEL=HIGH
ADDR = BMM350_ADDR_LOW    # change if needed

# ======== IMPORTANT REGISTERS (from Bosch docs) ========
REG_CHIP_ID       = 0x00  # should read 0x33
CHIP_ID_VALUE     = 0x33
REG_PMU_AGGR_SET  = 0x04  # ODR + averaging
REG_AXIS_EN       = 0x05  # enable XYZ bits
REG_PMU_CMD       = 0x06  # power/update commands
REG_MAG_X_XLSB    = 0x31  # start of 12 bytes: X,Y,Z,Temp (24-bit each)
REG_CMD           = 0x7E  # soft reset

# ======== COMMAND / CONFIG VALUES ========
CMD_SOFTRESET = 0xB6
# ODR = 50 Hz (0x5), AVG = 8 (0x3 in bits [5:4])
ODR_50HZ, AVG_8 = 0x05, 0x03
AGGR_SET_VALUE = (ODR_50HZ & 0x0F) | ((AVG_8 & 0x03) << 4)

# PMU commands
PMU_CMD_UPD_OAE = 0x02    # apply ODR/AVG
PMU_CMD_NM      = 0x01    # normal mode
AXIS_EN_XYZ     = 0x07    # enable X,Y,Z

# ======== SIMPLE SCALING (µT and °C) ========
# These constants mirror the driver’s default conversion so you get sane units.
def _lsb_scales():
    bxy_sens = 14.55
    bz_sens = 9.0
    temp_sens = 0.00204
    ina_xy_gain_trgt = 19.46
    ina_z_gain_trgt = 31.0
    adc_gain = 1/1.5
    lut_gain = 0.714607238769531
    power = 1000000.0 / 1048576.0
    sx = power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain)
    sy = sx
    sz = power / (bz_sens  * ina_z_gain_trgt  * adc_gain * lut_gain)
    st = 1 / (temp_sens * adc_gain * lut_gain * 1048576.0)
    return sx, sy, sz, st

SX, SY, SZ, ST = _lsb_scales()

# ======== HELPERS ========
def _sign_extend24(v):
    v &= 0xFFFFFF
    return v - 0x1000000 if (v & 0x800000) else v

def _read_after_reg(i2c, addr, reg, nbytes):
    # Bosch gets require two “dummy” bytes before real data; we emulate that.
    i2c.writeto(addr, bytes([reg]))
    rb = i2c.readfrom(addr, nbytes + 2)
    return rb[2:]

# ======== ONE SENSOR OBJECT ========
class BMM350:
    def __init__(self, i2c: SoftI2C, addr=ADDR, debug=False):
        self.i2c = i2c
        self.addr = addr
        self.debug = debug

    def _w(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]) if isinstance(val, int) else val)

    def check_id(self):
        return self.i2c.readfrom_mem(self.addr, REG_CHIP_ID, 1)[0]

    def reset_and_init(self):
        # 1) Soft reset, wait ~24 ms
        self._w(REG_CMD, CMD_SOFTRESET)
        time.sleep_ms(30)

        # 2) Verify chip ID
        cid = self.check_id()
        if cid != CHIP_ID_VALUE:
            raise RuntimeError("BMM350 chip_id 0x%02X (expected 0x%02X)" % (cid, CHIP_ID_VALUE))

        # 3) Set ODR/AVG, apply, wait ~1 ms
        self._w(REG_PMU_AGGR_SET, AGGR_SET_VALUE)
        self._w(REG_PMU_CMD, PMU_CMD_UPD_OAE)
        time.sleep_ms(2)

        # 4) Enable XYZ, enter Normal mode
        self._w(REG_AXIS_EN, AXIS_EN_XYZ)
        self._w(REG_PMU_CMD, PMU_CMD_NM)

        if self.debug:
            print("BMM350 init OK (chip_id=0x%02X)" % cid)

    def read_ut_c(self):
        # Read 12 bytes: X[3],Y[3],Z[3],Temp[3] (24-bit signed)
        b = _read_after_reg(self.i2c, self.addr, REG_MAG_X_XLSB, 12)
        if len(b) != 12:
            raise OSError("short read")
        x = _sign_extend24(b[0] | (b[1] << 8) | (b[2] << 16))
        y = _sign_extend24(b[3] | (b[4] << 8) | (b[5] << 16))
        z = _sign_extend24(b[6] | (b[7] << 8) | (b[8] << 16))
        t = _sign_extend24(b[9] | (b[10] << 8) | (b[11] << 16))

        # Convert to microtesla and Celsius using simple defaults
        return (x * SX, y * SY, z * SZ, t * ST)

# ======== DEFINE YOUR SIX BUSES HERE ========
# Replace these with YOUR (sda, scl) pins:
SENSORS_PINS = [
    (5, 6),
    (7, 8),
    (9, 10),
    (11, 12),
    (13, 14),
    (15, 16),
]

def make_sensors(freq=80000, debug=False):
    """Create six SoftI2C buses and init one BMM350 on each."""
    nodes = []
    for (sda_pin, scl_pin) in SENSORS_PINS:
        i2c = SoftI2C(sda=Pin(sda_pin), scl=Pin(scl_pin), freq=freq)
        dev = BMM350(i2c, ADDR, debug=debug)
        dev.reset_and_init()
        nodes.append(dev)
    return nodes

def csv_stream(nodes, hz=50, samples=1000):
    """Print CSV lines: idx,x[uT],y[uT],z[uT],temp[C] for all sensors."""
    period_ms = max(1, int(1000 / hz))
    print("# idx,x[uT],y[uT],z[uT],temp[C]")
    for _ in range(samples):
        t0 = time.ticks_ms()
        for idx, dev in enumerate(nodes):
            x, y, z, tc = dev.read_ut_c()
            print(f"{idx},{x:.6f},{y:.6f},{z:.6f},{tc:.3f}")
        # keep approximate rate
        dt = time.ticks_diff(time.ticks_ms(), t0)
        sleep_ms = period_ms - dt
        if sleep_ms > 0:
            time.sleep_ms(sleep_ms)

# Quick run helper (uncomment to auto-run on boot.py/main.py if you want):
# def main():
#     nodes = make_sensors(freq=80000, debug=True)
#     csv_stream(nodes, hz=50, samples=500)
# if __name__ == "__main__":
#     main()
