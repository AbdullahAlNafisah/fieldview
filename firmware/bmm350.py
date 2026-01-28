from machine import Pin, SoftI2C
import time
import config as C

# --- Registers / constants ---
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
OTP_PWR_OFF = 0x80

# ODR=50Hz (0x5), AVG=8 (0x3<<4) -> 0x35; enable XYZ axes
AGGR_SET = 0x35
AXIS_EN_XYZ = 0x07


# ---- scale factors (µT / °C) ----
def _lsb_scales():
    bxy_sens, bz_sens, temp_sens = 14.55, 9.0, 0.00204
    ina_xy_gain_trgt, ina_z_gain_trgt = 19.46, 31.0
    adc_gain = 1 / 1.5
    lut_gain = 0.714607238769531
    power = 1_000_000.0 / 1_048_576.0
    sx = power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain)
    sy = sx
    sz = power / (bz_sens * ina_z_gain_trgt * adc_gain * lut_gain)
    st = 1 / (temp_sens * adc_gain * lut_gain * 1_048_576.0)
    return sx, sy, sz, st


SX, SY, SZ, ST = _lsb_scales()


# ---- low-level helpers ----
def _sx24(lo, mi, hi):
    v = lo | (mi << 8) | (hi << 16)
    return v - 0x1000000 if (v & 0x800000) else v


def _bus_unstick(sda_pin, scl_pin, pulses=9):
    sda = Pin(sda_pin, Pin.OPEN_DRAIN, value=1)
    scl = Pin(scl_pin, Pin.OPEN_DRAIN, value=1)
    time.sleep_us(5)
    for _ in range(pulses):
        if sda.value():
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


def _wr1(i2c, reg, val):
    i2c.writeto(C.ADDR, bytes([reg, val]))


def _rdn(i2c, reg, n):
    # BMM350 quirk: repeated-start + drop two dummy bytes
    i2c.writeto(C.ADDR, bytes([reg]), False)
    b = i2c.readfrom(C.ADDR, n + 2)
    if not b or len(b) < (n + 2):
        return None
    b = b[2:]
    return b if len(b) == n else None


def _read_block12(i2c):
    b = _rdn(i2c, REG_MAG, 12)
    if not b or b == b"\x7f" * 12:
        return None
    return b


def _fm_one(i2c):
    _wr1(i2c, REG_PMU_CMD, CMD_FM)
    time.sleep_ms(16)
    return _read_block12(i2c)


# ---- public helpers ----
def read_xyz_t(i2c, forced=False):
    b = _fm_one(i2c) if forced else _read_block12(i2c)
    if not b:
        return None
    x = _sx24(b[0], b[1], b[2]) * SX
    y = _sx24(b[3], b[4], b[5]) * SY
    z = _sx24(b[6], b[7], b[8]) * SZ
    t = _sx24(b[9], b[10], b[11]) * ST
    # quick sanity gates
    if not (-2000 <= x <= 2000 and -2000 <= y <= 2000 and -2000 <= z <= 2000):
        return None
    if not (-40 <= t <= 125):
        return None
    return (x, y, z, t)


def init_bmm350(i2c):
    _wr1(i2c, REG_CMD, CMD_SOFTRESET)
    time.sleep_ms(30)
    cid = _rdn(i2c, REG_CHIP_ID, 1)
    cid = cid[0] if cid else None
    if cid != CHIP_ID:
        raise RuntimeError(
            "BMM350 chip_id mismatch (got 0x%02X, want 0x%02X)"
            % (cid if cid is not None else 0xFF, CHIP_ID)
        )
    _wr1(i2c, REG_OTP_CMD, OTP_PWR_OFF)
    time.sleep_ms(2)
    _wr1(i2c, REG_PMU_CMD, CMD_SUS)
    time.sleep_ms(40)
    _wr1(i2c, REG_PMU_CMD, CMD_BR)
    time.sleep_ms(14)
    _wr1(i2c, REG_PMU_CMD, CMD_FGR)
    time.sleep_ms(18)
    time.sleep_ms(40)
    _wr1(i2c, REG_PMU_CMD, CMD_NM)
    time.sleep_ms(40)
    _wr1(i2c, REG_AGGR, AGGR_SET)
    _wr1(i2c, REG_PMU_CMD, CMD_UPD)
    time.sleep_ms(2)
    _wr1(i2c, REG_AXIS_EN, AXIS_EN_XYZ)


class BMM350Node:
    def __init__(self, sda_pin, scl_pin):
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.i2c = None
        self.fail = 0
        self._create_bus()
        self._init_sensor()

    def _create_bus(self, freq=C.I2C_FREQ):
        self.i2c = SoftI2C(sda=Pin(self.sda_pin), scl=Pin(self.scl_pin), freq=int(freq))
        if C.DEBUG:
            print(
                f"[bus sda={self.sda_pin} scl={self.scl_pin}] "
                f"freq={freq} scan={[hex(x) for x in self.i2c.scan()]}"
            )

    def _init_sensor(self):
        init_bmm350(self.i2c)
        # throw away first few readings
        for _ in range(3):
            _ = read_xyz_t(self.i2c, forced=C.FORCED_PER_SAMPLE)

    def _recover(self):
        if C.DEBUG:
            print(f"[info sda={self.sda_pin} scl={self.scl_pin}] bus unstick + re-init")
        _bus_unstick(self.sda_pin, self.scl_pin)
        for freq in (50_000, 80_000, 100_000):
            try:
                self._create_bus(freq=freq)
                if C.ADDR not in self.i2c.scan():
                    continue
                self._init_sensor()
                return True
            except Exception as e:
                if C.DEBUG:
                    print(f"[warn] re-init @ {freq} Hz failed: {e}")
        return False

    def read(self):
        try:
            out = read_xyz_t(self.i2c, forced=C.FORCED_PER_SAMPLE)
            if out is None:
                self.fail += 1
            else:
                self.fail = 0
                return out
        except OSError as e:
            self.fail += 1
            if C.DEBUG:
                print(
                    f"[warn sda={self.sda_pin} scl={self.scl_pin}] read error: {e}; fail={self.fail}"
                )
        if self.fail >= C.MAX_FAIL_BEFORE_RECOVER:
            ok = self._recover()
            self.fail = 0
            if not ok and C.DEBUG:
                print("[error] re-init failed")
        return None
