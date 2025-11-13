# ===== BMM350 REPL bring-up (ESP32-S2 / MicroPython) =====
from machine import SoftI2C, Pin
import time

# ---- Pins / bus ----
#SCL, SDA = 0, 1           # <-- your working wiring
#SCL, SDA = 2, 3           # <-- your working wiring
#SCL, SDA = 4, 5           # <-- your working wiring
#SCL, SDA = 6, 7           # <-- your working wiring
#SCL, SDA = 8, 9           # <-- your working wiring
SCL, SDA = 10, 11           # <-- your working wiring
ADDR     = 0x14             # 0x15 if ADSEL=HIGH
FREQ     = 50_000           # start slow

# ---- Key registers / commands ----
REG_CHIP_ID   = 0x00;  CHIP_ID = 0x33
REG_AGGR      = 0x04
REG_AXIS_EN   = 0x05
REG_PMU_CMD   = 0x06
REG_PMU_ST0   = 0x07
REG_INT_STAT  = 0x30
REG_MAG_X_XLSB= 0x31
REG_OTP_CMD   = 0x50
REG_CMD       = 0x7E

CMD_SOFTRESET = 0xB6
CMD_SUS   = 0x00
CMD_NM    = 0x01
CMD_UPD   = 0x02
CMD_BR    = 0x07
CMD_FGR   = 0x05
OTP_PWR_OFF = 0x80          # from driver: BMM350_OTP_CMD_PWR_OFF_OTP

# ODR=50Hz, AVG=8 -> 0x35 ; enable XYZ -> 0x07
AGGR_SET   = 0x35
AXIS_EN_XYZ= 0x07

# ---- I2C helpers (BMM350 needs repeated-start + 2 dummy bytes on reads) ----
i2c = SoftI2C(sda=Pin(SDA), scl=Pin(SCL), freq=FREQ)
print("scan:", [hex(x) for x in i2c.scan()])

def rdn(reg, n):
    i2c.writeto(ADDR, bytes([reg]), False)   # repeated-start
    return i2c.readfrom(ADDR, n+2)[2:]       # drop 2 dummy bytes

def rd1(reg):
    return rdn(reg, 1)[0]

def wr1(reg, val):
    i2c.writeto(ADDR, bytes([reg, val]))

def dump12(tag):
    b = rdn(REG_MAG_X_XLSB, 12)
    print(tag, ":", " ".join(f"{x:02X}" for x in b))

# ---- 0) Soft reset, then *power off OTP* (critical), then magnetic reset ----
wr1(REG_CMD, CMD_SOFTRESET)
time.sleep_ms(30)

cid = rd1(REG_CHIP_ID)
print("chip:", hex(cid))
if cid != CHIP_ID:
    raise RuntimeError("chip_id mismatch")

# Power off OTP (as in driver init)
wr1(REG_OTP_CMD, OTP_PWR_OFF)
time.sleep_ms(2)

# Enter SUSPEND, then BR -> FGR (per driver sequence)
wr1(REG_PMU_CMD, CMD_SUS); time.sleep_ms(40)
wr1(REG_PMU_CMD, CMD_BR ); time.sleep_ms(14)
wr1(REG_PMU_CMD, CMD_FGR); time.sleep_ms(18)
time.sleep_ms(40)

# ---- 1) Enter NORMAL, set ODR/AVG, apply, enable axes ----
wr1(REG_PMU_CMD, CMD_NM );   time.sleep_ms(40)
wr1(REG_AGGR, AGGR_SET)
wr1(REG_PMU_CMD, CMD_UPD);   time.sleep_ms(2)
wr1(REG_AXIS_EN, AXIS_EN_XYZ)

print("AGGR_SET =", hex(rd1(REG_AGGR)), "AXIS_EN =", hex(rd1(REG_AXIS_EN)))
print("INT_STAT =", hex(rd1(REG_INT_STAT)))  # likely 0x00 without DRDY routing

# ---- 2) Read a few frames in NORMAL ----
for i in range(10):
    dump12(f"NORM{i}")
    time.sleep_ms(20)
