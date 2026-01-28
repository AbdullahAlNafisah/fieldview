
# ---- BMM350 + I2C configuration ----

# IMPORTANT: Keep same physical ordering (SCL, SDA)
SENSORS_PINS = [
    (0, 1),
    (2, 3),
    (4, 5),
    (6, 7),
    (8, 9),
    (10, 11),
]

ADDR = 0x14       # 0x15 if ADSEL high
I2C_FREQ = 100_000

# Acquisition
DEBUG = False
FORCED_PER_SAMPLE = False
MAX_FAIL_BEFORE_RECOVER = 3
