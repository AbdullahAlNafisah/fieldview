# config.py
MODE = "serial" # "wifi" or "serial"

# Device/hostname (used for pretty URLs)
HOSTNAME = "QubiCal"

USE_AP = False
USE_WIFI = (MODE == "wifi")

WIFI_STA = {
#    "ssid": "Nafisah_wifi",
    "ssid": "Galaxy S24 Ultra E577",
    "password": "s123123s",
}

AP_CFG = {"essid": "ESP32S2-Plotter", "password": "12345678"}

# BMM350 + I2C
# IMPORTANT: Keep the SAME ordering your original sketch used → (SCL, SDA)
SENSORS_PINS = [  # (SCL, SDA) pairs; add/remove to match your hardware
    (0, 1),
    (2, 3),
    (4, 5),
    (6, 7),
    (8, 9),
    (10, 11),
]
ADDR = 0x14  # 0x15 if ADSEL is high
I2C_FREQ = 100_000
HZ = 50  # stream rate (also chart update cadence)
DEBUG = False
FORCED_PER_SAMPLE = False  # robust but a bit slower
MAX_FAIL_BEFORE_RECOVER = 3
