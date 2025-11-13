# This file is executed on every boot (including wake-boot from deepsleep)
# import esp
# esp.osdebug(None)
# import webrepl
# webrepl.start()

import network
import os, machine, time
print(">> boot.py start", machine.reset_cause(), os.listdir('/'))
time.sleep_ms(200)  # brief pause so you can Ctrl-C here if needed

# =========================
# Wi‑Fi setup (STA mode)
# =========================
# WIFI_SSID = "Galaxy S24 Ultra E577"
WIFI_SSID = "Nafisah_wifi"
WIFI_PASS = "s123123s"

# If you prefer AP mode (board creates its own Wi‑Fi), set USE_AP = True
USE_AP = False
AP_SSID = "ESP32S2-Plotter"
AP_PASS = "12345678"  # 8+ chars required


def wifi_connect():
    if USE_AP:
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid=AP_SSID, password=AP_PASS, authmode=network.AUTH_WPA_WPA2_PSK)
        while not ap.active():
            time.sleep_ms(100)
        print("AP ready:", ap.ifconfig())
        return ap.ifconfig()[0]
    else:
        sta = network.WLAN(network.STA_IF)
        sta.active(True)
        if not sta.isconnected():
            print("Connecting to Wi‑Fi…")
            sta.connect(WIFI_SSID, WIFI_PASS)
            for _ in range(100):  # ~10s
                if sta.isconnected():
                    break
                time.sleep_ms(100)
        if not sta.isconnected():
            raise RuntimeError("Failed to connect to Wi‑Fi. Check SSID/PASS.")
        print("Wi‑Fi connected:", sta.ifconfig())
        return sta.ifconfig()[0]


ip = wifi_connect()
print("WiFi is connected successfully!\n")
print("Open this in your browser: http://%s/" % ip)
print(">> boot.py end")
