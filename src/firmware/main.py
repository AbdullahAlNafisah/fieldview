# main.py
import sys, machine
import uasyncio as asyncio
import config as C
from server import serve_forever  # Wi-Fi mode
from serial_mode import run_serial    # serial mode
# from app.serial_mode import run_serial   # your serial mode later
import network, time

def setup_wifi():
    HOSTNAME = getattr(C, "HOSTNAME", "esp32s2")

    if C.USE_AP:
        ap = network.WLAN(network.AP_IF); ap.active(True)
        ap.config(**C.AP_CFG, authmode=network.AUTH_WPA_WPA2_PSK)
        while not ap.active():
            time.sleep_ms(50)
        print("AP:", ap.ifconfig())
        return ap
    else:
        sta = network.WLAN(network.STA_IF); sta.active(True)
        for k in ("hostname", "dhcp_hostname"):
            try:
                sta.config(**{k: HOSTNAME})
                break
            except Exception:
                pass

        if not sta.isconnected():
            sta.connect(C.WIFI_STA["ssid"], C.WIFI_STA["password"])
            for _ in range(100):
                if sta.isconnected(): break
                time.sleep_ms(100)
        if not sta.isconnected():
            raise RuntimeError("Wi-Fi connect failed")
        print("Wi-Fi:", sta.ifconfig())
        return sta



def app_main():
    if C.USE_WIFI:
        setup_wifi()
        asyncio.run(serve_forever())
        asyncio.new_event_loop()
    else:
        run_serial()


