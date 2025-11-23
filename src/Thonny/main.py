# main.py
import sys, machine
import uasyncio as asyncio
from app import config as C
from app.server import serve_forever  # Wi-Fi mode
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
        # maybe setup_mdns(HOSTNAME) here too
        asyncio.run(serve_forever())
        asyncio.new_event_loop()
    else:
        # pure serial mode
        # run_serial()
        print("Serial mode placeholder")



print("> main.py start")

try:
    app_main()
except Exception as e:
    print("Fatal error in main:")
    sys.print_exception(e)
#finally: # This can cause infinite loop if not handled properly
#    machine.reset()
    
print("> main.py end")


