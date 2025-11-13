
import network, time
from app import config as C

print(">> boot.py start")

HOSTNAME = getattr(C, "HOSTNAME", None) or "esp32s2"

if C.USE_AP:
    ap = network.WLAN(network.AP_IF); ap.active(True)
    ap.config(**C.AP_CFG, authmode=network.AUTH_WPA_WPA2_PSK)
    while not ap.active():
        time.sleep_ms(50)
    print("AP:", ap.ifconfig())
else:
    sta = network.WLAN(network.STA_IF); sta.active(True)
    # Try to set DHCP hostname for nicer URLs
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
        raise RuntimeError("Wi‑Fi connect failed")
    print("Wi‑Fi:", sta.ifconfig())

# Try to announce via mDNS if supported (for Qubical.local)
try:
    import mdns
    try:
        s = mdns.Server()
    except TypeError:
        s = mdns.Server(None)
    s.start(HOSTNAME, "MicroPython ESP32-S2")
    # Optional service, handy on some OSes
    try:
        s.add_service('_http', '_tcp', 80, "ESP Web")
    except Exception:
        pass
    print("mDNS: http://"+HOSTNAME+".local/")
except Exception:
    print("mDNS not available; using IP URL")

print(">> boot.py end")
