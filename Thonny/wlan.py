import machine, network

SSID = "Nafisah_wifi"
Password = "s123123s"

def do_connect():
    wlan = network.WLAN()
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(SSID, Password)
        while not wlan.isconnected():
            machine.idle()
    print('Connected! network config (IP):', wlan.ipconfig('addr4'))

do_connect()