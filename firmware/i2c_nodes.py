from machine import Pin, SoftI2C
import config as C
from bmm350 import BMM350Node, _bus_unstick

def make_i2c_nodes():
    nodes = []
    for scl, sda in C.SENSORS_PINS:
        try:
            i2c = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=C.I2C_FREQ)
            if C.ADDR not in i2c.scan():
                _bus_unstick(sda, scl)
        except Exception:
            pass

        try:
            nodes.append(BMM350Node(sda, scl))
        except Exception:
            class Dummy:
                def read(self): return None
            nodes.append(Dummy())

    return nodes
