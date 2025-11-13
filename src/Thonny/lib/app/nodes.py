from machine import Pin, SoftI2C
from app import config as C
from app.bmm350 import BMM350Node, _bus_unstick


def make_nodes():
    """
    Build nodes using the SAME ordering as your original app: (SCL, SDA).
    Each tuple in C.SENSORS_PINS is (SCL, SDA). We then create the I2C bus with
    SoftI2C(sda=..., scl=...) and pass (sda, scl) to BMM350Node.
    """
    nodes = []
    for scl, sda in C.SENSORS_PINS:  # (SCL, SDA)
        try:
            # Quick scan to confirm the device is present; try an unstick if not.
            i2c_tmp = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=C.I2C_FREQ)
            seen = i2c_tmp.scan()
            if C.ADDR not in seen and 0x15 not in seen:
                _bus_unstick(sda, scl)
                seen = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=C.I2C_FREQ).scan()
            if C.DEBUG:
                print(f"scan[SCL={scl},SDA={sda}]: {[hex(x) for x in seen]}")
        except Exception as e:
            if C.DEBUG:
                print(f"[warn] pre-scan failed on SCL={scl} SDA={sda}: {e}")

        # Create the sensor node (with self-recovery if reads fail).
        try:
            nodes.append(BMM350Node(sda, scl))
        except Exception as e:
            if C.DEBUG:
                print(f"[error] node init failed on SCL={scl} SDA={sda}: {e}")
            # Keep array length stable so the frontend loop remains simple.
            class _Dummy:
                def read(self): return None
            nodes.append(_Dummy())

    return nodes
