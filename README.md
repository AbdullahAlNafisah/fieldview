# Magnetic Field Sensing using Multi-bus Software I2C and Live 3D Viewer

This project reads data from four magnetic field sensors (MAG3110) using four software I2C buses, and provides a visualization GUI of the live XYZ point clouds in a 3D space.

---

## Features

- Self-test, bus recovery (9 clocks), address scan.
- Sensors initialization + status + WHO_AM_I check.
- Request and response protocol (no spamming): host sends `S` (start) then `R` per batch; MCU replies with 4 lines
- GUI: 4x 3D scatter subplots, autoscale (smoothed), pause/resume, Reset All / Reset per bus.

---

## Firmware — Build & Flash

### Requirements

- AVR toolchain (`avr-g++`, `avr-libc`, `avrdude`).

### Build

```bash
cd firmware
make
```

### Flash (auto-detects `/dev/ttyACM*`/`/dev/ttyUSB*`)

```bash
make flash
```

### Serial monitor

```bash
make screen BAUD_SERIAL=115200
```

---

## Firmware — UART & Protocol

Host commands (ASCII, newline-terminated). Single-letter or full words are accepted:

- `S` or `START` → arm; MCU replies `ACK START`
- `R` or `REQ` → one batch (4 lines: Bus0..Bus3)
- `T` or `STOP` → disarm; MCU replies `ACK STOP`

### Response line format (per bus):

```
Bus0: X=123 Y=-45 Z=678
```

The GUI extracts all `BusN:` records from the stream.

---

## Python GUI — Install & Run

### Python deps

```bash
python -m venv .venv
source .venv/bin/activate
pip install pyqtgraph PyQt6 pyserial PyOpenGL PyOpenGL_accelerate
```

### Linux OpenGL libs (if needed)

```bash
sudo apt-get update
sudo apt-get install -y libgl1-mesa-glx libglu1-mesa freeglut3 mesa-utils
```

### Run

```bash
python tools/mag3110_viewer.py
```

- Select your serial **port** (e.g., `/dev/ttyUSB0`) and **baud** (match firmware, e.g., `115200`), then **Connect**.
- The GUI sends `S` once; when it sees `ACK START` it begins sending `R` every frame (`UPDATE_FPS`).
- Use **Pause**, **Reset All**, or **Reset Bus N** as needed.
- Toggle **Auto Scale** if you prefer a fixed view.
