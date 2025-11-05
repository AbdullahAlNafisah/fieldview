# Magnetic Field Sensing using Multi-bus Software I2C and Live 3D Viewer

This project reads data from four magnetic field sensors (MAG3110) using four software I2C buses, and provides a visualization GUI of the live XYZ point clouds in a 3D space.

## Features

- Self-test, bus recovery (9 clocks), address scan.
- Sensors initialization + status + WHO_AM_I check.
- Request and response protocol (no spamming): host sends `S` (start) then `R` per batch; MCU replies with 4 lines
- GUI: 4x 3D scatter subplots, autoscale (smoothed), pause/resume, Reset All / Reset per bus.

## Firmware — Build & Flash

### Build

```bash
cd firmware
make
```

### Flash (auto-detects `/dev/ttyACM*`/`/dev/ttyUSB*`)

```bash
make flash
```

## Firmware — UART & Protocol

Host commands (ASCII, newline-terminated). Single-letter or full words are accepted:

## Python GUI — Run

```bash
python mag3110_gui.py
```
