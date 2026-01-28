# FieldView

Magnetic field visualization around a cube using 6× BMM350 sensors and an ESP32-S2 running MicroPython.
Communication is USB-Serial using a binary, request-driven protocol.

The ESP32 supports:

- PING
- INFO
- READ (single frame)
- START <hz> / STOP (streaming)

Data is sent as fixed-size binary frames (6 sensors × x,y,z,temp).

## Notes

- Only one program may open the serial port at a time (close Thonny, serial monitors, etc.).
- Each notebook cell opens the port, communicates, then closes it.
- If you see boot messages, reset the board once and wait for READY before running cells.
- Calibration uses min/max ellipsoid approximation.

This setup is intended for research and notebook-based visualization and processing.
