# ESP32-C3 Companion Firmware

This folder contains the ESP-IDF project used to add a serial companion MCU for the STM32 firmware.

## Build

```bash
cd esp32-c3
idf.py build
```

## Flash

```bash
cd esp32-c3
idf.py -p /dev/ttyUSB0 flash monitor
```

## Current status

- ESP32-C3 project scaffold created.
- UART1 initialized for STM32 link (`GPIO4` TX, `GPIO5` RX, `115200` baud).
- Startup banner is sent to STM32 on boot.
- Incoming bytes from STM32 are logged on ESP32.

Update UART pins/baudrate in `main/main.c` to match wiring.
