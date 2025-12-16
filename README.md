# bt-gamepad

ESP32-S3 firmware that turns a **USB HID gamepad** into a **Bluetooth LE (BLE) gamepad**. Plug a wired controller into the board (USB Host), and the firmware advertises as a BLE gamepad so a phone/tablet/PC can connect to it wirelessly.

## Hardware

- **Seeed Studio XIAO ESP32S3** (the current PlatformIO target: `seeed_xiao_esp32s3`)
- A **USB HID gamepad/controller** (wired USB)
- A **USB-C OTG adapter** (USB-C male → USB-A female), or a **powered USB hub** + suitable cables
- Optional: a **3.3V UART adapter** for debug logs on pins **GPIO12 (TX)** / **GPIO13 (RX)** (see `platformio.ini`)

Power note: some controllers draw more current than the XIAO can comfortably supply. If your controller doesn’t enumerate or resets, use a powered hub.

## How to connect it

1. **Flashing / development:** connect the XIAO ESP32S3 to your computer via USB-C.
2. **Runtime (USB Host):** connect your USB controller to the XIAO’s USB-C port through an **OTG adapter** (or powered hub).
3. Pair the BLE device from your phone/tablet/PC like a normal Bluetooth controller.

## Flashing (PlatformIO)

This is a PlatformIO project (`platformio.ini`). The default environment enables USB Host (OTG) and routes debug output to a hardware UART.

- Build: `pio run -e seeed_xiao_esp32s3`
- Flash: `pio run -e seeed_xiao_esp32s3 -t upload`
- Serial monitor (if you use UART): `pio device monitor -b 115200`

There is also a development environment that disables USB Host and keeps USB CDC enabled (useful for quick flashing/monitoring over the built-in USB serial):

- Flash (dev): `pio run -e seeed_xiao_esp32s3_dev -t upload`

The project uses a custom partition table referenced by `platformio.ini`:

- `partitions_4MB.csv`

## Architecture overview

At a high level, the firmware runs a USB Host stack to enumerate a connected HID controller, parses the HID reports into a normalized internal `GamepadState`, then maps that state into BLE HID gamepad reports exposed via the BLE gamepad library. The USB side and BLE side are decoupled through this state representation so HID parsing and BLE report generation stay independent and easy to evolve.

## Code map

- `src/usb_host.*`: USB Host setup and device/report handling
- `src/hid_parser.*`: HID report parsing into a usable controller state
- `src/gamepad_state.h`: shared representation of buttons/axes
- `src/ble_gamepad.*`: BLE gamepad setup and report sending
- `src/main.cpp`: initialization and main loop orchestration

