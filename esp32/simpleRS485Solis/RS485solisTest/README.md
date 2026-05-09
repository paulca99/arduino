# RS485solisTest web monitor

This sketch keeps the existing Solis RS485 test folder but adds a lightweight ESP32-hosted monitor page.

## Setup

1. Copy `secrets_template.h` to `secrets.h` in the sketch folder.
2. Fill in your Wi-Fi SSID and password in `secrets.h`.
3. If your inverter uses a different Modbus slave ID, change `SOLIS_SLAVE_ID` in `secrets.h`.
4. Install the ESP32 Arduino core plus the `AsyncTCP` and `ESPAsyncWebServer` libraries.
5. Upload the sketch, open the serial monitor at 115200 baud, and browse to the printed IP address.

The page serves the UI on `/` and live JSON data on `/api/solis`.
