# RS485solisTest web monitor

This sketch keeps the existing Solis RS485 test folder but adds a lightweight ESP32-hosted monitor page.

## Setup

1. Copy `secrets_template.h` to `secrets.h` in the sketch folder.
2. Fill in your Wi-Fi SSID and password in `secrets.h`.
3. If your inverter uses a different Modbus slave ID, change `SOLIS_SLAVE_ID` in `secrets.h`.
4. Install the ESP32 Arduino core plus the `AsyncTCP` and `ESPAsyncWebServer` libraries.
5. Upload the sketch, open the serial monitor at 115200 baud, and browse to the printed IP address.

The page serves the UI on `/` and live JSON data on `/api/solis`.
The browser polls every **5 seconds**.

After each complete poll cycle the ESP32 also prints a compact one-line log to the serial monitor:

```
ms=<uptime> 33050=<raw> 33051=<raw> 33052=<raw> ... (all polled registers, ? if read failed)
```

This lets you correlate serial logs against your BMS / inverter readings while the web UI stays live.

## Live-watch registers

**Confirmed:**
- `33050` PV string 1 voltage (0.1 V), `33051` PV string 1 current (0.1 A)
- `33052` PV string 2 voltage (0.1 V), `33053` PV string 2 current (0.1 A)
- `33074` Grid voltage (0.1 V), `33095` Grid frequency (0.01 Hz)
- `33132` Grid power (signed 16-bit W; negative = importing, positive = exporting)
- `33140` Battery SOC (%), `33142` Battery voltage (0.01 V)

**Strong candidates being monitored:**
- `33135` Battery current (likely 0.1 A)
- `33136` Battery charge/discharge direction flag
- `33059` Battery/power-related unknown metric (not direct battery power)

## Derived values shown in the UI

- Battery power (derived): `(33142 / 100.0) * (33135 / 10.0)`
- PV1 power (derived): `(33050 / 10.0) * (33051 / 10.0)`
- PV2 power (derived): `(33052 / 10.0) * (33053 / 10.0)`
- Total PV power (derived): `PV1 power + PV2 power`
