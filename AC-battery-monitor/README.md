# AC Battery Monitor (second ESP32)

This sketch is a dedicated **AC-side JBD BLE monitor** for two batteries:

- `AC_Solax` (`a4:c1:37:20:4e:3b`)
- `AC_Growatt` (`a5:c2:37:49:c7:a2`)

It is intentionally separate from `BLE_BMS_MARK2` to avoid 4 persistent BLE client pressure on one ESP32.

## Purpose

- Keep `BLE_BMS_MARK2` focused on DC/Solis bridge duties.
- Monitor AC batteries on a second ESP32.
- Expose AC battery telemetry over Wi-Fi JSON (`/api/bms`).

## Dependencies

- ESP32 Arduino core
- `AsyncTCP`
- `ESPAsyncWebServer`
- ESP32 BLE libraries used by existing sketches (`BLEDevice`, `BLEScan`, etc.)

## Wi-Fi setup

The sketch includes `secrets.h` if present; otherwise it falls back to `secrets_template.h`.

### Recommended setup

1. Copy template:

```bash
cp AC-battery-monitor/secrets_template.h AC-battery-monitor/secrets.h
```

2. Edit `AC-battery-monitor/secrets.h` with your Wi-Fi credentials.

The template currently contains:

- SSID: `TP-LINK_73F3`
- Password: `DEADBEEF`

## Polling behavior

- Connects once at startup to both configured AC batteries.
- Keeps BLE links open and polls in `loop()`.
- Sends JBD read command `0x03` frequently (pack voltage/current/SOC/MOS/temp).
- Sends JBD read command `0x04` less frequently (`CELL_PACKET_EVERY_N_CYCLES`) for cell voltages.
- Reconnects using scan+connect behavior aligned with `BLE_BMS_MARK2` style.

## Safety / control behavior

- **Read-only JBD commands only** (`0x03`, `0x04`).
- No BMS configuration writes.
- No CAN behavior.
- No RS485 behavior.

## HTTP endpoints

- `/` : plain-text status
- `/status` : plain-text running status
- `/api/bms` : JSON telemetry for aggregate + both batteries

Example fields include:

- Root: `device`, `uptime_ms`, `connected_count`, `battery_count`
- `aggregate`: `valid`, `voltage_v`, `current_a`, `power_w`, `charge_power_w`, `discharge_power_w`, `temperature_c`
- Per battery: `name`, `mac`, `connected`, `has_data`, `voltage_v`, `current_a`, `power_w`, `soc`, `temperature_c`, `charge_mos`, `discharge_mos`, `cell_count`, `min_cell_mv`, `max_cell_mv`, `cell_delta_mv`, `ok_reads`, `failed_reads`, `disconnect_count`, `last_good_ms_ago`

## Quick test

```bash
curl http://<esp32-ip>/api/bms
```
