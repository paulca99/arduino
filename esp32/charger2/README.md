# charger2 — Refactored ESP32 Charger Firmware

This folder replaces the single-file `charger2usingrs485.ino` prototype with a
clean, modular firmware that performs **all the same tasks as `charger/`** while
adding async Solis RS485 polling, a richer web UI, and a JSON API.

> ⚠️ The original `charger/` firmware is **unchanged** and continues to work
> independently. This directory is a complete rewrite of the `charger2usingrs485.ino`
> prototype, expanding it into a full modular firmware.  The original prototype
> is preserved as `charger2usingrs485.ino.bak` for reference.

---

## What it does

| Feature | charger/ | charger2/ |
|---|---|---|
| EmonLib grid/charger/GTI readings | ✓ | ✓ |
| Local ADC battery (ADS1115) | ✓ | ✓ |
| PWM control of 5 series PSUs | ✓ | ✓ |
| GTI ↔ charger interlock | ✓ | ✓ |
| NTP time-based GTI schedule | ✓ | ✓ |
| Solis RS485/Modbus polling | — | ✓ |
| Solis battery/grid/PV data | — | ✓ |
| JSON API (`/api/status`) | — | ✓ |
| Async web UI with live cards | — | ✓ |
| Web-configurable parameters | ✓ (basic) | ✓ (improved) |

---

## File layout

```
charger2/
├── charger2.ino              Main sketch (setup + loop)
├── secrets_template.h        Copy → secrets.h, fill in WiFi credentials
├── secrets.h                 (git-ignored — do NOT commit)
├── solis_modbus.h/.cpp       RS485 Modbus poller (FreeRTOS task, Core 1)
├── measurements.h/.cpp       EmonLib AC + ADS1115 battery readings
├── control.h/.cpp            PWM/relay charger control algorithm
├── web.h/.cpp                AsyncWebServer UI (80) + JSON API (8080)
├── timestuff.h/.cpp          NTP + GTI time-of-day scheduling
├── espEmonLib.h/.cpp         Local copy of EmonLib (ESP32-tuned)
├── .gitignore                Ignores secrets.h
└── README.md                 This file
```

---

## Wiring

### RS485 (Solis inverter)

The RS485 interface uses **UART2** remapped to safe pins that do not conflict
with the PSU PWM outputs.

| ESP32 GPIO | RS485 module pin | Notes |
|---|---|---|
| **25** | RO (Receive Out) | UART2 RX |
| **26** | DI (Driver Input) | UART2 TX |
| GND | GND | Common ground |
| 3.3 V or 5 V | VCC | Per module spec |

If your RS485 module requires manual DE/RE direction control (not
auto-direction), connect a spare GPIO to DE/RE and set `RS485_DE_PIN` in
`solis_modbus.h` to that pin number.  The default is `-1` (auto-direction).

Solis RS485 wiring: connect module A/B to the inverter's RS485 terminals.
Baud rate is **9600**, 8N1.  Default Solis device address is **1**.

### PSU PWM outputs (unchanged from charger/)

| ESP32 GPIO | PWM channel | PSU |
|---|---|---|
| 19 | 0 | PSU 1 |
| 18 | 1 | PSU 2 |
|  5 | 2 | PSU 3 |
| 14 | 4 | PSU 4 |
| 16 | 3 | PSU 5 / Afterburner |

### Relay outputs

| ESP32 GPIO | Function | Active level |
|---|---|---|
| 13 | Charger main relay | LOW = ON |
| 27 | Afterburner relay | LOW = ON |
| 23 | GTI relay | HIGH = ON |

### AC current / voltage sensing (EmonLib)

| ESP32 GPIO | Signal |
|---|---|
| 34 | Grid voltage (transformer) |
| 32 | Grid current CT |
| 33 | Charger / GTI current CT |

### Battery voltage (ADS1115)

I2C address **0x48** (default).  Battery voltage divider on **AIN0**.
Gain set to GAIN_FOUR (±1.024 V) — calibrated multiplier `60.47`.

---

## WiFi setup

```bash
cp secrets_template.h secrets.h
# Edit secrets.h and replace YOUR_SSID / YOUR_PASSWORD
```

`secrets.h` is in `.gitignore` and will never be committed.

---

## Web interface

### Port 80 — Status dashboard

Visit `http://<esp32-ip>/` in a browser.

- **Live cards** — auto-refresh every 2 s via the JSON API:
  - Charger ON/OFF state, afterburner status
  - Grid power, voltage, power factor
  - Charger power, charger Irms, GTI power
  - Local battery voltage
  - Solis: PV1/PV2, Solis battery V/I/P, Solis grid V/I/P
  - Inverter temperature, daily/total energy
- **Configuration form** — adjust control parameters without reflashing:
  - Upper / lower grid limits (W)
  - Voltage limit (V)
  - Charger power limit (W)
  - Power creep (0/1)

Parameters are applied immediately on form submit (GET request with query
params).

### Port 8080 — JSON API

#### `GET /api/status`

Returns a JSON object with three nested objects:

```json
{
  "charger": {
    "powerOn": false,
    "afterburnerOn": false,
    "GTIenabled": true,
    "chargerPower": 0.0,
    "gtiPower": 45.2,
    "localBatteryV": 52.31,
    "upperLimit": 100,
    "lowerLimit": 0,
    "voltageLimit": 58.4,
    "chargerPLimit": 2900,
    "chargerPowerCreep": 0,
    "totalResistance": 2555,
    "psuValues": [511, 511, 511, 511, 511]
  },
  "grid": {
    "realPower": -35.0,
    "Vrms": 243.2,
    "Irms": 0.18,
    "powerFactor": 0.921
  },
  "solis": {
    "valid": true,
    "ageMs": 812,
    "pv1Voltage": 382.1,
    "pv1Current": 3.2,
    "pv2Voltage": 0.0,
    "pv2Current": 0.0,
    "batteryVoltage": 51.8,
    "batteryCurrent": -4.5,
    "batteryPower": -233,
    "gridVoltage": 242.8,
    "gridCurrent": 1.1,
    "gridPower": -265,
    "frequency": 50.02,
    "powerFactor": 0.998,
    "inverterTemp": 38.5,
    "totalPower": 1225,
    "dailyEnergy": 3.41,
    "totalEnergy": 4872.0
  }
}
```

`solis.ageMs` — milliseconds since the last successful RS485 poll.  Values
above ~5 000 ms indicate a communication problem.

#### `GET /api/solis` (legacy alias)

Returns only the Solis fields in the format used by `charger2usingrs485.ino`
for backwards compatibility with existing dashboards.

---

## Control strategy

Same as `charger/` — see `charger/README.md` for the state-machine diagram.

**Grid measurement source** — `readGrid()` uses a two-tier strategy:

1. **Primary (Solis Modbus)** — when the RS485 poller has valid data that is
   less than 5 000 ms old, `emonGrid.Vrms/Irms/realPower/powerFactor` are
   populated directly from the Solis inverter's grid registers
   (`gridVoltage / gridCurrent / gridPower / powerFactor`).
   The CT-specific −90 W offset is **not** applied in this mode.

2. **Fallback (CT clamp)** — if Solis data is stale or has never arrived,
   `readGrid()` falls back to the local CT + voltage-transformer measurement
   via EmonLib, including the −90 W calibration offset.

A Serial log message is printed once whenever the active source changes.

Key control parameters:

| Parameter | Default | Description |
|---|---|---|
| `upperChargerLimit` | 100 W | Reduce/stop charging when grid import exceeds this |
| `lowerChargerLimit` | 0 W | Start/increase charging when grid export exceeds this |
| `voltageLimit` | 58.4 V | Battery voltage ceiling (local ADC, with hysteresis) |
| `chargerPLimit` | 2 900 W | Absolute charger watt cap |
| `chargerPowerCreep` | 0 | If 1, chargerPLimit auto-increments by 5 W when hit |

---

## Library dependencies

Install via Arduino Library Manager or PlatformIO:

| Library | Version tested | Notes |
|---|---|---|
| `ESP32 Arduino core` | ≥ 2.x | Board support package |
| `AsyncTCP` | ≥ 1.1 | TCP foundation for async web server |
| `ESPAsyncWebServer` | ≥ 1.2 | Async HTTP server |
| `Adafruit ADS1X15` | ≥ 2.x | ADS1115 I2C ADC |
| `Adafruit BusIO` | ≥ 1.x | I2C/SPI abstraction (dependency of ADS1X15) |

`espEmonLib.h/.cpp` is a local copy bundled in this directory — no separate
install needed.

---

## How it differs from `charger/`

1. **RS485 Modbus polling** — a FreeRTOS task on Core 1 polls the Solis
   inverter every ~600 ms and updates `SolisData` behind a mutex.  The main
   control loop on Core 0 is never blocked by RS485 I/O.

2. **JSON API** — `/api/status` replaces the CSV endpoint.  It combines charger
   control state with live Solis inverter data in one response.

3. **Modular source** — code split into `solis_modbus`, `measurements`,
   `control`, `web`, and `timestuff` modules instead of one large file.

4. **Async web server** — both port 80 and port 8080 use `ESPAsyncWebServer`,
   eliminating the blocking `WiFiServer` poll loop from `charger/`.

5. **Two battery sources** — `localBatteryV` is the ADS1115 ADC (the battery
   being charged); `solis.batteryVoltage` is the Solis inverter's own battery
   (a different pack).  The charger voltage limit is based on `localBatteryV`.
