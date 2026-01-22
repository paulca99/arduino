# ESP32 Charger Diverter (5x series PSU) + GTI interlock

This folder contains an ESP32/Arduino sketch that **diverts excess PV power into a battery charger** by dynamically adjusting a chain of **five series-connected 9–13V power supply units (PSUs)**. It measures **grid import/export power**, **charger current**, and **battery voltage**, then modulates PWM outputs to keep grid power near a target band.

A critical safety feature of this project is the **GTI ↔ charger interlock**:
- When the **charger is ON**, the **GTI is forced OFF**.
- When the **charger is OFF**, the **GTI may be ON** (subject to time-of-day enable).
This prevents a dangerous feedback loop where *charger → GTI → charger*.

## What it does (high level)

- Reads **grid power** (`grid.realPower`) where:
  - negative = exporting
  - positive = importing
- If exporting beyond a threshold (default `lowerChargerLimit = -100W`), it **turns the charger on** and **increases charging power** until grid export is reduced.
- If importing beyond a threshold (default `upperChargerLimit = +20W`), or if the battery voltage/power limits are reached, it **reduces charging power** and may **turn the charger off**.
- Provides a simple HTTP status page and a CSV endpoint for monitoring and tuning.

## Control strategy (state machine)

```text
Legend:
  Pgrid = grid.realPower (W); negative=export, positive=import
  Vbatt = readBattery()
  Vlimit = 57.0V (with hysteresis latch)
  Upper = upperChargerLimit (import limit, default +20W)
  Lower = lowerChargerLimit (export threshold, default -100W)

                     (Vlimit hit OR Pgrid > Upper OR ChargerPower>PLimit)
          +-------------------------------------------------------------------+
          |                                                                   |
          v                                                                   |
  +---------------------+         (Pgrid < Lower AND SOC<99 AND NOT Vlimit)   |
  |  CHARGER OFF state  |---------------------------------------------------+ |
  |  - relay OFF         |                                                   | |
  |  - GTI ON *if allowed|                                                   | |
  +---------------------+                                                   | |
          ^                                                                 | |
          | (at min power and still need to reduce / stop charging)         | |
          |                                                                 v |
  +---------------------+      regulate to keep Pgrid in [Lower..Upper]  +---------------------+
  |  CHARGER ON state   |<----------------------------------------------->|  CHARGER ON state  |
  |  (GTI forced OFF)   |   increase power if Pgrid < Lower               |  (GTI forced OFF)  |
  |  - relay ON          |   reduce power if Pgrid > Upper or Vlimit       |  PWM “more” / “less|
  |  - PWM controls 5 PSUs|                                                |  power” sub-actions|
  +---------------------+                                                +---------------------+

GTI scheduling overlay:
  `timestuff.cpp` enables GTI between 05:00 and 22:00 via NTP time.
```

## Folder contents

- `charger.ino`  
  Main sketch. Calls setup for WiFi, NTP time, energy monitoring, battery ADC smoothing, and PWM/relay outputs. Runs the main loop and periodically calls `adjustCharger()`.

- `pcemon.cpp/.h`  
  Grid/charger/GTI measurement using a port of EmonLib (`espEmonLib`). Produces:
  - `grid.realPower`, `grid.Vrms`, etc.
  - `chargerPower` from charger RMS current × grid RMS voltage
  - `gtiPower` similarly (used for monitoring + GTI behavior)

- `battery.cpp/.h`  
  Battery voltage on ADC pin 39 with smoothing (150-sample average) and a linear correction mapping.

- `pwmFunctions.cpp/.h`  
  Output control:
  - Main charger relay/enable (`powerPin`)
  - GTI enable (`gtiPin`) with charger/GTI mutual exclusion
  - 5 PWM channels controlling the series PSUs (`psu_voltage_pins[]`)
  - The primary control algorithm: `adjustCharger()`

- `pcwifi.cpp/.h`  
  Wi-Fi connection + basic web UI:
  - Port **80** HTML status + configuration via query parameters
  - Port **8080** CSV endpoint for logging/graphing

- `timestuff.cpp/.h`  
  NTP time + GTI on/off scheduling by hour.

## Web interface

- **Port 80**: status page (grid power/voltage, charger current, battery voltage, PSU PWM values, etc.)
  - Can update power limits (upper/lower) via query parameters handled in `pcwifi.cpp`.
- **Port 8080**: returns a CSV line of current stats (for dashboards).

## Key configuration knobs

In `pwmFunctions.cpp`:
- `upperChargerLimit` (W): above this import level the code reduces power / may shut charger off
- `lowerChargerLimit` (W): below this export level the code increases charger power
- `voltageLimit` (V): battery voltage ceiling with hysteresis latch
- `chargerPLimit` (W): absolute cap on charger watts
- `current_limit` (A): cap based on charger RMS current

In `timestuff.cpp`:
- `GTI_on_time`, `GTI_off_time` (hours): GTI allowed window

## Safety notes

This project controls mains-related equipment and multiple power supplies. Ensure:
- proper isolation, fusing, and enclosures
- correct CT orientation (negative must mean export for the control logic)
- GTI/charger interlock behavior matches your wiring (the code assumes **charger ON disables GTI**)

## Build notes

Open `charger.ino` in Arduino IDE (ESP32 core required) and upload to an ESP32. Wiring/pin assignments are in the source (`pcemon.cpp`, `battery.cpp`, `pwmFunctions.cpp`).
