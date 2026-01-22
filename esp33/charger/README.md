# ESP32 Charger Diverter (Reorganized)

**Reorganized version of `esp32/charger` with improved code structure while maintaining identical functionality.**

This folder contains an ESP32/Arduino sketch that **diverts excess PV power into a battery charger** by dynamically adjusting a chain of **five series-connected 9–13V power supply units (PSUs)**. It measures **grid import/export power**, **charger current**, and **battery voltage**, then modulates PWM outputs to keep grid power near a target band.

## Key Features

### Safety: GTI ↔ Charger Interlock

A critical safety feature of this project is the **GTI (Grid Tie Inverter) ↔ charger mutual exclusion interlock**:
- When the **charger is ON**, the **GTI is forced OFF**.
- When the **charger is OFF**, the **GTI may be ON** (subject to time-of-day enable).
- This prevents a dangerous feedback loop where *charger → GTI → charger*.

### Power Sign Convention

- **Grid power** (`grid.realPower`):
  - **Negative** = exporting to grid
  - **Positive** = importing from grid

### Control Strategy

- If exporting beyond a threshold (default `lowerChargerLimit = -100W`), it **turns the charger on** and **increases charging power** until grid export is reduced.
- If importing beyond a threshold (default `upperChargerLimit = +20W`), or if the battery voltage/power limits are reached, it **reduces charging power** and may **turn the charger off**.
- Provides a simple HTTP status page and a CSV endpoint for monitoring and tuning.

## State Machine Diagram

```text
Legend:
  Pgrid = grid.realPower (W); negative=export, positive=import
  Vbatt = readBattery()
  Vlimit = 57.0V (with hysteresis latch)
  Upper = upperChargerLimit (import limit, default +20W)
  Lower = lowerChargerLimit (export threshold, default -100W)
  PLimit = chargerPLimit (max charger power, default 3000W)
  ILimit = current_limit (max charger current, default 10A)

                     (Vlimit hit OR Pgrid > Upper OR ChargerPower>PLimit)
          +-------------------------------------------------------------------+
          |                                                                   |
          v                                                                   |
  +---------------------+         (Pgrid < Lower AND SOC<99 AND NOT Vlimit)  |
  |  CHARGER OFF state  |---------------------------------------------------+ |
  |  - relay OFF        |                                                   | |
  |  - GTI ON *if time  |                                                   | |
  |    schedule allows  |                                                   | |
  +---------------------+                                                   | |
          ^                                                                 | |
          | (at min power and still need to reduce / stop charging)        | |
          |                                                                 v |
  +---------------------+      regulate to keep Pgrid in [Lower..Upper]  +---------------------+
  |  CHARGER ON state   |<----------------------------------------------->|  CHARGER ON state  |
  |  (GTI forced OFF)   |   increase power if Pgrid < Lower              |  (GTI forced OFF)  |
  |  - relay ON         |   reduce power if Pgrid > Upper or Vlimit      |  PWM "more" / "less|
  |  - PWM controls 5   |   reduce power if ChargerPower > PLimit        |  power" sub-actions|
  |    PSUs in series   |   reduce power if Irms > ILimit                |                    |
  +---------------------+                                                +---------------------+

GTI scheduling overlay:
  TimeSchedule module enables GTI between 05:00 and 22:00 via NTP time.
  Outside these hours, GTI is disabled regardless of charger state.
```

## Project Structure

This reorganized version follows a cleaner Arduino project structure:

```
esp33/charger/
├── charger.ino              # Thin entry point (setup/loop)
├── README.md                # This file
└── src/                     # Implementation modules
    ├── Config.h             # Centralized configuration constants
    ├── Battery.h/cpp        # Battery voltage reading with smoothing
    ├── Emon.h/cpp           # Energy monitoring (grid/charger/GTI)
    ├── EmonLib.h/cpp        # ESP32 energy monitoring library
    ├── PwmController.h/cpp  # PWM control and charger adjustment algorithm
    ├── WiFiUI.h/cpp         # WiFi connection and web interface
    └── TimeSchedule.h/cpp   # NTP time and GTI scheduling
```

### Module Descriptions

- **`charger.ino`**  
  Main sketch entry point. Calls setup for all modules and runs the main control loop.

- **`Config.h`**  
  Centralized configuration constants: pins, calibration defaults, limits, WiFi credentials, GTI hours, etc.

- **`Battery.h/cpp`**  
  Battery voltage measurement on ADC pin 39 with 150-sample moving average smoothing and linear calibration correction.

- **`Emon.h/cpp`**  
  Grid/charger/GTI measurement using a port of EmonLib. Produces:
  - `grid.realPower`, `grid.Vrms`, etc.
  - `chargerPower` from charger RMS current × grid RMS voltage
  - `gtiPower` similarly (used for monitoring + GTI behavior)

- **`EmonLib.h/cpp`**  
  ESP32-compatible energy monitoring library (based on OpenEnergyMonitor EmonLib).

- **`PwmController.h/cpp`**  
  Output control:
  - Main charger relay/enable (`CHARGER_RELAY_PIN`)
  - GTI enable (`GTI_RELAY_PIN`) with charger/GTI mutual exclusion
  - 5 PWM channels controlling the series PSUs
  - The primary control algorithm: `adjustCharger()`

- **`WiFiUI.h/cpp`**  
  Wi-Fi connection + basic web UI:
  - Port **80**: HTML status page + configuration via query parameters
  - Port **8080**: CSV endpoint for logging/graphing

- **`TimeSchedule.h/cpp`**  
  NTP time synchronization + GTI on/off scheduling by hour (default: 05:00–22:00).

## Web Interface

- **Port 80**: Status page showing:
  - Grid power/voltage
  - Charger current
  - Battery voltage
  - PSU PWM values (0–255)
  - Configuration form to update upper/lower power limits

- **Port 8080**: Returns a CSV line of current stats (for dashboards/logging):
  ```
  BatteryV,GridP,GridV,TotalResistance,ChargerP,GTIP,Hour,GTIenabled,EOT
  ```

## Configuration

All configuration constants are centralized in `src/Config.h`:

### Power Control Limits
- `DEFAULT_UPPER_CHARGER_LIMIT` (W): Above this import level, reduce power or shut charger off
- `DEFAULT_LOWER_CHARGER_LIMIT` (W): Below this export level, increase charger power
- `VOLTAGE_LIMIT` (V): Battery voltage ceiling with hysteresis latch
- `CHARGER_POWER_LIMIT` (W): Absolute cap on charger watts
- `CURRENT_LIMIT` (A): Cap based on charger RMS current

### GTI Scheduling
- `GTI_ON_HOUR`: Hour to enable GTI (default: 5)
- `GTI_OFF_HOUR`: Hour to disable GTI (default: 22)

### WiFi
- `WIFI_SSID`: Your WiFi network name
- `WIFI_PASSWORD`: Your WiFi password

### Pin Assignments
- Battery, grid/charger/GTI current sensors, relay controls, PSU PWM outputs
- All defined in `Config.h`

### Calibration
- Energy monitor calibration constants for voltage/current/phase
- Battery voltage calibration parameters
- All defined with sensible defaults in `Config.h`

## Improvements Over Original

### Code Quality
1. **Centralized configuration**: All constants in `Config.h` instead of scattered across files
2. **Fixed bug**: Uninitialized variable `retval` in `getTotalResistance()` now initialized to 0
3. **Reduced global state**: More encapsulation within modules
4. **Better naming**: Clear module names (`Battery.*`, `Emon.*`, `PwmController.*`, etc.)
5. **Improved comments**: Comprehensive documentation throughout

### Structure
1. **Organized hierarchy**: Implementation in `src/` subdirectory
2. **Thin entry point**: `charger.ino` is minimal, delegates to modules
3. **Clear module boundaries**: Each module has a specific responsibility
4. **Consistent style**: `constexpr`, proper includes, namespace awareness

### Maintainability
1. **Single source of truth**: Configuration in one place
2. **Easier testing**: Modular structure supports testing individual components
3. **Clear dependencies**: Explicit includes show module relationships
4. **Better documentation**: README with state machine diagram

## Safety Notes

This project controls mains-related equipment and multiple power supplies. Ensure:
- Proper isolation, fusing, and enclosures
- Correct CT orientation (negative must mean export for the control logic)
- GTI/charger interlock behavior matches your wiring (the code assumes **charger ON disables GTI**)
- All safety regulations and electrical codes are followed
- Appropriate over-current and over-voltage protection

## Build Notes

1. Open `charger.ino` in Arduino IDE with ESP32 core installed
2. Install required libraries:
   - WiFi (built-in with ESP32 core)
   - ESPAsyncWebSrv
   - time (built-in)
3. Configure WiFi credentials in `src/Config.h`
4. Upload to ESP32

## Hardware Requirements

- ESP32 development board
- 5 series-connected 9–13V power supply units (PSUs) with PWM control
- Current transformers (CTs) for grid, charger, and GTI measurements
- Voltage divider for battery voltage measurement
- Relays for charger and GTI control
- Appropriate power electronics, isolation, and safety equipment

## Pin Wiring

See `src/Config.h` for complete pin assignments:
- **Battery voltage**: ADC pin 39
- **Grid voltage**: ADC pin 34
- **Grid current**: ADC pin 32
- **Charger current**: ADC pin 33
- **GTI current**: ADC pin 36
- **Charger relay**: GPIO 21
- **GTI relay**: GPIO 23
- **PSU PWM**: GPIOs 19, 18, 5, 22, 16

## Differences from esp32/charger

This `esp33/charger` folder is a reorganized version of `esp32/charger` with:
- Identical functionality (behavior-equivalent)
- Improved code structure and organization
- Centralized configuration
- Fixed minor bugs (uninitialized variables)
- Enhanced documentation
- Same libraries and dependencies

The original `esp32/charger` folder remains unchanged.

## License

Same as original (GNU GPL from EmonLib).

## Credits

- Original EmonLib by Trystan Lea (OpenEnergyMonitor project)
- ESP32 port and charger diverter application by Paul Carroll
- Reorganization and documentation improvements: 2026
