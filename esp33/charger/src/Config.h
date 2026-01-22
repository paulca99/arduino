#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Centralized Configuration for ESP32 Charger Diverter
// ============================================================================

// --- WiFi Configuration ---
// Replace with your network credentials
constexpr const char* WIFI_SSID = "mesh";
constexpr const char* WIFI_PASSWORD = "deadbeef";

// --- Web Server Configuration ---
constexpr int HTTP_PORT = 80;
constexpr int CSV_PORT = 8080;
constexpr long HTTP_TIMEOUT_MS = 2000;

// --- NTP Time Configuration ---
constexpr const char* NTP_SERVER = "pool.ntp.org";
constexpr long GMT_OFFSET_SEC = 0;
constexpr int DAYLIGHT_OFFSET_SEC = 3600;

// --- GTI Scheduling Configuration ---
constexpr int GTI_ON_HOUR = 5;   // GTI enabled from 05:00
constexpr int GTI_OFF_HOUR = 22; // GTI disabled at 22:00

// --- Pin Assignments ---
// Battery voltage measurement
constexpr int BATTERY_PIN = 39;

// Energy monitoring pins
constexpr int GRID_VOLTAGE_PIN = 34;
constexpr int GRID_CURRENT_PIN = 32;
constexpr int CHARGER_CURRENT_PIN = 33;
constexpr int GTI_CURRENT_PIN = 36;

// Control output pins
constexpr int CHARGER_RELAY_PIN = 21;
constexpr int GTI_RELAY_PIN = 23;

// PSU PWM control pins (5 series PSUs)
constexpr int PSU_PIN_1 = 19;
constexpr int PSU_PIN_2 = 18;
constexpr int PSU_PIN_3 = 5;
constexpr int PSU_PIN_4 = 22;
constexpr int PSU_PIN_5 = 16;

// --- PWM Configuration ---
constexpr int PWM_FREQUENCY = 200;
constexpr int PWM_RESOLUTION = 8; // 2^8 = 256 levels
constexpr int PWM_CHANNEL_0 = 0;
constexpr int PWM_CHANNEL_1 = 1;
constexpr int PWM_CHANNEL_2 = 2;
constexpr int PWM_CHANNEL_3 = 3;
constexpr int PWM_CHANNEL_4 = 4;

// --- Charger Control Limits ---
// Grid power thresholds (W): negative=export, positive=import
constexpr int DEFAULT_UPPER_CHARGER_LIMIT = 20;   // Above this import: reduce/stop charging
constexpr int DEFAULT_LOWER_CHARGER_LIMIT = -100; // Below this export: increase charging

// Battery voltage limits
constexpr float VOLTAGE_LIMIT = 57.0f;         // Battery voltage ceiling (V)
constexpr float VOLTAGE_HYSTERESIS = 1.0f;     // Hysteresis for voltage limit (V)

// Power and current limits
constexpr int CHARGER_POWER_LIMIT = 3000;      // Max charger power (W)
constexpr float CURRENT_LIMIT = 10.0f;         // Max charger current (A)

// State of charge (SOC) threshold
constexpr int SOC_FULL_THRESHOLD = 99;         // Consider battery full at this SOC (%)

// --- Energy Monitor Calibration Defaults ---
// Grid monitoring
constexpr float GRID_VOLTAGE_CALIBRATION = 1120.0f;
constexpr float GRID_CURRENT_CALIBRATION = 56.0f;
constexpr float GRID_PHASE_OFFSET = 0.2f;

// Charger monitoring
constexpr float CHARGER_VOLTAGE_CALIBRATION = 740.0f;
constexpr float CHARGER_CURRENT_CALIBRATION = 16.0f;
constexpr float CHARGER_PHASE_OFFSET = 0.2f;

// GTI monitoring
constexpr float GTI_VOLTAGE_CALIBRATION = 740.0f;
constexpr float GTI_CURRENT_CALIBRATION = 80.0f;
constexpr float GTI_PHASE_OFFSET = 0.2f;

// Grid power offset correction
constexpr float GRID_POWER_OFFSET = -90.0f; // Added to measured grid power

// Charger current offset correction
constexpr float CHARGER_CURRENT_OFFSET = -0.2f; // Added to measured charger current

// GTI current offset correction
constexpr float GTI_CURRENT_OFFSET = -0.2f; // Added to measured GTI current

// --- Battery Voltage Calibration ---
constexpr float BATTERY_VOLTAGE_MULTIPLIER = 22.85f; // ADC to voltage conversion
constexpr int BATTERY_HISTORY_SIZE = 150;            // Samples for moving average

// Battery voltage linear correction parameters
// Formula: corrected = (raw - V_CAL_X1) * (V_CAL_Y2 - V_CAL_Y1) / (V_CAL_X2 - V_CAL_X1) + V_CAL_Y1
constexpr float BATTERY_CAL_X1 = 39.5f;
constexpr float BATTERY_CAL_Y1 = 44.5f;
constexpr float BATTERY_CAL_X2 = 63.5f;
constexpr float BATTERY_CAL_Y2 = 61.6f;

// --- Control Algorithm Parameters ---
constexpr float POWER_ADJUST_DAMPING = 0.7f;  // Damping factor to avoid overshoot (0-1)
constexpr int POWER_STEP_DIVISOR = 50;        // Larger changes get bigger steps (W per step)

// --- Watchdog Timer ---
constexpr int TIMER_PRESCALER = 320;
constexpr uint64_t TIMER_ALARM_US = 3600000000ULL; // 1 hour in microseconds

// --- Main Loop Timing ---
constexpr int CHARGER_ADJUST_INTERVAL = 15; // Adjust charger every N loops

#endif // CONFIG_H
