/*
 * measurements.cpp — EmonLib AC measurements + ADS1115 battery voltage
 */
#include "measurements.h"
#include "control.h"

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ── Pin assignments ────────────────────────────────────────────────
static const int GRID_VOLTAGE_PIN    = 34;
static const int GRID_CURRENT_PIN    = 32;
static const int CHARGER_CURRENT_PIN = 33;

// ── Calibration constants ──────────────────────────────────────────
float gridVoltageCalibration    = 297.0f;
float gridPhaseOffset           = 1.4f;
float gridCurrentCalibration    = 52.6f;
float chargerCurrentCalibration = 30.0f;

// ── Live readings ──────────────────────────────────────────────────
float chargerPower        = 0.0f;
float gtiPower            = 0.0f;
float localBatteryVoltage = 0.0f;

// ── EmonLib instances ──────────────────────────────────────────────
EnergyMonitor emonGrid;
EnergyMonitor emonCharger;

// ── ADS1115 ───────────────────────────────────────────────────────
static Adafruit_ADS1115 ads1;

// Calibrated multiplier: 48.2 V / 0.757 V = 60.47
static const float BATT_ADC_MULTIPLIER    = 60.47f;
// Wiring resistance correction (charger CT Irms × R)
static const float WIRING_RESISTANCE_OHMS = 0.10f;

// Rolling average for battery voltage (60 samples)
static const int   BATT_HISTORY_SIZE = 60;
static float battHistory[BATT_HISTORY_SIZE] = {};
static int   battHistPtr = 0;

static void battAddHistory(float v) {
  battHistory[battHistPtr++] = v;
  if (battHistPtr >= BATT_HISTORY_SIZE) battHistPtr = 0;
}

static float battAverage() {
  float sum = 0.0f;
  for (int i = 0; i < BATT_HISTORY_SIZE; i++) sum += battHistory[i];
  return sum / BATT_HISTORY_SIZE;
}

// ── Public functions ───────────────────────────────────────────────

void reconfigureEmon() {
  emonGrid.voltage(GRID_VOLTAGE_PIN, gridVoltageCalibration, gridPhaseOffset);
  emonGrid.current(GRID_CURRENT_PIN, gridCurrentCalibration);
  // Charger uses the same voltage pin for phase reference
  emonCharger.voltage(GRID_VOLTAGE_PIN, 1500.0f, gridPhaseOffset);
  emonCharger.current(CHARGER_CURRENT_PIN, chargerCurrentCalibration);
}

void measurementsSetup() {
  reconfigureEmon();

  bool ads1ok = ads1.begin(0x48);
  if (!ads1ok) {
    Serial.println("[Meas] ADS1115 not found at 0x48!");
  } else {
    ads1.setGain(GAIN_FOUR);  // ±1.024 V — covers divider output up to ~60 V battery
    Serial.println("[Meas] ADS1115 OK, GAIN_FOUR");
  }

  // Pre-fill rolling average with current readings
  for (int i = 0; i < BATT_HISTORY_SIZE; i++) {
    readLocalBattery();
  }
}

void readGrid() {
  emonGrid.calcVI(40, 1000);
  emonGrid.realPower -= 90;  // known offset calibration
}

float readCharger() {
  if (!powerOn) {
    chargerPower = 0.0f;
    return chargerPower;
  }
  emonCharger.calcIrms(500);
  float cur = emonCharger.Irms - 0.2f;  // remove CT offset
  if (cur < 0.0f) cur = 0.0f;
  chargerPower = cur * emonGrid.Vrms;
  return chargerPower;
}

float readGti() {
  if (powerOn) {
    gtiPower = 0.0f;
    return gtiPower;
  }
  emonCharger.calcIrms(500);
  float cur = emonCharger.Irms - 0.2f;
  if (cur < 0.0f) cur = 0.0f;
  gtiPower = cur * emonGrid.Vrms;

  // Dynamically adjust lower charger limit based on GTI output
  if (gtiPower > 120) {
    lowerChargerLimit = (int)(gtiPower * -1.0f);
  } else {
    lowerChargerLimit = -120;
  }
  return gtiPower;
}

void readLocalBattery() {
  int16_t adc     = ads1.readADC_SingleEnded(0);
  float pinVolts  = ads1.computeVolts(adc);
  float measured  = pinVolts * BATT_ADC_MULTIPLIER;
  // Correct for voltage drop across wiring resistance
  float corrected = measured - (emonCharger.Irms * WIRING_RESISTANCE_OHMS);
  battAddHistory(corrected);
  localBatteryVoltage = battAverage();
}
