#include "Arduino.h"
#include "Emon.h"
#include "Config.h"

// ============================================================================
// Energy Monitoring Module Implementation
// ============================================================================

// Energy monitor instances
EnergyMonitor grid;
EnergyMonitor charger;
EnergyMonitor gti;

// Calibration values (initialized from Config.h, can be updated at runtime)
float gridVoltageCalibration = GRID_VOLTAGE_CALIBRATION;
float gridCurrentCalibration = GRID_CURRENT_CALIBRATION;
float gridPhaseOffset = GRID_PHASE_OFFSET;

float chargerVoltageCalibration = CHARGER_VOLTAGE_CALIBRATION;
float chargerCurrentCalibration = CHARGER_CURRENT_CALIBRATION;
float chargerPhaseOffset = CHARGER_PHASE_OFFSET;

float gtiVoltageCalibration = GTI_VOLTAGE_CALIBRATION;
float gtiCurrentCalibration = GTI_CURRENT_CALIBRATION;
float gtiPhaseOffset = GTI_PHASE_OFFSET;

// Calculated power values
float chargerPower = 0.0f;
float gtiPower = 0.0f;

// Initialize energy monitoring
void setupEmon()
{
  grid.voltage(GRID_VOLTAGE_PIN, gridVoltageCalibration, gridPhaseOffset);
  grid.current(GRID_CURRENT_PIN, gridCurrentCalibration);
  
  charger.voltage(GRID_VOLTAGE_PIN, chargerVoltageCalibration, chargerPhaseOffset);
  charger.current(CHARGER_CURRENT_PIN, chargerCurrentCalibration);
  
  gti.voltage(GRID_VOLTAGE_PIN, gtiVoltageCalibration, gtiPhaseOffset);
  gti.current(GTI_CURRENT_PIN, gtiCurrentCalibration);
}

// Read grid voltage and power
void readGrid()
{
  grid.calcVI(60, 4000);  // Calculate all. No.of half wavelengths (crossings), time-out
  grid.realPower = grid.realPower + GRID_POWER_OFFSET;
}

// Read charger current and calculate power
float readCharger(bool powerOn)
{
  if (!powerOn) {
    chargerPower = 0.0f;
    return chargerPower;
  }

  charger.calcIrms(1000);
  float current = charger.Irms;
  current = current + CHARGER_CURRENT_OFFSET;
  if (current < 0.0f) {
    current = 0.0f;
  }
  chargerPower = current * grid.Vrms;
  return chargerPower;
}

// Read GTI current and calculate power
float readGti(bool powerOn)
{
  if (powerOn) {
    gtiPower = 0.0f;
    return gtiPower;
  }
  
  gti.calcIrms(100);
  float current = gti.Irms;
  current = current + GTI_CURRENT_OFFSET;
  if (current < 0.0f) {
    current = 0.0f;
  }
  gtiPower = current * grid.Vrms;
  return gtiPower;
}
