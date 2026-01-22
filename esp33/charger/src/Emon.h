#ifndef EMON_H
#define EMON_H

#include "EmonLib.h"

// ============================================================================
// Energy Monitoring Module
// ============================================================================
// Measures grid power, charger current, and GTI current using CT sensors.
// Uses EmonLib for RMS calculations.

// Energy monitor instances
extern EnergyMonitor grid;
extern EnergyMonitor charger;
extern EnergyMonitor gti;

// Calibration values (can be updated at runtime via web interface)
extern float gridVoltageCalibration;
extern float gridCurrentCalibration;
extern float gridPhaseOffset;
extern float chargerVoltageCalibration;
extern float chargerCurrentCalibration;
extern float chargerPhaseOffset;
extern float gtiVoltageCalibration;
extern float gtiCurrentCalibration;
extern float gtiPhaseOffset;

// Calculated power values
extern float chargerPower;
extern float gtiPower;

// Power state (managed by PwmController)
extern bool powerOn;

void setupEmon();
void readGrid();
float readCharger();
float readGti();

#endif // EMON_H
