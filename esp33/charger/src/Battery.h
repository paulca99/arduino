#ifndef BATTERY_H
#define BATTERY_H

// ============================================================================
// Battery Voltage Measurement Module
// ============================================================================
// Reads battery voltage from ADC with smoothing via moving average filter.
// Applies linear calibration correction to measured voltage.

void setupBattery();
float readBattery();

#endif // BATTERY_H
