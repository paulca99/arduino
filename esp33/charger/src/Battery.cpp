#include "Arduino.h"
#include "Battery.h"
#include "Config.h"

// ============================================================================
// Battery Module Implementation
// ============================================================================

// Static state for battery voltage smoothing
static float batteryHistory[BATTERY_HISTORY_SIZE];
static int historyPointer = 0;

// Add value to circular history buffer
static void addToHistory(float value)
{
  batteryHistory[historyPointer] = value;
  historyPointer++;
  if (historyPointer == BATTERY_HISTORY_SIZE) {
    historyPointer = 0;
  }
}

// Get average of all values in history
static float getAverageValue()
{
  float total = 0.0f;
  for (int i = 0; i < BATTERY_HISTORY_SIZE; i++)
  {
    total += batteryHistory[i];
  }
  return total / BATTERY_HISTORY_SIZE;
}

// Read battery voltage with calibration and smoothing
float readBattery()
{
  int adcValue = 0;
  delay(5);
  adcValue += analogRead(BATTERY_PIN);
  
  // Convert ADC reading to voltage
  // 4095 = 3.3V (12-bit ADC)
  float voltageOnPin = (adcValue * 3.3f) / 4095.0f;
  float rawVoltage = voltageOnPin * BATTERY_VOLTAGE_MULTIPLIER;

  // Apply linear calibration correction
  // Formula: y = (x - x1) * (y2 - y1) / (x2 - x1) + y1
  float calibratedVoltage = (rawVoltage - BATTERY_CAL_X1) * 
                            (BATTERY_CAL_Y2 - BATTERY_CAL_Y1) / 
                            (BATTERY_CAL_X2 - BATTERY_CAL_X1) + 
                            BATTERY_CAL_Y1;
  
  addToHistory(calibratedVoltage);
  return getAverageValue();
}

// Initialize battery voltage reading
void setupBattery()
{
  pinMode(BATTERY_PIN, INPUT);
  
  // Pre-fill history buffer with initial readings
  for (int i = 0; i < BATTERY_HISTORY_SIZE; i++)
  {
    readBattery();
  }
}
