#include "Arduino.h"
#include "espEmonLib.h"
#include "pwmFunctions.h"
#include "battery.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads1;  // Single ADS1115 at 0x48, 3.3V VDD

extern boolean afterburnerOn;

float batteryTotalVoltage = 0.0;
float history[60];
int arraySize = 60;
int historyPointer = 0;

void addToHistory(float value)
{
  history[historyPointer] = value;
  historyPointer++;
  if (historyPointer == arraySize)
  {
    historyPointer = 0;
  }
}

float getAverageValue()
{
  float retval = 0.0;
  for (int i = 0; i < arraySize; i++)
  {
    retval += history[i];
  }
  retval = retval / arraySize;
  return retval;
}

float getMinValue()
{
  float retval = 70.0;
  for (int i = 0; i < arraySize; i++)
  {
    if (history[i] < retval)
    {
      retval = history[i];
    }
  }
  return retval;
}

float readBattery()
{
  batteryTotalVoltage = readBatteryOnce();
  addToHistory(batteryTotalVoltage);
  float batt = getAverageValue();
  return batt;
}

float readBatteryOnce()
{
  int16_t adc = ads1.readADC_SingleEnded(0);
  float voltageOnPinADS = ads1.computeVolts(adc);
  // Ohm's law: Vbatt = Vout * multiplier
  // Calibrated by direct measurement: 48.2V total, 0.757V at ADS input => 48.2/0.757 = 63.67
  batteryTotalVoltage = voltageOnPinADS * 63.67;
  Serial.println("ADS V=" + (String)voltageOnPinADS + " batt=" + (String)batteryTotalVoltage);
  return batteryTotalVoltage;
}

float checkBattery()  // accurate reading by stopping charger and GTI first
{
  turnGTIOff();
  turnPowerOff();
  delay(8000);
  float accbatt = readBatteryOnce();
  Serial.println("Accurate Battery reading =:" + (String)accbatt);
  delay(1000);
  return accbatt;
}

void setupBattery()
{
  bool ads1_ok = ads1.begin(0x48);
  if (!ads1_ok)
  {
    Serial.println("Failed to initialize ADS1.");
  }
  else
  {
    ads1.setGain(GAIN_FOUR);  // ±1.024V — covers battery divider output (max ~1.017V at 60V)
    Serial.println("ADS1 initialized with GAIN_FOUR");
  }

  for (int i = 0; i < arraySize; i++)
  {
    readBattery();
  }
}
