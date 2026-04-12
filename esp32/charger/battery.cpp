#include "Arduino.h"
#include "espEmonLib.h"
#include "pwmFunctions.h"
#include "battery.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads1;  // Single ADS1115 at 0x48, 3.3V VDD

extern boolean afterburnerOn;

int batteryPin = 39;
float batteryTotalVoltage = 0.0;
float history[60];
int arraySize = 60;
int historyPointer = 0;

bool useADSForBattery = false;  // false = pin 39 (original), true = ADS1115 ads1 channel 0
int adsBatteryChannel = 0;      // ADS1115 channel wired to battery divider (51k+6.8k / 1k)

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
  float rV;

  // Always read and print pin 39
  int adcValue = 0;
  delay(50);
  adcValue += analogRead(batteryPin);
  float voltageOnPin39 = (adcValue * 3.3) / 4095;
  float rV39 = voltageOnPin39 * 22.85;
  float batt39 = (rV39 - 45.8) * 1.3663 + 45.0;
  Serial.println("PIN39 V=" + (String)voltageOnPin39 + " rV=" + (String)rV39 + " batt=" + (String)batt39);

  // Always read and print ADS
  int16_t adc = ads1.readADC_SingleEnded(adsBatteryChannel);
  float voltageOnPinADS = ads1.computeVolts(adc);
  // Multiplier calibrated: real=47.8V, rV=46.78V => 58.8 * (47.8/46.78) = 60.08
  float rVADS = voltageOnPinADS * (60080.0 / 1000.0);
  Serial.println("ADS   V=" + (String)voltageOnPinADS + " rV=" + (String)rVADS);

  // Use whichever source is selected by the flag
  if (useADSForBattery)
    rV = rVADS;
  else
    rV = rV39;

  // ADS1115 is linear — rV is already accurate, no correction needed
  // Pin 39 requires correction for ESP32 ADC non-linearity
  if (useADSForBattery)
    batteryTotalVoltage = rV;
  else
    batteryTotalVoltage = (rV - 45.8) * 1.3663 + 45.0;

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
  pinMode(batteryPin, INPUT);

  bool ads1_ok = ads1.begin(0x48);
  if (!ads1_ok)
  {
    Serial.println("Failed to initialize ADS1. Falling back to pin 39.");
    useADSForBattery = false;
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
