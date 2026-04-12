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

  if (useADSForBattery)
  {
    int16_t adc = ads1.readADC_SingleEnded(adsBatteryChannel);
    float voltageOnPin = ads1.computeVolts(adc);
    // R1 = 51kΩ + 6.8kΩ = 57800Ω, R2 = 1000Ω
    // multiply by (R1+R2)/R2 = 58800/1000
    rV = voltageOnPin * (58800.0 / 1000.0);
    Serial.println("ADS batt pin V =:" + (String)voltageOnPin + " rV=" + (String)rV);
  }
  else
  {
    int adcValue = 0;
    delay(50);
    adcValue += analogRead(batteryPin);
    float voltageOnPin = (adcValue * 3.3) / 4095;
    rV = voltageOnPin * 22.85;
    Serial.println("PIN39 batt pin V =:" + (String)voltageOnPin + " rV=" + (String)rV);
  }

  // Calibration: re-verify these constants once switched to ADS path
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
