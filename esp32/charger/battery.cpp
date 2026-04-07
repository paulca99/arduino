#include "Arduino.h"
#include "espEmonLib.h"
#include "pwmFunctions.h"
#include "battery.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads1; /* Use this for the 16-bit version */
Adafruit_ADS1115 ads2;
extern boolean afterburnerOn;
/*
OK , we can read each PSU's output using a relay to connect each to
a potential divider 8 K  / 470R  uses 0.4W of power

need 5 output pins to turn on relays
reuse batteries analog read pin
select 1 = 12v
select2 = 24v
select3 = 36v
select4 = 48v
select 5 = 60v

to work out each psu voltage

1 = 1
2 = 2-1
3 = 3-2-1   etc

leave battery pin alone for total as need to read that constantly
instead we'll re-use the gti pin as the voltSamplePin and read the PSU voltages
less frequently .... say once per minute... then we could have a balance function that
subtracts 1 from the highest voltage found so that after a while they all read the same voltage.

*/
// for some bizarre reason the first in the list doesn't operate the relay
/*int selectorPins[] = {13, 12, 14, 26, 25};*/
double psuVoltages[] = {
    12.1,
    12.2,
    12.3,
    12.4,
    12.5};
int adc_channels[] = {
    3, 2, 1, 0, 0};

/*1 5.04   8k  multiplier = 2.6002
5.03     21.6k  multiplier = 5.29334
5.05      32.6k  multiplier = 7.4534
5.11       47.4k multiplier = 10.2739726
4.99       59.5k   multiplier = 12.917115*/
double voltMultiplier[] = {
    2.6002, 5.29334, 7.4534, 10.2739726, 12.807115};
bool adc_enabled = true;
int batteryPin = 39;
int voltSamplePin = 35;
float batteryTotalVoltage = 0.0;
float history[60];
int arraySize = 60;
int historyPointer = 0;

int findHighestPSU()
{
  int max=4;
  if(afterburnerOn)
    max=5;
  double highestVolts = 0;
  int pos = 0;

  for (int i = 0; i < max; i++)
  {
    if (psuVoltages[i] > highestVolts)
    {
      highestVolts = psuVoltages[i];
      pos = i;
    }
  }
  return pos;
}

int findLowestPSU()
{
  int max=4;
  if(afterburnerOn)
    max=5;
  double lowestVolts = 99;
  int pos = 0;

  for (int i = 0; i < max; i++)
  {
    if (psuVoltages[i] < lowestVolts)
    {
      lowestVolts = psuVoltages[i];
      pos = i;
    }
  }
  return pos;
}

float findVoltageSpan()
{
  int lowest=findLowestPSU();
  int highest=findHighestPSU();
  return psuVoltages[highest] - psuVoltages[lowest];

}

double readPSUValue(int selection)
{
  int16_t adc;
  float volts, scaledVolts = 0.0;
  if (adc_enabled)
  {
    //Serial.println("Reading Voltage for PSU " + (String)selection);
    if (selection == 4)
    {
      adc = ads2.readADC_SingleEnded(adc_channels[selection]);
      volts = ads2.computeVolts(adc);
    }
    else
    {
      adc = ads1.readADC_SingleEnded(adc_channels[selection]);
      volts = ads1.computeVolts(adc);
    }

    scaledVolts = volts * voltMultiplier[selection];

    // now subtract the other voltages to get the one we have selected
    //Serial.println("Voltage= " + (String)scaledVolts);
    for (int i = selection; i > 0; i--)
    {
      scaledVolts = scaledVolts - psuVoltages[i - 1];
    }
    // store it in the array
    psuVoltages[selection] = scaledVolts;
  }
  return scaledVolts;
}

double populateVoltages() // returns the total
{
  double retval = 0;
  if (adc_enabled)
  {
    for (int i = 0; i < 5; i++)
    {

      //readPSUValue(i);
      //retval = retval + psuVoltages[i];
    }
  }
  else
  {
    Serial.println("ADC disabled, Check wires 21 & 22, disconnect USB, turn AC off and on, ADC needs power from 5V supply" );
  }
  return retval;
}

float checkBattery() // intended to get a more accurate reading by stopping charger and gti then reading battery
{
  turnGTIOff();
  turnPowerOff();
  delay(8000);
  float accbatt = readBatteryOnce();
  Serial.println("Accurate Battery reading =:" + (String)accbatt);
  delay(1000);
  return accbatt;
}

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
  // Serial.println("VBatt:"+(String)batt);
  return batt;
}

float readBatteryOnce()
{
  int adcValue = 0;
  delay(50);
  adcValue += analogRead(batteryPin);
  // Serial.println("Batt pin adv val =:"+(String)adcValue);

  // 4096=3.3V
  //  39.5V  is really 44.5
  //  63.5   is really 61.6
  /*
  y = (x - 39.5) * (61.6 - 44.5) / (63.5 - 39.5) + 44.5
  */
  //
  float voltageOnPin = (adcValue * 3.3) / 4095;
  float rV = voltageOnPin * 22.85;

  batteryTotalVoltage = (rV - 39.5) * (61.6 - 44.5) / (63.5 - 39.5) + 44.5;

  return batteryTotalVoltage;
}
void setupBattery()
{
  pinMode(batteryPin, INPUT);
  pinMode(voltSamplePin, INPUT);
 /* adc_enabled = ads1.begin(0x48);
  if (!adc_enabled)
  {
    Serial.println("Failed to initialize ADS1.");
  }
  if (adc_enabled)
  {
    adc_enabled = ads2.begin(0x49);
    if (!adc_enabled)
    {
      Serial.println("Failed to initialize ADS2.");
    }
  }*/
  for (int i = 0; i < arraySize; i++)
  {
    readBattery();
  }
}
