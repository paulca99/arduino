#include "Arduino.h"
#include "espEmonLib.h"
#include "pwmFunctions.h"
#include "battery.h"

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

int batteryPin = 39;
int voltSamplePin = 35;
float batteryTotalVoltage = 0.0;
float history[60];
int arraySize = 60;
int historyPointer = 0;

int findHighestPSU()
{
  double highestVolts = 0;
  int pos = 0;

  for (int i = 0; i < 5; i++)
  {
    if (psuVoltages[i] > highestVolts)
    {
      highestVolts = psuVoltages[i];
      pos = i;
    }
  }
  return pos;
}
// nicked from github.... makes non linear adc linear.
// 3.3 = 60, so to scale up....
// multiply by 18.1818181818

double ReadVoltage(int pin)
{
  double reading = 0.0;
  for (int i = 0; i < 5; i++) // average of 5 reads
  {
    reading += analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  }
  reading = reading / 5;

  if (reading < 1 || reading > 4095)
    return 0;
  return -0.000000000009824 * pow(reading, 3) + 0.000000016557283 * pow(reading, 2) + 0.000854596860691 * reading + 0.065440348345433;
  //  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
} // Added an improved polynomial, use either, comment out as required

void selectPSU(int selection) // pass in 0 to 4
{
  for (int i = 0; i < 5; i++)
  {
    //digitalWrite(selectorPins[i], HIGH); // disconnect all
  }
  //digitalWrite(selectorPins[selection], LOW); // select PSU we want
  //Serial.println("selecting psu " + (String)selection);
  //delay(25);
}

double readPSUValue(int selection)
{
  selectPSU(selection);
  delay(25);
  double adcVolts = ReadVoltage(voltSamplePin);
 // digitalWrite(selectorPins[selection], HIGH); // turn it back off
  delay(50);                                  // DO NOT want to short 2 together
  double voltage = adcVolts * 18.1818181818;

  // now subtract the other voltages to get the one we have selected
  Serial.println("Voltage= " + (String)voltage);
  for (int i = selection; i > 0; i--)
  {
    voltage = voltage - psuVoltages[i - 1];
  }
  // store it in the array
  psuVoltages[selection] = voltage;
  return voltage;
}

double populateVoltages() // returns the total
{
  double retval = 0;
  for (int i = 0; i < 5; i++)
  {

    readPSUValue(i);
    retval = retval + psuVoltages[i];
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

  for (int i = 0; i < arraySize; i++)
  {
    readBattery();
  }
}
