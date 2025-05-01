#include "Arduino.h"
#include "espEmonLib.h"
#include "pcemon.h"
#include "battery.h"
#include "pwmFunctions.h"
/*
Thinking about having PSU 5 as an afterburner
This will make the morning startup easier as we'll only be using 4 PSU
Morning startup gets harder with more batteries in parallel.
46.6 = emptyvoltage
48 = afterburner engaged voltage
to avoid the afterburner flicking on and off we'd need
AFTERBURNER_ON_VOLTAGE = 48
AFTERBURNER_OFF_VOLTAGE = 47
even then, may flicker quite a bit
consider making battery moving average larger num samples.
47V appears to be the max voltage we can get from 4 PSU ...which seems low.

Maybe if grid < -300 and afterburner off, engage afterburner.

*/
boolean VOLTAGE_HIGH = false;
boolean powerOn = false;
boolean afterburnerOn = false;
int gtiPin = 23;
int upperChargerLimit = 100; // point to turn charger off
int lowerChargerLimit = 0;   // point to turn charger on
float voltageLimit = 57.2;
int chargerPLimit = 3300; // max watts into charger ( prob 2000 into battery)
bool GTIenabled = true;
const int freq = 200;
int SOC = 90; // TODO neds calculating

const float current_limit = 14;
const int resolution = 8; // 2^8 = 256
extern EnergyMonitor grid;
extern EnergyMonitor charger;
extern float chargerPower;
int powerPin = 21;
int afterBurnerPin = 27;
int psu_voltage_pins[] = {19, 18, 5, 22, 16};
int pwmChannels[] = {0, 1, 2, 3, 4};
const int delayIn = 1;

int range = (pow(2, resolution)) - 1;
int psu_resistance_values[] = {range, range, range, range, range};
int psu_count = sizeof psu_resistance_values / sizeof psu_resistance_values[0];
int psu_pointer = 0;

int getTotalResistance()
{
  int retval;
  for (int i = 0; i < psu_count; i++)
  {
    retval += psu_resistance_values[i];
  }
  return retval;
}
bool isAtMinPower()
{
  for (int i = 0; i < psu_count; i++)
  {
    if (psu_resistance_values[i] < range)
    {
      return false;
    }
  }
  // Serial.println("IsAtMinPower");
  if (afterburnerOn)
  {
    turnAfterburnerOff();
    return false;
  }
  return true;
}

bool voltageLimitReached2()
{

  float presentVoltage = readBattery();
  if (VOLTAGE_HIGH && (presentVoltage < (voltageLimit - 1)))
  {
    VOLTAGE_HIGH = false;
  }

  if (presentVoltage >= voltageLimit || VOLTAGE_HIGH)
  {
    Serial.println("VOLTAGE LIMIT TRIPPED" + String(presentVoltage));
    VOLTAGE_HIGH = true;
    return true;
  }
  // Serial.println("VOLTAGE OK");
  return false;
}
void writePowerValuesToPSUs()
{
  for (int i = 0; i < psu_count; i++)
  {
    // Serial.println("Writing " +(String)psu_resistance_values[i] + " to psu " + (String)i);
    ledcWrite(pwmChannels[i], psu_resistance_values[i]);
  }
}
void turnGTIOn()
{
  VOLTAGE_HIGH = false;
  if (GTIenabled)
  {
    digitalWrite(gtiPin, HIGH);
  }
}
void turnGTIOff()
{
  digitalWrite(gtiPin, LOW);
}

void turnPowerOff()
{
  turnAfterburnerOff();
  digitalWrite(powerPin, HIGH);
  powerOn = false;
  turnGTIOn();
  upperChargerLimit = 0;
  lowerChargerLimit = -100;
}
void turnPowerOn()
{
  digitalWrite(powerPin, LOW);
  powerOn = true;
  turnGTIOff();
  upperChargerLimit = 100;
  lowerChargerLimit = 0;
}
void turnAfterburnerOff()
{

  digitalWrite(afterBurnerPin, HIGH);
  afterburnerOn = false;
  Serial.println("Afterburner OFF");
}
void turnAfterburnerOn()
{
  digitalWrite(afterBurnerPin, LOW);
  afterburnerOn = true;
    Serial.println("Afterburner ON");
}
void pwmSetup()
{
  // configure LED PWM functionalitites
  pinMode(powerPin, OUTPUT);
  pinMode(afterBurnerPin, OUTPUT);
  pinMode(gtiPin, OUTPUT);
  turnPowerOff();
  for (int i = 0; i < psu_count; i++)
  {

    pinMode(psu_voltage_pins[i], OUTPUT);
    ledcSetup(pwmChannels[i], freq, resolution);
    ledcAttachPin(psu_voltage_pins[i], pwmChannels[i]);
    ledcWrite(pwmChannels[i], psu_resistance_values[i]);
  }
  writePowerValuesToPSUs();
}
void setMinPower()
{
  for (int i = 0; i < psu_count; i++)
  {
    psu_resistance_values[i] = range - 1;
  }
  psu_pointer = 0;
}

void incrementPower(boolean write, int stepAmount)
{ // means reducinng resistance
  // Serial.println("IncrementPower");
  // 255,255,255,255,255   ->  254,255,255,255,255  ->  254,254,255,255,255

  // Serial.println("decrementing values "+(String)psu_pointer);
  float vbatt = readBattery();
  if (vbatt < voltageLimit)
  {
    for (int i = 0; i < stepAmount; i++)
    {
      psu_resistance_values[psu_pointer] = psu_resistance_values[psu_pointer] - 1;
      if (psu_resistance_values[psu_pointer] < 0)
      {
        psu_resistance_values[psu_pointer] = 0;
      }
      psu_pointer++;
      if (psu_pointer == psu_count)
      {
        psu_pointer = 0;
      }
    }
    if (write)
    {
      writePowerValuesToPSUs();
    }
  }
}

void decrementPower(boolean write, int stepAmount)
{ // means increasing resistance
  // Serial.println("DecrementPower");
  // 00000 , 00001 , 00011 , 00111 , 01111 , 11111 , 11112
  if (!isAtMinPower())
  {
    for (int i = 0; i < stepAmount; i++)
    {
      psu_pointer--;
      if (psu_pointer == -1)
      {
        psu_pointer = psu_count - 1;
      }
      psu_resistance_values[psu_pointer] = psu_resistance_values[psu_pointer] + 1;
      if (psu_resistance_values[psu_pointer] > range)
      {
        psu_resistance_values[psu_pointer] = range;
      }
    }
    if (write)
    {
      writePowerValuesToPSUs();
    }
  }
}

void setupTest()
{
  turnPowerOn();
  turnAfterburnerOn();
}
void goBottom()
{

  for (int i = 0; i < psu_count; i++)
  {
    psu_resistance_values[i] = 0;
  }
  writePowerValuesToPSUs();
}
void goMid()
{
  for (int i = 0; i < psu_count; i++)
  {
    psu_resistance_values[i] = 128;
  }
  writePowerValuesToPSUs();
}
void goTop()
{
  for (int i = 0; i < psu_count; i++)
  {
    psu_resistance_values[i] = 255;
  }
  writePowerValuesToPSUs();
}
void rampUp()
{

  for (int dutyCycle = 0; dutyCycle <= (range * 5); dutyCycle++)
  {
    // changing the LED brightness with PWM
    incrementPower(true, 1);
    delay(1);
  }
}
void rampDown()
{
 
  for (int dutyCycle = (range * 5); dutyCycle >= 0; dutyCycle--)
  {
    // changing the LED brightness with PWM
    decrementPower(true, 1);
    delay(1);
  }
}

void increaseChargerPower(float startingChargerPower)
{
  // here gridp is lower than the lowerChargerLimit.
  float vbattery = readBattery();
  float fakeGridp = grid.realPower + 10000; // cancel out -ve values
  float fakeLowerLimit = lowerChargerLimit + 10000;
  float increaseAmount = fakeLowerLimit - fakeGridp; // will always be +ve
  Serial.println("startingCpower=" + (String)startingChargerPower + (String)lowerChargerLimit + " - " + (String)grid.realPower + " = " + (String)increaseAmount);
  if(afterburnerOn)
    increaseAmount = increaseAmount * 0.7; // don't overshoot
  if (vbattery > 56)
  {
    if (increaseAmount > 50)
    {
      increaseAmount = 50;
    }
  }

  int stepAmount = (increaseAmount / 30) + 1; // react faster to large change
  float target = startingChargerPower + increaseAmount;

  if (target > chargerPLimit)
    target = chargerPLimit; //  pin it at 3000

  Serial.println("increase amount = " + (String)increaseAmount);
  float chargerPower = startingChargerPower;
  // float gtiPower = readGti();
  int attemptCount = 0;
  while (chargerPower < target && (charger.Irms < current_limit) && !voltageLimitReached2() && !isAtMaxPower() && chargerPower < chargerPLimit && vbattery < voltageLimit && attemptCount < 50)
  {
    incrementPower(true, stepAmount);
    chargerPower = readCharger();
    vbattery = readBattery();
    attemptCount++;
  }
}

void reduceChargerPower(float startingChargerPower)
{
  float vbattery = readBattery();
  // for starters here gridp is higher than the upper limit
  decrementPower(true, 1);                  // always reduce by 1 in case vbatt is too high
  float fakeGridp = grid.realPower + 10000; // cancel out -ve values
  float fakeUpperLimit = upperChargerLimit + 10000;
  float reductionAmount = fakeGridp - fakeUpperLimit; // will always be +ve unless voltage too high may be -ve
  reductionAmount = reductionAmount * 0.7;            // don't overshoot
  if (vbattery > 55)
  {
    if (reductionAmount > 200)
    {
      reductionAmount = 200;
    }
  }
  int stepAmount = (reductionAmount / 30) + 1; // react faster to large change
  float target = startingChargerPower - reductionAmount;
  if (reductionAmount > 0) // don't bother if its -ve
  {
    float chargerPower = startingChargerPower;
    Serial.println("decrease amount = " + (String)reductionAmount);
    // Serial.println("reduce target:" + String(target));
    while (chargerPower > target && !isAtMinPower())
    {
      chargerPower = readCharger();
      decrementPower(true, stepAmount);
      // delay(5);  // Damping coefficient, can be reduced if we don't overshoot too badly
    }
  }
}

void adjustCharger()
{
  float vbatt = readBattery();
  bool vLimitHit = voltageLimitReached2();
  if (vbatt > voltageLimit)
  {
    decrementPower(true, 30);
  }

  //  if (vbatt )
  //  {
  //    maintainVoltage();
  //  }
  //  else
  //  {
  float presentChargerPower = readCharger();
  if (presentChargerPower > chargerPLimit)
  {
    Serial.println(" " + (String)presentChargerPower);
  }
  if (grid.realPower > upperChargerLimit || vbatt > voltageLimit || presentChargerPower > chargerPLimit)
  {
    if (isAtMinPower() && powerOn)
    {
      Serial.println("turning off , setting pin HIGH");
      turnPowerOff(); // turn off
    }
    else
    {
      reduceChargerPower(presentChargerPower);
    }
  }
  else if (SOC < 99 && grid.realPower < lowerChargerLimit)
  {
    if (!powerOn)
    {
      Serial.println("turning on , setting pin LOW");
      turnPowerOn(); // turn on
                     // allow spike
      readGrid();
      for (int i = 0; i < 10; i++) // reading the charger a few times stops the power on spike
      {
        readCharger();
      }
      presentChargerPower = readCharger();
    }
    increaseChargerPower(presentChargerPower);
  }
}

bool isAtMaxPower()
{
  for (int i = 0; i < psu_count; i++)
  {
    if (psu_resistance_values[i] > 0)
    {
      return false;
    }
  }
  Serial.println("MAX POWER");

  for (int i = 0; i < psu_count; i++)
  {
    psu_resistance_values[i] = 254;
  }
  writePowerValuesToPSUs();
  if (afterburnerOn)
  {
    turnPowerOff();
    delay(5000);
    turnPowerOn();
  }
  else
  {
    
    turnAfterburnerOn();
    delay(3000);
    return false;
  }
  return true;
}