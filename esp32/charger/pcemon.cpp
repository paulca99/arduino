#include "Arduino.h"
#include "espEmonLib.h"
#include "pwmFunctions.h"

extern int lowerChargerLimit;
const int freq = 5000;
const int resolution = 8; // 2^8 = 256

int gridVoltagePin = 34;
int gridCurrentPin = 32;
int chargerCurrentPin = 33;
int gtiCurrentPin = 36;

float chargerPower = 0.0;
float gtiPower = 0.0;

float gridVoltageCalibration = 280;
float gridPhaseOffset = 1.4;
float gridCurrentCalibration = 56;

float chargerVoltageCalibration = 1500;
float chargerPhaseOffset = 1.4;
// float chargerCurrentCalibration=24;
float chargerCurrentCalibration = 32;

float gtiVoltageCalibration = 740;
float gtiPhaseOffset = 1.4;
// float gtiCurrentCalibration=36;
float gtiCurrentCalibration = 80;

EnergyMonitor grid;
EnergyMonitor charger;
EnergyMonitor gti;

extern int range;
extern boolean powerOn;
void setupEmon()
{
  grid.voltage(gridVoltagePin, gridVoltageCalibration, gridPhaseOffset); // Voltage: input pin, calibration, phase_shift
  grid.current(gridCurrentPin, gridCurrentCalibration);
  charger.voltage(gridVoltagePin, chargerVoltageCalibration, chargerPhaseOffset); // Voltage: input pin, calibration, phase_shift
  charger.current(chargerCurrentPin, chargerCurrentCalibration);

  gti.voltage(gridVoltagePin, gtiVoltageCalibration, gtiPhaseOffset); // Voltage: input pin, calibration, phase_shift
  gti.current(gtiCurrentPin, gtiCurrentCalibration);
}

void readGrid()
{
  grid.calcVI(40, 1000); // Calculate all. No.of half wavelengths (crossings), time-out
  grid.realPower = grid.realPower - 90;
  /*Serial.println("gridp:"+(String)grid.realPower);
   Serial.println("grridv:"+(String)grid.Vrms);*/
  // Serial.println("grridPF:"+(String)grid.powerFactor);
}

float readCharger()
{
  if (!powerOn) // power is off
  {
    chargerPower = 0;
    return chargerPower;
  }

  charger.calcIrms(500);
  float current = charger.Irms;
  //  Serial.println("current "+(String)current);
  current = current - 0.2; // offset is out..
  if (current < 0)
  {
    current = 0;
  }
  chargerPower = current * grid.Vrms;
  return chargerPower;
}

float readGti()
{
  if (powerOn)
  {
    gtiPower = 0;
    return gtiPower;
  }
  gti.calcIrms(500);
  float current = gti.Irms;
  current = current - 0.2; // offset is out..
  if (current < 0)
  {
    current = 0;
  }
  gtiPower = current * grid.Vrms;
  if (gtiPower > 200)
  {
    lowerChargerLimit = (gtiPower / 2) * -1;
  }
  else
    lowerChargerLimit = -120;

  return gtiPower;
  // need to average over 20 like the battery average
  // then need to set lower limit to half of GTI power to stop the charger
  // coming on when the GTI's are at high power.
}