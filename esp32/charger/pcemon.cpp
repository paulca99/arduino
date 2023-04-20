#include "Arduino.h"
#include "EmonLib.h"


const int freq = 5000;
const int resolution = 8; //2^8 = 256

int gridVoltagePin=34;
int gridCurrentPin=32;
int chargerCurrentPin=33;

EnergyMonitor grid;
EnergyMonitor charger;

void setupEmon()
{
  grid.voltage(gridVoltagePin, 160, 0.2);  // Voltage: input pin, calibration, phase_shift
  grid.current(gridCurrentPin, 12); 
  charger.voltage(gridVoltagePin, 140, 0.2);  // Voltage: input pin, calibration, phase_shift
  charger.current(chargerCurrentPin, 14); 
}

void readGrid()
{
  grid.calcVI(40,2000);  // Calculate all. No.of half wavelengths (crossings), time-out
  Serial.println(grid.realPower);
   Serial.println(grid.Vrms);
}

void readCharger()
{
  grid.calcVI(20,2000);  // Calculate all. No.of half wavelengths (crossings), time-out
}
