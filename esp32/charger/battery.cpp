#include "Arduino.h"
#include "EmonLib.h"


int batteryPin=22;
float batteryTotalVoltage=0.0;

void setupBattery()
{
  pinMode(batteryPin,INPUT);
}

float readBattery()
{
  int adcValue=0;
  for (int i=0; i<10; i++)
  {
      adcValue += analogRead(batteryPin);
      delay(6);
  }


  //4096=3.3V
  float voltageOnPin = (adcValue * 3.3) / 40960;
  batteryTotalVoltage=voltageOnPin*18;
  return batteryTotalVoltage;
}

