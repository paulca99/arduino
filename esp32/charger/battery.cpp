#include "Arduino.h"
#include "espEmonLib.h"


int batteryPin=39;
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
  }


  //4096=3.3V
  float voltageOnPin = (adcValue * 3.3) / 40960;
  batteryTotalVoltage=voltageOnPin*22.3;
  return batteryTotalVoltage;
}

