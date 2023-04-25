#include "Arduino.h"
#include "espEmonLib.h"


const int freq = 5000;
const int resolution = 8; //2^8 = 256

int gridVoltagePin=34;
int gridCurrentPin=32;
int chargerCurrentPin=33;

EnergyMonitor grid;
EnergyMonitor charger;

void setupEmon()
{
  grid.voltage(gridVoltagePin, 690, 0.2);  // Voltage: input pin, calibration, phase_shift
  grid.current(gridCurrentPin, 50); 
  charger.voltage(gridVoltagePin, 690, 0.2);  // Voltage: input pin, calibration, phase_shift
  charger.current(chargerCurrentPin, 25); 
  charger.calcVI(40,2000);
}

void readGrid()
{
  grid.calcVI(20,1000);  // Calculate all. No.of half wavelengths (crossings), time-out
  Serial.println("gridp:"+(String)grid.realPower);
  Serial.println("grridv:"+(String)grid.Vrms);
  Serial.println("grridPF:"+(String)grid.powerFactor);
}

float readCharger()
{
  float power=0;

  charger.calcIrms(300);
  float current=charger.Irms;
  //Serial.println(""+(String)current);
  //current = current -2.5 ; // offset is out..
  power = current * 234;
   //Serial.println("chargerp1:"+(String)power);
  /*if(power*power < 1000)
  {
    float gap = 1000 - power;
    float mult = gap / 1000; // so 0.7
    mult=1-mult;
    power = power*mult;
  }*/




    Serial.println("chargerp:"+(String)power);
  //      Serial.println("current:"+(String)charger.Irms);
   // Serial.println("pfactor:"+(String)charger.powerFactor);
  // Serial.println("chargerRealp="+(String)(charger.realPower));
   return power;
}
