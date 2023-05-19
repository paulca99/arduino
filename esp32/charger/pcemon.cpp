#include "Arduino.h"
#include "espEmonLib.h"
#include "pwmFunctions.h"


const int freq = 5000;
const int resolution = 8; //2^8 = 256

int gridVoltagePin=34;
int gridCurrentPin=32;
int chargerCurrentPin=33;
int gtiCurrentPin=36;

float chargerPower=0.0;
float gtiPower=0.0;

float gridVoltageCalibration=740;
float gridPhaseOffset=0.2;
float gridCurrentCalibration=56;

float chargerVoltageCalibration=740;
float chargerPhaseOffset=0.2;
float chargerCurrentCalibration=24;

float gtiVoltageCalibration=740;
float gtiPhaseOffset=0.2;
float gtiCurrentCalibration=36;

EnergyMonitor grid;
EnergyMonitor charger;
EnergyMonitor gti;

extern int range;
extern boolean powerOn;
void setupEmon()
{
  grid.voltage(gridVoltagePin, gridVoltageCalibration, gridPhaseOffset);  // Voltage: input pin, calibration, phase_shift
  grid.current(gridCurrentPin, gridCurrentCalibration); 
  charger.voltage(gridVoltagePin, chargerVoltageCalibration, chargerPhaseOffset);  // Voltage: input pin, calibration, phase_shift
  charger.current(chargerCurrentPin, chargerCurrentCalibration); 
    
  gti.voltage(gridVoltagePin, gtiVoltageCalibration, gtiPhaseOffset);  // Voltage: input pin, calibration, phase_shift
  gti.current(gtiCurrentPin, gtiCurrentCalibration); 
  gti.calcVI(40,2000);
}

void readGrid()
{
  grid.calcVI(40,1000);  // Calculate all. No.of half wavelengths (crossings), time-out
  grid.realPower = grid.realPower-90;
  Serial.println("gridp:"+(String)grid.realPower);
  Serial.println("grridv:"+(String)grid.Vrms);
  Serial.println("grridPF:"+(String)grid.powerFactor);
}

float readCharger()
{
  if(!powerOn) //power is off
  {
     chargerPower=0;
    return chargerPower;
  }

  charger.calcIrms(300);
  float current=charger.Irms;
  //Serial.println(""+(String)current);
  current = current -0.2 ; // offset is out..
  if(current < 0){
    current=0;
  }
  chargerPower = current * grid.Vrms;
   //Serial.println("chargerp1:"+(String)power);
  /*if(power*power < 1000)
  {
    float gap = 1000 - power;
    float mult = gap / 1000; // so 0.7
    mult=1-mult;
    power = power*mult;
  }*/




   // Serial.println("chargerp:"+(String)power);
  //      Serial.println("current:"+(String)charger.Irms);
   // Serial.println("pfactor:"+(String)charger.powerFactor);
  // Serial.println("chargerRealp="+(String)(charger.realPower));
   return chargerPower;
}


float readGti()
{
  if(powerOn)
  {
     gtiPower=0;
    return gtiPower;
  }
  gti.calcIrms(200);
  float current=gti.Irms;
  current = current -0.2 ; // offset is out..
  if(current < 0){
    current=0;
  }
  gtiPower = current * grid.Vrms;
  
   return gtiPower;
}