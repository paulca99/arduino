#include "pwmFunctions.h"
#include "pc-wifi.h"
#include "EmonLib.h"
EnergyMonitor grid;

// setting PWM properties


 
void setup()
{
  wifiSetup();
  pwmSetup();
}
 
void loop(){
  rampUp();
  rampDown();
}