#include "pcwifi.h"
#include "pcemon.h"
#include "pwmFunctions.h"

void setup(){
  pwmSetup();
  wifiSetup();
}
 
void loop(){
  readGrid();
  rampUp();
  rampDown();
  readCharger();
}