#include "pcwifi.h"
#include "pcemon.h"
#include "pwmFunctions.h"

void setup(){
  pwmSetup();
  wifiSetup();
  setupEmon();
}
 
void loop(){
  autoLoop();
 // rampUp();
 // rampDown();
 // readCharger();

}


void autoLoop() {
  readGrid();
  readCharger();
  adjustCharger();
}

