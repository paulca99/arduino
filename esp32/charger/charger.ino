#include "pcwifi.h"
#include "pcemon.h"
#include "pwmFunctions.h"
#include "battery.h"

  int loopcount=0;

void setup(){

  wifiSetup();
  setupEmon();
  setupBattery();
  pwmSetup();
}
 
void loop(){
  autoLoop();
  wifiLoop();
 // rampUp();
 // rampDown();
 // readCharger();

}


void autoLoop() {
 // wait till stable before adjusting anything

  readGrid();
  readCharger();
  if(loopcount>15)
  {
  adjustCharger();
  Serial.println("VBBatt="+(String)readBattery());
  loopcount=16;
  }
  loopcount++;

}

