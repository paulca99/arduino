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
  turnPowerOff();
}

void loop(){
  autoLoop();
  //testLoop();

}

void testLoop()
{
   rampDown();
   delay(5000);
   //rampUp();
  // wifiLoop();
}
void autoLoop() {
 // wait till stable before adjusting anything


  readCharger();
  readGrid();
  readGti();
  readBattery();
    wifiLoop();
  if(loopcount>15)
  {
  adjustCharger();
  //Serial.println("VBBatt="+(String)readBattery());
  loopcount=16;
  }
  loopcount++;

}

