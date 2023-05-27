#include "pcwifi.h"
#include "pcemon.h"
#include "pwmFunctions.h"
#include "battery.h"

  int loopcount=0;
  hw_timer_t * timer = NULL;
void ARDUINO_ISR_ATTR onTimer(){
    ESP.restart();
}

void setup(){
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3600000000, true);//1hour
  timerAlarmEnable(timer);
  wifiSetup();
  setupEmon();
  setupBattery();
  pwmSetup();
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

 	
  Serial.println("FreeHeap:"+(String)ESP.getFreeHeap());
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

