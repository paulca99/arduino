#include "pcwifi.h"
#include "pcemon.h"
#include "pwmFunctions.h"
#include "battery.h"
#include "timestuff.h"
  int loopcount=0;
  hw_timer_t * timer = NULL;
void ARDUINO_ISR_ATTR onTimer(){
    //ESP.restart();
    Serial.println("Timer fired");
}

void setup(){
  timer = timerBegin(0, 320, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3600000000, true);//4hour
  timerAlarmEnable(timer);
  wifiSetup();
  timeSetup();
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
    timeLoop();
  if(loopcount>5)
  {
  adjustCharger();
  //Serial.println("VBBatt="+(String)readBattery());
  loopcount=6;
  }
  loopcount++;

}

