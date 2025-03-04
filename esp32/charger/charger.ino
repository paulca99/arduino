#include "pcwifi.h"
#include "pcemon.h"
#include "pwmFunctions.h"
#include "battery.h"
#include "timestuff.h"
int loopcount = 0;
hw_timer_t *timer = NULL;
void ARDUINO_ISR_ATTR onTimer()
{
  // ESP.restart();
  Serial.println("Timer fired");
}

void setup()
{
    
  Serial.begin(115200);
    while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  delay(2000);
  Serial.println("setupStart");
  timer = timerBegin(0, 320, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3600000000, true); // 4hour
  timerAlarmEnable(timer);
       Serial.println("wifisetupStart");
  wifiSetup();
     Serial.println("timesetupStart");
  timeSetup();
  setupEmon();
  setupBattery();
  pwmSetup();
}

void loop()
{
  autoLoop();
  // testLoop();
}

void testLoop()
{
  rampDown();
  delay(5000);
  // rampUp();
  // wifiLoop();
}
void autoLoop()
{
  // wait till stable before adjusting anything

  //Serial.println("FreeHeap:" + (String)ESP.getFreeHeap());
  readCharger();
  readGrid();
  readBattery();
  wifiLoop();
  timeLoop();
  if (loopcount > 15)
  {
    readGti();
    adjustCharger();
    // Serial.println("VBBatt="+(String)readBattery());
    loopcount = 16;
  }
  loopcount++;
}
