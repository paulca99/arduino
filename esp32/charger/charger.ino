#include "pcwifi.h"
#include "pcemon.h"
#include "pwmFunctions.h"
#include "battery.h"
#include "timestuff.h"
extern int psu_resistance_values[];
int loopcount = 0;
int checkTime = 0;
// int timeToCheckBattery=0;

void setup()
{

  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(2000);
  Serial.println("setupStart");
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
  //testLoop();
}

void testLoop()
{
  setupTest();
  while (true)
  {
    Serial.println("TEST LOOP: ramping up");
    bool limitHit = rampUp();
    if (limitHit)
      Serial.println("TEST LOOP: limit hit, ramping down");
    else
      Serial.println("TEST LOOP: max power reached, ramping down");
    rampDown();
    delay(500); // brief pause at min power before next ramp
  }
}

void autoLoop()
{
  // wait till stable before adjusting anything

  // Serial.println("FreeHeap:" + (String)ESP.getFreeHeap());
  readCharger();
  readGrid();
  readBattery();
  wifiLoop();
  if (checkTime == 50)
  {
    timeLoop();
    checkTime = 0;
  }
  /*if(timeToCheckBattery == 16)
  //{
    checkBattery();
    timeToCheckBattery=0;
  }*/
  if (loopcount > 15)
  {
    readGti();
    adjustCharger();
    // Serial.println("VBBatt="+(String)readBattery());
    loopcount = 16;
  }
  loopcount++;
  checkTime++;
  // timeToCheckBattery++;
}
