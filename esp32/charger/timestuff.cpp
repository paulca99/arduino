#include "Arduino.h"
#include "timestuff.h"
#include "time.h"
#include "pwmFunctions.h"

extern bool GTIenabled;
struct tm timeinfo;
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

int GTI_on_time = 6;
int GTI_off_time = 22;
void timeSetup() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void timeLoop() {

  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  } else {
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    if ((timeinfo.tm_hour >= GTI_on_time) && (timeinfo.tm_hour < GTI_off_time) && GTIenabled == false) {
      GTIenabled = true;
      turnPowerOn();
      turnPowerOff();
    } else {
      if (GTIenabled == true) {
        GTIenabled = false;
        turnGTIOff();
      }
    }
  }
}