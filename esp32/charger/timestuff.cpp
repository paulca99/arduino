#include "Arduino.h"
#include "timestuff.h"
#include "time.h"
#include "pwmFunctions.h"
#include "pcwifi.h"

extern bool GTIenabled;
struct tm timeinfo;
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

int GTI_on_time = 5;
int GTI_off_time = 22;
void timeSetup() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void timeLoop() {

  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    connectToWifi();
    return;
  } else {
   // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    if ((timeinfo.tm_hour >= GTI_on_time) && (timeinfo.tm_hour < GTI_off_time)) { // Within GTI hours
      if(GTIenabled==false)
      { 
       GTIenabled = true; // should happen ONCE at 5am
       turnGTIOn();
      }
    } else { // Out of GTI hours

        GTIenabled = false;
        turnGTIOff();
      
    }
  }
}