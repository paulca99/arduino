#include "Arduino.h"
#include "timestuff.h"
#include "time.h"
#include "pwmFunctions.h"
#include "pcwifi.h"

extern bool GTIenabled;
extern int upperChargerLimit;
extern int lowerChargerLimit;
struct tm timeinfo;
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

int GTI_on_time = 5;
int GTI_off_time = 23;
int startChargingTime = 6;
int stopChargingTime = 18;

// ── Eon Next Drive cheap-rate ─────────────────────────────────────
static const int CHEAP_RATE_START = 0;   // 00:00
static const int CHEAP_RATE_END   = 6;   // 06:00
bool inCheapRateWindow = false;

void timeSetup()
{
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void timeLoop()
{

  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    connectToWifi();
    return;
  }
  else
  {

    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    if ((timeinfo.tm_hour >= GTI_on_time) && (timeinfo.tm_hour < GTI_off_time))
    { // Within GTI hours
      if (GTIenabled == false)
      {
        GTIenabled = true; // should happen ONCE at 5am
        turnGTIOn();
      }
    }
    else
    { // Out of GTI hours

     // GTIenabled = false;
     // turnGTIOff();
    }

    // ── Cheap-rate window management ─────────────────────────────────
    bool nowCheap = (timeinfo.tm_hour >= CHEAP_RATE_START && timeinfo.tm_hour < CHEAP_RATE_END);

    if (nowCheap && !inCheapRateWindow) {
      // Entering cheap-rate window
      inCheapRateWindow = true;
      lowerChargerLimit = -9999;
      GTIenabled = false;
      turnGTIOff();
      Serial.println("[CheapRate] STARTED: lowerChargerLimit=-9999, GTI OFF");
    }
    else if (!nowCheap && inCheapRateWindow) {
      // Leaving cheap-rate window
      inCheapRateWindow = false;
      lowerChargerLimit = 0;
      GTIenabled = true;
      Serial.println("[CheapRate] ENDED: lowerChargerLimit=0, GTI re-enabled");
      // The GTI_on_time block above will call turnGTIOn() on the next timeLoop() call if appropriate
    }
  }
}

/*switch PV off rules 
if  2>month>10  
    pv off time =18:40 minus 10 mins per month away from june
else
    pv off time = sunset

or

2 hours before sunset in June
1 hour before sunset in march   nope TODO*/
