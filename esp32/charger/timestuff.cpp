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
// Change these four values to test at a different time of day.
// e.g. to test 09:30-10:00: set START_HOUR=9, START_MIN=30,
//                                   END_HOUR=10, END_MIN=0
static const int CHEAP_RATE_START_HOUR = 0;   // real: 0  (midnight)
static const int CHEAP_RATE_START_MIN  = 0;   // real: 0
static const int CHEAP_RATE_END_HOUR   = 6;   // real: 6  (06:00)
static const int CHEAP_RATE_END_MIN    = 0;   // real: 0
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
    // Convert everything to minutes-since-midnight so start/end minutes
    // work correctly (e.g. 09:30 = 570, 10:00 = 600).
    int nowMins   = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    int startMins = CHEAP_RATE_START_HOUR * 60 + CHEAP_RATE_START_MIN;
    int endMins   = CHEAP_RATE_END_HOUR   * 60 + CHEAP_RATE_END_MIN;
    bool nowCheap = (nowMins >= startMins && nowMins < endMins);

    if (nowCheap && !inCheapRateWindow) {
      // Entering cheap-rate window.
      // lowerChargerLimit = -9999 : grid is always below this so charger always
      //   wants to increase power (until hardware limits fire).
      // upperChargerLimit = 9999  : grid can never exceed this so the
      //   reduce/stop branch in adjustCharger() only triggers via voltage or
      //   chargerPLimit — never from a momentary grid reading.
      //   This is critical: without it, after a voltage-limit shutdown the
      //   charger cannot restart because the idle-load grid reading (~-80W to
      //   +50W) is always inside the band [-9999, 0] left by turnPowerOff().
      inCheapRateWindow = true;
      lowerChargerLimit = -9999;
      upperChargerLimit = 9999;
      GTIenabled = false;
      turnGTIOff();
      Serial.printf("[CheapRate] STARTED at %02d:%02d: lower=-9999 upper=9999, GTI OFF\n",
                    timeinfo.tm_hour, timeinfo.tm_min);
    }
    else if (!nowCheap && inCheapRateWindow) {
      // Leaving cheap-rate window — restore normal daytime limits.
      inCheapRateWindow = false;
      lowerChargerLimit = 0;
      upperChargerLimit = 100;
      GTIenabled = true;
      Serial.printf("[CheapRate] ENDED at %02d:%02d: lower=0 upper=100, GTI re-enabled\n",
                    timeinfo.tm_hour, timeinfo.tm_min);
      // The GTI_on_time block above will call turnGTIOn() on the next
      // timeLoop() call if we are inside GTI hours (hour >= 5).
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
