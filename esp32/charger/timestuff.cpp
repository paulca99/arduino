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

// Cheap charge overnight window (24-hour clock, hour and minute).
// Default: 00:30 – 05:00 (crosses midnight).
int cheapChargeStartHour   = 01;
int cheapChargeStartMinute = 00;
int cheapChargeStopHour    = 05;
int cheapChargeStopMinute  = 45;

// Returns true if the current NTP time falls inside the cheap-charge window.
// Handles windows that cross midnight (e.g. 00:30 → 05:00).
static bool isInsideCheapChargeWindow()
{
  int nowMinutes   = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int startMinutes = cheapChargeStartHour * 60 + cheapChargeStartMinute;
  int stopMinutes  = cheapChargeStopHour  * 60 + cheapChargeStopMinute;

  if (startMinutes < stopMinutes) {
    // Same-day window (e.g. 01:00 – 05:00)
    return (nowMinutes >= startMinutes && nowMinutes < stopMinutes);
  } else {
    // Overnight window crossing midnight (e.g. 23:30 – 05:00)
    return (nowMinutes >= startMinutes || nowMinutes < stopMinutes);
  }
}

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
    { // Within GTI hours — only enable if cheap charge is not active
      if (GTIenabled == false && !cheapChargeActive)
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

    // Cheap charge window management
    bool insideWindow = isInsideCheapChargeWindow();

    if (insideWindow)
    {
      if (!cheapChargeActive && !cheapChargeVoltageExit)
      {
        cheapChargeActive = true;
        Serial.println("CheapCharge: entered cheap charge window");
      }
    }
    else
    {
      // Outside the cheap-charge window
      if (cheapChargeActive)
      {
        exitCheapCharge(false);
      }
      // Reset the voltage-exit latch so the next window starts fresh
      if (cheapChargeVoltageExit)
      {
        cheapChargeVoltageExit = false;
        Serial.println("CheapCharge: voltage exit latch reset outside window");
      }
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
