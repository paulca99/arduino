#include "Arduino.h"
#include "TimeSchedule.h"
#include "Config.h"
#include "PwmController.h"

// ============================================================================
// Time Schedule Module Implementation
// ============================================================================

struct tm timeinfo;

// Forward declaration (defined in WiFiUI module)
extern void connectToWifi();

void timeSetup()
{
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
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
    // Check if within GTI hours (5:00 to 22:00)
    if ((timeinfo.tm_hour >= GTI_ON_HOUR) && (timeinfo.tm_hour < GTI_OFF_HOUR))
    {
      // Within GTI hours
      if (GTIenabled == false)
      {
        GTIenabled = true; // Should happen ONCE at 5am
        turnGTIOn();
      }
    }
    else
    {
      // Out of GTI hours
      GTIenabled = false;
      turnGTIOff();
    }
  }
}
