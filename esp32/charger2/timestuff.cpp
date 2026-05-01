/*
 * timestuff.cpp — NTP time + GTI scheduling
 */
#include "timestuff.h"
#include "control.h"

#include <time.h>
#include <WiFi.h>

struct tm timeinfo;

static const char* NTP_SERVER       = "pool.ntp.org";
static const long  GMT_OFFSET_SEC   = 0;
static const int   DAYLIGHT_OFFSET  = 3600;  // BST = UTC+1

// GTI enable/disable hours (24-hour clock)
static const int GTI_ON_HOUR  = 5;
static const int GTI_OFF_HOUR = 23;

void timeSetup() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);
}

void timeLoop() {
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[Time] Failed to get NTP time");
    return;
  }
  Serial.printf("[Time] %s", asctime(&timeinfo));

  if (timeinfo.tm_hour >= GTI_ON_HOUR && timeinfo.tm_hour < GTI_OFF_HOUR) {
    if (!GTIenabled) {
      GTIenabled = true;
      turnGTIOn();
      Serial.println("[Time] GTI enabled");
    }
  }
  // Note: GTI is not forcibly disabled out of hours here —
  // same behaviour as the original charger/ firmware.
}
