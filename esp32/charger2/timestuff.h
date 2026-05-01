/*
 * timestuff.h — NTP time + GTI schedule
 */
#pragma once

#include <Arduino.h>

extern struct tm timeinfo;

/** Initialise NTP.  Call once after WiFi is connected. */
void timeSetup();

/**
 * Check NTP time and apply GTI on/off schedule.
 * Call periodically (e.g. every 50 main-loop iterations).
 */
void timeLoop();
