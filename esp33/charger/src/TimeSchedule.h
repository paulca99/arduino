#ifndef TIME_SCHEDULE_H
#define TIME_SCHEDULE_H

#include <time.h>

// ============================================================================
// Time Schedule Module
// ============================================================================
// Manages NTP time synchronization and GTI on/off scheduling by hour.

// Current time info (shared with web UI for display)
extern struct tm timeinfo;

void timeSetup();
void timeLoop();

#endif // TIME_SCHEDULE_H
