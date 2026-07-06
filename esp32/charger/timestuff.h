#ifndef TIME_STUFF_H
#define TIME_STUFF_H
void timeLoop();
void timeSetup();

// True when current time is in the cheap-rate window.
// Start/end time configured by CHEAP_RATE_START/END_HOUR/MIN in timestuff.cpp.
// Read by pwmFunctions.cpp and pcemon.cpp to protect charger limits.
extern bool inCheapRateWindow;
#endif
