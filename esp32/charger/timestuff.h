#ifndef TIME_STUFF_H
#define TIME_STUFF_H
void timeLoop();
void timeSetup();

// True when current time is in the cheap-rate window (00:00-06:00).
// Read by pwmFunctions.cpp and pcemon.cpp to protect lowerChargerLimit.
extern bool inCheapRateWindow;
#endif