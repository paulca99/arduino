#ifndef PC_WIFI_H
#define PC_WIFI_H
void wifiSetup();
void wifiLoop();
void connectToWifi();
// Poll Pi /energy-state endpoint and update gtiInhibited.
// Must be called regularly (wifiLoop handles interval internally).
void pollEnergyState();
#endif