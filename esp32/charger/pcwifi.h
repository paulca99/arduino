#ifndef PC_WIFI_H
#define PC_WIFI_H
extern float solisBatteryPower;
extern int solisBatterySoc;
extern float solisPvTotalPower;
extern bool solisEnergyOk;
extern bool solisChargerAllowed;
extern bool gtiInhibited;
void wifiSetup();
void wifiLoop();
void connectToWifi();
void pollEnergyStateIfDue();
#endif