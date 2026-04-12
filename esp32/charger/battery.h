#ifndef PC_BATT_H
#define PC_BATT_H
float readBattery();
float readBatteryOnce();
float checkBattery();
void setupBattery();
extern bool useADSForBattery;  // toggle: false=pin39, true=ADS1115 ads1 ch0
#endif