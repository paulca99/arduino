#ifndef PC_BATT_H
#define PC_BATT_H
float readBattery();
float readBatteryOnce();
float checkBattery();
void setupBattery();
double populateVoltages();
int  findHighestPSU();
int  findLowestPSU();
float findVoltageSpan();
extern bool useADSForBattery;  // toggle: false=pin39, true=ADS1115 ads2 ch1
extern bool ads2_enabled;      // true if ads2 initialized successfully
#endif