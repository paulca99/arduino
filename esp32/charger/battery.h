#ifndef PC_BATT_H
#define PC_BATT_H
extern float batteryTotalVoltage;
float readBattery();
float readBatteryOnce();
float checkBattery();
void setupBattery();
#endif