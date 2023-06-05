#ifndef PWM_FUNCTION_H
#define PWM_FUNCTION_H
void pwmSetup();
void rampUp();
void rampDown();
bool isAtMinPower();
bool isAtMaxPower();
void incrementPower();
int getTotalResistance();
void turnPowerOff();
void adjustCharger();
void turnGTIOff();
void turnGTIOn();
void turnPowerOn();
void turnPowerOff();
#endif