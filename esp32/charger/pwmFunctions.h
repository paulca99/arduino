#ifndef PWM_FUNCTION_H
#define PWM_FUNCTION_H
void pwmSetup();
bool rampUp();
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
void turnAfterburnerOn();
void turnAfterburnerOff();
void setupTest();
void goBottom();
void goMid();
void goTop();
void writePowerValuesToPSUs();

// GTI inhibit state: true = GTI is blocked regardless of schedule
extern bool gtiInhibited;
// charger relay state: true = charger is on
extern bool powerOn;
#endif