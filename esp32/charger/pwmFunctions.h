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

// Cheap charge helpers
void applyCheapChargeLimits();
void restoreNormalChargeLimits();
void exitCheapCharge(bool voltageLimitExit);

// GTI inhibit state: true = GTI is blocked regardless of schedule
extern bool gtiInhibited;
// charger relay state: true = charger is on
extern bool powerOn;
// Solis charger permission: true = AC charger may start/run.
// Set by pollEnergyState(). Fail-safe is false (charger blocked on any poll error).
extern bool solisChargerAllowed;
// cheapChargeActive: true while inside the overnight cheap-charge window and voltage OK.
extern bool cheapChargeActive;
// cheapChargeVoltageExit: latched true when voltage limit is hit during cheap charge.
// Cleared by timeLoop() once the cheap-charge window has ended.
extern bool cheapChargeVoltageExit;
#endif