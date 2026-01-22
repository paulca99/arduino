#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

// ============================================================================
// PWM Controller Module
// ============================================================================
// Controls charger relay, GTI relay, and 5 series PSU PWM outputs.
// Implements the main control algorithm for charger power adjustment.

// Control limits (can be updated at runtime via web interface)
extern int upperChargerLimit;
extern int lowerChargerLimit;

// GTI enable flag (controlled by time schedule)
extern bool GTIenabled;

// State of charge (TODO: needs calculating)
extern int SOC;

// PSU resistance values (for web UI display)
extern int psuResistanceValues[];

void pwmSetup();
void turnPowerOn();
void turnPowerOff();
void turnGTIOn();
void turnGTIOff();
void adjustCharger();

bool isAtMinPower();
bool isAtMaxPower();
void incrementPower();
int getTotalResistance();

// Test functions
void rampUp();
void rampDown();

#endif // PWM_CONTROLLER_H
