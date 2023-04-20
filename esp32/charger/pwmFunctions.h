#ifndef PWM_FUNCTION_H
#define PWM_FUNCTION_H
void pwmSetup();
void rampUp();
void rampDown();
bool isAtMinPower();
bool isAtMaxPower();
void incrementPower();
void adjustCharger();
#endif