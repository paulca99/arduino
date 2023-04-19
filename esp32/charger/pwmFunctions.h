#ifndef pwmFunctions.h
#define pwmFunctions.h
void pwmSetup();
void rampUp();
void rampDown();
bool isAtMinPower();
bool isAtMaxPower();
void incrementPower();

#endif