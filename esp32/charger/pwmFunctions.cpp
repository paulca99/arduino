#include "Arduino.h"
#include "espEmonLib.h"
#include "pcemon.h"
#include "battery.h"

int upperChargerLimit = -100;  //point to turn charger off
int lowerChargerLimit = -300;  // point to turn charger on
float voltageLimit = 57.6;

const int freq = 500;
int SOC = 90;  // TODO neds calculating

const float current_limit = 10;
const int resolution = 8;  //2^8 = 256
extern EnergyMonitor grid;
extern EnergyMonitor charger;
int powerPin = 21;
int psu_voltage_pins[] = { 19, 18, 5, 17, 16 };
int pwmChannels[] = { 0, 1, 2, 3, 4 };
const int delayIn = 1;


int range = (pow(2, resolution)) - 1;
int psu_resistance_values[] = { range, range, range, range, range };
int psu_count = sizeof psu_resistance_values / sizeof psu_resistance_values[0];
int psu_pointer = 0;

bool isAtMaxPower() {
  for (int i = 0; i < psu_count; i++) {
    if (psu_resistance_values[i] > 0) {
      return false;
    }
  }
  Serial.println("MAX POWER");
  return true;
}

bool isAtMinPower() {
  for (int i = 0; i < psu_count; i++) {
    if (psu_resistance_values[i] < range) {
      return false;
    }
  }
  Serial.println("IsAtMinPower");
  return true;
}

boolean voltageLimitReached() {
  float presentVoltage = readBattery();
  Serial.println("VBATT:"+(String)presentVoltage);
  if (presentVoltage > voltageLimit) {
    Serial.println("VOLTAGE LIMIT REACHED");
    return true;
  }
  return false;
}
void writePowerValuesToPSUs() {
  for (int i = 0; i < psu_count; i++) {
    //Serial.println("Writing " +(String)psu_resistance_values[i] + " to psu " + (String)i);
    ledcWrite(pwmChannels[i], psu_resistance_values[i]);
  }
}
void pwmSetup() {
  // configure LED PWM functionalitites
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH);
  delay(20);
  for (int i = 0; i < psu_count; i++) {

    pinMode(psu_voltage_pins[i], OUTPUT);
    ledcSetup(pwmChannels[i], freq, resolution);
    ledcAttachPin(psu_voltage_pins[i], pwmChannels[i]);
    ledcWrite(pwmChannels[i], psu_resistance_values[i]);
  }
  writePowerValuesToPSUs();
}
void setMinPower() {
  for (int i = 0; i < psu_count; i++) {
    psu_resistance_values[i] = range - 1;
  }
  psu_pointer = 0;
}


void turnPowerOff() {
  digitalWrite(powerPin, HIGH);
}
void turnPowerOn() {
  digitalWrite(powerPin, LOW);
}
void incrementPower(boolean write,int stepAmount) {  //means reducinng resistance
                                      // Serial.println("IncrementPower");
                                      //255,255,255,255,255   ->  254,255,255,255,255  ->  254,254,255,255,255

  // Serial.println("decrementing values "+(String)psu_pointer);
  psu_resistance_values[psu_pointer] = psu_resistance_values[psu_pointer] - stepAmount;
  if (psu_resistance_values[psu_pointer] < 0) {
    psu_resistance_values[psu_pointer] = 0;
  }
  psu_pointer++;
  if (psu_pointer == psu_count) {
    psu_pointer = 0;
  }

  if (write) {
    writePowerValuesToPSUs();
  }
}

void decrementPower(boolean write,int stepAmount) {  //means increasing resistance
                                      // Serial.println("DecrementPower");
  //00000 , 00001 , 00011 , 00111 , 01111 , 11111 , 11112
  if (!isAtMinPower()) {
    psu_pointer--;
    if (psu_pointer == -1) {
      psu_pointer = psu_count - 1;
    }
    psu_resistance_values[psu_pointer] = psu_resistance_values[psu_pointer] + stepAmount;
    if (psu_resistance_values[psu_pointer] > range) {
      psu_resistance_values[psu_pointer] = range;
    }
  }
  if (write) {
    writePowerValuesToPSUs();
  }
}

void rampUp() {
  for (int dutyCycle = 0; dutyCycle <= (range * 5); dutyCycle++) {
    // changing the LED brightness with PWM
    incrementPower(true,1);
  }
}
void rampDown() {
  for (int dutyCycle = (range * 5); dutyCycle >= 0; dutyCycle--) {
    // changing the LED brightness with PWM
    decrementPower(true,1);
    delay(delayIn);
  }
}

void increaseChargerPower(float startingChargerPower) {
  //here gridp is lower than the lowerChargerLimit.
  float fakeGridp = grid.realPower + 10000;  // cancel out -ve values
  float fakeLowerLimit = lowerChargerLimit + 10000;
  float increaseAmount = fakeLowerLimit - fakeGridp;  // will always be +ve
  increaseAmount = increaseAmount * 0.75;             //don't overshoot
  int stepAmount = (increaseAmount / 100)+1; // react faster to large change 
  float target = startingChargerPower + increaseAmount;

  Serial.println("increase target:" + String(target));
  float chargerPower = startingChargerPower;
  while (chargerPower < target && (charger.Irms < current_limit) && !voltageLimitReached() && !isAtMaxPower()) {
    incrementPower(true,stepAmount);
    chargerPower = readCharger();
    //delay(5);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}

void reduceChargerPower(float startingChargerPower) {
  //for starters here gridp is higher than the upper limit

  float fakeGridp = grid.realPower + 10000;  // cancel out -ve values
  float fakeUpperLimit = upperChargerLimit + 10000;
  float reductionAmount = fakeGridp - fakeUpperLimit;  // will always be +ve
  reductionAmount = reductionAmount * 0.75;            //don't overshoot
  int stepAmount = (reductionAmount / 100)+1; // react faster to large change
  float target = startingChargerPower - reductionAmount;

  float chargerPower = startingChargerPower;

  Serial.println("reduce target:" + String(target));
  while (chargerPower > target && !isAtMinPower()) {
    chargerPower = readCharger();
    decrementPower(true,stepAmount);
    //delay(5);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}

void adjustCharger() {
  float presentChargerCurrent = charger.Irms;
  float presentChargerPower = presentChargerCurrent * grid.Vrms;
  if (grid.realPower > upperChargerLimit) {
    if (isAtMinPower()) {
      Serial.println("turning off , setting pin HIGH");
      turnPowerOff();  //turn off
    } else {
      reduceChargerPower(presentChargerPower);
    }
  } else if (SOC < 99 && grid.realPower < lowerChargerLimit) {
    if (isAtMinPower()) {
      Serial.println("turning on , setting pin LOW");
      turnPowerOn();  // turn on
      //delay(8000);    //allow spike
    }
    increaseChargerPower(presentChargerPower);
  }
}