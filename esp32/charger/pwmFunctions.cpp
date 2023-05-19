#include "Arduino.h"
#include "espEmonLib.h"
#include "pcemon.h"
#include "battery.h"

boolean powerOn=false;
int gtiPin=23;
int upperChargerLimit = 50;  //point to turn charger off
int lowerChargerLimit = -50;  // point to turn charger on
float voltageLimit = 59.6;
int chargerPLimit = 4000; //max watts into charger ( prob 2000 into battery)

const int freq = 200;
int SOC = 90;  // TODO neds calculating

const float current_limit = 10;
const int resolution = 8;  //2^8 = 256
extern EnergyMonitor grid;
extern EnergyMonitor charger;
extern float chargerPower;
int powerPin = 21;
int psu_voltage_pins[] = { 19, 18, 5, 22, 16 };
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
int getTotalResistance() {
  int retval;
  for (int i = 0; i < psu_count; i++) {
    retval += psu_resistance_values[i];
  }
  return retval;
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
    Serial.println("Writing " +(String)psu_resistance_values[i] + " to psu " + (String)i);
    ledcWrite(pwmChannels[i], psu_resistance_values[i]);
  }
}
void enableGti(){  //only enable discharge if grid is over 300W
  if(grid.realPower > 300){
    digitalWrite(gtiPin, HIGH);
  }
  else
  {
    digitalWrite(gtiPin, HIGH);
  }

}
void turnPowerOff() {
  enableGti();
  delay(100);
  digitalWrite(powerPin, HIGH);
  powerOn=false;

}
void turnPowerOn() {
  digitalWrite(powerPin, LOW);
  enableGti();
  powerOn=true;
}

void pwmSetup() {
  // configure LED PWM functionalitites
  pinMode(powerPin, OUTPUT);
  pinMode(gtiPin, OUTPUT);
  turnPowerOff();
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
  turnPowerOn();
  for (int dutyCycle = 0; dutyCycle <= (range * 5); dutyCycle++) {
    // changing the LED brightness with PWM
    incrementPower(true,1);
    delay(10);
  }
}
void rampDown() {
  turnPowerOn();
  for (int dutyCycle = (range * 5); dutyCycle >= 0; dutyCycle--) {
    // changing the LED brightness with PWM
    decrementPower(true,1);
    //delay(200);
  }
}

void increaseChargerPower(float startingChargerPower) {
  //here gridp is lower than the lowerChargerLimit.
 
  float fakeGridp = grid.realPower + 10000;  // cancel out -ve values
  float fakeLowerLimit = lowerChargerLimit + 10000;
  float increaseAmount = fakeLowerLimit - fakeGridp;  // will always be +ve
  Serial.println("startingCpower="+(String)startingChargerPower+ (String)lowerChargerLimit+" - "+(String)grid.realPower+" = "+(String)increaseAmount);
  increaseAmount = increaseAmount * 0.9;             //don't overshoot
  int stepAmount = (increaseAmount / 100)+1; // react faster to large change 
  float target = startingChargerPower + increaseAmount;

  Serial.println("increase target:" + String(target));
  float chargerPower = startingChargerPower;
  //float gtiPower = readGti();
  while (chargerPower < target && (charger.Irms < current_limit) && !voltageLimitReached() && !isAtMaxPower() && chargerPower < chargerPLimit) {
    incrementPower(true,stepAmount);
    chargerPower = readCharger();
   // gtiPower = readGti();
    //delay(5);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}

void reduceChargerPower(float startingChargerPower) {
  //for starters here gridp is higher than the upper limit
  decrementPower(true,1); //always reduce by 1 in case vbatt is too high
  float fakeGridp = grid.realPower + 10000;  // cancel out -ve values
  float fakeUpperLimit = upperChargerLimit + 10000;
  float reductionAmount = fakeGridp - fakeUpperLimit;  // will always be +ve
  reductionAmount = reductionAmount * 0.95;            //don't overshoot
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
  float vbatt=readBattery();
  float presentChargerPower = readCharger();
  if (presentChargerPower > chargerPLimit){
    Serial.println("CHARGE POWER LIMIT REACHED "+(String)presentChargerPower);
  }
  if (grid.realPower > upperChargerLimit || vbatt > voltageLimit || presentChargerPower > chargerPLimit) {
    if (isAtMinPower()) {
      Serial.println("turning off , setting pin HIGH");
      turnPowerOff();  //turn off
    } else {
      reduceChargerPower(presentChargerPower);
    }
  } else if (SOC < 99 && grid.realPower < lowerChargerLimit) {
    if (!powerOn) {
      Serial.println("turning on , setting pin LOW");
      turnPowerOn();  // turn on
         //allow spike
      readGrid();
      for(int i=0; i<50; i++) // reading the charger a few times stops the power on spike
      {
         readCharger();
      } 
      presentChargerPower = readCharger();
    }
    increaseChargerPower(presentChargerPower);
  }
}