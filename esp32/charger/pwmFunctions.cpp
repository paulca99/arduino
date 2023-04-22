
#include "Arduino.h"
#include "EmonLib.h"
#include "pcemon.h"

const int freq = 500;
int SOC = 90; // TODO neds calculating
const float current_limit=10;
const int resolution = 13; //2^8 = 256
extern EnergyMonitor grid;
extern EnergyMonitor charger;
int powerPin=27;
int psu_voltage_pins[] = { 19, 18, 5, 17, 16 };
int pwmChannels[] = { 0, 1, 2, 3, 4 }; 
const int delayIn = 1;
int upperChargerLimit = -100; //point to turn charger off
int lowerChargerLimit = -300; // point to turn charger on

int range=(pow(2, resolution))-1;
int psu_resistance_values[] = { range-1, range-1, range-1, range-1, range-1 }; 
int psu_count = sizeof psu_resistance_values / sizeof psu_resistance_values[0];
int psu_pointer=0;

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
    if (psu_resistance_values[i] < 255) {
      return false;
    }
  }

  return true;
}

boolean voltageLimitReached() {
//  if (getChargerVoltage() > voltage_limit) {
//    return true;
//  }
  return false;
}

void pwmSetup(){
  // configure LED PWM functionalitites
      pinMode(powerPin,OUTPUT);
      for(int i=0; i<psu_count; i++)
      {

        pinMode(psu_voltage_pins[i],OUTPUT);
        ledcSetup(pwmChannels[i], freq, resolution);
        ledcAttachPin(psu_voltage_pins[i], pwmChannels[i]);
        ledcWrite(pwmChannels[i], psu_resistance_values[i]);
      }
}
void setMinPower(){
      for (int i = 0; i < psu_count; i++) {
        psu_resistance_values[i]=255;
      }
      psu_pointer=0;
}

void writePowerValuesToPSUs() {
  for (int i = 0; i < psu_count; i++) {
   // Serial.println("Writing " +(String)psu_resistance_values[i] + " to psu " + (String)i);
    ledcWrite(pwmChannels[i], psu_resistance_values[i]);
  }
}


void incrementPower(boolean write) {  //means reducinng resistance
 // Serial.println("IncrementPower");
  //255,255,255,255,255   ->  254,255,255,255,255  ->  254,254,255,255,255
  if(isAtMinPower()){
      digitalWrite(powerPin,HIGH);
  }

   // Serial.println("decrementing values "+(String)psu_pointer);
    psu_resistance_values[psu_pointer]--;
    psu_pointer++;
    if (psu_pointer == psu_count) {
      psu_pointer = 0;
    }

  if(write)
  {
  writePowerValuesToPSUs();
  }
}

void decrementPower(boolean write) {  //means increasing resistance
 // Serial.println("DecrementPower");
  //00000 , 00001 , 00011 , 00111 , 01111 , 11111 , 11112
  if (!isAtMinPower()) {
    psu_pointer--;
    if (psu_pointer == -1) {
      psu_pointer = psu_count - 1;
    }
    psu_resistance_values[psu_pointer]++;
  }
  else
  {
      digitalWrite(powerPin,LOW);
  }
   if(write)
  {
  writePowerValuesToPSUs();
  }
}

void rampUp()
{
  for(int dutyCycle = 0; dutyCycle <= (range*5); dutyCycle++){   
    // changing the LED brightness with PWM
    incrementPower(true);
    delay(delayIn);
  }
}
void rampDown()
{
  for(int dutyCycle = (range*5); dutyCycle >= 0; dutyCycle--){
    // changing the LED brightness with PWM
    decrementPower(true); 
    delay(delayIn);
  }
}

void increaseChargerPower(float startingChargerPower) {
  //the grid will be -ve .... need to work out the gap between the charger lower limit
  // and where the grid is...then thats the target we need to increase by.

  float target = ((grid.realPower *-1 ) +lowerChargerLimit) + startingChargerPower;
  //eg..  -1000 * -1 = +1000 + (-300) = 700 + 200 = 900.....so need charger to use 700 more watts to get grid to -300
  //eg..  -1000 * -1 = +1000 + (0) = 1000 +200 = 1200 .... so need charger to use 1000W more to get grid to 0
  target = target *0.95; // onnly go to 95% to be safe
  Serial.println("target:"+String(target));
  float chargerPower = startingChargerPower;
  while (chargerPower < target && (charger.Irms < current_limit) && !voltageLimitReached() && !isAtMaxPower()) {
     incrementPower(true);
     readCharger();
     chargerPower = charger.Irms*grid.Vrms;
     delay(5);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}

void reduceChargerPower(float startingChargerPower) {
  //the grid could be -ve or positive .... need to work out the gap between the charger upper limit
  // and where the grid is...then thats the target we need to decrease by.

//e.g.   grid= 200W , chargerUpperLimit= -100W ....need to decrease by 300W
//e.g.   grid= -50W , chargerUpperLimit = -100W .... need to decrease by 50W
// so    chargerUpperLimit - grid.realPower  = reductionAmount... will be negative
// startingChargerPower + reductionAmount = target 
//e.g.  500W + (-300) = 200W
//e.g.  500W + (-50) = 450W
// if target is -ve , we'll be turning the charger off
  float reductionAmount = upperChargerLimit - grid.realPower;
  float target = startingChargerPower + reductionAmount;

  float chargerPower = startingChargerPower;
  target = target *0.95;  //keep grid negative
  Serial.println("target:"+String(target));
  while (chargerPower > target && !isAtMinPower()) {
    readCharger();
    chargerPower = charger.Irms*grid.Vrms;
    decrementPower(true);
    delay(5);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}

void adjustCharger() {
  float presentChargerCurrent = charger.Irms;
  float presentChargerPower=presentChargerCurrent*grid.Vrms;
  if (grid.realPower > upperChargerLimit ) {
    if(isAtMinPower())
    {
      digitalWrite(powerPin,LOW); //turn off
    }
    reduceChargerPower(presentChargerPower);
  } else if (SOC < 99 && grid.realPower < lowerChargerLimit) {
    if(isAtMinPower())
    {
      digitalWrite(powerPin,HIGH); // turn on
    }
    increaseChargerPower(presentChargerPower);
  }
}