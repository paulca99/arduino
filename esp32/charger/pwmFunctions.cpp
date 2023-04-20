
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
  //the grid is -ve, increasePower to charger by the magnitude
  float target = (grid.realPower *-1 ) + startingChargerPower;
  target = target - 50;  //keep grid negative
    Serial.println("target:"+String(target));
  float chargerCurrent = startingChargerPower/grid.Vrms;
  float chargerPower = startingChargerPower;
  while (chargerPower< target && (charger.Irms < current_limit) && !voltageLimitReached() && !isAtMaxPower()) {
     incrementPower(true);
     readCharger();
     chargerPower = charger.Irms*grid.Vrms;
     delay(5);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}

void reduceChargerPower(float startingChargerPower) {
  //the grid is +ve, decrease power to charger by the magnitude
  float target = startingChargerPower - grid.realPower;
  float chargerCurrent = startingChargerPower/grid.Vrms;
  float chargerPower = startingChargerPower;
  target = target - 50;  //keep grid negative
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
  if (grid.realPower > -100 ) {
    if(isAtMinPower())
    {
      digitalWrite(powerPin,LOW); //turn off
    }
    reduceChargerPower(presentChargerPower);
  } else if (SOC < 99 && grid.realPower < -300) {
    if(isAtMinPower())
    {
      digitalWrite(powerPin,HIGH); // turn on
    }
    increaseChargerPower(presentChargerPower);
  }
}