
#include "Arduino.h"

const int freq = 5000;
const int resolution = 8; //2^8 = 256

int powerPin=27;
int psu_voltage_pins[] = { 19, 18, 5, 17, 16 };
int psu_resistance_values[] = { 255, 255, 255, 255, 255 }; 
int pwmChannels[] = { 0, 1, 2, 3, 4 }; 
const int delayIn = 4;

int range=(pow(2, resolution))-1;
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
    Serial.println("Writing " +(String)psu_resistance_values[i] + " to psu " + (String)i);
    ledcWrite(pwmChannels[i], psu_resistance_values[i]);
  }
}


void incrementPower(boolean write) {  //means reducinng resistance
  //Serial.println("IncrementPower");
  //255,255,255,255,255   ->  254,255,255,255,255  ->  254,254,255,255,255
  if(isAtMinPower()){
      digitalWrite(powerPin,HIGH);
  }
  else
  {
    psu_resistance_values[psu_pointer]--;
    psu_pointer++;
    if (psu_pointer == psu_count) {
      psu_pointer = 0;
    }
  }
  if(write)
  {
  writePowerValuesToPSUs();
  }
}

void decrementPower(boolean write) {  //means increasing resistance
  Serial.println("DecrementPower");
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
  for(int dutyCycle = 0; dutyCycle <= range; dutyCycle++){   
    // changing the LED brightness with PWM
    incrementPower(true);
    delay(delayIn);
  }
}
void rampDown()
{
  for(int dutyCycle = range; dutyCycle >= 0; dutyCycle--){
    // changing the LED brightness with PWM
    decrementPower(true); 
    delay(delayIn);
  }
}