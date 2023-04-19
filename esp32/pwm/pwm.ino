#include "pwmFunctions.h"

const int pwmPin1 = 16;  // 16 corresponds to GPIO16
const int pwmPin2 = 17;  // 16 corresponds to GPIO16

const int powerPin = 27; 

// setting PWM properties
const int freq = 5000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 10;
const int delayIn = 13;
int range=(pow(2, resolution))-1;
 
void setup(){
  // configure LED PWM functionalitites
  pinMode(pwmPin1,OUTPUT);
  pinMode(pwmPin2,OUTPUT);
  pinMode(powerPin,OUTPUT);
  digitalWrite(powerPin, HIGH);
  ledcSetup(pwmChannel1, freq, resolution);
    ledcSetup(pwmChannel2, 2000, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pwmPin1, pwmChannel1);
  ledcAttachPin(pwmPin2, pwmChannel2);

}
 
void loop(){
  // increase the LED brightness
  rampUp(pwmChannel1,range,delayIn);
  // decrease the LED brightness
  rampDown(pwmChannel1,range,delayIn);
    rampUp(pwmChannel2,range,delayIn);
  // decrease the LED brightness
  rampDown(pwmChannel2,range,delayIn);
}