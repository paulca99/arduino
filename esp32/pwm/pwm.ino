#include "pwmFunctions.h"

const int pwmPin = 16;  // 16 corresponds to GPIO16
const int powerPin = 27; 

// setting PWM properties
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;
const int delayIn = 15;
int range=(pow(2, resolution))-1;
 
void setup(){
  // configure LED PWM functionalitites
  pinMode(pwmPin,OUTPUT);
  pinMode(powerPin,OUTPUT);
  digitalWrite(powerPin, HIGH);
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pwmPin, pwmChannel);

}
 
void loop(){
  // increase the LED brightness
  rampUp(pwmChannel,range,delayIn);

  // decrease the LED brightness
  rampDown(pwmChannel,range,delayIn);
}