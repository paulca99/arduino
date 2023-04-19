#include "pwmFunctions.h"
#include "Arduino.h"

void rampUp(int pwmChannel,int range,int delayIn)
{
  for(int dutyCycle = 0; dutyCycle <= range; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(pwmChannel, dutyCycle);
    delay(delayIn);
  }
}
void rampDown(int pwmChannel,int range,int delayIn)
{
  for(int dutyCycle = range; dutyCycle >= 0; dutyCycle--){
    // changing the LED brightness with PWM
    ledcWrite(pwmChannel, dutyCycle);   
    delay(delayIn);
  }
}