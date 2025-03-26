#include "Arduino.h"
#include "espEmonLib.h"


int batteryPin=39;
float batteryTotalVoltage=0.0;
float history[15]; 
int arraySize=15;
int historyPointer=0;

void addToHistory(float value)
{
  history[historyPointer]=value;
  historyPointer++;
  if (historyPointer==arraySize){
    historyPointer=0;
  }
}

float getAverageValue()
{
  float retval=0.0;
  for(int i=0; i<arraySize; i++)
  {
     retval+=history[i];
  }
  retval=retval/arraySize;
  return retval;
}

float getMinValue()
{
  float retval=70.0;
  for(int i=0; i<arraySize; i++)
  {
     if(history[i] < retval)
      {
        retval=history[i];
      }  
  }
  return retval;
}

float readBattery()
{
  int adcValue=0;
  delay(5);
  adcValue += analogRead(batteryPin);
  


  //4096=3.3V
  // 39.5V  is really 44.5
  // 63.5   is really 61.6
  /*
  y = (x - 39.5) * (61.6 - 44.5) / (63.5 - 39.5) + 44.5
  */
  //
  float voltageOnPin = (adcValue * 3.3) / 4095;
  float rV = voltageOnPin*22.85;

  batteryTotalVoltage= (rV - 39.5)* (61.6-44.5) / (63.5 - 39.5) + 44.5;
  
  addToHistory(batteryTotalVoltage);
  float batt= getMinValue();
  //Serial.println("VBatt:"+(String)batt);
  return batt;
}


void setupBattery()
{
  pinMode(batteryPin,INPUT);
   for(int i=0; i<arraySize; i++)
    {
      readBattery();
    }
}

