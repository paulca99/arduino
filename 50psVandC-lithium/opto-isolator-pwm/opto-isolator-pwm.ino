// Initializing PWM Pin
#include <SPI.h>
#include <Ethernet.h>
#include "EmonLib.h"  // Include Emon Library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

const int MANUAL_STATE = 0;
const int AUTO_STATE = 1;
const int TEST_STATE = 2;
int state = TEST_STATE;
int SOC = 0;

int psu_pointer = 0;
const int psu_count = 5;
const int grid_current_pin = 0;
const int grid_voltage_pin = 1;
const int current_limit = 20;            //1kw for starters
const float voltage_limit = 58.6;        // for growatt GBLI5001
const float halfVCC = 2.3700;            //TODO check ...for current calcs
const float howManyVoltsPerAmp = 0.066;  // for current calcs, how much voltage changes per AMP
int chargerVoltagePin = 3;               //voltage is 1 : 20 ratio.
int chargerCurrentSensorPins[] = { 6, 7 };
int power240pins[] = { 31, 33, 35, 37, 39 };  // 240V relays
int manualButtonPin = 29;
int controlPotPin = 2;  //ANALOG
int psu_voltage_pins[] = { 3, 4, 5, 6, 7 };
int psu_resistance_values[] = { 255, 255, 255, 255, 255 };  // higher the R the lower the V
int dampingCoefficient = 10;                                // How many ms to wait after adjusting charger voltage before taking next reading


EnergyMonitor grid;

void setup() {


  //INPUTS
  pinMode(manualButtonPin, INPUT);
  pinMode(controlPotPin, INPUT);
  pinMode(chargerVoltagePin, INPUT);
  pinMode(chargerCurrentSensorPins[0], INPUT);
  pinMode(chargerCurrentSensorPins[1], INPUT);
  pinMode(grid_current_pin, INPUT);
  pinMode(grid_voltage_pin, INPUT);

  //OUTPUTS

  for (int i = 0; i < psu_count; i++) {
    pinMode(power240pins[i], OUTPUT);
    pinMode(psu_voltage_pins[i], OUTPUT);
    analogWrite(psu_voltage_pins[i], psu_resistance_values[i]);  // set resistance to max (voltage to min)
    digitalWrite(power240pins[i], HIGH);                         // turn off the 240 supply..HIGH=off
  }

  grid.voltage(grid_voltage_pin, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  grid.current(grid_current_pin, 50);
  attachInterrupt(manualButtonPin, manualButtonOn, RISING);
  attachInterrupt(manualButtonPin, manualButtonOff, FALLING);
}

//////INTERRUPTS//////
void manualButtonOn() {
  //TODO add logic here to check for a full sweep of the pot to avoid accidental manual switch
  state = MANUAL_STATE;
}
void manualButtonOff() {
  state = AUTO_STATE;
}
/////END INTERRUPTS/////



float getChargerCurrent() {
  //2.5V = ZERO amps .... 66mv per AMP
  float avgSensorVal = 0.0;
  int sampleCount = 20;
  for (int i = 0; i < sampleCount; i = i + 2) {
    avgSensorVal += analogRead(chargerCurrentSensorPins[i]);
    delay(2);
    avgSensorVal += analogRead(chargerCurrentSensorPins[i + 1]);
    delay(2);
  }
  avgSensorVal = (avgSensorVal / sampleCount);
  float volts = avgSensorVal - halfVCC;  // remove the offset
  float amps = volts / howManyVoltsPerAmp;
  return amps;
}
void setSOC(float voltage) {
  /*range is 0 :48V  100 : 58V
  so subtract 48 multiply by 10 and you've got SOC*/
  float diff = voltage - 48;
  SOC = (int)(diff * 10.0);
}

float getChargerVoltage() {
  //voltage is 1 : 20 ratio.
  float avgSensorVal = 0.0;
  int sampleCount = 5;
  for (int i = 0; i < sampleCount; i++) {
    avgSensorVal += analogRead(chargerVoltagePin);
    delay(2);
  }
  float voltage = 20 * (avgSensorVal / sampleCount);  //needs work
  setSOC(voltage);
  return voltage;
}

float getChargerPower() {
  return getChargerCurrent() * getChargerVoltage();
}

int getOverallResistanceValue() {
  int avgSensorVal = 0;
  for (int i = 0; i < psu_count; i++) {
    avgSensorVal = avgSensorVal + psu_resistance_values[i];
  }
  return avgSensorVal;
}

void changeToTargetVoltage(int choice) {
  //choice is between 0 and 1023. make same as our range (1275)
  float newchoice = choice * 1.24633; //1023 ->1275
  int totalcounter = 0;
  for (int i = 0; i < psu_count; i++) {
    for (int x = 0; x < 255; x++) {
      if (newchoice < totalcounter) {
        analogWrite(power240pins[i], LOW);
        psu_resistance_values[i] = x;
      } else {
        if (x == 0) { analogWrite(power240pins[i], HIGH); }        
      }
      totalcounter++;
    }
  }
  writePowerValuesToPSUs();
}


void writePowerValuesToPSUs() {
  for (int i = 0; i < psu_count; i++) {
    analogWrite(psu_voltage_pins[i], psu_resistance_values[i]);
  }
}

void incrementPower() {  //means reducinng resistance
  //255,255,255,255,255   ->  254,255,255,255,255  ->  254,254,255,255,255
  if (!isAtMaxPower()) {
    psu_resistance_values[psu_pointer]--;
    psu_pointer++;
    if (psu_pointer == psu_count) {
      psu_pointer = 0;
    }
  }
  writePowerValuesToPSUs();
}

void decrementPower() {  //means increasing resistance
  //00000 , 00001 , 00011 , 00111 , 01111 , 11111 , 11112
  if (!isAtMinPower()) {
    psu_pointer--;
    if (psu_pointer == -1) {
      psu_pointer = psu_count - 1;
    }
    psu_resistance_values[psu_pointer]++;
  }
  writePowerValuesToPSUs();
}

boolean isAtMaxPower() {
  for (int i = 0; i < psu_count; i++) {
    if (psu_resistance_values[i] > 0) {
      return false;
    }
  }
  return true;
}

boolean isAtMinPower() {
  for (int i = 0; i < psu_count; i++) {
    if (psu_resistance_values[i] < 255) {
      return false;
    }
  }

  return true;
}

boolean currentLimitReached() {
  if (getChargerCurrent() > current_limit) {
    return true;
  }
  return false;
}
boolean voltageLimitReached(){
  if (getChargerVoltage() > voltage_limit) {
    return true;
  }
  return false;

}

void increaseChargerPower(float startingChargerPower) {
  //the grid is -ve, increasePower to charger by the magnitude
  float target = (grid.realPower * -1) + startingChargerPower;
  target = target - 50;  //keep grid negative

  while (getChargerPower() < target && !currentLimitReached() && !voltageLimitReached() && !isAtMaxPower()) {
    incrementPower();
    delay(dampingCoefficient);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}

void reduceChargerPower(float startingChargerPower) {
  //the grid is +ve, decrease power to charger by the magnitude
  float target = startingChargerPower - grid.realPower;
  target = target + 50;  //keep grid negative

  while (getChargerPower() > target && !isAtMinPower()) {
    decrementPower();
    delay(dampingCoefficient);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}


void adjustCharger() {
  float startingChargerPower = getChargerPower();
  if (grid.realPower > 0 || currentLimitReached()) {
    reduceChargerPower(startingChargerPower);
  } else if (SOC < 99) {
    increaseChargerPower(startingChargerPower);
  }
}

void updateAutoDisplay() {
  Serial.println(getChargerVoltage());
  int choice = analogRead(controlPotPin);  //0 to 1023
  //if(choice > 100)
  //display power set 1
  //display power set 2
  // display voltages
  // display battery etc
}

void updateManualDisplay() {
  Serial.println(getChargerVoltage());

  //displayDCVoltageand current
}
void autoLoop() {
  grid.calcVI(20, 1000);
  adjustCharger();
  updateAutoDisplay();
}

void manualLoop() {
  int choice = analogRead(controlPotPin);
  changeToTargetVoltage(choice);
  updateManualDisplay();
}

void testLoop()
{
  Serial.println("testing");
}


void loop() {
  if (state == MANUAL_STATE)
    manualLoop();
  else if (state == AUTO_STATE)
    autoLoop();
  else
    testLoop();
}