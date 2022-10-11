// Initializing PWM Pin
const int psu_count = 5;
const int current_limit = 20;  //1kw for starters
const float halfVCC = 2.5000; // for current calcs
const float howManyVoltsPerAmp = 0.066; // for current calcs
int chargerCurrentSensorPins [] = {13,14};
int power240pins[] = { 8, 9, 10, 11, 12 };  // 240V relays
int psu_voltage_pins[] = { 3, 4, 5, 6, 7 };
int psu_resistance_values[] = { 255, 255, 255, 255, 255 }; // higher the R the lower the V
boolean search_upwards=true;


void setup() {

  for (int i = 0; i < psu_count; i++) {
    pinMode(psu_voltage_pins[i], OUTPUT);
    analogWrite(psu_voltage_pins[i], psu_resistance_values[i]);  // set resistance to max (voltage to min)
    digitalWrite(power240pins[i], HIGH);                         // turn on the 240 supply
  }
}

float getChargerCurrent()
{
  //2.5V = ZERO amps .... 66mv per AMP
  float avgSensorVal = 0.0;
  int sampleCount=50;
  for (int i = 0; i < sampleCount; i=i+2)
  {
    avgSensorVal += analogRead(chargerCurrentSensorPins[i]);
    avgSensorVal += analogRead(chargerCurrentSensorPins[i+1]);
    delay(2);
  }
  avgSensorVal = (avgSensorVal / sampleCount) / 2;
  float volts = avgSensorVal - halfVCC // remove the offset
  float amps = volts / howManyVoltsPerAmp;  
  return amps;

}



int getOverallPowerPosition() {
  int avgSensorVal = 0;
  for (int i = 0; i < psu_count; i++) {
    avgSensorVal = avgSensorVal + psu_resistance_values[i];
  }
  return avgSensorVal;
}



void searchUpwards() {
// read the Grid value , if it's say -500W , we know we have to add 500W to the battery....BINARY SEARCH....if only a few watts out ....FINE TUNING
//BINARY:-
// read the present battery voltage and battery current, and we have present battery power (lets say 200W) so target power into battery is 700W
// set state to BINARY-SEARCHING.
// set initial jump-value as total available jump range .. 0 to 1275  divided by 2...OR ....for starters, maybe set it to 500 ???
// set search direction as UP/DOWN
//
// **** jump in search direction by jump-value
// read the present battery voltage and battery current, if we're within 650W-750W break out;
// reduce jump-value by half..... jump-value=jump-value / 2
// if we've overshot flip-direction, GO BACK TO *****
//
// Once broken out ,set state to FINE_TUNING
// read grid value again , if adjustment required use fine tuner ....1 by 1 .
//
// Now we're stuck in fine tuning based, need to fine tune incrementally to within 50W then switch state to STABLE.
// If we're more than 500W away , scrap it , flip to BINARY.



 
}
void jumpUpwards() {
  //Increase voltage means decreasing values
  setTargetResistance(getOverallPowerPosition() / 2);
}

void jumpDownwards() {
  //Decreasing voltage means increasing values
  // find difference from max value , halve it , and add it where we are.
  int maxValue = psu_count * 255;
  int currentPosition = getOverallPowerPosition();
  int incrementRequired = (maxValue - currentPosition) / 2;
  int newTarget = currentPosition + incrementRequired;
  setTargetResistance(newTarget);
}


void setTargetPositiosetTargetResistancen(int target) {
  int numPSUsThatNeedToBeOnFull = target / 255;
  int remainder = target % 255;
  for (int i = 0; i < numPSUsThatNeedToBeOnFull; i++) {
    analogWrite(psu_voltage_pins[i], 255);
  }
  analogWrite(psu_voltage_pins[numPSUsThatNeedToBeOnFull], remainder);
  //set rest to 0
  for (int i = numPSUsThatNeedToBeOnFull + 1; i < 5; i++) {
    analogWrite(psu_voltage_pins[i], 0);
  }
}


void incrementPower() {
  for (int i = 0; i < psu_count; i++) {
    if (psu_resistance_values[i] > 0) {
      psu_resistance_values[i]--;
      analogWrite(psu_voltage_pins[i], psu_resistance_values[i]);
      break;
    }
  }
}

void decrementPower() {
  for (int i = 0; i < psu_count; i++) {
    if (psu_resistance_values[i] < 255) {
      psu_resistance_values[i]++;
      analogWrite(psu_voltage_pins[i], psu_resistance_values[i]);
      break;
    }
  }
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



void loop() {
  while (!isAtMaxPower()) {
    incrementPower();
  }
  while (!isAtMinPower()) {
    decrementPower();
  }
}