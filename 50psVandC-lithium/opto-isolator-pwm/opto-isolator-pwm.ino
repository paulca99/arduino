// Initializing PWM Pin
const int psu_count = 5;
int psu_pins[] = { 3, 4, 5, 6, 7 };
int psu_values[] = { 255, 255, 255, 255, 255 };
void setup() {
  // Declaring PWM pin output
  for (int i = 0; i < psu_count; i++) {
    pinMode(psu_pins[i], OUTPUT);
  }
}

int getOverallPowerPosition(
{
  int retval = 0;
  for (int i = 0; i < psu_count; i++) {
    retval = retval + psu_values[i];
  }
  return retval;
}

// read the Grid value , if it's say -500W , we know we have to add 500W to the battery....BINARY SEARCH....if only a few watts out ....FINE TUNING
//BINARY:-
// read the present battery voltage and battery current, and we have present battery power (lets say 200W) so target power into battery is 700W
// set state to BINARY-SEARCHING.
// set initial jump-value as total available jump range divided by 2.
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



void searchUpwards()
{
  //we want to keep searching upwards until we pass the desired level plus or minus 100W
}
void jumpUpwards()
{
  //Increase voltage means decreasing values
  setTargetPosition(getOverallPowerPosition()/2);
}

void jumpDownwards()
{
  //Decreasing voltage means increasing values
  // find difference from max value , halve it , and add it where we are.
  int maxValue=psu_count*255;
  int currentPosition=getOverallPowerPosition();
  int incrementRequired= (maxValue - currentPosition)   /2;
  int newTarget = currentPosition+incrementRequired;
  setTargetPosition( newTarget );
}


void setTargetPosition(int target)
{
  int numberFull=target/255;
  int remainder=target%255;
  for (int i = 0; i < numberFull; i++) {
          analogWrite(psu_pins[i], 255]);
  }
  analogWrite(psu_pins[numberFull], remainder]);
}

void incrementPower()
{
  for (int i = 0; i < psu_count; i++) {
    if (psu_values[i] > 0) {
      psu_values[i]--;
      analogWrite(psu_pins[i], psu_values[i]);
      break;
    }
  }
}

void decrementPower()
{
  for (int i = 0; i < psu_count; i++) {
    if (psu_values[i] < 255) {
      psu_values[i]++;
      analogWrite(psu_pins[i], psu_values[i]);
      break;
    }
  }
}

boolean atMaxPower()
{
  for (int i = 0; i < psu_count; i++) {
    if (psu_values[i] > 0) {
      return false;
    }
  }
  return true;  
}

boolean atMinPower()
{
  for (int i = 0; i < psu_count; i++) {
    if (psu_values[i] < 255) {
      return false;
    }
  }
  return true;  
}



void loop()
{
  while (!atMaxPower()) {
    incrementPower();
  }
  while (!atMinPower()) {
    decrementPower();
  }
}