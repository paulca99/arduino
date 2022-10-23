// Initializing PWM Pin
#include <SPI.h>
#include "ACS712.h"
#include <Ethernet.h>
#include "EmonLib.h"  // Include Emon Library
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin # (or -1 if sharing Arduino reset pin)
ACS712 currentSensor1(A6, 5.1, 1023, 66);
ACS712 currentSensor2(A7, 5.1, 1023, 66);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
EthernetServer server(80);

static const unsigned char PROGMEM logo_bmp[] = {
  B01110111, B01010100,
  B01010101, B01010100,
  B01110111, B01010100,
  B01000101, B01010100,
  B01000101, B01110111,
  B00000000, B00000000,
  B01110111, B01110111,
  B01000101, B01000010,
  B01110101, B01110010,
  B00010101, B01000010,
  B01110111, B01000010
};

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

const int MANUAL_STATE = 0;
const int AUTO_STATE = 1;
const int TEST_STATE = 2;
int state = MANUAL_STATE;
int SOC = 0;

int psu_pointer = 0;
const int psu_count = 5;
const int charger_ct_pin = 2;
const int grid_current_pin = 0;
const int grid_voltage_pin = 1;
const int solar_current_pin = 2;
const int current_limit = 10;            //500w for starters
const float voltage_limit = 58.6;        // for growatt GBLI5001
const float halfVCC = 2.3700;            //TODO check ...for current calcs
const float howManyVoltsPerAmp = 0.066;  // for current calcs, how much voltage changes per AMP
int chargerVoltagePin = 3;               //voltage is 1 : 20 ratio.
int chargerCurrentSensorPins[] = { 6, 7 };
int power240pins[] = { 31, 33, 35, 37, 39 };  // 240V relays
int manualButtonPin = 29;
int controlPotPin = 5;  //ANALOG
int psu_voltage_pins[] = { 3, 4, 5, 6, 7 };
int psu_resistance_values[] = { 255, 255, 255, 255, 255 };  // higher the R the lower the V
int dampingCoefficient = 10;                                // How many ms to wait after adjusting charger voltage before taking next reading


EnergyMonitor grid;
EnergyMonitor charger;
EnergyMonitor solar;


void setup() {

  currentSensor1.autoMidPoint();
  currentSensor2.autoMidPoint();
  Serial.begin(9600);

  //INPUTS
  pinMode(manualButtonPin, INPUT);
  pinMode(controlPotPin, INPUT);
  pinMode(chargerVoltagePin, INPUT);
  pinMode(chargerCurrentSensorPins[0], INPUT);
  pinMode(chargerCurrentSensorPins[1], INPUT);
  pinMode(grid_current_pin, INPUT);
  pinMode(grid_voltage_pin, INPUT);
  pinMode(solar_current_pin, INPUT);

  //OUTPUTS

  for (int i = 0; i < psu_count; i++) {
    pinMode(power240pins[i], OUTPUT);
    pinMode(psu_voltage_pins[i], OUTPUT);
    analogWrite(psu_voltage_pins[i], psu_resistance_values[i]);  // set resistance to max (voltage to min)
    digitalWrite(power240pins[i], HIGH);                         // turn off the 240 supply..HIGH=off
  }


  grid.voltage(grid_voltage_pin, 163.36, 2.40);  // Voltage: input pin, calibration, phase_shift
  grid.current(grid_current_pin, 29);
  charger.current(charger_ct_pin, 15);
  solar.voltage(1, 163.36, 2.40);  // Voltage: input pin, calibration, phase_shift
  solar.current(solar_current_pin, 29);
  attachInterrupt(manualButtonPin, manualButtonOn, RISING);
  attachInterrupt(manualButtonPin, manualButtonOff, FALLING);

  setupDisplay();
  setupEthernet();
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

void setupDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.display();
  delay(5000);  // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();
  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);
  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
}

void setupEthernet() {
  Ethernet.begin(mac, ip);
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
}

float getChargerCurrent() {
  double Irms = charger.calcIrms(200);

  return Irms;  //draws 0.8 at idle
  /*int mA1 = currentSensor1.mA_DC();
  int mA2 = currentSensor2.mA_DC();

  Serial.println(mA1);
  Serial.println(mA2);
  delay(1000);
return (float)mA1+mA2;*/
}
void setSOC(float voltage) {
  /*range is 0 :48V  100 : 58V
  so subtract 48 multiply by 10 and you've got SOC*/
  float diff = voltage - 48;
  SOC = (int)(diff * 10.0);
}

float getChargerVoltage() {
  float retval = 0.0;
  for (int i = 0; i < 10; i++) {
    retval += (1023.0 - analogRead(chargerVoltagePin));
    delay(2);
  }
  retval = retval / 10;
  //40=45.6 V 980=58.5V spread=13.1
  retval = retval / 39;
  retval = retval + 41;
  return retval;
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
void setMinPower()
{
psu_resistance_values[] = { 255, 255, 255, 255, 255 }; 
psu_pointer=0;
}

void changeToTargetVoltage(int choice) {
  //choice is between 0 and 1023. make same as our range (1275)
  float newchoice = choice * 1.24633;  //1023 ->1275
  setMinPower();
  int totalcounter = 0;
  for (int x = 0; x < 255; x++) {
    for (int i = 0; i < psu_count; i++) {
      if (newchoice > totalcounter) {

       incrementPower(false);
      }
      else
      {
       x=256;
       break;
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

void incrementPower(boolean write) {  //means reducinng resistance
  //255,255,255,255,255   ->  254,255,255,255,255  ->  254,254,255,255,255
  if (!isAtMaxPower()) {
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
  //00000 , 00001 , 00011 , 00111 , 01111 , 11111 , 11112
  if (!isAtMinPower()) {
    psu_pointer--;
    if (psu_pointer == -1) {
      psu_pointer = psu_count - 1;
    }
    psu_resistance_values[psu_pointer]++;
  }
   if(write)
  {
  writePowerValuesToPSUs();
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

boolean currentLimitReached() {
  if (getChargerCurrent() > current_limit) {
    return true;
  }
  return false;
}
boolean voltageLimitReached() {
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
    incrementPower(true);
    delay(dampingCoefficient);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}

void reduceChargerPower(float startingChargerPower) {
  //the grid is +ve, decrease power to charger by the magnitude
  float target = startingChargerPower - grid.realPower;
  target = target - 50;  //keep grid negative

  while (getChargerPower() > target && !isAtMinPower()) {
    decrementPower(true);
    delay(dampingCoefficient);  // Damping coefficient, can be reduced if we don't overshoot too badly
  }
}


void adjustCharger() {
  float presentChargerPower = getChargerPower();
  if (grid.realPower > 0 || currentLimitReached()) {
    reduceChargerPower(presentChargerPower);
  } else if (SOC < 99) {
    increaseChargerPower(presentChargerPower);
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
  getChargerVoltage();


  //displayDCVoltageand current
}
void autoLoop() {
  grid.calcVI(20, 1000);
  solar.calcVI(20, 1000);  // TODO reduce frequency , not used for calcs , could be 1 loop in 5/10/20.
  adjustCharger();
  updateAutoDisplay();
}

void manualLoop() {
  // grid.calcVI(20, 1000);

  delay(3);
  int choice = analogRead(controlPotPin);
  //Serial.println(choice);
  changeToTargetVoltage(choice);
  updateManualDisplay();
  String stats = "Power:" + (String)getChargerPower() + ":" + (String)getChargerVoltage() + ":" + (String)getChargerCurrent();
  Serial.println(stats);
}

void testLoop() {
  for (int i = 0; i < psu_count; i++) {
    delay(1000);
    analogWrite(power240pins[i], LOW);
  }
  for (int i = 0; i < psu_count; i++) {
    delay(1000);
    analogWrite(power240pins[i], HIGH);
  }
}


void loop() {
  if (state == MANUAL_STATE)
    manualLoop();
  else if (state == AUTO_STATE)
    autoLoop();
  else
    testLoop();
}

void ethernetLoop() {
  EthernetClient client = server.available();
  if (client) {
    // Serial.println("new client");
    // an http request ends with a blank line
    String command = "";
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        command += c;
        //Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply

        if (c == '\n' && currentLineIsBlank) {
          //processCommand(command);
          // output the value of each analog input pin
          // send a power summary
          client.println(String(grid.realPower) + "," + String(grid.apparentPower) + "," + String(grid.Vrms) + "," + String(grid.Irms) + "," + String(grid.powerFactor) + "," + String(solar.realPower) + "," + String(solar.apparentPower) + "," + String(solar.Vrms) + "," + String(solar.Irms) + "," + String(solar.powerFactor) + "," + String("0") + "," + String("0") + "," + String(getChargerVoltage(), 2) + "," + String("0.0") + "," + String("0.0") + "," + String("0.0") + "," + String(getChargerPower()) + "," + String("0.0") + ",EOT");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    //  Serial.println("client disconnected");
  }
}