/*
Home energy monitor
To upload with ethernet shield attached we need to reset the power, click upload, then power on 0.5 seconds later

*/

#include <SPI.h>
#include <Ethernet.h>
#include "EmonLib.h"             // Include Emon Library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000
};

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);
EnergyMonitor grid;             // Gridmon
EnergyMonitor solar;            // Solarmon
EnergyMonitor gti;            // Solarmon
EnergyMonitor charger;            // Solarmon


  // listen for incoming clients

int appHomePower;  

int realsolarp;
int realHomePower;
float solarIRMS;
float solarVRMS;
float solarPFactor;
int appsolarp;

int realgridp;
int appgridp;
float gridVRMS;
float gridIRMS;
float gridPFactor;

int realchargerp;
int appchargerp;
float chargerVRMS;
float chargerIRMS;
float chargerPFactor;

int realgtip;
int appgtip;
float gtiVRMS;
float gtiIRMS;
float gtiPFactor;

int INITIAL=0;
int TUNING=1;
int NIGHT=2;
int STATE=INITIAL;

int displayCounter=0;
boolean batteriesHaveCharge=false;
boolean beenCharging=false;
int chargerVoltage=100;
int chargeUpperThreshold=-50;
int chargeLowerThreshold=chargeUpperThreshold-150;

int chargerMaxPower=760;
int chargerPin=47;
int gtiPin=53;

float b1volts;
float b2volts;
float b3volts;
float b4volts;
float btotalvolts;

float bmaxvolts=42.0;

int minChargerVolts=100; //if 3 batteries
const byte numResistorPins = 8;
byte resistorPins[] = {22, 24, 26, 28, 30, 32, 34, 36};
byte resistorBasePins[] = {49,51};

//22-34 = resistor pins, 36 is Charger power relay

float readApin(int pin)
{
  int samples=10;
  delay(10);
  float retval=0.0;
  for(int i = 0; i < samples; i++)
  {
      retval = retval + (analogRead(pin) * 0.0049);  // read the input pin
      delay(5);
  }
  return retval / samples;
}

void setBaseR(int i)
{
  // 0 is 31.7K , 1 is 41.7K , 2 is 68K , 3 is 78K
      Serial.println("Setting base R to " + String(i));
  if(i == 0){
    digitalWrite(49, HIGH);digitalWrite(51, HIGH);
  }
  if(i == 1){
    digitalWrite(49, LOW);digitalWrite(51, HIGH);
  }
  if(i == 2){
    digitalWrite(49, HIGH);digitalWrite(51, LOW);
  }
  if(i == 3){
    digitalWrite(49, LOW);digitalWrite(51, LOW);
  }
}

void readBatteryVoltages()
{
    Serial.println("");
  float v1 = readApin(14);  // read the input pin
  //Serial.println("v1= " + String (v1,2));
  b1volts = v1 * 4.108;
  float v2 = readApin(13);  // read the input pin
  //Serial.println("v2= " + String (v2,2));  
  b2volts = (v2 * 7.82496) - b1volts;
  float v3 = readApin(12);  // read the input pin
  //Serial.println("v3= " + String (v3,2));
  b3volts = (v3 * 11.132) - b2volts -b1volts;
  float v4 = readApin(11);  // read the input pin
  //Serial.println("v4= " + String (v4,2));
  b4volts = (v4 * 16) - b3volts -b2volts -b1volts;
  if(b4volts  <38 || b4volts > 60)
  {
    b4volts=0.0; // it's floating
  }
  btotalvolts= b1volts+b2volts+b3volts+b4volts;
  
}


void setChargerVoltage(int i)
{
    for (byte b=0; b<numResistorPins; b++) 
    {
      byte state = bitRead(255-i, b);
      digitalWrite(resistorPins[b], state);
    }
}

void switchChargerOn()
{
      digitalWrite(chargerPin, LOW);
}

void switchGTIOn()
{
      digitalWrite(gtiPin, LOW);
}

void switchChargerOff()
{
      digitalWrite(chargerPin, HIGH);
}

void switchGTIOff()
{
      digitalWrite(gtiPin, HIGH);
}

void processCommand(String command)
{
 // Serial.print("command="+command);
}
void displayPowerStats()
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 2);
  // Display static text
  display.println("GRD:" + String(realgridp));
  display.setCursor(0, 22);
  display.println("SLR:" + String(realsolarp));
  display.setCursor(0, 42);

  display.println("HME:" + String(realHomePower));
  display.display();
}

void displayBatteryVolts()
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 2);
  // Display static text
  display.println("B1:" + String(b1volts));
  display.setCursor(0, 22);
  display.println("B2:" + String(b2volts));
  display.setCursor(0, 42);
  display.println("B3:" + String(b3volts));
  display.display();
}

void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
 for(byte i = 0; i < 15; i=i+2)
 { 
   pinMode(i+22,OUTPUT);
 }
 for(byte i = 0; i < 7; i=i+2)
 { 
   pinMode(i+47,OUTPUT);
 }

 switchGTIOn();
 switchChargerOff();

 
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
 
  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  
  grid.voltage(2, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  grid.current(1, 50);
  
  solar.voltage(2, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  solar.current(3, 50);
  
  gti.voltage(2, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  gti.current(5, 50);
  
  charger.voltage(2, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  charger.current(4, 50);
  
  //*********init DISPLAY
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();

   readBatteryVoltages();
  if(b4volts > 6)
  {
    setBaseR(2);
    minChargerVolts=0;
  }
  else
  {
    setBaseR(1);
    minChargerVolts=90;
  }
  chargerVoltage=minChargerVolts;
}


int getRampDown()
{
  int diff = chargerVoltage - minChargerVolts;
  int rampdown= diff/10;
  if(rampdown<1)
  {
    rampdown=1;
  }
  return rampdown;
}

int getRampUp()
{
  int diff = 130-(chargerVoltage - minChargerVolts);
  int ramp= diff/10;
  if(ramp <1)
  {
    ramp=1;
  }
  return ramp;
}
boolean inStableCondition()
{
  boolean retval=false;
  retval = (realgridp < chargeUpperThreshold && realgridp > chargeLowerThreshold);
  retval = (retval || ( btotalvolts < bmaxvolts && btotalvolts >  (bmaxvolts-0.5)));
  retval = retval && (realchargerp < chargerMaxPower );
  retval = retval && (btotalvolts < bmaxvolts);
  retval = retval && (realgridp < chargeUpperThreshold);
  return retval;
}

boolean mustRampDown()
{
  boolean retval=false;
  retval = (realgridp > chargeUpperThreshold);
  retval = retval || (btotalvolts > bmaxvolts);
  retval = retval || (realchargerp > chargerMaxPower );
  retval = retval || (realgtip > 200);
  return retval;
}

void checkBatteriesHaveCharge()
{ 
  int b4check=15; 
    if(b4volts > 6)
    {
      b4check=b4volts;
    }
    if(b1volts <10.9 || b2volts < 10.9 || b3volts < 10.9 || b4check < 10.9)
    {
       batteriesHaveCharge= false;
    }
    else
    {
      batteriesHaveCharge = true;
    }
  
}
void loop() {
  // listen for incoming clients

  
  grid.calcVI(10, 1000);        // Calculate all. No.of half wavelengths (crossings), time-out
  realgridp = (int)grid.realPower;
  appgridp = (int)grid.apparentPower;
  gridVRMS = grid.Vrms;
  gridIRMS = grid.Irms;
  gridPFactor = grid.powerFactor;
  

  solar.calcVI(10, 1000); // Calculate all. No.of half wavelengths (crossings), time-out
  realsolarp = (int)solar.realPower;
  appsolarp = (int)solar.apparentPower;
  solarVRMS = solar.Vrms;
  solarIRMS = solar.Irms;
  solarPFactor = solar.powerFactor;

  //May be able to use DC current sensors directly for these
  charger.calcVI(5, 1000);        // Calculate all. No.of half wavelengths (crossings), time-out
  realchargerp = (int)charger.realPower;
  appchargerp = (int)charger.apparentPower;
  chargerVRMS = charger.Vrms;
  chargerIRMS = charger.Irms;
  chargerPFactor = charger.powerFactor;

  gti.calcVI(5, 1000);        // Calculate all. No.of half wavelengths (crossings), time-out
  realgtip = (int)gti.realPower;
  appgtip = (int)gti.apparentPower;
  gtiVRMS = gti.Vrms;
  gtiIRMS = gti.Irms;
  gtiPFactor = gti.powerFactor;

  realHomePower = realsolarp + realgridp + realgtip -realchargerp;
  appHomePower = appsolarp + appgridp + appgtip - appchargerp;
  
  Serial.println(" chargerp=" + String(realchargerp) + "  gtip=" + String(realgtip) + "   solar:"+String(realsolarp)+"  gridp="+String(realgridp)  );



//******************CONTROL LOOP
//******************CONTROL LOOP
//******************CONTROL LOOP

/*Lithium requires different charger states.
(1) 0.1C constant current
(2) 0.5C constant current
(3) constant voltage (4.2 * 14s) = 58.8V

Resistors in pot divider for battery V will need to change for 44 -> 58.8 V
if boost converter experiments fail , Possible new circuit using LM338/LM317 adjusttable voltage regulators.
New circuit will allow more current , hence more power.*/


if(STATE == TUNING)
{
 
  if(inStableCondition())
  {
    // do nowt
    Serial.println("power stable");
   // switchChargerOn(); // will aready be  on, unless we've just dropped into the -50 -> -200 range.
  }
  else
  {
    if(!mustRampDown())
    { 
      Serial.println("Ramping up");
      //switchGTIOff();
      switchChargerOn();
      chargerVoltage=chargerVoltage+getRampUp();
      batteriesHaveCharge=true;
      beenCharging=true;
    }
    else //  must rampDown
    {
       Serial.println("Ramping DOWN");
       chargerVoltage=chargerVoltage-getRampDown();  
       checkBatteriesHaveCharge();
    }
  }

  
  if(!batteriesHaveCharge)  // if nightTime or batteries in danger switch GTI off
  {
      switchGTIOff();
  }
  

  
  if(chargerVoltage < minChargerVolts) // 100 @ 36V // happens every cycle
  {
    switchChargerOff();

    if(batteriesHaveCharge && beenCharging) // only happens after charging 
    {
       Serial.println("Switching GTI on");
       switchGTIOn();
       beenCharging=false;
    }   
    else if (!batteriesHaveCharge)
    {   
       switchGTIOff(); 
    }
    chargerVoltage=minChargerVolts;
  }
  if(chargerVoltage > 255)
  {
    chargerVoltage=255;
  } 
  String  chargerVoltStr = String(chargerVoltage);
  Serial.print("currentChargerV="+chargerVoltStr);
  setChargerVoltage(chargerVoltage );
  
}

if(STATE == INITIAL)
{
    STATE = TUNING;  
}



//******************CONTROL LOOP 
//******************CONTROL LOOP
//******************CONTROL LOOP

  if(displayCounter < 2)
    displayPowerStats();
  else
    displayBatteryVolts();
  
  //delay(500);

  
  EthernetClient client = server.available();
  if (client) {
   // Serial.println("new client");
    // an http request ends with a blank line
    String command="";
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
          processCommand(command);
          // output the value of each analog input pin
          // send a power summary
          client.println(String(realgridp) + "," + String(appgridp) + "," + String(gridVRMS) + "," + String(gridIRMS) + "," + String(gridPFactor)+","+String(realsolarp) + "," + String(appsolarp) + "," + String(solarVRMS) + "," + String(solarIRMS) + "," + String(solarPFactor)+","+String(realHomePower)+","+String(appHomePower)+","+String(b1volts,2)+","+String(b2volts,2)+","+String(b3volts,2)+","+String(b4volts,2)+","+String(realchargerp)+","+String(realgtip)+",EOT");
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
  displayCounter++;
  if (displayCounter>20) { displayCounter=0;}
  readBatteryVoltages();
}
