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

int chargerVoltage=100;
int chargeUpperThreshold=-50;
int chargeLowerThreshold=chargeUpperThreshold-150;
//maxPower is actually 50 % for some weird reason.
int chargerMaxPower=300;
int chargerPin=40;
int gtiPin=46;

float b1volts;
float b2volts;
float b3volts;
float b4volts;
float btotalvolts;
float bmaxvolts=42.0;

const byte numResistorPins = 8;
byte resistorPins[] = {22, 24, 26, 28, 30, 32, 34, 36};
byte resistorBasePins[] = {42,44};

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
    digitalWrite(42, 1);digitalWrite(44, 1);
  }
  if(i == 1){
    digitalWrite(42, 0);digitalWrite(44, 1);
  }
  if(i == 2){
    digitalWrite(42, 1);digitalWrite(44, 0);
  }
  if(i == 3){
    digitalWrite(42, 0);digitalWrite(44, 0);
  }
}

void readBatteryVoltages()
{
    Serial.println("");
  float v1 = readApin(14);  // read the input pin
  Serial.println("v1= " + String (v1,2));
  b1volts = v1 * 4.042;
  float v2 = readApin(13);  // read the input pin
  Serial.println("v2= " + String (v2,2));  
  b2volts = (v2 * 7.67826) - b1volts;
  float v3 = readApin(12);  // read the input pin
  Serial.println("v3= " + String (v3,2));
  b3volts = (v3 * 10.948) - b2volts -b1volts;
  float v4 = readApin(11);  // read the input pin
  Serial.println("v4= " + String (v4,2));
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
      digitalWrite(chargerPin, 0);
}

void switchGTIOn()
{
      digitalWrite(gtiPin, 0);
}

void switchChargerOff()
{
      digitalWrite(chargerPin, 1);
}

void switchGTIOff()
{
      digitalWrite(gtiPin, 1);
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
 for(byte i = 0; i < 9; i=i+2)
 { 
   pinMode(i+40,OUTPUT);
 }

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

 /*delay(5000);
 setChargerVoltage(0);
 for(int i=0; i<4;i++)
 {
   setBaseR(i);
   delay(10000);
 }
*/
 
  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  
  grid.voltage(2, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  grid.current(1, 50);
  
  solar.voltage(2, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  solar.current(3, 50);
  
  gti.voltage(2, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  gti.current(4, 50);
  
  charger.voltage(2, 155.16, 2.40);  // Voltage: input pin, calibration, phase_shift
  charger.current(5, 50);
  
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
  delay(2000);
}



void loop() {
  // listen for incoming clients
  readBatteryVoltages();
  if(b4volts > 6)
  {
    setBaseR(2);
  }
  else
  {
    setBaseR(1);
  }
  
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

  charger.calcVI(10, 1000);        // Calculate all. No.of half wavelengths (crossings), time-out
  realchargerp = (int)charger.realPower;
  appchargerp = (int)charger.apparentPower;
  chargerVRMS = charger.Vrms;
  chargerIRMS = charger.Irms;
  chargerPFactor = charger.powerFactor;

  gti.calcVI(10, 1000);        // Calculate all. No.of half wavelengths (crossings), time-out
  realgtip = (int)gti.realPower;
  appgtip = (int)gti.apparentPower;
  gtiVRMS = gti.Vrms;
  gtiIRMS = gti.Irms;
  gtiPFactor = gti.powerFactor;

  realHomePower = realsolarp + realgridp ;
  appHomePower = appsolarp + appgridp;
  
  Serial.println(" chargerp=" + String(realchargerp) + "  gtip=" + String(realgtip) + "   solar:"+String(realsolarp)+"  gridp="+String(realgridp)  );



//******************CONTROL LOOP
//******************CONTROL LOOP
//******************CONTROL LOOP

if(STATE == TUNING)

{
  if(realgridp < chargeUpperThreshold && realgridp > chargeLowerThreshold)
  {
    Serial.println("power stable");
  }
  else
  {
    if((realgridp < chargeLowerThreshold) && (bmaxvolts > btotalvolts))
    { 
      Serial.println("Switching on charger");
      switchChargerOn();
      switchGTIOff();
      chargerVoltage++;
      if (realchargerp > chargerMaxPower ) // DO NOT Exceed 22A output
      {
        chargerVoltage=chargerVoltage-5;
      }
    }
    else
    {
       chargerVoltage=chargerVoltage-5;  
    }
  }
  
  if(chargerVoltage < 100)
  {
    switchChargerOff();
    switchGTIOn();
    chargerVoltage=100;
  }
  if(chargerVoltage > 255)
  {
    chargerVoltage=255;
  } 
  String  chargerVoltStr = String(chargerVoltage);
  Serial.print("currentV="+chargerVoltStr);
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
