/*
Home energy monitor
To upload with ethernet shield attached we need to reset the power, click upload, then power on 0.5 seconds later

*/

#include <SPI.h>
#include <Ethernet.h>
#include "EmonLib.h"             // Include Emon Library
EnergyMonitor grid;             // Gridmon
EnergyMonitor solar;            // Solarmon
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

void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

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
  grid.voltage(2, 224.26, 1.7);  // Voltage: input pin, calibration, phase_shift
  grid.current(1, 49.9);
  solar.voltage(2, 224.26, 1.7);  // Voltage: input pin, calibration, phase_shift
  solar.current(3, 49.9);
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
  int realgridp = (int)grid.realPower;
  int realsolarp = (int)solar.realPower;
  int realHomePower = (int)(solar.realPower - grid.realPower);
  int appgridp = (int)grid.apparentPower;
  int appsolarp = (int)solar.apparentPower;
  int appHomePower = (int)(solar.apparentPower - grid.apparentPower);

  
  Serial.print("GRID : ");
  grid.calcVI(20, 500);        // Calculate all. No.of half wavelengths (crossings), time-out
  grid.serialprint();
  Serial.print("SOLAR : ");
  solar.calcVI(20, 500); // Calculate all. No.of half wavelengths (crossings), time-out
  solar.serialprint();
 

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
  delay(500);

  
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply

        if (c == '\n' && currentLineIsBlank) {

          // output the value of each analog input pin
          // send a power summary
          client.println(String(grid.realPower) + "," + String(grid.apparentPower) + "," + String(grid.Vrms) + "," + String(grid.Irms) + "," + String(grid.powerFactor)+","+String(solar.realPower) + "," + String(solar.apparentPower) + "," + String(solar.Vrms) + "," + String(solar.Irms) + "," + String(solar.powerFactor)+","+String(realHomePower)+","+String(appHomePower));
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
    Serial.println("client disconnected");
  }

}
