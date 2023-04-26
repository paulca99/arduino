
#include "Arduino.h"
#include <WiFi.h>
#include "espEmonLib.h"
#include "battery.h"


extern EnergyMonitor grid;
extern EnergyMonitor charger;

// Replace with your network credentials
const char* ssid = "TP-LINK_73F3";
const char* password = "DEADBEEF";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void wifiSetup() {
  Serial.begin(115200);
  Serial.print(grid.Irms);
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void wifiLoop() {
  WiFiClient client = server.available();  // Listen for incoming clients

  if (client) {  // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");                                             // print a message out in the serial port
    String currentLine = "";                                                   // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {  // if there's bytes to read from the client,
        char c = client.read();  // read a byte, then
        Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            client.println("<!DOCTYPE html><html><head><style>table, th, td {border: 1px solid black;border-collapse: collapse;}</style></head><body><h2>Power Stats</h2><table style='width:100%'><tr><th>GridP</th><th>GridV</th><th>GridPF</th><th>ChargerIrms</th><th>BatteryV</th></tr><tr>");

            client.println("<td>" + (String)grid.realPower + "</td>");
            client.println("<td>" + (String)grid.Vrms + "</td>");
            client.println("<td>" + (String)grid.powerFactor + "</td>");
            client.println("<td>" + (String)charger.Irms + "</td>");
            client.println("<td>" + (String)readBattery() + "</td>");

            client.println("</tr></table></body></html>");
            client.println();
            // Break out of the while loop
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
