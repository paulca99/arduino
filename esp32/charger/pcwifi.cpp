#include "Arduino.h"
#include <WiFi.h>
#include "espEmonLib.h"
#include "battery.h"
#include "pwmFunctions.h"
#include "pcemon.h"

extern EnergyMonitor grid;
extern EnergyMonitor charger;
extern int psu_resistance_values[];
extern int upperChargerLimit;
extern int lowerChargerLimit;
extern float gridVoltageCalibration;
extern float gridPhaseOffset;
extern float gridCurrentCalibration;
extern float chargerVoltageCalibration;
extern float chargerPhaseOffset;
extern float chargerCurrentCalibration;

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
        //Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          Serial.println(header);
          int upperpos= header.indexOf("?upper=");
          int lowerpos= header.indexOf("&lower=");
          Serial.println((String)upperpos);
          Serial.println((String)lowerpos);
          if(upperpos==21)
          {
            String upper=header.substring(upperpos+7,lowerpos);
            upper.trim();
            String lower=header.substring(lowerpos+7,lowerpos+11);
            lower.trim();
            upperChargerLimit=upper.toInt();
            lowerChargerLimit=lower.toInt();
          }

          int gridVCpos= header.indexOf("?gridVC=");
          int gridCCpos= header.indexOf("&gridCC=");
          Serial.println((String)gridVCpos);
          Serial.println((String)gridCCpos);
          if(gridVCpos==18)
          {
            String gridVC=header.substring(gridVCpos+7,gridCCpos);
            gridVC.trim();
            String gridCC=header.substring(gridCCpos+7,gridCCpos+11);
            gridCC.trim();
            gridVoltageCalibration=gridVC.toFloat();
            gridCurrentCalibration=gridCC.toFloat();
            setupEmon();
          }

          
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            client.println("<!DOCTYPE html><html><head><style>table, th, td {border: 1px solid black;border-collapse: collapse;}</style></head><body><h2>Power Stats</h2><table style='width:100%'><tr>");
            client.println("<th>GridP</th>");
            client.println("<th>GridV</th>");
            client.println("<th>GridPF</th>");
            client.println("<th>ChargerIrms</th>");
            client.println("<th>BatteryV</th> ");
            client.println("<th>PSU1</th> ");
            client.println("<th>PSU2</th> ");
            client.println("<th>PSU3</th> ");
            client.println("<th>PSU4</th> ");
            client.println("<th>PSU5</th> ");
            client.println("       </tr><tr>");

            client.println("<td>" + (String)grid.realPower + "</td>");
            client.println("<td>" + (String)grid.Vrms + "</td>");
            client.println("<td>" + (String)grid.powerFactor + "</td>");
            client.println("<td>" + (String)charger.Irms + "</td>");
            client.println("<td>" + (String)readBattery() + "</td>");
            client.println("<td>" + (String)psu_resistance_values[0] + "</td>");
            client.println("<td>" + (String)psu_resistance_values[1] + "</td>");
            client.println("<td>" + (String)psu_resistance_values[2] + "</td>");
            client.println("<td>" + (String)psu_resistance_values[3] + "</td>");
            client.println("<td>" + (String)psu_resistance_values[4] + "</td>");


            client.println("</tr></table>");

            client.println("<form action='/updatePowerLimits'>");
              client.println("<label for='upper'>Upper Limit(W):</label>");
              client.println("<input type='text' id='upper' name='upper' value='"+(String)upperChargerLimit+"'><br><br>");
              client.println("<label for='lower'>Lower Limit(W):</label>");
              client.println("<input type='text' id='lower' name='lower' value='"+(String)lowerChargerLimit+"'><br><br>");
              client.println("<input type='submit' value='Submit'>");
            client.println("</form>");

            client.println("<form action='/updateEmonVars'>");
              client.println("<label for='gridVC'>Grid Voltage Calibration:</label>");
              client.println("<input type='text' id='gridVC' name='gridVC' value='"+(String)gridVoltageCalibration+"'><br><br>");
              client.println("<label for='gridCC'>Grid Current Calibration:</label>");
              client.println("<input type='text' id='gridCC' name='gridCC' value='"+(String)gridCurrentCalibration+"'><br><br>");
              client.println("<input type='submit' value='Submit'>");
            client.println("</form>");

            client.println("");
            client.println("</body></html>");
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