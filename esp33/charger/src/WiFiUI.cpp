#include "Arduino.h"
#include <WiFi.h>
#include "WiFiUI.h"
#include "Config.h"
#include "EmonLib.h"
#include "Battery.h"
#include "PwmController.h"
#include "Emon.h"
#include "TimeSchedule.h"
#include <ESPAsyncWebSrv.h>

// ============================================================================
// WiFi and Web Interface Module Implementation
// ============================================================================

// Web servers
static WiFiServer server(HTTP_PORT);
static AsyncWebServer csvServer(CSV_PORT);

// HTTP request tracking
static String header;
static unsigned long currentTime = millis();
static unsigned long previousTime = 0;

// CSV data string
static String csvString = "1,2,3,4";

// WiFi event handlers
static void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

static void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToWifi()
{
  int wifiwait = 0;
  WiFi.disconnect();
  delay(2000);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED && wifiwait < 10)
  {
    delay(2000);
    Serial.print(".");
    wifiwait++;
  }
}

void wifiSetup()
{
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  connectToWifi();

  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  
  // CSV endpoint on port 8080
  csvServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", csvString);
  });
  csvServer.begin();
}

static void generateCsvString()
{
  csvString = String(readBattery()) + "," +
              String(grid.realPower) + "," +
              String(grid.Vrms) + "," +
              String(getTotalResistance()) + "," +
              String(chargerPower) + "," +
              String(gtiPower) + "," +
              String(timeinfo.tm_hour) + "," +
              String(GTIenabled) + ",EOT\n";
}

void wifiLoop()
{
  generateCsvString();
  WiFiClient client = server.available(); // Listen for incoming clients
  if (client)
  { // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");
    String currentLine = ""; // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= HTTP_TIMEOUT_MS)
    { // loop while the client's connected
      currentTime = millis();
      if (client.available())
      { // if there's bytes to read from the client,
        char c = client.read(); // read a byte, then
        header += c;
        if (c == '\n')
        { // if the byte is a newline character
          Serial.println(header);
          
          // Parse upper/lower power limit updates
          int upperpos = header.indexOf("?upper=");
          int lowerpos = header.indexOf("&lower=");
          Serial.println(String(upperpos));
          Serial.println(String(lowerpos));
          if (upperpos == 22)
          {
            String upper = header.substring(upperpos + 7, lowerpos);
            upper.trim();
            String lower = header.substring(lowerpos + 7, lowerpos + 11);
            lower.trim();
            upperChargerLimit = upper.toInt();
            lowerChargerLimit = lower.toInt();
          }

          // Parse grid calibration updates
          int gridVCpos = header.indexOf("?gridVC=");
          int gridCCpos = header.indexOf("&gridCC=");
          Serial.println(String(gridVCpos));
          Serial.println(String(gridCCpos));
          if (gridVCpos == 19)
          {
            String gridVC = header.substring(gridVCpos + 8, gridCCpos);
            gridVC.trim();
            String gridCC = header.substring(gridCCpos + 8, gridCCpos + 13);
            gridCC.trim();
            gridVoltageCalibration = gridVC.toFloat();
            gridCurrentCalibration = gridCC.toFloat();
            setupEmon();
          }

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0)
          {
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

            client.println("<td>" + String(grid.realPower) + "</td>");
            client.println("<td>" + String(grid.Vrms) + "</td>");
            client.println("<td>" + String(grid.powerFactor) + "</td>");
            client.println("<td>" + String(charger.Irms) + "</td>");
            client.println("<td>" + String(readBattery()) + "</td>");
            client.println("<td>" + String(psuResistanceValues[0]) + "</td>");
            client.println("<td>" + String(psuResistanceValues[1]) + "</td>");
            client.println("<td>" + String(psuResistanceValues[2]) + "</td>");
            client.println("<td>" + String(psuResistanceValues[3]) + "</td>");
            client.println("<td>" + String(psuResistanceValues[4]) + "</td>");

            client.println("</tr></table>");

            client.println("<form action='/updatePowerLimits'>");
            client.println("<label for='upper'>Upper Limit(W):</label>");
            client.println("<input type='text' id='upper' name='upper' value='" + String(upperChargerLimit) + "'><br><br>");
            client.println("<label for='lower'>Lower Limit(W):</label>");
            client.println("<input type='text' id='lower' name='lower' value='" + String(lowerChargerLimit) + "'><br><br>");
            client.println("<input type='submit' value='Submit'>");
            client.println("</form>");
            client.println("");
            client.println("</body></html>");
            client.println();

            // Break out of the while loop
            break;
          }
          else
          { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        }
        else if (c != '\r')
        {                     // if you got anything else but a carriage return character,
          currentLine += c; // add it to the end of the currentLine
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
