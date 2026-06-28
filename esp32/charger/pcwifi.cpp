#include "Arduino.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "espEmonLib.h"
#include "battery.h"
#include "pwmFunctions.h"
#include "pcemon.h"
#include <ESPAsyncWebSrv.h>

extern bool GTIenabled;
extern bool gtiInhibited;
extern bool powerOn;
extern struct tm timeinfo;
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
extern float chargerPower;
extern float gtiPower;
extern int chargerPLimit;
extern int chargerPowerCreep;

// Replace with your network credentials
//const char* ssid = "TP-LINK_73F3";
//const char* password = "DEADBEEF";
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
const long interval=5000;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
AsyncWebServer csvserver(8080);
String csvString="1,2,3,4";

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid, password);
}

void connectToWifi(){
  int wifiwait=0;
  WiFi.disconnect();
  delay(2000);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && wifiwait < 10 ) {
    delay(2000);
    Serial.print(".");
    wifiwait++;
  }

}

void wifiSetup() {


  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  connectToWifi();

  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  csvserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", csvString);
  });
  csvserver.begin();
}

void generateCsvString()
{
  csvString=(String)readBattery()+","+(String)grid.realPower+","+(String)grid.Vrms+","+(String)getTotalResistance()+","+(String)(chargerPower)+","+(String)(gtiPower)+","+(String)timeinfo.tm_hour+","+(String)GTIenabled+","+(String)chargerPLimit+",EOT\n";
}

// ── Pi energy-state polling ──────────────────────────────────────────────────
// Poll http://192.168.1.218/energy-state every ENERGY_POLL_INTERVAL_MS ms.
// Updates gtiInhibited based on Solis battery state.
// Fail-safe: on any error or ok=0, gtiInhibited is cleared (GTI allowed).

static const char*         ENERGY_STATE_URL          = "http://192.168.1.218/energy-state";
static const unsigned long ENERGY_POLL_INTERVAL_MS   = 10000UL; // 10 seconds
static const int           GTI_ALLOW_MAX_SOC_PCT     = 15;      // GTI is allowed when SoC is at or below this threshold.
static const float         GTI_ALLOW_MIN_DISCHARGE_W = -2000.0f; // GTI is allowed when discharge is at or beyond this negative-power threshold.
static unsigned long       lastEnergyPoll            = 0;
static bool                gtiAllowedByDischarge     = false; // latched true when discharge > 2000W; cleared when Solis stops discharging

static void clearGTIInhibit()
{
  if (gtiInhibited)
  {
    gtiInhibited = false;
    if (!powerOn) turnGTIOn();
  }
}

static bool parseIntStrict(String value, int& out)
{
  value.trim();
  if (value.length() == 0) return false;

  char* end = nullptr;
  long parsed = strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0') return false;

  out = (int)parsed;
  return true;
}

static bool parseFloatStrict(String value, float& out)
{
  value.trim();
  if (value.length() == 0) return false;

  char* end = nullptr;
  float parsed = strtof(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0') return false;

  out = parsed;
  return true;
}

void pollEnergyState()
{
  unsigned long now = millis();
  if (now - lastEnergyPoll < ENERGY_POLL_INTERVAL_MS) return;
  lastEnergyPoll = now;

  // Only poll when GTI is enabled by the schedule; inhibit logic is irrelevant otherwise.
  if (!GTIenabled) return;

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("EnergyPoll: WiFi not connected - GTI allowed (fail-safe)");
    clearGTIInhibit();
    return;
  }

  Serial.println("EnergyPoll: GET " + String(ENERGY_STATE_URL) + " (WiFi IP=" + WiFi.localIP().toString() + ")");

  HTTPClient http;
  http.begin(ENERGY_STATE_URL);
  http.setTimeout(500); // 500 ms timeout

  unsigned long t0 = millis();
  int httpCode = http.GET();
  unsigned long elapsed = millis() - t0;

  if (httpCode != HTTP_CODE_OK)
  {
    Serial.println("EnergyPoll: HTTP error " + String(httpCode) + " after " + String(elapsed) + "ms - GTI allowed (fail-safe)");
    http.end();
    clearGTIInhibit();
    return;
  }

  String body = http.getString();
  http.end();

  // Parse plain-text key=value lines
  float solisP   = 0.0f;
  int   solisSoc = 0;
  int   ok       = 0;
  bool  gotP     = false;
  bool  gotSoc   = false;
  bool  gotOk    = false;

  int lineStart = 0;
  while (lineStart < (int)body.length())
  {
    int lineEnd = body.indexOf('\n', lineStart);
    if (lineEnd < 0) lineEnd = (int)body.length();
    String line = body.substring(lineStart, lineEnd);
    line.trim();
    int eq = line.indexOf('=');
    if (eq > 0)
    {
      String key = line.substring(0, eq);
      String val = line.substring(eq + 1);
      if (key == "solis_battery_power")    { gotP   = parseFloatStrict(val, solisP); }
      else if (key == "solis_battery_soc") { gotSoc = parseIntStrict(val, solisSoc); }
      else if (key == "ok")                { gotOk  = parseIntStrict(val, ok); }
    }
    lineStart = lineEnd + 1;
  }

  if (!gotOk || ok != 1 || !gotP || !gotSoc)
  {
    Serial.println("EnergyPoll: parse failed or ok=0 - GTI allowed (fail-safe)");
    clearGTIInhibit();
    return;
  }

  // Allow GTI when the battery SoC is low, or when the inverter is (or was recently)
  // discharging heavily. Once allowed by discharge, stay allowed until Solis stops
  // discharging entirely (power >= 0), so a brief dip below the threshold doesn't
  // immediately re-inhibit the GTI.
  if (solisP <= GTI_ALLOW_MIN_DISCHARGE_W)      gtiAllowedByDischarge = true;
  else if (solisP >= 0.0f)                       gtiAllowedByDischarge = false;
  // else: still discharging but under threshold — keep the latch as-is

  bool shouldAllow = (solisSoc <= GTI_ALLOW_MAX_SOC_PCT || gtiAllowedByDischarge);
  bool newInhibited = !shouldAllow;

  if (newInhibited != gtiInhibited)
  {
    gtiInhibited = newInhibited;
    Serial.println("EnergyPoll: solis_power=" + String(solisP, 1) +
                   " W soc=" + String(solisSoc) +
                   "% dischLatch=" + String(gtiAllowedByDischarge ? "Y" : "N") +
                   " => GTI " + (gtiInhibited ? "INHIBITED" : "ALLOWED"));
    if (gtiInhibited)
    {
      turnGTIOff();
    }
    else if (!powerOn)
    {
      turnGTIOn();
    }
  }
}
// ────────────────────────────────────────────────────────────────────────────

void wifiLoop() {
  pollEnergyState();
  generateCsvString();
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
          int chargerPLimitpos= header.indexOf("&chargerPLimit=");
          int chargerPowerCreeppos= header.indexOf("&chargerPowerCreep=");

 
          Serial.println((String)upperpos);
          Serial.println((String)lowerpos);
          if(upperpos==22)
          {
            String upper=header.substring(upperpos+7,lowerpos);
            upper.trim();
            String lower=header.substring(lowerpos+7,lowerpos+11);
            lower.trim();
            String chargerPLimitstr=header.substring(chargerPLimitpos+15,chargerPLimitpos+19);
            chargerPLimitstr.trim();
            String chargerPowerCreepstr=header.substring(chargerPowerCreeppos+19,chargerPowerCreeppos+20);
            chargerPowerCreepstr.trim();
            Serial.println("power web settings=:"+chargerPLimitstr + ":" + chargerPowerCreepstr+":");
            upperChargerLimit=upper.toInt();
            lowerChargerLimit=lower.toInt();
            chargerPLimit=chargerPLimitstr.toInt();
            chargerPowerCreep=chargerPowerCreepstr.toInt();
          }

          int gridVCpos= header.indexOf("?gridVC=");
          int gridCCpos= header.indexOf("&gridCC=");
          Serial.println((String)gridVCpos);
          Serial.println((String)gridCCpos);
          if(gridVCpos==19)
          {
            String gridVC=header.substring(gridVCpos+8,gridCCpos);
            gridVC.trim();
            String gridCC=header.substring(gridCCpos+8,gridCCpos+13);
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
            client.println("<th>chargerPLimit</th> ");
            client.println("<th>chargerPowerCreep</th> ");
            
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
            client.println("<td>" + (String)chargerPLimit + "</td>");
            client.println("<td>" + (String)chargerPowerCreep + "</td>");

            client.println("</tr></table>");

            client.println("<form action='/updatePowerLimits'>");
              client.println("<label for='upper'>Upper Limit(W):</label>");
              client.println("<input type='text' id='upper' name='upper' value='"+(String)upperChargerLimit+"'><br><br>");
              client.println("<label for='lower'>Lower Limit(W):</label>");
              client.println("<input type='text' id='lower' name='lower' value='"+(String)lowerChargerLimit+"'><br><br>");
              client.println("<label for='chargerPLimit'>chargerPLimit:</label>");
              client.println("<input type='text' id='chargerPLimit' name='chargerPLimit' value='"+(String)chargerPLimit+"'><br><br>");
              client.println("<label for='chargerPowerCreep'>chargerPowerCreep:</label>");
              client.println("<input type='text' id='chargerPowerCreep' name='chargerPowerCreep' value='"+(String)chargerPowerCreep+"'><br><br>");
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