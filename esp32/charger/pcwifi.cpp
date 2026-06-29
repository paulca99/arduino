#include "Arduino.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include "espEmonLib.h"
#include "battery.h"
#include "pwmFunctions.h"
#include "pcemon.h"
#include <ESPAsyncWebSrv.h>

extern bool GTIenabled;
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
extern boolean powerOn;

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

float solisBatteryPower = 0.0;
int solisBatterySoc = 0;
float solisPvTotalPower = 0.0;
bool solisEnergyOk = false;
bool solisChargerAllowed = false;
bool gtiInhibited = false;

static const char *energyStateUrl = "http://192.168.1.218/energy-state";
static const unsigned long energyStatePollIntervalMs = 10000;
static const uint16_t energyStateTimeoutMs = 500;
static const int solisLowSocThreshold = 15;
static const float solisHardDischargeThresholdW = -2000.0;
static const float solisMinPvForChargingW = 100.0;

static unsigned long lastEnergyStatePollMs = 0;
static bool gtiHardDischargeLatched = false;
static bool lastLoggedSolisState = false;
static bool lastLoggedEnergyOk = false;
static bool lastLoggedGtiInhibited = false;
static bool lastLoggedChargerAllowed = false;
static bool lastEnergyFailureLogged = false;
static unsigned long lastEnergyFailureLogMs = 0;

static void logEnergyDecisionState()
{
  if (!lastLoggedSolisState ||
      lastLoggedEnergyOk != solisEnergyOk ||
      lastLoggedGtiInhibited != gtiInhibited ||
      lastLoggedChargerAllowed != solisChargerAllowed)
  {
    Serial.println("Solis energy-state battP=" + (String)solisBatteryPower +
                   " soc=" + (String)solisBatterySoc +
                   " pv=" + (String)solisPvTotalPower +
                   " ok=" + (String)solisEnergyOk +
                   " GTI=" + (gtiInhibited ? String("INHIBITED") : String("ALLOWED")) +
                   " charger=" + (solisChargerAllowed ? String("ALLOWED") : String("BLOCKED")));
    lastLoggedSolisState = true;
    lastLoggedEnergyOk = solisEnergyOk;
    lastLoggedGtiInhibited = gtiInhibited;
    lastLoggedChargerAllowed = solisChargerAllowed;
  }
}

static void applyEnergyPermissions()
{
  bool previousGtiInhibited = gtiInhibited;
  bool gtiAllowed = true;
  if (!solisEnergyOk)
  {
    gtiHardDischargeLatched = false;
  }
  else
  {
    if (solisBatteryPower <= solisHardDischargeThresholdW)
    {
      gtiHardDischargeLatched = true;
    }
    else if (solisBatteryPower >= 0.0)
    {
      gtiHardDischargeLatched = false;
    }
    gtiAllowed = (solisBatterySoc <= solisLowSocThreshold) || gtiHardDischargeLatched;
  }

  gtiInhibited = !gtiAllowed;
  solisChargerAllowed = solisEnergyOk &&
                        (solisPvTotalPower > solisMinPvForChargingW) &&
                        (solisBatteryPower > 0.0);

  if (!powerOn && (gtiInhibited != previousGtiInhibited))
  {
    if (gtiInhibited)
    {
      turnGTIOff();
    }
    else
    {
      turnGTIOn();
    }
  }

  logEnergyDecisionState();
}

static void logEnergyFailure(const String &reason, unsigned long elapsedMs, const String &decisionSummary)
{
  unsigned long now = millis();
  if (!lastEnergyFailureLogged || (now - lastEnergyFailureLogMs) >= 60000)
  {
    Serial.println("energy-state poll failed after " + (String)elapsedMs + "ms: " + reason +
                   " -> " + decisionSummary);
    lastEnergyFailureLogged = true;
    lastEnergyFailureLogMs = now;
  }
}

void pollEnergyStateIfDue()
{
  unsigned long now = millis();
  if (lastEnergyStatePollMs != 0 && (now - lastEnergyStatePollMs) < energyStatePollIntervalMs)
  {
    return;
  }
  lastEnergyStatePollMs = now;

  solisBatteryPower = 0.0;
  solisBatterySoc = 0;
  solisPvTotalPower = 0.0;
  solisEnergyOk = false;

  unsigned long startMs = millis();
  if (WiFi.status() != WL_CONNECTED)
  {
    logEnergyFailure("WiFi disconnected", millis() - startMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
    applyEnergyPermissions();
    return;
  }

  HTTPClient http;
  http.setTimeout(energyStateTimeoutMs);
  if (!http.begin(energyStateUrl))
  {
    logEnergyFailure("HTTP begin failed", millis() - startMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
    applyEnergyPermissions();
    return;
  }

  int httpCode = http.GET();
  unsigned long elapsedMs = millis() - startMs;
  if (httpCode != HTTP_CODE_OK)
  {
    String reason = (httpCode > 0) ? ("HTTP " + String(httpCode)) : http.errorToString(httpCode);
    http.end();
    logEnergyFailure(reason, elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
    applyEnergyPermissions();
    return;
  }

  String payload = http.getString();
  http.end();

  bool haveBatteryPower = false;
  bool haveBatterySoc = false;
  bool havePvTotalPower = false;
  bool haveOk = false;
  bool payloadOk = false;

  int lineStart = 0;
  while (lineStart < payload.length())
  {
    int lineEnd = payload.indexOf('\n', lineStart);
    if (lineEnd == -1)
    {
      lineEnd = payload.length();
    }

    String line = payload.substring(lineStart, lineEnd);
    line.trim();
    if (line.length() > 0)
    {
      int equalsPos = line.indexOf('=');
      if (equalsPos <= 0)
      {
        logEnergyFailure("parse error", elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
        applyEnergyPermissions();
        return;
      }

      String key = line.substring(0, equalsPos);
      String value = line.substring(equalsPos + 1);
      value.trim();

      if (key == "solis_battery_power")
      {
        if (value.length() == 0)
        {
          logEnergyFailure("missing solis_battery_power", elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
          applyEnergyPermissions();
          return;
        }
        solisBatteryPower = value.toFloat();
        haveBatteryPower = true;
      }
      else if (key == "solis_battery_soc")
      {
        if (value.length() == 0)
        {
          logEnergyFailure("missing solis_battery_soc", elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
          applyEnergyPermissions();
          return;
        }
        solisBatterySoc = value.toInt();
        haveBatterySoc = true;
      }
      else if (key == "solis_pv_total_power")
      {
        if (value.length() == 0)
        {
          logEnergyFailure("missing solis_pv_total_power", elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
          applyEnergyPermissions();
          return;
        }
        solisPvTotalPower = value.toFloat();
        havePvTotalPower = true;
      }
      else if (key == "ok")
      {
        if (value.length() == 0)
        {
          logEnergyFailure("missing ok value", elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
          applyEnergyPermissions();
          return;
        }
        payloadOk = (value.toInt() == 1);
        haveOk = true;
      }
    }

    lineStart = lineEnd + 1;
  }

  if (!haveOk)
  {
    logEnergyFailure("missing ok", elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
    applyEnergyPermissions();
    return;
  }

  if (!payloadOk)
  {
    logEnergyFailure("ok=0", elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
    applyEnergyPermissions();
    return;
  }

  if (!haveBatteryPower || !haveBatterySoc)
  {
    logEnergyFailure("missing GTI keys", elapsedMs, "GTI ALLOWED fail-safe, AC charger BLOCKED fail-safe");
    applyEnergyPermissions();
    return;
  }

  if (!havePvTotalPower)
  {
    logEnergyFailure("missing charger PV key", elapsedMs, "GTI decision kept from valid Solis data, AC charger BLOCKED fail-safe");
  }
  else
  {
    lastEnergyFailureLogged = false;
  }

  solisEnergyOk = true;
  solisChargerAllowed = havePvTotalPower &&
                        (solisPvTotalPower > solisMinPvForChargingW) &&
                        (solisBatteryPower > 0.0);
  applyEnergyPermissions();
}

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
void wifiLoop() {
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