/*
 * web.cpp — WiFi connection, HTML status UI (port 80) and JSON API (port 8080)
 *
 * Port 80:  Full status dashboard with live-updating cards and a
 *           configuration form (upper/lower limits, voltage limit, etc.)
 * Port 8080: /api/status — JSON containing charger state + Solis data
 */
#include "web.h"
#include "secrets.h"
#include "control.h"
#include "measurements.h"
#include "solis_modbus.h"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer serverUI(80);
static AsyncWebServer serverAPI(8080);

// ── HTML Page ─────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Charger2 Dashboard</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:Arial,sans-serif;background:#111;color:#eee;padding:16px}
    h1{text-align:center;margin-bottom:16px;font-size:1.4em}
    h2{margin:20px 0 10px;font-size:1.1em;color:#aaa;border-bottom:1px solid #333;padding-bottom:4px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px}
    .card{background:#1e1e1e;padding:14px;border-radius:8px;box-shadow:0 0 8px #000}
    .label{font-size:.8em;color:#888;margin-bottom:4px}
    .value{font-size:1.3em;font-weight:bold}
    .value.on{color:#4caf50}.value.off{color:#f44336}
    .badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:.85em;margin-left:6px}
    .badge-green{background:#1b5e20;color:#a5d6a7}
    .badge-red{background:#b71c1c;color:#ef9a9a}
    .badge-grey{background:#333;color:#aaa}
    form{background:#1e1e1e;padding:16px;border-radius:8px;margin-top:8px}
    .form-row{display:flex;flex-wrap:wrap;gap:12px;margin-bottom:10px}
    .form-field{display:flex;flex-direction:column;min-width:160px}
    .form-field label{font-size:.8em;color:#888;margin-bottom:4px}
    .form-field input{background:#2a2a2a;color:#eee;border:1px solid #444;border-radius:4px;padding:6px 8px;font-size:.95em}
    input[type=submit]{background:#1565c0;color:#fff;border:none;border-radius:4px;padding:8px 20px;cursor:pointer;font-size:1em;margin-top:4px}
    input[type=submit]:hover{background:#1976d2}
    #age{font-size:.75em;color:#666;text-align:right;margin-top:8px}
    .stale{color:#f44336!important}
  </style>
</head>
<body>
  <h1>&#9889; Charger2 Dashboard</h1>

  <h2>Charger Status</h2>
  <div class="grid">
    <div class="card">
      <div class="label">Charger</div>
      <div class="value" id="chargerState">--</div>
    </div>
    <div class="card">
      <div class="label">Grid Power</div>
      <div class="value" id="gridPower">-- W</div>
    </div>
    <div class="card">
      <div class="label">Grid Voltage</div>
      <div class="value" id="gridVoltage">-- V</div>
    </div>
    <div class="card">
      <div class="label">Grid PF</div>
      <div class="value" id="gridPF">--</div>
    </div>
    <div class="card">
      <div class="label">Charger Power</div>
      <div class="value" id="chargerPower">-- W</div>
    </div>
    <div class="card">
      <div class="label">Charger Irms</div>
      <div class="value" id="chargerIrms">-- A</div>
    </div>
    <div class="card">
      <div class="label">GTI Power</div>
      <div class="value" id="gtiPower">-- W</div>
    </div>
    <div class="card">
      <div class="label">Local Battery</div>
      <div class="value" id="localBatt">-- V</div>
    </div>
    <div class="card">
      <div class="label">Total Resistance</div>
      <div class="value" id="totalR">--</div>
    </div>
  </div>

  <h2>Solis Inverter</h2>
  <div class="grid">
    <div class="card">
      <div class="label">PV1</div>
      <div class="value" id="pv1">-- V / -- A</div>
    </div>
    <div class="card">
      <div class="label">PV2</div>
      <div class="value" id="pv2">-- V / -- A</div>
    </div>
    <div class="card">
      <div class="label">Solis Battery</div>
      <div class="value" id="solisBatt">-- V / -- A / -- W</div>
    </div>
    <div class="card">
      <div class="label">Solis Grid</div>
      <div class="value" id="solisGrid">-- V / -- A / -- W</div>
    </div>
    <div class="card">
      <div class="label">Frequency / PF</div>
      <div class="value" id="freqpf">-- Hz / --</div>
    </div>
    <div class="card">
      <div class="label">Inverter Temp</div>
      <div class="value" id="invTemp">-- &deg;C</div>
    </div>
    <div class="card">
      <div class="label">Daily / Total Energy</div>
      <div class="value" id="energy">-- / -- kWh</div>
    </div>
  </div>

  <h2>Configuration</h2>
  <form id="configForm" method="get" action="/">
    <div class="form-row">
      <div class="form-field">
        <label>Upper Limit (W)</label>
        <input type="number" name="upper" id="fUpper" value="">
      </div>
      <div class="form-field">
        <label>Lower Limit (W)</label>
        <input type="number" name="lower" id="fLower" value="">
      </div>
      <div class="form-field">
        <label>Voltage Limit (V)</label>
        <input type="number" step="0.1" name="voltageLimit" id="fVLimit" value="">
      </div>
      <div class="form-field">
        <label>Charger P Limit (W)</label>
        <input type="number" name="chargerPLimit" id="fPLimit" value="">
      </div>
      <div class="form-field">
        <label>Power Creep (0/1)</label>
        <input type="number" name="powerCreep" id="fCreep" value="">
      </div>
    </div>
    <input type="submit" value="Update Settings">
  </form>

  <div id="age">Last update: --</div>

  <script>
    const API = 'http://' + location.hostname + ':8080/api/status';

    function fmt(v, dp) { return (v != null) ? v.toFixed(dp) : '--'; }

    function colorPower(id, w) {
      const el = document.getElementById(id);
      el.textContent = fmt(w, 0) + ' W';
      el.className = 'value ' + (w < 0 ? 'on' : (w > 50 ? 'off' : ''));
    }

    async function fetchData() {
      try {
        const res = await fetch(API);
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const d = await res.json();
        const c = d.charger;
        const g = d.grid;
        const s = d.solis;

        // Charger section
        const stateEl = document.getElementById('chargerState');
        stateEl.textContent = c.powerOn ? 'ON' + (c.afterburnerOn ? ' + AB' : '') : 'OFF';
        stateEl.className   = 'value ' + (c.powerOn ? 'on' : 'off');

        colorPower('gridPower', g.realPower);
        document.getElementById('gridVoltage').textContent = fmt(g.Vrms, 1) + ' V';
        document.getElementById('gridPF').textContent      = fmt(g.powerFactor, 3);
        document.getElementById('chargerPower').textContent= fmt(c.chargerPower, 0) + ' W';
        document.getElementById('chargerIrms').textContent = fmt(g.Irms, 2) + ' A';
        colorPower('gtiPower', c.gtiPower);
        document.getElementById('localBatt').textContent   = fmt(c.localBatteryV, 2) + ' V';
        document.getElementById('totalR').textContent      = c.totalResistance;

        // Solis section
        if (s.valid) {
          document.getElementById('pv1').textContent  = fmt(s.pv1Voltage,1)+' V / '+fmt(s.pv1Current,1)+' A';
          document.getElementById('pv2').textContent  = fmt(s.pv2Voltage,1)+' V / '+fmt(s.pv2Current,1)+' A';
          document.getElementById('solisBatt').textContent =
            fmt(s.batteryVoltage,1)+' V / '+fmt(s.batteryCurrent,1)+' A / '+fmt(s.batteryPower,0)+' W';
          document.getElementById('solisGrid').textContent =
            fmt(s.gridVoltage,1)+' V / '+fmt(s.gridCurrent,1)+' A / '+fmt(s.gridPower,0)+' W';
          document.getElementById('freqpf').textContent  = fmt(s.frequency,2)+' Hz / '+fmt(s.powerFactor,3);
          document.getElementById('invTemp').textContent = fmt(s.inverterTemp,1)+' \u00b0C';
          document.getElementById('energy').textContent  =
            fmt(s.dailyEnergy,2)+' / '+fmt(s.totalEnergy,1)+' kWh';
        }

        // Populate config form on first fill
        if (!document.getElementById('fUpper').dataset.filled) {
          document.getElementById('fUpper').value  = c.upperLimit;
          document.getElementById('fLower').value  = c.lowerLimit;
          document.getElementById('fVLimit').value = c.voltageLimit;
          document.getElementById('fPLimit').value = c.chargerPLimit;
          document.getElementById('fCreep').value  = c.chargerPowerCreep;
          document.getElementById('fUpper').dataset.filled = '1';
        }

        // Data age
        const ageEl = document.getElementById('age');
        const ageS  = (s.ageMs != null && s.valid) ? (s.ageMs / 1000).toFixed(1) + ' s ago' : '?';
        ageEl.textContent = 'Last Solis update: ' + ageS;
        ageEl.className   = (s.ageMs > 5000) ? 'stale' : '';

      } catch(e) {
        console.warn('Fetch error', e);
      }
    }

    setInterval(fetchData, 2000);
    fetchData();
  </script>
</body>
</html>
)HTML";

// ── JSON builder ──────────────────────────────────────────────────
static String buildJson() {
  SolisData s = getSolisSnapshot();

  String j = "{";

  // -- charger object --
  j += "\"charger\":{";
  j += "\"powerOn\":"         + String(powerOn        ? "true" : "false") + ",";
  j += "\"afterburnerOn\":"   + String(afterburnerOn  ? "true" : "false") + ",";
  j += "\"GTIenabled\":"      + String(GTIenabled      ? "true" : "false") + ",";
  j += "\"chargerPower\":"    + String(chargerPower,   1) + ",";
  j += "\"gtiPower\":"        + String(gtiPower,       1) + ",";
  j += "\"localBatteryV\":"   + String(localBatteryVoltage, 2) + ",";
  j += "\"upperLimit\":"      + String(upperChargerLimit) + ",";
  j += "\"lowerLimit\":"      + String(lowerChargerLimit) + ",";
  j += "\"voltageLimit\":"    + String(voltageLimit,   1) + ",";
  j += "\"chargerPLimit\":"   + String(chargerPLimit) + ",";
  j += "\"chargerPowerCreep\":" + String(chargerPowerCreep) + ",";
  j += "\"totalResistance\":" + String(getTotalResistance()) + ",";
  j += "\"psuValues\":[";
  for (int i = 0; i < PSU_COUNT; i++) {
    j += String(psu_resistance_values[i]);
    if (i < PSU_COUNT - 1) j += ",";
  }
  j += "]},";

  // -- grid object --
  j += "\"grid\":{";
  j += "\"realPower\":"   + String(emonGrid.realPower,   1) + ",";
  j += "\"Vrms\":"        + String(emonGrid.Vrms,        1) + ",";
  j += "\"Irms\":"        + String(emonGrid.Irms,        2) + ",";
  j += "\"powerFactor\":" + String(emonGrid.powerFactor, 3);
  j += "},";

  // -- solis object --
  j += "\"solis\":{";
  j += "\"valid\":"          + String(s.valid ? "true" : "false") + ",";
  j += "\"ageMs\":"          + String(s.valid ? (long)(millis() - s.lastUpdateMs) : 0L) + ",";
  j += "\"pv1Voltage\":"     + String(s.pv1Voltage,    1) + ",";
  j += "\"pv1Current\":"     + String(s.pv1Current,    1) + ",";
  j += "\"pv2Voltage\":"     + String(s.pv2Voltage,    1) + ",";
  j += "\"pv2Current\":"     + String(s.pv2Current,    1) + ",";
  j += "\"batteryVoltage\":" + String(s.batteryVoltage,1) + ",";
  j += "\"batteryCurrent\":" + String(s.batteryCurrent,1) + ",";
  j += "\"batteryPower\":"   + String(s.batteryPower,  0) + ",";
  j += "\"gridVoltage\":"    + String(s.gridVoltage,   1) + ",";
  j += "\"gridCurrent\":"    + String(s.gridCurrent,   1) + ",";
  j += "\"gridPower\":"      + String(s.gridPower,     0) + ",";
  j += "\"frequency\":"      + String(s.frequency,     2) + ",";
  j += "\"powerFactor\":"    + String(s.powerFactor,   3) + ",";
  j += "\"inverterTemp\":"   + String(s.inverterTemp,  1) + ",";
  j += "\"totalPower\":"     + String(s.totalPower,    0) + ",";
  j += "\"dailyEnergy\":"    + String(s.dailyEnergy,   2) + ",";
  j += "\"totalEnergy\":"    + String(s.totalEnergy,   1);
  j += "}";

  j += "}";
  return j;
}

// ── WiFi connect (with auto-reconnect handler) ─────────────────────
static void onWiFiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.printf("[WiFi] Disconnected (reason %d), reconnecting…\n",
                info.wifi_sta_disconnected.reason);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

static void onWiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print("[WiFi] Connected — IP: ");
  Serial.println(WiFi.localIP());
}

// ── Config parameter parser ───────────────────────────────────────
static void applyConfigParams(AsyncWebServerRequest* req) {
  if (req->hasParam("upper"))       upperChargerLimit = req->getParam("upper")->value().toInt();
  if (req->hasParam("lower"))       lowerChargerLimit = req->getParam("lower")->value().toInt();
  if (req->hasParam("voltageLimit"))voltageLimit      = req->getParam("voltageLimit")->value().toFloat();
  if (req->hasParam("chargerPLimit"))chargerPLimit    = req->getParam("chargerPLimit")->value().toInt();
  if (req->hasParam("powerCreep"))  chargerPowerCreep = req->getParam("powerCreep")->value().toInt();

  Serial.printf("[Web] Config: upper=%d lower=%d vLimit=%.1f pLimit=%d creep=%d\n",
                upperChargerLimit, lowerChargerLimit, voltageLimit,
                chargerPLimit, chargerPowerCreep);
}

// ── Public setup ──────────────────────────────────────────────────
void wifiSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWiFiDisconnect, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent(onWiFiGotIP,      WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[WiFi] Connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Failed to connect — will retry via event handler");
  }

  // ── Port 80: HTML UI ──────────────────────────────────────────
  serverUI.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    // Apply any config parameters first
    applyConfigParams(req);
    req->send_P(200, "text/html", INDEX_HTML);
  });
  serverUI.begin();
  Serial.println("[Web] UI server started on port 80");

  // ── Port 8080: JSON API ───────────────────────────────────────
  serverAPI.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    String json = buildJson();
    AsyncWebServerResponse* resp = req->beginResponse(200, "application/json", json);
    resp->addHeader("Access-Control-Allow-Origin", "*");
    req->send(resp);
  });
  // Legacy alias kept for compatibility
  serverAPI.on("/api/solis", HTTP_GET, [](AsyncWebServerRequest* req) {
    SolisData s = getSolisSnapshot();
    String json = "{";
    json += "\"pv1Voltage\":"     + String(s.pv1Voltage,    1) + ",";
    json += "\"pv1Current\":"     + String(s.pv1Current,    1) + ",";
    json += "\"pv2Voltage\":"     + String(s.pv2Voltage,    1) + ",";
    json += "\"pv2Current\":"     + String(s.pv2Current,    1) + ",";
    json += "\"batteryVoltage\":" + String(s.batteryVoltage,1) + ",";
    json += "\"batteryCurrent\":" + String(s.batteryCurrent,1) + ",";
    json += "\"batteryPower\":"   + String(s.batteryPower,  0) + ",";
    json += "\"gridVoltage\":"    + String(s.gridVoltage,   1) + ",";
    json += "\"gridCurrent\":"    + String(s.gridCurrent,   1) + ",";
    json += "\"gridPower\":"      + String(s.gridPower,     0) + ",";
    json += "\"frequency\":"      + String(s.frequency,     2) + ",";
    json += "\"powerFactor\":"    + String(s.powerFactor,   3) + ",";
    json += "\"inverterTemp\":"   + String(s.inverterTemp,  1) + ",";
    json += "\"totalPower\":"     + String(s.totalPower,    0) + ",";
    json += "\"dailyEnergy\":"    + String(s.dailyEnergy,   2) + ",";
    json += "\"totalEnergy\":"    + String(s.totalEnergy,   1);
    json += "}";
    AsyncWebServerResponse* resp = req->beginResponse(200, "application/json", json);
    resp->addHeader("Access-Control-Allow-Origin", "*");
    req->send(resp);
  });
  serverAPI.begin();
  Serial.println("[Web] JSON API started on port 8080");
}
