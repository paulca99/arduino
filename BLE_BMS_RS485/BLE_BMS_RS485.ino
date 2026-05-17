#include <NimBLEDevice.h>
#include <bms2.h>
#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <float.h>
#include <string.h>

#if __has_include("secrets.h")
#include "secrets.h"
#elif __has_include("secrets_template.h")
#include "secrets_template.h"
#endif

// -----------------------------------------------------------------------
// Combined BLE BMS + Pylontech CAN + Solis RS485 monitor sketch.
//
// Coexistence tradeoffs (single ESP32):
// - BLE + Wi-Fi share the same radio, so throughput/latency can vary.
// - CAN and RS485 are independent wired buses, but RS485 Modbus reads can
//   block for timeout windows; this is why RS485 polling runs in its own task.
// - Keep CAN send timing conservative and poll intervals moderate to avoid
//   starving BLE processing.
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// BLE BMS configuration
// -----------------------------------------------------------------------
#define SERVICE_UUID   "ff00"
#define RX_UUID        "ff01"
#define TX_UUID        "ff02"

// -----------------------------------------------------------------------
// Pin assignment plan (safe defaults, no overlap)
// -----------------------------------------------------------------------
// CAN transceiver pins (TWAI)
#define CAN_TX_PIN     GPIO_NUM_21
#define CAN_RX_PIN     GPIO_NUM_19
// RS485 transceiver pins (UART2)
#define RS485_RX_PIN   16
#define RS485_TX_PIN   17

#define CAN_INTERVAL_MS             100
#define LOG_INTERVAL_MS            5000
#define BLE_TIMEOUT_MS  (3UL * 60UL * 1000UL)
#define BLE_RETRY_INTERVAL_MS      5000
#define BLE_RETRY_LOG_INTERVAL_MS  1000

#define WIFI_CONNECT_TIMEOUT_MS      10000

// RS485 poll timing; tune here if inverter responses are slow/noisy.
#define MODBUS_TIMEOUT_MS              180
#define POLL_INTERVAL_MS              1000
#define INTER_REGISTER_DELAY_MS         15
#define MONITOR_MUTEX_TIMEOUT_MS       100
#define SERIAL_SETTLE_DELAY_MS        1200
#define SOLIS_SLAVE_ID                   1

#ifndef BMS_MAC 
#define BMS_MAC "a5:c2:37:51:85:89"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "TP-LINK_73F3"
#endif

#ifndef WIFI_PASSWORD_PRIMARY
#define WIFI_PASSWORD_PRIMARY "DEADBEEF"
#endif

#ifndef WIFI_PASSWORD_FALLBACK
#define WIFI_PASSWORD_FALLBACK "deadbeef"
#endif

// JBD BLE command frame terminator byte.
static const uint8_t BMS_FRAME_TERMINATOR = 0x77;

// BLE connection parameter defaults:
// interval min/max 6 units (6 * 1.25ms = 7.5ms), latency 0 events,
// supervision timeout 51 units (51 * 10ms = 510ms).
static const uint16_t BLE_CONN_INTERVAL_MIN = 6;
static const uint16_t BLE_CONN_INTERVAL_MAX = 6;
static const uint16_t BLE_CONN_LATENCY = 0;
static const uint16_t BLE_CONN_TIMEOUT = 51;

// -----------------------------------------------------------------------
// BLE globals
// -----------------------------------------------------------------------
static NimBLEClient*               pClient  = nullptr;
static NimBLERemoteCharacteristic* pTxChar  = nullptr;
static NimBLERemoteCharacteristic* pRxChar  = nullptr;
static NimBLEAddress               bmsMacAddress(BMS_MAC, BLE_ADDR_PUBLIC);

static volatile bool doConnect = false;
static volatile bool connected = false;
static bool wifiReady = false;
static volatile unsigned long lastBLEDataMs = 0;
static unsigned long bleConnectAttempts = 0;
static unsigned long nextBLEConnectAttemptMs = 0;
static unsigned long lastBLERetryLogMs = 0;

// -----------------------------------------------------------------------
// BMS ring buffer and Stream wrapper
// -----------------------------------------------------------------------
#define RX_BUF_SIZE 4096
static uint8_t rxBuf[RX_BUF_SIZE];
static volatile int rxHead = 0;
static volatile int rxTail = 0;
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

static void rxPush(uint8_t b) {
  taskENTER_CRITICAL(&rxMux);
  int next = (rxHead + 1) % RX_BUF_SIZE;
  if (next != rxTail) {
    rxBuf[rxHead] = b;
    rxHead = next;
  }
  taskEXIT_CRITICAL(&rxMux);
}

static int rxPop() {
  taskENTER_CRITICAL(&rxMux);
  if (rxHead == rxTail) {
    taskEXIT_CRITICAL(&rxMux);
    return -1;
  }
  uint8_t b = rxBuf[rxTail];
  rxTail = (rxTail + 1) % RX_BUF_SIZE;
  taskEXIT_CRITICAL(&rxMux);
  return b;
}

static int rxAvailable() {
  taskENTER_CRITICAL(&rxMux);
  int count = (rxHead - rxTail + RX_BUF_SIZE) % RX_BUF_SIZE;
  taskEXIT_CRITICAL(&rxMux);
  return count;
}

class BmsStream : public Stream {
 private:
  uint8_t txBuf[32];
  size_t txLen = 0;

 public:
  int available() override { return rxAvailable(); }
  int read() override { return rxPop(); }
  int peek() override {
    taskENTER_CRITICAL(&rxMux);
    if (rxHead == rxTail) {
      taskEXIT_CRITICAL(&rxMux);
      return -1;
    }
    int b = rxBuf[rxTail];
    taskEXIT_CRITICAL(&rxMux);
    return b;
  }

  size_t write(uint8_t b) override {
    if (txLen >= sizeof(txBuf)) {
      static unsigned long lastOverflowLogMs = 0;
      unsigned long now = millis();
      if (now - lastOverflowLogMs > 2000) {
        Serial.println("BMS TX buffer overflow, dropping frame");
        lastOverflowLogMs = now;
      }
      txLen = 0;  // Drop oversized frame and resync.
      return 0;
    }
    txBuf[txLen++] = b;
    if (b == BMS_FRAME_TERMINATOR) {
      if (!pTxChar || !connected) {
        txLen = 0;
        return 0;
      }
      pTxChar->writeValue(txBuf, txLen, false);
      txLen = 0;
    }
    return 1;
  }

  size_t write(const uint8_t* buf, size_t size) override {
    size_t written = 0;
    for (size_t i = 0; i < size; i++) written += write(buf[i]);
    return written;
  }

  void flush() override {}
} bmsStream;

OverkillSolarBms2 bms;
WebServer server(80);
HardwareSerial RS485(2);

// -----------------------------------------------------------------------
// RS485 Solis state
// -----------------------------------------------------------------------
struct RegisterSpec {
  uint16_t reg;
};

static const RegisterSpec REGISTER_SPECS[] = {
    {33050}, {33051}, {33052}, {33053}, {33059}, {33072}, {33074},
    {33080}, {33081}, {33085}, {33095}, {33129}, {33130}, {33131},
    {33132}, {33134}, {33135}, {33136}, {33137}, {33140}, {33142},
};

static const size_t REGISTER_COUNT = sizeof(REGISTER_SPECS) / sizeof(REGISTER_SPECS[0]);
static const uint16_t SOLIS_REG_BATTERY_DIRECTION = 33136;  // Confirmed: 0=charging, 1=discharging.
static const uint16_t SOLIS_REG_BATTERY_CURRENT = 33135;    // Register current, unsigned 0.1A magnitude.
static const uint16_t SOLIS_REG_BATTERY_VOLTAGE = 33142;    // Register voltage, unsigned 0.01V.

struct RegisterValue {
  uint16_t raw;
  bool valid;
};

struct SolisState {
  RegisterValue values[REGISTER_COUNT];
  uint32_t lastPollMs;
  uint32_t lastSuccessMs;
  uint32_t pollCount;
  uint32_t readErrors;
  uint32_t lockTimeouts;
};

static SolisState solisState = {};
static SemaphoreHandle_t solisMutex = nullptr;

// Explicit prototypes avoid Arduino auto-generated prototypes being emitted
// before the custom type definitions above.
static RegisterValue getRegisterValueByDocReg(const SolisState& state, uint16_t docReg);
static bool isSolisBatteryDischarging(bool valid, uint16_t raw);
static bool tryBuildSignedBatteryPowerW(const SolisState& state, float& powerW);

static RegisterValue getRegisterValueByDocReg(const SolisState& state, uint16_t docReg) {
  for (size_t i = 0; i < REGISTER_COUNT; i++) {
    if (REGISTER_SPECS[i].reg == docReg) return state.values[i];
  }
  return {0, false};
}

static bool isSolisBatteryDischarging(bool valid, uint16_t raw) {
  return valid && raw == 1;
}

static bool tryBuildSignedBatteryPowerW(const SolisState& state, float& powerW) {
  RegisterValue current = getRegisterValueByDocReg(state, SOLIS_REG_BATTERY_CURRENT);
  RegisterValue voltage = getRegisterValueByDocReg(state, SOLIS_REG_BATTERY_VOLTAGE);
  if (!current.valid || !voltage.valid) return false;

  RegisterValue direction = getRegisterValueByDocReg(state, SOLIS_REG_BATTERY_DIRECTION);
  float currentAmps = float(current.raw) / 10.0f;
  float voltageVolts = float(voltage.raw) / 100.0f;
  powerW = voltageVolts * currentAmps;
  // Apply sign convention: positive = charging, negative = discharging.
  if (isSolisBatteryDischarging(direction.valid, direction.raw)) powerW = -powerW;
  return true;
}

// -----------------------------------------------------------------------
// CAN API implemented in companion file
// -----------------------------------------------------------------------
void setupCAN();
void sendCANFrames(OverkillSolarBms2& bms);

// -----------------------------------------------------------------------
// Generic helpers
// -----------------------------------------------------------------------
static String jsonBool(bool v) { return v ? "true" : "false"; }

static bool tryConnectWiFi(const char* ssid, const char* password) {
  Serial.printf("Trying WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

// -----------------------------------------------------------------------
// BLE callbacks/connect
// -----------------------------------------------------------------------
void notifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  (void)pChar;
  (void)isNotify;
  for (size_t i = 0; i < length; i++) rxPush(pData[i]);
}

static void resetBLEClient();
static void scheduleBLEReconnect(const char* reason);

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pC) override {
    (void)pC;
    Serial.println("BLE Connected");
    connected = true;
  }

  void onDisconnect(NimBLEClient* pC, int reason) override {
    (void)pC;
    Serial.printf("BLE disconnected (reason: %d).\n", reason);
    pTxChar = nullptr;
    pRxChar = nullptr;
    connected = false;
    scheduleBLEReconnect("BLE disconnected.");
  }
};

static ClientCallbacks gClientCallbacks;

static void resetBLEClient() {
  connected = false;
  pTxChar = nullptr;
  pRxChar = nullptr;

  if (!pClient) return;

  if (pClient->isConnected()) pClient->disconnect();
  NimBLEDevice::deleteClient(pClient);
  pClient = nullptr;
}

static void scheduleBLEReconnect(const char* reason) {
  nextBLEConnectAttemptMs = millis() + BLE_RETRY_INTERVAL_MS;
  lastBLERetryLogMs = 0;
  doConnect = false;
  Serial.printf("%s Retrying BLE in %lu ms and will keep retrying until connected.\n", reason,
                BLE_RETRY_INTERVAL_MS);
}

static bool connectToBMS() {
  resetBLEClient();
  bleConnectAttempts++;
  Serial.printf("BLE connect attempt %lu to BMS %s...\n", bleConnectAttempts, BMS_MAC);

  pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(&gClientCallbacks, false);
  pClient->setConnectionParams(BLE_CONN_INTERVAL_MIN, BLE_CONN_INTERVAL_MAX, BLE_CONN_LATENCY,
                               BLE_CONN_TIMEOUT);
  pClient->setConnectTimeout(30);

  if (!pClient->connect(bmsMacAddress)) {
    Serial.println("BLE connect() failed");
    resetBLEClient();
    scheduleBLEReconnect("BLE connect() failed.");
    return false;
  }

  NimBLERemoteService* pSvc = pClient->getService(SERVICE_UUID);
  if (!pSvc) {
    Serial.println("Service FF00 not found");
    resetBLEClient();
    scheduleBLEReconnect("Service FF00 not found.");
    return false;
  }

  pRxChar = pSvc->getCharacteristic(RX_UUID);
  if (!pRxChar) {
    Serial.println("RX char FF01 not found");
    resetBLEClient();
    scheduleBLEReconnect("RX char FF01 not found.");
    return false;
  }
  if (pRxChar->canNotify()) pRxChar->subscribe(true, notifyCallback);

  pTxChar = pSvc->getCharacteristic(TX_UUID);
  if (!pTxChar) {
    Serial.println("TX char FF02 not found");
    resetBLEClient();
    scheduleBLEReconnect("TX char FF02 not found.");
    return false;
  }

  nextBLEConnectAttemptMs = 0;
  lastBLERetryLogMs = 0;
  Serial.println("BLE fully connected");
  return true;
}

// -----------------------------------------------------------------------
// RS485 Modbus helpers
// -----------------------------------------------------------------------
static uint16_t modbusCRC(const uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (int i = 0; i < 8; i++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

static void flush485() {
  while (RS485.available()) RS485.read();
}

static int readReply(uint8_t* buf, int maxLen, uint32_t timeoutMs) {
  int len = 0;
  uint32_t last = millis();
  while (millis() - last < timeoutMs) {
    while (RS485.available()) {
      uint8_t b = RS485.read();
      if (len < maxLen) buf[len++] = b;
      last = millis();
    }
    delay(1);
  }
  return len;
}

static bool validCRC(const uint8_t* buf, int len) {
  if (len < 4) return false;
  uint16_t rxCRC = buf[len - 2] | (uint16_t(buf[len - 1]) << 8);
  return rxCRC == modbusCRC(buf, len - 2);
}

static void sendReadInput(uint8_t slave, uint16_t rawAddr, uint16_t count) {
  uint8_t frame[8];
  frame[0] = slave;
  frame[1] = 0x04;
  frame[2] = rawAddr >> 8;
  frame[3] = rawAddr & 0xFF;
  frame[4] = count >> 8;
  frame[5] = count & 0xFF;
  uint16_t crc = modbusCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;
  RS485.write(frame, 8);
  RS485.flush();
}

static bool readDocRegU16(uint8_t slave, uint16_t docReg, uint16_t& value) {
  uint8_t buf[32];
  // Solis docs use 1-based register numbers while Modbus frame address is 0-based.
  const uint16_t rawAddr = docReg - 1;

  flush485();
  sendReadInput(slave, rawAddr, 1);

  const int len = readReply(buf, sizeof(buf), MODBUS_TIMEOUT_MS);
  if (len < 7) return false;
  if (!validCRC(buf, len)) return false;
  if (buf[0] != slave || buf[1] != 0x04 || buf[2] != 2) return false;

  value = (uint16_t(buf[3]) << 8) | buf[4];
  return true;
}

static void pollTask(void* pv) {
  (void)pv;

  for (;;) {
    uint32_t errorsThisPass = 0;
    uint32_t lockTimeoutsThisPass = 0;
    bool anySuccess = false;

    for (size_t i = 0; i < REGISTER_COUNT; i++) {
      uint16_t raw = 0;
      if (readDocRegU16(SOLIS_SLAVE_ID, REGISTER_SPECS[i].reg, raw)) {
        uint32_t now = millis();
        if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(MONITOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
          solisState.values[i].raw = raw;
          solisState.values[i].valid = true;
          solisState.lastSuccessMs = now;
          xSemaphoreGive(solisMutex);
          anySuccess = true;
        } else {
          lockTimeoutsThisPass++;
        }
      } else {
        errorsThisPass++;
      }
      vTaskDelay(pdMS_TO_TICKS(INTER_REGISTER_DELAY_MS));
    }

    if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(MONITOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
      solisState.lastPollMs = millis();
      solisState.pollCount++;
      solisState.readErrors += errorsThisPass;
      solisState.lockTimeouts += lockTimeoutsThisPass;
      xSemaphoreGive(solisMutex);
    }

    if (!anySuccess) Serial.println("Solis poll pass had no successful reads");
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// -----------------------------------------------------------------------
// JSON builders (simple String output for conservative first merged version)
// -----------------------------------------------------------------------
static String buildSolisJson() {
  SolisState snapshot = {};
  if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(MONITOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
    snapshot = solisState;
    xSemaphoreGive(solisMutex);
  }

  String json;
  json.reserve(1400);
  json += "{";
  json += "\"uptimeMs\":";
  json += String(millis());
  json += ",\"lastPollMs\":";
  json += String(snapshot.lastPollMs);
  json += ",\"lastSuccessMs\":";
  json += String(snapshot.lastSuccessMs);
  json += ",\"pollCount\":";
  json += String(snapshot.pollCount);
  json += ",\"readErrors\":";
  json += String(snapshot.readErrors);
  json += ",\"lockTimeouts\":";
  json += String(snapshot.lockTimeouts);

  float batteryPowerW = 0.0f;
  json += ",\"batteryPowerW\":";
  if (tryBuildSignedBatteryPowerW(snapshot, batteryPowerW)) {
    json += String(batteryPowerW, 1);
  } else {
    json += "null";
  }

  for (size_t i = 0; i < REGISTER_COUNT; i++) {
    json += ",\"";
    json += String(REGISTER_SPECS[i].reg);
    json += "\":{";
    json += "\"valid\":";
    json += jsonBool(snapshot.values[i].valid);
    json += ",\"raw\":";
    json += String(snapshot.values[i].raw);
    json += ",\"signed\":";
    json += String((int16_t)snapshot.values[i].raw);
    json += "}";
  }
  json += "}";
  return json;
}

static String buildBmsJson() {
  uint8_t numCells = bms.get_num_cells();
  String json;
  json.reserve(1500);
  json += "{";
  json += "\"connected\":";
  json += jsonBool(connected);
  json += ",\"uptimeMs\":";
  json += String(millis());
  json += ",\"lastBLEDataMs\":";
  json += String(lastBLEDataMs);
  json += ",\"voltage\":";
  json += String(bms.get_voltage(), 3);
  json += ",\"current\":";
  json += String(bms.get_current(), 3);
  json += ",\"soc\":";
  json += String(bms.get_state_of_charge());
  json += ",\"temperatureC\":";
  json += String(bms.get_ntc_temperature(0), 2);
  json += ",\"chargeMos\":";
  json += jsonBool(bms.get_charge_mosfet_status());
  json += ",\"dischargeMos\":";
  json += jsonBool(bms.get_discharge_mosfet_status());
  json += ",\"numCells\":";
  json += String(numCells);
  json += ",\"cells\":[";
  for (uint8_t c = 0; c < numCells; c++) {
    if (c) json += ',';
    json += "{";
    json += "\"index\":";
    json += String(c + 1);
    json += ",\"voltage\":";
    json += String(bms.get_cell_voltage(c), 3);
    json += ",\"balance\":";
    json += jsonBool(bms.get_balance_status(c));
    json += "}";
  }
  json += "]}";
  return json;
}

static String buildAllJson() {
  String json = "{";
  json += "\"bms\":";
  json += buildBmsJson();
  json += ",\"solis\":";
  json += buildSolisJson();
  json += "}";
  return json;
}

// -----------------------------------------------------------------------
// Web handlers
// -----------------------------------------------------------------------
static void handleRoot() {
  const unsigned long ageSec = (millis() - lastBLEDataMs) / 1000UL;
  const uint8_t numCells = bms.get_num_cells();
  // Use the primary BMS NTC reading for dashboard temperature.
  const float temperatureC = bms.get_ntc_temperature(0);

  SolisState snapshot = {};
  if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(MONITOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
    snapshot = solisState;
    xSemaphoreGive(solisMutex);
  }

  String html;
  html.reserve(2600 + size_t(numCells) * 100);
  // Keep browser refresh simple and conservative for low ESP32 load.
  html += F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<meta http-equiv='refresh' content='5'>"
            "<title>BLE BMS + CAN + RS485</title>"
            "<style>body{font-family:Arial,sans-serif;margin:16px;background:#101820;color:#e6eef8}"
            "h1{margin-bottom:6px}h2{margin:14px 0 8px}.card{background:#172533;padding:10px;border-radius:8px;margin:8px 0}"
            ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:8px}"
            "table{width:100%;border-collapse:collapse}th,td{border-bottom:1px solid #2a3a4a;padding:6px 8px;text-align:left}"
            "th{background:#1f3142}.mono{font-family:monospace}a{color:#9bd0ff}code{color:#9bd0ff}</style></head><body>");
  html += F("<h1>ESP32 merged monitor</h1>");
  html += F("<p>Conservative merged sketch: BLE BMS + Pylontech CAN in loop, Solis RS485 in dedicated task. Auto-refreshes every 5s.</p>");

  html += F("<h2>BMS summary</h2><div class='card'><div class='grid'>");
  html += String("<div><b>BLE</b><br>") + (connected ? "Connected" : "Disconnected") + "</div>";
  html += String("<div><b>Last frame age</b><br>") + String(ageSec) + " s</div>";
  html += String("<div><b>Voltage</b><br>") + String(bms.get_voltage(), 2) + " V</div>";
  html += String("<div><b>Current</b><br>") + String(bms.get_current(), 2) + " A</div>";
  html += String("<div><b>SoC</b><br>") + String(bms.get_state_of_charge()) + " %</div>";
  html += String("<div><b>Temp</b><br>") + String(temperatureC, 1) + " C</div>";
  html += String("<div><b>Charge MOS</b><br>") + (bms.get_charge_mosfet_status() ? "ON" : "OFF") + "</div>";
  html += String("<div><b>Discharge MOS</b><br>") + (bms.get_discharge_mosfet_status() ? "ON" : "OFF") + "</div>";
  html += String("<div><b>Cells reported</b><br>") + String(numCells) + "</div>";
  html += F("</div>");
  html += F("</div>");

  html += F("<h2>BMS cell voltages</h2><div class='card'>");
  if (numCells == 0) {
    html += F("No cell data yet");
  } else {
    html += F("<table><tr><th>Cell</th><th>Voltage (V)</th><th>Balance</th></tr>");
    for (uint8_t c = 0; c < numCells; c++) {
      html += F("<tr><td>");
      html += String(c + 1);
      html += F("</td><td class='mono'>");
      html += String(bms.get_cell_voltage(c), 3);
      html += F("</td><td>");
      html += bms.get_balance_status(c) ? "ON" : "-";
      html += F("</td></tr>");
    }
    html += F("</table>");
  }
  html += F("</div>");

  html += F("<h2>Solis summary</h2><div class='card'>");
  float batteryPowerW = 0.0f;
  html += String("Polls: ") + String(snapshot.pollCount) + "<br>";
  html += String("Read errors: ") + String(snapshot.readErrors) + "<br>";
  html += String("Lock timeouts: ") + String(snapshot.lockTimeouts) + "<br>";
  html += String("Last poll ms: ") + String(snapshot.lastPollMs) + "<br>";
  html += String("Last success ms: ") + String(snapshot.lastSuccessMs) + "<br>";
  if (tryBuildSignedBatteryPowerW(snapshot, batteryPowerW)) {
    html += String("Battery power: ") + String(batteryPowerW, 1) + " W";
  } else {
    html += F("Battery power: --");
  }
  html += F("</div>");

  html += F("<div class='card'><b>API endpoints</b><br>");
  html += F("<a href='/api/bms'>/api/bms</a><br>");
  html += F("<a href='/api/solis'>/api/solis</a><br>");
  html += F("<a href='/api/all'>/api/all</a><br>");
  html += F("</div>");

  html += F("<p><small>Tuning points: <code>CAN_INTERVAL_MS</code>, <code>POLL_INTERVAL_MS</code>, <code>MODBUS_TIMEOUT_MS</code>.</small></p>");
  html += F("</body></html>");

  server.send(200, "text/html", html);
}

static void handleBmsApi() { server.send(200, "application/json", buildBmsJson()); }
static void handleSolisApi() { server.send(200, "application/json", buildSolisJson()); }
static void handleAllApi() { server.send(200, "application/json", buildAllJson()); }

// -----------------------------------------------------------------------
// Serial log
// -----------------------------------------------------------------------
static void logBMSData() {
  uint8_t numCells = bms.get_num_cells();
  float voltage = bms.get_voltage();
  if (numCells == 0 || voltage == 0.0f) {
    Serial.println("Waiting for valid BMS data...");
    return;
  }

  float minV = FLT_MAX, maxV = 0.0f, sum = 0.0f;
  for (uint8_t c = 0; c < numCells; c++) {
    float v = bms.get_cell_voltage(c);
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    sum += v;
  }

  Serial.printf("BMS: V=%.2fV I=%.2fA SoC=%u%% T=%.1fC Min=%.3f Max=%.3f Avg=%.3f dV=%.0fmV\n",
                voltage,
                bms.get_current(),
                bms.get_state_of_charge(),
                bms.get_ntc_temperature(0),
                minV,
                maxV,
                (numCells ? (sum / numCells) : 0.0f),
                (maxV - minV) * 1000.0f);
}

// -----------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(SERIAL_SETTLE_DELAY_MS);
  Serial.println();
  Serial.println("=== BLE BMS + CAN + RS485 merged sketch ===");
  Serial.printf("Pins: CAN TX=%d RX=%d | RS485 RX=%d TX=%d\n", (int)CAN_TX_PIN, (int)CAN_RX_PIN,
                RS485_RX_PIN, RS485_TX_PIN);

  setupCAN();

  solisMutex = xSemaphoreCreateMutex();
  RS485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  xTaskCreatePinnedToCore(
      pollTask,
      "SolisPoll",
      4096,
      nullptr,
      1,
      nullptr,
      1);

  bool wifiOk = false;
  if (strlen(WIFI_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    wifiOk = tryConnectWiFi(WIFI_SSID, WIFI_PASSWORD_PRIMARY);
    if (!wifiOk) {
      WiFi.disconnect(true);
      delay(500);
      wifiOk = tryConnectWiFi(WIFI_SSID, WIFI_PASSWORD_FALLBACK);
    }
  } else {
    Serial.println("WiFi SSID not configured; skipping web server startup");
  }

  if (wifiOk) {
    Serial.printf("WiFi connected - http://%s\n", WiFi.localIP().toString().c_str());
    server.on("/", handleRoot);
    server.on("/api/bms", handleBmsApi);
    server.on("/api/solis", handleSolisApi);
    server.on("/api/all", handleAllApi);
    server.begin();
    wifiReady = true;
  } else {
    Serial.println("WiFi failed - continuing without web server");
  }

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  doConnect = true;
}

static unsigned long lastCAN = 0;
static unsigned long lastLog = 0;

void loop() {
  if (doConnect) {
    doConnect = false;
    taskENTER_CRITICAL(&rxMux);
    rxHead = 0;
    rxTail = 0;
    taskEXIT_CRITICAL(&rxMux);
    if (connectToBMS()) {
      bms.begin(&bmsStream);
      delay(500);
      for (int i = 0; i < 10; i++) {
        bms.main_task(true);
        delay(100);
      }
    }
  }

  unsigned long now = millis();

  if (connected) {
    bms.main_task(true);
    lastBLEDataMs = now;
  } else if (!doConnect) {
    if (nextBLEConnectAttemptMs != 0 && now >= nextBLEConnectAttemptMs) {
      doConnect = true;
    } else if (nextBLEConnectAttemptMs != 0 &&
               now - lastBLERetryLogMs >= BLE_RETRY_LOG_INTERVAL_MS) {
      unsigned long remainingMs = nextBLEConnectAttemptMs - now;
      Serial.printf("BLE still disconnected - next retry in %lu ms\n", remainingMs);
      lastBLERetryLogMs = now;
    }
    delay(200);
  }

  if (now - lastCAN >= CAN_INTERVAL_MS) {
    lastCAN = now;
    if (connected || (now - lastBLEDataMs < BLE_TIMEOUT_MS)) {
      sendCANFrames(bms);
    }
  }

  if (now - lastLog >= LOG_INTERVAL_MS) {
    lastLog = now;
    if (connected) logBMSData();
  }

  if (wifiReady) server.handleClient();
}
