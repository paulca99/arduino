#include <NimBLEDevice.h>
#include <bms2.h>
#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>
#include <float.h>

// -----------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------
#define BMS_MAC        "a5:c2:37:51:85:89"
#define SERVICE_UUID   "ff00"
#define RX_UUID        "ff01"
#define TX_UUID        "ff02"

#define CAN_TX_PIN     GPIO_NUM_21
#define CAN_RX_PIN     GPIO_NUM_19

#define CAN_INTERVAL_MS   100    // Send CAN frames every 100ms
#define LOG_INTERVAL_MS   5000   // Log to serial every 5s
#define BLE_TIMEOUT_MS  (3UL * 60UL * 1000UL)   // 3 minutes

#define WIFI_SSID           "TP-LINK_73F3"
#define WIFI_PASSWORD_UPPER "DEADBEEF"
#define WIFI_PASSWORD_LOWER "deadbeef"

#define WIFI_CONNECT_TIMEOUT_MS  10000   // 10 s WiFi connection timeout

// Cell voltage colour thresholds
#define CELL_V_GREEN_LO   3.4f
#define CELL_V_GREEN_HI   4.15f
#define CELL_V_AMBER_LO   3.0f

// Cell delta thresholds (mV)
#define CELL_DELTA_AMBER_MV   50.0f
#define CELL_DELTA_RED_MV    100.0f

// -----------------------------------------------------------------------
// BLE globals
// -----------------------------------------------------------------------
static NimBLEClient*               pClient  = nullptr;
static NimBLERemoteCharacteristic* pTxChar  = nullptr;
static NimBLERemoteCharacteristic* pRxChar  = nullptr;
static NimBLEAddress               bmsMacAddress(BMS_MAC, BLE_ADDR_PUBLIC);

static bool doConnect = false;
static bool connected = false;
static bool wifiReady = false;
static unsigned long lastBLEDataMs = 0;

// -----------------------------------------------------------------------
// Ring buffer
// -----------------------------------------------------------------------
#define RX_BUF_SIZE 4096
uint8_t rxBuf[RX_BUF_SIZE];
volatile int rxHead = 0;
volatile int rxTail = 0;

void rxPush(uint8_t b) {
    int next = (rxHead + 1) % RX_BUF_SIZE;
    if (next != rxTail) { rxBuf[rxHead] = b; rxHead = next; }
}
int rxPop() {
    if (rxHead == rxTail) return -1;
    uint8_t b = rxBuf[rxTail];
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
    return b;
}
int rxAvailable() {
    return (rxHead - rxTail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

// -----------------------------------------------------------------------
// Stream wrapper
// -----------------------------------------------------------------------
class BmsStream : public Stream {
private:
    uint8_t txBuf[32];
    size_t  txLen = 0;
public:
    int available() override { return rxAvailable(); }
    int read()      override { return rxPop(); }
    int peek()      override {
        if (rxHead == rxTail) return -1;
        return rxBuf[rxTail];
    }
    size_t write(uint8_t b) override {
        if (txLen < sizeof(txBuf)) txBuf[txLen++] = b;
        if (b == 0x77) {
            if (!pTxChar || !connected) { txLen = 0; return 0; }
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

// -----------------------------------------------------------------------
// Helper: build one summary card
// -----------------------------------------------------------------------
static String card(const String& label, const String& value, const String& cls = "") {
    String c = "<div class='card'><div class='label'>" + label + "</div><div class='value";
    if (cls.length()) c += " " + cls;
    c += "'>" + value + "</div></div>";
    return c;
}

// -----------------------------------------------------------------------
// Web server — root page
// -----------------------------------------------------------------------
void handleRoot() {
    unsigned long now = millis();
    uint8_t numCells  = bms.get_num_cells();
    float   voltage   = bms.get_voltage();
    float   current   = bms.get_current();
    uint8_t soc       = bms.get_state_of_charge();
    float   temp      = bms.get_ntc_temperature(0);
    bool    chgMos    = bms.get_charge_mosfet_status();
    bool    dsgMos    = bms.get_discharge_mosfet_status();
    unsigned long ageSec = (now - lastBLEDataMs) / 1000UL;

    float minV = FLT_MAX, maxV = 0, sumV = 0;
    for (uint8_t c = 0; c < numCells; c++) {
        float v = bms.get_cell_voltage(c);
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sumV += v;
    }
    float avgV   = numCells ? sumV / numCells : 0;
    float deltaMv = (maxV - minV) * 1000.0f;

    String html = F("<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='5'>"
        "<title>BMS Monitor</title>"
        "<style>"
        "body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}"
        "h1{color:#e0c97f;margin-bottom:4px}"
        "h2{color:#a0b4cc;margin-top:20px;margin-bottom:6px}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:10px;margin-bottom:16px}"
        ".card{background:#16213e;border-radius:8px;padding:12px;text-align:center}"
        ".card .label{font-size:0.75em;color:#8899aa;margin-bottom:4px}"
        ".card .value{font-size:1.4em;font-weight:bold}"
        "table{width:100%;border-collapse:collapse}"
        "th,td{padding:8px 10px;text-align:center;border-bottom:1px solid #2a2a4a}"
        "th{background:#16213e;color:#a0b4cc}"
        ".green{color:#4caf50}.amber{color:#ff9800}.red{color:#f44336}"
        ".footer{margin-top:16px;font-size:0.8em;color:#556}"
        "</style></head><body>"
        "<h1>Battery BMS</h1>");

    html += F("<div class='grid'>");
    html += card("Pack Voltage",   String(voltage, 2) + " V");
    html += card("Current",        String(current, 2) + " A");
    html += card("SoC",            String(soc) + " %");
    html += card("Temperature",    String(temp, 1) + " C");
    html += card("Charge MOSFET",  String(chgMos  ? "ON" : "OFF"), String(chgMos  ? "green" : "red"));
    html += card("Discharge MOSFET", String(dsgMos ? "ON" : "OFF"), String(dsgMos ? "green" : "red"));
    html += card("BLE",            String(connected ? "Connected" : "Disconnected"), String(connected ? "green" : "red"));
    html += card("Last BMS Data",  String(ageSec) + " s ago");
    html += F("</div>");

    html += F("<h2>Cell Voltages</h2>"
              "<table><tr><th>#</th><th>Voltage (V)</th><th>Status</th></tr>");

    for (uint8_t c = 0; c < numCells; c++) {
        float v = bms.get_cell_voltage(c);
        bool  bal = bms.get_balance_status(c);
        String cls;
        if      (v >= CELL_V_GREEN_LO && v <= CELL_V_GREEN_HI)      cls = "green";
        else if (v >= CELL_V_AMBER_LO && v <  CELL_V_GREEN_LO)      cls = "amber";
        else                                                          cls = "red";

        html += String("<tr><td>") + String(c + 1) + "</td>"
              + "<td class='" + cls + "'>" + String(v, 3) + "</td>"
              + "<td>" + (bal ? "bal" : "-") + "</td></tr>";
    }
    html += F("</table>");

    String deltaCls;
    if      (deltaMv > CELL_DELTA_RED_MV)   deltaCls = "red";
    else if (deltaMv >= CELL_DELTA_AMBER_MV) deltaCls = "amber";
    else                                     deltaCls = "green";

    html += F("<h2>Cell Summary</h2><div class='grid'>");
    html += card("Min",   String(minV, 3) + " V");
    html += card("Max",   String(maxV, 3) + " V");
    html += card("Avg",   String(avgV, 3) + " V");
    html += card("Delta", String(deltaMv, 0) + " mV", deltaCls);
    html += F("</div>");

    html += F("<div class='footer'>Auto-refreshes every 5 s</div></body></html>");

    server.send(200, "text/html", html);
}

// -----------------------------------------------------------------------
// Notify callback
// -----------------------------------------------------------------------
void notifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    for (size_t i = 0; i < length; i++) rxPush(pData[i]);
}

// -----------------------------------------------------------------------
// Client callbacks
// -----------------------------------------------------------------------
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pC) override {
        Serial.println("BLE Connected!");
        connected = true;
    }
    void onDisconnect(NimBLEClient* pC, int reason) override {
        Serial.printf("BLE Disconnected (reason: %d) - retrying\n", reason);
        connected = false;
        doConnect = true;
    }
};

// -----------------------------------------------------------------------
// BLE connect
// -----------------------------------------------------------------------
bool connectToBMS() {
    Serial.printf("Connecting to BMS %s...\n", BMS_MAC);

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks(), false);
    pClient->setConnectionParams(6, 6, 0, 51);
    pClient->setConnectTimeout(30);

    if (!pClient->connect(bmsMacAddress)) {
        Serial.println("BLE connect() failed");
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }

    NimBLERemoteService* pSvc = pClient->getService(SERVICE_UUID);
    if (!pSvc) { Serial.println("Service FF00 not found"); pClient->disconnect(); return false; }

    pRxChar = pSvc->getCharacteristic(RX_UUID);
    if (!pRxChar) { Serial.println("RX char FF01 not found"); pClient->disconnect(); return false; }
    if (pRxChar->canNotify()) pRxChar->subscribe(true, notifyCallback);

    pTxChar = pSvc->getCharacteristic(TX_UUID);
    if (!pTxChar) { Serial.println("TX char FF02 not found"); pClient->disconnect(); return false; }

    Serial.println("BLE fully connected!");
    return true;
}

// -----------------------------------------------------------------------
// Serial log
// -----------------------------------------------------------------------
void logBMSData() {
    uint8_t numCells = bms.get_num_cells();
    float   voltage  = bms.get_voltage();

    if (numCells == 0 || voltage == 0.0f) {
        Serial.println("Waiting for valid BMS data...");
        return;
    }

    float minV = 9999, maxV = 0, sum = 0;
    for (uint8_t c = 0; c < numCells; c++) {
        float v = bms.get_cell_voltage(c);
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sum += v;
    }
    float avgV   = sum / numCells;
    float deltaV = maxV - minV;

    Serial.println("\n========== BMS STATUS ==========");
    Serial.printf("Voltage:   %.2f V\n",   voltage);
    Serial.printf("Current:   %.2f A\n",   bms.get_current());
    Serial.printf("SoC:       %d %%\n",    bms.get_state_of_charge());
    Serial.printf("Capacity:  %.1f Ah\n",  bms.get_balance_capacity());
    Serial.printf("Temp:      %.1f C\n",   bms.get_ntc_temperature(0));
    Serial.printf("CHG: %s  DSG: %s\n",
        bms.get_charge_mosfet_status()    ? "ON" : "OFF",
        bms.get_discharge_mosfet_status() ? "ON" : "OFF");
    Serial.println("--- Cells ---");
    Serial.printf("Min: %.3fV  Max: %.3fV  Avg: %.3fV  Delta: %.0fmV\n",
        minV, maxV, avgV, deltaV * 1000.0f);
    for (uint8_t c = 0; c < numCells; c++) {
        Serial.printf("  Cell %2d: %.3f V%s\n", c + 1,
            bms.get_cell_voltage(c),
            bms.get_balance_status(c) ? " *bal*" : "");
    }
    Serial.println("=================================");
}

// -----------------------------------------------------------------------
// WiFi helper
// -----------------------------------------------------------------------
static bool tryConnectWiFi(const char* ssid, const char* password) {
    Serial.printf("Trying WiFi: %s\n", ssid);
    WiFi.begin(ssid, password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();
    return WiFi.status() == WL_CONNECTED;
}

// -----------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== JBD BMS -> Solis CAN Bridge ===");

    setupCAN();

    WiFi.mode(WIFI_STA);
    bool wifiOk = tryConnectWiFi(WIFI_SSID, WIFI_PASSWORD_UPPER);
    if (!wifiOk) {
        WiFi.disconnect(true);
        delay(500);
        wifiOk = tryConnectWiFi(WIFI_SSID, WIFI_PASSWORD_LOWER);
    }
    if (wifiOk) {
        Serial.printf("WiFi connected - http://%s\n", WiFi.localIP().toString().c_str());
        server.on("/", handleRoot);
        server.begin();
        Serial.println("Web server started on port 80");
        wifiReady = true;
    } else {
        Serial.println("WiFi failed - continuing without web server");
    }

    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    doConnect = true;
}

// -----------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------
static unsigned long lastCAN = 0;
static unsigned long lastLog = 0;

void loop() {
    if (doConnect) {
        doConnect = false;
        rxHead = 0; rxTail = 0;
        if (connectToBMS()) {
            bms.begin(&bmsStream);
            delay(500);
            for (int i = 0; i < 10; i++) { bms.main_task(true); delay(100); }
        } else {
            delay(5000);
            doConnect = true;
        }
    }

    if (connected) {
        bms.main_task(true);
        lastBLEDataMs = millis();
    } else if (!doConnect) {
        delay(1000);
        Serial.print(".");
        return;
    }

    unsigned long now = millis();

    if (now - lastCAN >= CAN_INTERVAL_MS) {
        lastCAN = now;
        if (connected || (millis() - lastBLEDataMs < BLE_TIMEOUT_MS)) {
            sendCANFrames(bms);
        }
    }

    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        if (connected) logBMSData();
    }

    if (wifiReady) server.handleClient();
}
