#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <bms2.h>
#include "BatteryAggregate.h"
#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>
#include <float.h>

// -----------------------------------------------------------------------
// Battery config (set enabled=true/false as needed)
// -----------------------------------------------------------------------
struct BatteryConfig {
    const char* name;
    const char* mac;
    bool        enabled;
};

static BatteryConfig batteryConfigs[] = {
    {"Growatt",           "a5:c2:37:49:c7:a2", true},
    {"Solax",             "a4:c1:37:20:4e:3b", true},
    {"SP14S004P14S40A",   "a5:c2:37:51:85:7f", true},
};

static const uint8_t BATTERY_COUNT = sizeof(batteryConfigs) / sizeof(batteryConfigs[0]);

// -----------------------------------------------------------------------
// BLE UUIDs
// -----------------------------------------------------------------------
static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb");
static BLEUUID rxUUID("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID txUUID("0000ff02-0000-1000-8000-00805f9b34fb");

// -----------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------
#define CAN_TX_PIN     GPIO_NUM_21
#define CAN_RX_PIN     GPIO_NUM_19

#define CAN_INTERVAL_MS          100
#define LOG_INTERVAL_MS         5000
#define BLE_TIMEOUT_MS     (3UL * 60UL * 1000UL)
#define BATTERY_BUDGET_MS       5000
#define POLL_SETTLE_MS           500
#define READ_SLICE_DELAY_MS       20

#define WIFI_SSID           "TP-LINK_73F3"
#define WIFI_PASSWORD_UPPER "DEADBEEF"
#define WIFI_PASSWORD_LOWER "deadbeef"
#define WIFI_CONNECT_TIMEOUT_MS  10000

// Cell voltage colour thresholds
#define CELL_V_GREEN_LO   3.4f
#define CELL_V_GREEN_HI   4.15f
#define CELL_V_AMBER_LO   3.0f

// Cell delta thresholds (mV)
#define CELL_DELTA_AMBER_MV   50.0f
#define CELL_DELTA_RED_MV    100.0f

// -----------------------------------------------------------------------
// Per-battery runtime model
// -----------------------------------------------------------------------
#define RX_BUF_SIZE 4096

struct BatteryRuntime;

class BmsStream : public Stream {
public:
    BatteryRuntime* owner = nullptr;

    int available() override;
    int read() override;
    int peek() override;
    size_t write(uint8_t b) override;
    size_t write(const uint8_t* buf, size_t size) override;
    void flush() override {}

private:
    uint8_t txBuf[32];
    size_t  txLen = 0;
};

struct BatteryRuntime {
    const char* name = nullptr;
    const char* mac = nullptr;
    bool enabled = false;

    OverkillSolarBms2 bms;
    BmsStream stream;

    uint8_t rxBuf[RX_BUF_SIZE];
    volatile int rxHead = 0;
    volatile int rxTail = 0;

    BLERemoteCharacteristic* txChar = nullptr;

    bool hasData = false;
    bool lastPollOk = false;
    unsigned long lastGoodDataMs = 0;
    unsigned long lastPollMs = 0;

    uint32_t okReads = 0;
    uint32_t failReads = 0;
};

static BatteryRuntime batteries[BATTERY_COUNT];

// -----------------------------------------------------------------------
// BLE globals
// -----------------------------------------------------------------------
static BLEScan*                 pBLEScan = nullptr;
static BLEClient*               pClient = nullptr;
static BLERemoteCharacteristic* pTxChar = nullptr;
static BLERemoteCharacteristic* pRxChar = nullptr;
static BLEAdvertisedDevice*     pRemoteDevice = nullptr;

static BatteryRuntime* gActiveBattery = nullptr;
static BatteryRuntime* gScanTargetBattery = nullptr;
static bool gScanFoundTarget = false;

static bool wifiReady = false;

WebServer server(80);

// -----------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------
void setupCAN();
void sendCANFrames(const AggregatedBmsData& agg);

static void runPeriodicTasks();
static AggregatedBmsData buildAggregate(unsigned long now);

// -----------------------------------------------------------------------
// Ring buffer helpers
// -----------------------------------------------------------------------
static inline void rxPush(BatteryRuntime& br, uint8_t b) {
    int next = (br.rxHead + 1) % RX_BUF_SIZE;
    if (next != br.rxTail) {
        br.rxBuf[br.rxHead] = b;
        br.rxHead = next;
    }
}

static inline int rxPop(BatteryRuntime& br) {
    if (br.rxHead == br.rxTail) return -1;
    uint8_t b = br.rxBuf[br.rxTail];
    br.rxTail = (br.rxTail + 1) % RX_BUF_SIZE;
    return b;
}

static inline int rxAvailable(BatteryRuntime& br) {
    return (br.rxHead - br.rxTail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

static inline void clearRx(BatteryRuntime& br) {
    br.rxHead = 0;
    br.rxTail = 0;
}

// -----------------------------------------------------------------------
// Stream wrapper implementation
// -----------------------------------------------------------------------
int BmsStream::available() {
    return owner ? rxAvailable(*owner) : 0;
}

int BmsStream::read() {
    return owner ? rxPop(*owner) : -1;
}

int BmsStream::peek() {
    if (!owner) return -1;
    if (owner->rxHead == owner->rxTail) return -1;
    return owner->rxBuf[owner->rxTail];
}

size_t BmsStream::write(uint8_t b) {
    if (!owner) return 0;

    if (txLen < sizeof(txBuf)) txBuf[txLen++] = b;

    if (b == 0x77) {
        if (!owner->txChar) {
            txLen = 0;
            return 0;
        }
        owner->txChar->writeValue(txBuf, txLen, false);
        txLen = 0;
    }
    return 1;
}

size_t BmsStream::write(const uint8_t* buf, size_t size) {
    size_t written = 0;
    for (size_t i = 0; i < size; i++) written += write(buf[i]);
    return written;
}

// -----------------------------------------------------------------------
// Generic helpers
// -----------------------------------------------------------------------
static bool hasValidBatteryData(const BatteryRuntime& br) {
    return br.bms.get_num_cells() > 0 && br.bms.get_voltage() > 0.0f;
}

static bool isBatteryFresh(const BatteryRuntime& br, unsigned long now) {
    return br.enabled && br.hasData && (now - br.lastGoodDataMs < BLE_TIMEOUT_MS);
}

static bool anyEnabledBattery() {
    for (uint8_t i = 0; i < BATTERY_COUNT; i++) {
        if (batteries[i].enabled) return true;
    }
    return false;
}

static String card(const String& label, const String& value, const String& cls = "") {
    String c = "<div class='card'><div class='label'>" + label + "</div><div class='value";
    if (cls.length()) c += " " + cls;
    c += "'>" + value + "</div></div>";
    return c;
}

static void appendBatterySection(String& html, const BatteryRuntime& br, unsigned long now) {
    uint8_t numCells = br.bms.get_num_cells();
    float voltage = br.bms.get_voltage();
    float current = br.bms.get_current();
    uint8_t soc = br.bms.get_state_of_charge();
    float temp = br.bms.get_ntc_temperature(0);
    bool chgMos = br.bms.get_charge_mosfet_status();
    bool dsgMos = br.bms.get_discharge_mosfet_status();
    unsigned long ageSec = (now - br.lastGoodDataMs) / 1000UL;

    float minV = FLT_MAX, maxV = 0.0f, sumV = 0.0f;
    for (uint8_t c = 0; c < numCells; c++) {
        float v = br.bms.get_cell_voltage(c);
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sumV += v;
    }
    float avgV = numCells ? (sumV / numCells) : 0.0f;
    float deltaMv = (numCells ? (maxV - minV) : 0.0f) * 1000.0f;

    String deltaCls;
    if      (deltaMv > CELL_DELTA_RED_MV)     deltaCls = "red";
    else if (deltaMv >= CELL_DELTA_AMBER_MV)  deltaCls = "amber";
    else                                       deltaCls = "green";

    html += "<h2>" + String(br.name) + "</h2>";
    html += F("<div class='grid'>");
    html += card("MAC", String(br.mac));
    html += card("Pack Voltage", String(voltage, 2) + " V");
    html += card("Current", String(current, 2) + " A");
    html += card("SoC (BMS)", String(soc) + " %");
    html += card("Temperature", String(temp, 1) + " C");
    html += card("Charge MOSFET", String(chgMos ? "ON" : "OFF"), String(chgMos ? "green" : "red"));
    html += card("Discharge MOSFET", String(dsgMos ? "ON" : "OFF"), String(dsgMos ? "green" : "red"));
    html += card("Last Data", String(ageSec) + " s ago");
    html += F("</div>");

    html += F("<table><tr><th>#</th><th>Voltage (V)</th><th>Status</th></tr>");
    for (uint8_t c = 0; c < numCells; c++) {
        float v = br.bms.get_cell_voltage(c);
        bool  bal = br.bms.get_balance_status(c);
        String cls;
        if      (v >= CELL_V_GREEN_LO && v <= CELL_V_GREEN_HI) cls = "green";
        else if (v >= CELL_V_AMBER_LO && v < CELL_V_GREEN_LO)  cls = "amber";
        else                                                    cls = "red";

        html += String("<tr><td>") + String(c + 1) + "</td>"
              + "<td class='" + cls + "'>" + String(v, 3) + "</td>"
              + "<td>" + (bal ? "bal" : "-") + "</td></tr>";
    }
    html += F("</table>");

    html += F("<div class='grid'>");
    html += card("Cell Min", String(minV, 3) + " V");
    html += card("Cell Max", String(maxV, 3) + " V");
    html += card("Cell Avg", String(avgV, 3) + " V");
    html += card("Cell Delta", String(deltaMv, 0) + " mV", deltaCls);
    html += F("</div>");
}

// -----------------------------------------------------------------------
// Web server — root page
// -----------------------------------------------------------------------
void handleRoot() {
    unsigned long now = millis();
    AggregatedBmsData agg = buildAggregate(now);

    uint8_t enabledCount = 0;
    uint8_t withDataCount = 0;
    for (uint8_t i = 0; i < BATTERY_COUNT; i++) {
        if (!batteries[i].enabled) continue;
        enabledCount++;
        if (isBatteryFresh(batteries[i], now)) withDataCount++;
    }

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
        ".card .value{font-size:1.1em;font-weight:bold;word-break:break-word}"
        "table{width:100%;border-collapse:collapse;margin-bottom:14px}"
        "th,td{padding:8px 10px;text-align:center;border-bottom:1px solid #2a2a4a}"
        "th{background:#16213e;color:#a0b4cc}"
        ".green{color:#4caf50}.amber{color:#ff9800}.red{color:#f44336}"
        ".footer{margin-top:16px;font-size:0.8em;color:#556}"
        "</style></head><body>"
        "<h1>Battery BMS</h1>");

    html += F("<h2>Combined Summary</h2><div class='grid'>");
    html += card("Enabled Batteries", String(enabledCount));
    html += card("Batteries with Data", String(withDataCount));
    html += card("Avg Pack Voltage", agg.valid ? String(agg.avgVoltage, 2) + " V" : "n/a");
    html += card("Total Current", agg.valid ? String(agg.totalCurrent, 2) + " A" : "n/a");
    html += card("Avg Temperature", agg.valid ? String(agg.avgTemperature, 1) + " C" : "n/a");
    html += card("CAN Source", agg.valid ? "aggregate" : "no fresh data", agg.valid ? "green" : "amber");
    html += F("</div>");

    bool renderedAnyBattery = false;
    for (uint8_t i = 0; i < BATTERY_COUNT; i++) {
        if (!batteries[i].enabled || !isBatteryFresh(batteries[i], now)) continue;
        renderedAnyBattery = true;
        appendBatterySection(html, batteries[i], now);
    }

    if (!renderedAnyBattery) {
        html += F("<h2>Batteries</h2><p>No fresh battery data yet.</p>");
    }

    html += F("<div class='footer'>Auto-refreshes every 5 s</div></body></html>");
    server.send(200, "text/html", html);
}

// -----------------------------------------------------------------------
// BLE callbacks / connect / poll
// -----------------------------------------------------------------------
static void cleanupConnection() {
    if (pClient) {
        if (pClient->isConnected()) {
            pClient->disconnect();
            unsigned long settleStart = millis();
            while (millis() - settleStart < 100) {
                runPeriodicTasks();
                delay(5);
            }
        }
        delete pClient;
        pClient = nullptr;
    }
    pTxChar = nullptr;
    pRxChar = nullptr;

    if (pRemoteDevice) {
        delete pRemoteDevice;
        pRemoteDevice = nullptr;
    }

    if (gActiveBattery) {
        gActiveBattery->txChar = nullptr;
        gActiveBattery = nullptr;
    }
}

static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify
) {
    (void)pBLERemoteCharacteristic;
    (void)isNotify;

    if (!gActiveBattery) return;
    for (size_t i = 0; i < length; i++) {
        rxPush(*gActiveBattery, pData[i]);
    }
}

class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (!gScanTargetBattery) return;

        String addr = advertisedDevice.getAddress().toString().c_str();
        addr.toLowerCase();

        String targetMac = gScanTargetBattery->mac;
        targetMac.toLowerCase();

        bool addrOk = (addr == targetMac);
        bool svcOk = advertisedDevice.haveServiceUUID() &&
                     advertisedDevice.isAdvertisingService(serviceUUID);

        if (addrOk && svcOk) {
            gScanFoundTarget = true;
            if (pRemoteDevice) {
                delete pRemoteDevice;
                pRemoteDevice = nullptr;
            }
            pRemoteDevice = new BLEAdvertisedDevice(advertisedDevice);
            BLEDevice::getScan()->stop();
        }
    }
};

static bool scanForBattery(BatteryRuntime& br, unsigned long deadlineMs) {
    gScanTargetBattery = &br;
    gScanFoundTarget = false;

    if (pRemoteDevice) {
        delete pRemoteDevice;
        pRemoteDevice = nullptr;
    }

    while (!gScanFoundTarget && millis() < deadlineMs) {
        pBLEScan->setActiveScan(true);
        // Keep the tested interval/window pair from proven single-battery polling sketches.
        // Values are in BLE units (0.625 ms): 1349≈843.125 ms interval, 449≈280.625 ms window.
        pBLEScan->setInterval(1349);
        pBLEScan->setWindow(449);
        pBLEScan->start(1, false);
        runPeriodicTasks();
    }

    return gScanFoundTarget && pRemoteDevice;
}

static bool connectToBattery(BatteryRuntime& br) {
    if (!pRemoteDevice) return false;

    pClient = BLEDevice::createClient();
    if (!pClient) return false;

    pClient->setMTU(517); // request a large MTU to reduce packet fragmentation
    pClient->connect(pRemoteDevice);

    if (!pClient->isConnected()) {
        return false;
    }

    BLERemoteService* pSvc = pClient->getService(serviceUUID);
    if (!pSvc) return false;

    pRxChar = pSvc->getCharacteristic(rxUUID);
    if (!pRxChar || !pRxChar->canNotify()) return false;
    pRxChar->registerForNotify(notifyCallback);

    pTxChar = pSvc->getCharacteristic(txUUID);
    if (!pTxChar || (!pTxChar->canWrite() && !pTxChar->canWriteNoResponse())) return false;

    gActiveBattery = &br;
    br.txChar = pTxChar;
    clearRx(br);

    return true;
}

static bool readBatteryData(BatteryRuntime& br, unsigned long deadlineMs) {
    unsigned long settleEnd = millis() + POLL_SETTLE_MS;

    while (millis() < settleEnd && millis() < deadlineMs) {
        br.bms.main_task(true);
        runPeriodicTasks();
        delay(READ_SLICE_DELAY_MS);
    }

    while (millis() < deadlineMs) {
        br.bms.main_task(true);
        if (hasValidBatteryData(br)) {
            br.hasData = true;
            br.lastGoodDataMs = millis();
            return true;
        }
        runPeriodicTasks();
        delay(READ_SLICE_DELAY_MS);
    }

    return false;
}

static bool pollBattery(BatteryRuntime& br) {
    if (!br.enabled) return false;

    unsigned long startMs = millis();
    unsigned long deadlineMs = startMs + BATTERY_BUDGET_MS;
    br.lastPollMs = startMs;

    cleanupConnection();

    bool ok = false;
    if (scanForBattery(br, deadlineMs)) {
        if (millis() < deadlineMs && connectToBattery(br)) {
            ok = readBatteryData(br, deadlineMs);
        }
    }

    cleanupConnection();

    br.lastPollOk = ok;
    if (ok) {
        br.okReads++;
        Serial.printf("%s %.2fV\n", br.name, br.bms.get_voltage());
    } else {
        br.failReads++;
        Serial.printf("%s FAIL\n", br.name);
    }
    return ok;
}

// -----------------------------------------------------------------------
// Aggregation
// -----------------------------------------------------------------------
static AggregatedBmsData buildAggregate(unsigned long now) {
    AggregatedBmsData agg{};
    agg.minCellVoltage = FLT_MAX;
    agg.maxCellVoltage = 0.0f;
    agg.chargeAllowed = true;
    agg.dischargeAllowed = true;

    float sumV = 0.0f;
    float sumTemp = 0.0f;

    for (uint8_t i = 0; i < BATTERY_COUNT; i++) {
        BatteryRuntime& br = batteries[i];
        if (!isBatteryFresh(br, now)) continue;

        float packV = br.bms.get_voltage();
        if (packV <= 0.0f) continue;

        agg.valid = true;
        agg.validCount++;
        sumV += packV;
        // Parallel-pack aggregate: currents add, while pack voltage is averaged.
        agg.totalCurrent += br.bms.get_current();
        sumTemp += br.bms.get_ntc_temperature(0);

        // Conservative inverter gating: any battery disallowing charge/discharge blocks it.
        agg.chargeAllowed = agg.chargeAllowed && br.bms.get_charge_mosfet_status();
        agg.dischargeAllowed = agg.dischargeAllowed && br.bms.get_discharge_mosfet_status();

        uint8_t cells = br.bms.get_num_cells();
        for (uint8_t c = 0; c < cells; c++) {
            float cv = br.bms.get_cell_voltage(c);
            if (cv < agg.minCellVoltage) agg.minCellVoltage = cv;
            if (cv > agg.maxCellVoltage) agg.maxCellVoltage = cv;
        }
    }

    if (agg.validCount > 0) {
        agg.avgVoltage = sumV / agg.validCount;
        agg.avgTemperature = sumTemp / agg.validCount;
        // Defensive fallback if a pack reports voltage but no cells.
        if (agg.minCellVoltage == FLT_MAX) {
            agg.minCellVoltage = 0.0f;
            agg.maxCellVoltage = 0.0f;
        }
    } else {
        agg.chargeAllowed = false;
        agg.dischargeAllowed = false;
        agg.minCellVoltage = 0.0f;
        agg.maxCellVoltage = 0.0f;
    }

    return agg;
}

// -----------------------------------------------------------------------
// Serial log
// -----------------------------------------------------------------------
static void logBMSData() {
    unsigned long now = millis();
    AggregatedBmsData agg = buildAggregate(now);
    uint8_t enabledCount = 0;
    for (uint8_t i = 0; i < BATTERY_COUNT; i++) if (batteries[i].enabled) enabledCount++;

    Serial.println("\n========== BMS STATUS ==========");
    Serial.printf("Enabled: %u  Fresh: %u\n", (unsigned)enabledCount, (unsigned)agg.validCount);
    if (agg.valid) {
        Serial.printf("Aggregate V: %.2fV  I: %.2fA  T: %.1fC\n", agg.avgVoltage, agg.totalCurrent, agg.avgTemperature);
    }

    for (uint8_t i = 0; i < BATTERY_COUNT; i++) {
        BatteryRuntime& br = batteries[i];
        if (!br.enabled) continue;

        if (!isBatteryFresh(br, now)) {
            Serial.printf("%s: no fresh data\n", br.name);
            continue;
        }

        uint8_t cells = br.bms.get_num_cells();
        Serial.printf("%s: %.2fV %.2fA SoC:%u%% Cells:%u Last:%lus\n",
            br.name,
            br.bms.get_voltage(),
            br.bms.get_current(),
            br.bms.get_state_of_charge(),
            cells,
            (unsigned long)((now - br.lastGoodDataMs) / 1000UL));
    }
    Serial.println("================================");
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
// Housekeeping / periodic tasks
// -----------------------------------------------------------------------
static unsigned long lastCAN = 0;
static unsigned long lastLog = 0;

static void runPeriodicTasks() {
    unsigned long now = millis();

    if (now - lastCAN >= CAN_INTERVAL_MS) {
        lastCAN = now;
        AggregatedBmsData agg = buildAggregate(now);
        if (agg.valid) sendCANFrames(agg);
    }

    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        logBMSData();
    }

    if (wifiReady) server.handleClient();
}

// -----------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== JBD BMS -> Solis CAN Bridge (multi battery poll) ===");

    setupCAN();

    for (uint8_t i = 0; i < BATTERY_COUNT; i++) {
        batteries[i].name = batteryConfigs[i].name;
        batteries[i].mac = batteryConfigs[i].mac;
        batteries[i].enabled = batteryConfigs[i].enabled;
        batteries[i].stream.owner = &batteries[i];
        batteries[i].bms.begin(&batteries[i].stream);
    }

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

    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P9);
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());

    uint8_t enabledCount = 0;
    for (uint8_t i = 0; i < BATTERY_COUNT; i++) if (batteries[i].enabled) enabledCount++;
    Serial.printf("Configured batteries: %u enabled\n", enabledCount);
}

// -----------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------
void loop() {
    static uint8_t nextBattery = 0;

    runPeriodicTasks();

    if (!anyEnabledBattery()) {
        delay(250);
        return;
    }

    for (uint8_t checked = 0; checked < BATTERY_COUNT; checked++) {
        BatteryRuntime& br = batteries[nextBattery];
        nextBattery = (nextBattery + 1) % BATTERY_COUNT;
        if (!br.enabled) continue;
        pollBattery(br);
        break;
    }
}
