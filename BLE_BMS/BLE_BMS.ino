#include <NimBLEDevice.h>
#include <bms2.h>
#include "driver/twai.h"

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

// -----------------------------------------------------------------------
// BLE globals
// -----------------------------------------------------------------------
static NimBLEClient*               pClient  = nullptr;
static NimBLERemoteCharacteristic* pTxChar  = nullptr;
static NimBLERemoteCharacteristic* pRxChar  = nullptr;
static NimBLEAddress               bmsMacAddress(BMS_MAC, BLE_ADDR_PUBLIC);

static bool doConnect = false;
static bool connected = false;

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
// Stream wrapper — buffers until full JBD packet (0x77) then sends at once
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
        Serial.println("✅ BLE Connected!");
        connected = true;
    }
    void onDisconnect(NimBLEClient* pC, int reason) override {
        Serial.printf("⚠️  BLE Disconnected (reason: %d) — retrying\n", reason);
        connected = false;
        doConnect = true;
    }
};

// -----------------------------------------------------------------------
// BLE connect
// -----------------------------------------------------------------------
bool connectToBMS() {
    Serial.printf("🔌 Connecting to BMS %s...\n", BMS_MAC);

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks(), false);
    pClient->setConnectionParams(6, 6, 0, 51);
    pClient->setConnectTimeout(30);

    if (!pClient->connect(bmsMacAddress)) {
        Serial.println("❌ BLE connect() failed");
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }

    NimBLERemoteService* pSvc = pClient->getService(SERVICE_UUID);
    if (!pSvc) { Serial.println("❌ Service FF00 not found"); pClient->disconnect(); return false; }

    pRxChar = pSvc->getCharacteristic(RX_UUID);
    if (!pRxChar) { Serial.println("❌ RX char FF01 not found"); pClient->disconnect(); return false; }
    if (pRxChar->canNotify()) pRxChar->subscribe(true, notifyCallback);

    pTxChar = pSvc->getCharacteristic(TX_UUID);
    if (!pTxChar) { Serial.println("❌ TX char FF02 not found"); pClient->disconnect(); return false; }

    Serial.println("✅ BLE fully connected!");
    return true;
}

// -----------------------------------------------------------------------
// Serial log — every 5 seconds
// -----------------------------------------------------------------------
void logBMSData() {
    uint8_t numCells = bms.get_num_cells();
    float   voltage  = bms.get_voltage();

    if (numCells == 0 || voltage == 0.0f) {
        Serial.println("⏳ Waiting for valid BMS data...");
        return;
    }

    // Calculate cell stats
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
// Setup
// -----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== JBD BMS → Solis CAN Bridge ===");

    // Start CAN (defined in CAN_Pylontech.ino)
    setupCAN();

    // Start BLE
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
    // --- BLE connect / reconnect ---
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

    // --- BMS state machine ---
    if (connected) {
        bms.main_task(true);
    } else if (!doConnect) {
        delay(1000);
        Serial.print(".");
        return;
    }

    unsigned long now = millis();

    // --- Send CAN frames every 100ms ---
    if (now - lastCAN >= CAN_INTERVAL_MS) {
        lastCAN = now;
        if (connected) sendCANFrames(bms);
    }

    // --- Log to serial every 5 seconds ---
    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        if (connected) logBMSData();
    }
}