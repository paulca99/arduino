#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>

static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_rx("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_tx("0000ff02-0000-1000-8000-00805f9b34fb");

#define STARTUP_SCAN_TIMEOUT_MS   15000
#define RECONNECT_SCAN_TIMEOUT_MS  6000
#define SCAN_SLICE_MS              1200
#define MIN_SCAN_SLICE_MS           200
// Matches the scan settings used in WORKING_BMS_BLE/WORKING_BMS_BLE.ino.
#define BLE_SCAN_INTERVAL_UNITS    1349
#define BLE_SCAN_WINDOW_UNITS       449
#define MS_PER_SECOND              1000
#define CONNECT_DELAY_MS            100
#define DISCONNECT_CLEANUP_DELAY_MS  50
#define REQUEST_DELAY_MS            300
#define RESPONSE_TIMEOUT_MS        2500
#define RESPONSE_POLL_MS             20
#define BETWEEN_BATTERIES_MS        200
#define BETWEEN_CYCLES_MS          1000
#define LOG_INTERVAL_MS            5000
#define RECONNECT_INTERVAL_MS     30000
#define MAX_PACKET_LEN             128

struct BatteryState {
    const char* name;
    const char* mac;
    bool enabled;

    BLEAdvertisedDevice* advertisedDevice = nullptr;
    BLEClient* client = nullptr;
    BLERemoteService* service = nullptr;
    BLERemoteCharacteristic* rx = nullptr;
    BLERemoteCharacteristic* tx = nullptr;

    bool seen = false;
    bool connected = false;

    unsigned long connectedAtMs = 0;
    unsigned long lastGoodDataMs = 0;
    unsigned long lastRequestMs = 0;
    unsigned long lastDisconnectMs = 0;
    unsigned long nextReconnectMs = 0;
    unsigned long okReads = 0;
    unsigned long failedReads = 0;
    unsigned long disconnectCount = 0;

    float voltage = 0.0f;
    float current = 0.0f;
    uint8_t soc = 0;

    uint8_t packetBuf[MAX_PACKET_LEN] = {0};
    int packetLen = 0;
    int expectedLen = 0;
    bool packetError = false;
    bool gotPacket03 = false;
};

static BatteryState batteries[] = {
    {"Growatt", "a5:c2:37:49:c7:a2", true},
    {"Solax", "a4:c1:37:20:4e:3b", true},
    {"SP14S004P14S40A", "a5:c2:37:51:85:7f", true},
};

static const int BATTERY_COUNT = sizeof(batteries) / sizeof(batteries[0]);

static BLEScan* pBLEScan = nullptr;
static unsigned long cycleCount = 0;
static unsigned long lastSummaryMs = 0;

static int findBatteryByClient(BLEClient* client) {
    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (batteries[i].client == client) return i;
    }
    return -1;
}

static int findBatteryByRx(BLERemoteCharacteristic* characteristic) {
    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (batteries[i].rx == characteristic) return i;
    }
    return -1;
}

static bool isBatteryConnected(const BatteryState& battery) {
    return battery.enabled &&
           battery.client != nullptr &&
           battery.connected &&
           battery.client->isConnected() &&
           battery.rx != nullptr &&
           battery.tx != nullptr;
}

static bool hasDeadlinePassed(unsigned long deadlineMs) {
    return (unsigned long)(millis() - deadlineMs) < 0x80000000UL;
}

static uint32_t scanDurationSeconds(unsigned long durationMs) {
    return (uint32_t)((durationMs + (MS_PER_SECOND - 1)) / MS_PER_SECOND);
}

static int enabledBatteryCount() {
    int count = 0;
    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (batteries[i].enabled) count++;
    }
    return count;
}

static int seenBatteryCount() {
    int count = 0;
    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (batteries[i].enabled && batteries[i].seen) count++;
    }
    return count;
}

static void resetPacketAssembly(BatteryState& battery) {
    battery.packetLen = 0;
    battery.expectedLen = 0;
    battery.packetError = false;
    battery.gotPacket03 = false;
}

static uint16_t calcChecksum(const uint8_t* data, int length) {
    if (data == nullptr || length < 7) return 0;

    int checksum = 0x10000;
    int dataLength = data[3];
    if (length < dataLength + 7) return 0;

    for (int i = 0; i < dataLength + 1; i++) checksum -= data[i + 3];
    return (uint16_t)checksum;
}

static bool checksumValid(const uint8_t* data, int length) {
    if (data == nullptr) return false;
    if (length < 7) return false;
    int checksumIndex = data[3] + 4;
    if (checksumIndex + 1 >= length) return false;
    uint16_t got = ((uint16_t)data[checksumIndex] << 8) | data[checksumIndex + 1];
    return calcChecksum(data, length) == got;
}

static void notifyCallback(BLERemoteCharacteristic* characteristic,
                           uint8_t* pData,
                           size_t length,
                           bool isNotify) {
    (void)isNotify;

    int index = findBatteryByRx(characteristic);
    if (index < 0) return;

    BatteryState& battery = batteries[index];
    if (battery.packetError) return;

    if (battery.packetLen == 0) {
        if (length == 0 || pData[0] != 0xDD) return;
        battery.packetError = (pData[2] != 0x00);
        battery.expectedLen = pData[3] + 7;
    }

    if (battery.packetLen + (int)length > MAX_PACKET_LEN) {
        battery.packetError = true;
        return;
    }

    memcpy(battery.packetBuf + battery.packetLen, pData, length);
    battery.packetLen += (int)length;

    if (battery.packetError ||
        battery.expectedLen <= 0 ||
        battery.packetLen != battery.expectedLen) {
        return;
    }

    if (!checksumValid(battery.packetBuf, battery.packetLen) ||
        battery.packetBuf[1] != 0x03) {
        battery.packetError = true;
        return;
    }

    if (battery.packetLen <= 23) {
        battery.packetError = true;
        return;
    }

    uint16_t rawVolts = ((uint16_t)battery.packetBuf[4] << 8) | battery.packetBuf[5];
    int16_t rawCurrent = ((int16_t)battery.packetBuf[6] << 8) | battery.packetBuf[7];

    battery.voltage = rawVolts / 100.0f;
    battery.current = rawCurrent / 100.0f;
    battery.soc = battery.packetBuf[23];
    battery.gotPacket03 = true;
    battery.lastGoodDataMs = millis();
}

class ClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* client) override {
        int index = findBatteryByClient(client);
        if (index < 0) return;

        batteries[index].connected = true;
        batteries[index].connectedAtMs = millis();
        batteries[index].nextReconnectMs = 0;
        Serial.printf("[%s] connected\n", batteries[index].name);
    }

    void onDisconnect(BLEClient* client) override {
        int index = findBatteryByClient(client);
        if (index < 0) return;

        BatteryState& battery = batteries[index];
        battery.connected = false;
        battery.service = nullptr;
        battery.rx = nullptr;
        battery.tx = nullptr;
        battery.lastDisconnectMs = millis();
        battery.nextReconnectMs = battery.lastDisconnectMs + RECONNECT_INTERVAL_MS;
        battery.disconnectCount++;

        Serial.printf("[%s] disconnected (drops=%lu)\n",
                      battery.name,
                      battery.disconnectCount);
    }
};

class DiscoveryCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (!advertisedDevice.haveServiceUUID() ||
            !advertisedDevice.isAdvertisingService(serviceUUID)) {
            return;
        }

        String seenMac = advertisedDevice.getAddress().toString().c_str();
        seenMac.toLowerCase();

        for (int i = 0; i < BATTERY_COUNT; i++) {
            if (!batteries[i].enabled) continue;

            String targetMac = batteries[i].mac;
            targetMac.toLowerCase();
            if (seenMac != targetMac) continue;

            batteries[i].seen = true;
            if (batteries[i].advertisedDevice != nullptr) {
                delete batteries[i].advertisedDevice;
            }
            batteries[i].advertisedDevice = new BLEAdvertisedDevice(advertisedDevice);

            Serial.printf("[%s] discovered at %s\n",
                          batteries[i].name,
                          batteries[i].mac);

            if (seenBatteryCount() >= enabledBatteryCount()) {
                BLEDevice::getScan()->stop();
            }
            return;
        }
    }
};

static ClientCallbacks clientCallbacks;

static void cleanupBatteryClient(BatteryState& battery) {
    if (battery.client != nullptr) {
        if (battery.client->isConnected()) {
            battery.client->disconnect();
            delay(DISCONNECT_CLEANUP_DELAY_MS);
        }
        delete battery.client;
        battery.client = nullptr;
    }

    battery.connected = false;
    battery.service = nullptr;
    battery.rx = nullptr;
    battery.tx = nullptr;
    resetPacketAssembly(battery);
}

static bool scanForAllEnabledBatteries(unsigned long timeoutMs) {
    for (int i = 0; i < BATTERY_COUNT; i++) {
        batteries[i].seen = false;
        if (batteries[i].advertisedDevice != nullptr) {
            delete batteries[i].advertisedDevice;
            batteries[i].advertisedDevice = nullptr;
        }
    }

    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(BLE_SCAN_INTERVAL_UNITS);
    pBLEScan->setWindow(BLE_SCAN_WINDOW_UNITS);

    unsigned long deadlineMs = millis() + timeoutMs;
    while (millis() < deadlineMs && seenBatteryCount() < enabledBatteryCount()) {
        unsigned long remainingMs = deadlineMs - millis();
        unsigned long sliceMs = (remainingMs < SCAN_SLICE_MS) ? remainingMs : SCAN_SLICE_MS;
        if (sliceMs < MIN_SCAN_SLICE_MS) sliceMs = MIN_SCAN_SLICE_MS;

        pBLEScan->start(scanDurationSeconds(sliceMs), false);
        pBLEScan->clearResults();
    }

    return seenBatteryCount() == enabledBatteryCount();
}

static bool scanForBattery(BatteryState& battery, unsigned long timeoutMs) {
    battery.seen = false;
    if (battery.advertisedDevice != nullptr) {
        delete battery.advertisedDevice;
        battery.advertisedDevice = nullptr;
    }

    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(BLE_SCAN_INTERVAL_UNITS);
    pBLEScan->setWindow(BLE_SCAN_WINDOW_UNITS);

    unsigned long deadlineMs = millis() + timeoutMs;
    while (millis() < deadlineMs && !battery.seen) {
        unsigned long remainingMs = deadlineMs - millis();
        unsigned long sliceMs = (remainingMs < SCAN_SLICE_MS) ? remainingMs : SCAN_SLICE_MS;
        if (sliceMs < MIN_SCAN_SLICE_MS) sliceMs = MIN_SCAN_SLICE_MS;

        pBLEScan->start(scanDurationSeconds(sliceMs), false);
        pBLEScan->clearResults();
    }

    return battery.seen;
}

static bool connectBattery(BatteryState& battery) {
    if (!battery.enabled) return false;
    if (isBatteryConnected(battery)) return true;
    if (battery.advertisedDevice == nullptr) return false;

    cleanupBatteryClient(battery);

    battery.client = BLEDevice::createClient();
    if (battery.client == nullptr) {
        Serial.printf("[%s] failed to create BLE client\n", battery.name);
        return false;
    }

    battery.client->setClientCallbacks(&clientCallbacks);

    delay(CONNECT_DELAY_MS);
    if (!battery.client->connect(battery.advertisedDevice)) {
        Serial.printf("[%s] connect() failed\n", battery.name);
        cleanupBatteryClient(battery);
        return false;
    }

    battery.service = battery.client->getService(serviceUUID);
    if (battery.service == nullptr) {
        Serial.printf("[%s] FF00 service not found\n", battery.name);
        cleanupBatteryClient(battery);
        return false;
    }

    battery.rx = battery.service->getCharacteristic(charUUID_rx);
    if (battery.rx == nullptr || !battery.rx->canNotify()) {
        Serial.printf("[%s] FF01 notify characteristic not found\n", battery.name);
        cleanupBatteryClient(battery);
        return false;
    }
    battery.rx->registerForNotify(notifyCallback);

    battery.tx = battery.service->getCharacteristic(charUUID_tx);
    if (battery.tx == nullptr ||
        (!battery.tx->canWrite() && !battery.tx->canWriteNoResponse())) {
        Serial.printf("[%s] FF02 write characteristic not found\n", battery.name);
        cleanupBatteryClient(battery);
        return false;
    }

    delay(REQUEST_DELAY_MS);
    battery.connected = battery.client->isConnected();
    battery.connectedAtMs = millis();

    Serial.printf("[%s] ready: connected + notifications registered\n", battery.name);
    return battery.connected;
}

static bool requestBatterySnapshot(BatteryState& battery) {
    static uint8_t cmd3[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};

    if (!isBatteryConnected(battery)) return false;

    resetPacketAssembly(battery);
    battery.lastRequestMs = millis();
    battery.tx->writeValue(cmd3, sizeof(cmd3), false);

    unsigned long deadlineMs = millis() + RESPONSE_TIMEOUT_MS;
    while (millis() < deadlineMs) {
        if (!isBatteryConnected(battery)) break;
        if (battery.gotPacket03) {
            battery.okReads++;
            return true;
        }
        delay(RESPONSE_POLL_MS);
    }

    battery.failedReads++;
    return false;
}

static bool reconnectBattery(BatteryState& battery) {
    if (isBatteryConnected(battery)) return true;

    Serial.printf("[%s] reconnect attempt\n", battery.name);

    if (battery.advertisedDevice == nullptr) {
        if (!scanForBattery(battery, RECONNECT_SCAN_TIMEOUT_MS)) {
            Serial.printf("[%s] not seen during reconnect scan\n", battery.name);
            battery.nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
            return false;
        }
    }

    if (connectBattery(battery)) return true;

    if (!scanForBattery(battery, RECONNECT_SCAN_TIMEOUT_MS)) {
        Serial.printf("[%s] rediscovery failed\n", battery.name);
        battery.nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
        return false;
    }

    bool ok = connectBattery(battery);
    if (!ok) battery.nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
    return ok;
}

static void printSummary() {
    unsigned long now = millis();
    int connectedCount = 0;

    Serial.printf("\n=== Persistent BLE summary @ %lus ===\n", now / 1000UL);
    for (int i = 0; i < BATTERY_COUNT; i++) {
        BatteryState& battery = batteries[i];
        if (!battery.enabled) continue;

        if (isBatteryConnected(battery)) connectedCount++;

        unsigned long ageSec = battery.lastGoodDataMs == 0
                             ? 0
                             : (now - battery.lastGoodDataMs) / 1000UL;

        Serial.printf("[%s] connected=%s seen=%s ok=%lu fail=%lu drops=%lu last=%lu s",
                      battery.name,
                      isBatteryConnected(battery) ? "yes" : "no",
                      battery.seen ? "yes" : "no",
                      battery.okReads,
                      battery.failedReads,
                      battery.disconnectCount,
                      ageSec);

        if (battery.lastGoodDataMs != 0) {
            Serial.printf("  %.2f V  %.2f A  SoC %u%%",
                          battery.voltage,
                          battery.current,
                          battery.soc);
        }
        Serial.println();
    }

    Serial.printf("Connected %d/%d enabled batteries\n",
                  connectedCount,
                  enabledBatteryCount());
    Serial.println("========================================");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== Experimental classic BLE persistent 3-battery test ===");

    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new DiscoveryCallbacks());

    Serial.printf("Configured entries: %d\n", BATTERY_COUNT);
    for (int i = 0; i < BATTERY_COUNT; i++) {
        Serial.printf("  [%d] %s  %s  enabled=%s\n",
                      i,
                      batteries[i].name,
                      batteries[i].mac,
                      batteries[i].enabled ? "true" : "false");
    }

    bool allSeen = scanForAllEnabledBatteries(STARTUP_SCAN_TIMEOUT_MS);
    Serial.printf("Startup discovery: seen=%d/%d\n",
                  seenBatteryCount(),
                  enabledBatteryCount());
    if (!allSeen) {
        Serial.println("Some enabled batteries were not found during startup scan.");
    }

    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (!batteries[i].enabled) continue;

        if (!batteries[i].seen) {
            Serial.printf("[%s] startup connect skipped (not discovered)\n",
                          batteries[i].name);
            batteries[i].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
            continue;
        }

        bool ok = connectBattery(batteries[i]);
        Serial.printf("[%s] startup connect %s\n",
                      batteries[i].name,
                      ok ? "OK" : "FAIL");
        if (!ok) {
            batteries[i].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
        }
    }

    printSummary();
}

void loop() {
    cycleCount++;
    Serial.printf("\n--- Persistent cycle %lu ---\n", cycleCount);

    for (int i = 0; i < BATTERY_COUNT; i++) {
        BatteryState& battery = batteries[i];
        if (!battery.enabled) continue;

        if (isBatteryConnected(battery)) {
            bool ok = requestBatterySnapshot(battery);
            if (ok) {
                Serial.printf("[%s] OK   %.2f V  %.2f A  SoC %u%%\n",
                              battery.name,
                              battery.voltage,
                              battery.current,
                              battery.soc);
            } else {
                Serial.printf("[%s] FAIL waiting for 0x03 response\n",
                              battery.name);
            }
        } else if (battery.nextReconnectMs != 0 &&
                   hasDeadlinePassed(battery.nextReconnectMs)) {
            bool ok = reconnectBattery(battery);
            Serial.printf("[%s] reconnect %s\n",
                          battery.name,
                          ok ? "OK" : "FAIL");
        } else {
            Serial.printf("[%s] offline\n", battery.name);
        }

        delay(BETWEEN_BATTERIES_MS);
    }

    if (millis() - lastSummaryMs >= LOG_INTERVAL_MS) {
        lastSummaryMs = millis();
        printSummary();
    }

    delay(BETWEEN_CYCLES_MS);
}
