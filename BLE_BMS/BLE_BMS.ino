#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <new>
#include <string.h>
#include "driver/twai.h"

// -----------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------
static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_rx("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_tx("0000ff02-0000-1000-8000-00805f9b34fb");

#define CAN_TX_PIN     GPIO_NUM_21
#define CAN_RX_PIN     GPIO_NUM_19

#define CAN_INTERVAL_MS            100
#define LOG_INTERVAL_MS           5000
#define HEARTBEAT_INTERVAL_MS     1000
#define BLE_TIMEOUT_MS  (3UL * 60UL * 1000UL)
#define DATA_FRESH_MS            15000UL

#define WIFI_SSID           "TP-LINK_73F3"
#define WIFI_PASSWORD_UPPER "DEADBEEF"
#define WIFI_PASSWORD_LOWER "deadbeef"
#define WIFI_CONNECT_TIMEOUT_MS  10000

#define STARTUP_SCAN_TIMEOUT_MS   15000
#define RECONNECT_SCAN_TIMEOUT_MS  6000
#define SCAN_SLICE_MS              1200
#define MIN_SCAN_SLICE_MS           200
#define USER_SCAN_TIMEOUT_MS      10000
#define BLE_SCAN_INTERVAL_UNITS    1349
#define BLE_SCAN_WINDOW_UNITS       449
#define CONNECT_DELAY_MS            100
#define REQUEST_DELAY_MS            150
#define REQUEST_INTERVAL_MS        2000
#define RESPONSE_TIMEOUT_MS        1200
#define RECONNECT_INTERVAL_MS     30000
#define MAX_PACKET_LEN             128
#define MAX_CELLS                   24
#define PACKET03_MIN_LEN            24
#define PACKET03_SOC_INDEX          23
#define PACKET03_FET_INDEX          24
#define PACKET03_TEMP_COUNT_IDX_A   26
#define PACKET03_TEMP_COUNT_IDX_B   25

enum BatteryRequestStage : uint8_t {
    REQUEST_STAGE_IDLE = 0,     // No outstanding request for this battery.
    REQUEST_STAGE_WAIT_03,      // Waiting for the basic-info response.
    REQUEST_STAGE_READY_04,     // 0x03 completed; send 0x04 on the next poll step.
    REQUEST_STAGE_WAIT_04       // Waiting for the cell-voltage response.
};

#define MAX_BATTERIES     6
#define MAX_NAME_LEN     20
#define MAX_MAC_LEN      18
#define MAX_SCAN_RESULTS 10

struct BatteryConfig {
    char name[MAX_NAME_LEN];
    char mac[MAX_MAC_LEN];
    bool enabled;
};

static BatteryConfig batteryConfigs[MAX_BATTERIES];
static int batteryCount = 0;

struct ScanResult {
    char mac[MAX_MAC_LEN];
    char name[MAX_NAME_LEN];
};

static ScanResult scanResults[MAX_SCAN_RESULTS];
static int scanResultCount = 0;
static bool userScanActive = false;
static bool scanDone = false;

struct BatteryState {
    BLEAdvertisedDevice* advertisedDevice = nullptr;
    BLEClient* client = nullptr;
    BLERemoteService* service = nullptr;
    BLERemoteCharacteristic* rx = nullptr;
    BLERemoteCharacteristic* tx = nullptr;

    bool seen = false;
    bool connected = false;
    bool hasData = false;
    bool hasTemperature = false;
    bool chargeMos = false;
    bool dischargeMos = false;

    unsigned long connectedAtMs = 0;
    unsigned long lastGoodDataMs = 0;
    unsigned long lastDisconnectMs = 0;
    unsigned long nextReconnectMs = 0;
    unsigned long lastRequestMs = 0;
    unsigned long requestDeadlineMs = 0;
    unsigned long lastRequestStartedMs = 0;
    unsigned long lastRequestCompletedMs = 0;
    unsigned long okReads = 0;
    unsigned long failedReads = 0;
    unsigned long requestTimeouts = 0;
    unsigned long disconnectCount = 0;

    float voltage = 0.0f;
    float current = 0.0f;
    float temperature = 0.0f;
    uint8_t soc = 0;

    uint16_t cellMv[MAX_CELLS] = {0};
    uint8_t cellCount = 0;
    bool hasCellData = false;
    unsigned long lastCellDataMs = 0;

    uint8_t packetBuf[MAX_PACKET_LEN] = {0};
    int packetLen = 0;
    int expectedLen = 0;
    bool packetError = false;
    bool gotPacket03 = false;
    bool gotPacket04 = false;
    bool requestInFlight = false;
    BatteryRequestStage requestStage = REQUEST_STAGE_IDLE;
};

struct AggregateSnapshot {
    bool valid = false;
    uint8_t contributingBatteries = 0;
    float voltage = 0.0f;
    float current = 0.0f;
    uint8_t soc = 0;
    float temperature = 0.0f;
    bool hasTemperature = false;
    bool chargeAllowed = false;
    bool dischargeAllowed = false;
    unsigned long lastFreshMs = 0;
};

static BatteryState batteries[MAX_BATTERIES];
static AggregateSnapshot aggregate;

static BLEScan* pBLEScan = nullptr;
static bool wifiReady = false;

WebServer server(80);

// CAN API from CAN_Pylontech.ino
void setupCAN();
void sendCANFrames(float voltage,
                   float current,
                   uint8_t soc,
                   float temperature,
                   bool chargeAllowed,
                   bool dischargeAllowed);

// Battery management web handlers (defined later)
static void handleBatteriesPage();
static void handleBatteriesScan();
static void handleBatteriesAdd();
static void handleBatteriesRemove();

static const char* wifiStatusToString(wl_status_t status) {
    switch (status) {
        case WL_NO_SHIELD: return "NO_SHIELD";
        case WL_IDLE_STATUS: return "IDLE";
        case WL_NO_SSID_AVAIL: return "NO_SSID";
        case WL_SCAN_COMPLETED: return "SCAN_DONE";
        case WL_CONNECTED: return "CONNECTED";
        case WL_CONNECT_FAILED: return "CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "CONNECTION_LOST";
        case WL_DISCONNECTED: return "DISCONNECTED";
        default: return "UNKNOWN";
    }
}

static int findBatteryByClient(BLEClient* client) {
    for (int i = 0; i < batteryCount; i++) {
        if (batteries[i].client == client) return i;
    }
    return -1;
}

static int findBatteryByRx(BLERemoteCharacteristic* characteristic) {
    for (int i = 0; i < batteryCount; i++) {
        if (batteries[i].rx == characteristic) return i;
    }
    return -1;
}

static bool hasDeadlinePassed(unsigned long deadlineMs) {
    return (int32_t)(millis() - deadlineMs) >= 0;
}

static uint32_t scanDurationSeconds(unsigned long durationMs) {
    return (uint32_t)((durationMs + 999UL) / 1000UL);
}

static bool isBatteryEnabled(int index) {
    return index >= 0 && index < batteryCount && batteryConfigs[index].enabled;
}

static bool isBatteryConnected(int index) {
    if (!isBatteryEnabled(index)) return false;

    BatteryState& battery = batteries[index];
    return battery.client != nullptr &&
           battery.connected &&
           battery.client->isConnected() &&
           battery.rx != nullptr &&
           battery.tx != nullptr;
}

static bool shouldAttemptReconnect(int index, unsigned long nowMs) {
    if (!isBatteryEnabled(index)) return false;
    if (isBatteryConnected(index)) return false;
    if (batteries[index].nextReconnectMs == 0) return false;
    return (int32_t)(nowMs - batteries[index].nextReconnectMs) >= 0;
}

static bool isAggregateUsable(const AggregateSnapshot& snap, unsigned long nowMs) {
    if (snap.valid) return true;
    return snap.lastFreshMs != 0 && (nowMs - snap.lastFreshMs < BLE_TIMEOUT_MS);
}

static int enabledBatteryCount() {
    int count = 0;
    for (int i = 0; i < batteryCount; i++) {
        if (batteryConfigs[i].enabled) count++;
    }
    return count;
}

static int seenBatteryCount() {
    int count = 0;
    for (int i = 0; i < batteryCount; i++) {
        if (batteryConfigs[i].enabled && batteries[i].seen) count++;
    }
    return count;
}

static int connectedBatteryCount() {
    int count = 0;
    for (int i = 0; i < batteryCount; i++) {
        if (isBatteryConnected(i)) count++;
    }
    return count;
}

static void logBatteryDebugState(int index, const char* prefix) {
    if (index < 0 || index >= batteryCount) return;

    BatteryState& battery = batteries[index];
    unsigned long now = millis();
    unsigned long ageGood = battery.lastGoodDataMs == 0 ? 0 : (now - battery.lastGoodDataMs);
    unsigned long ageReq = battery.lastRequestMs == 0 ? 0 : (now - battery.lastRequestMs);
    long deadlineIn = battery.requestInFlight ? (long)(battery.requestDeadlineMs - now) : 0L;

    Serial.printf(
        "%s [%s] enabled=%s seen=%s connected=%s hasData=%s hasCellData=%s reqInFlight=%s "
        "lastReqAge=%lu lastGoodAge=%lu deadlineIn=%ld packetLen=%d expectedLen=%d "
        "packetError=%s stage=%u gotPacket03=%s gotPacket04=%s ok=%lu fail=%lu timeouts=%lu drops=%lu heap=%u wifi=%s\n",
        prefix,
        batteryConfigs[index].name,
        batteryConfigs[index].enabled ? "yes" : "no",
        battery.seen ? "yes" : "no",
        isBatteryConnected(index) ? "yes" : "no",
        battery.hasData ? "yes" : "no",
        battery.hasCellData ? "yes" : "no",
        battery.requestInFlight ? "yes" : "no",
        ageReq,
        ageGood,
        deadlineIn,
        battery.packetLen,
        battery.expectedLen,
        battery.packetError ? "yes" : "no",
        (unsigned)battery.requestStage,
        battery.gotPacket03 ? "yes" : "no",
        battery.gotPacket04 ? "yes" : "no",
        battery.okReads,
        battery.failedReads,
        battery.requestTimeouts,
        battery.disconnectCount,
        ESP.getFreeHeap(),
        wifiStatusToString(WiFi.status())
    );
}

static void logSystemDebugSummary(const char* prefix) {
    unsigned long now = millis();
    Serial.printf(
        "%s now=%lu heap=%u wifi=%s connected=%d/%d enabled=%d\n",
        prefix,
        now,
        ESP.getFreeHeap(),
        wifiStatusToString(WiFi.status()),
        connectedBatteryCount(),
        batteryCount,
        enabledBatteryCount()
    );

    for (int i = 0; i < batteryCount; i++) {
        if (!batteryConfigs[i].enabled) continue;
        logBatteryDebugState(i, "  state");
    }
}

static void resetPacketAssembly(BatteryState& battery) {
    battery.packetLen = 0;
    battery.expectedLen = 0;
    battery.packetError = false;
    battery.gotPacket03 = false;
    battery.gotPacket04 = false;
}

static void clearCellData(BatteryState& battery) {
    battery.hasCellData = false;
    battery.cellCount = 0;
    battery.lastCellDataMs = 0;
    memset(battery.cellMv, 0, sizeof(battery.cellMv));
}

static uint16_t calcChecksum(const uint8_t* data, int length) {
    if (data == nullptr || length < 7) return 0;

    int dataLength = data[3];
    if (length < dataLength + 7) return 0;

    uint32_t checksum = 0x10000UL;
    for (int i = 0; i < dataLength + 1; i++) checksum -= data[i + 3];
    return (uint16_t)checksum;
}

static bool checksumValid(const uint8_t* data, int length) {
    if (data == nullptr || length < 7) return false;

    int checksumIndex = data[3] + 4;
    if (checksumIndex + 1 >= length) return false;

    uint16_t got = ((uint16_t)data[checksumIndex] << 8) | data[checksumIndex + 1];
    return calcChecksum(data, length) == got;
}

static void parsePacket03(BatteryState& battery) {
    if (battery.packetLen < PACKET03_MIN_LEN) {
        battery.packetError = true;
        return;
    }

    uint16_t rawVolts = ((uint16_t)battery.packetBuf[4] << 8) | battery.packetBuf[5];
    int16_t rawCurrent = (int16_t)(((uint16_t)battery.packetBuf[6] << 8) | battery.packetBuf[7]);

    battery.voltage = rawVolts / 100.0f;
    battery.current = rawCurrent / 100.0f;
    battery.soc = battery.packetBuf[PACKET03_SOC_INDEX];

    uint8_t mosStatus = battery.packetLen > PACKET03_FET_INDEX ? battery.packetBuf[PACKET03_FET_INDEX] : 0;
    battery.chargeMos = (mosStatus & 0x01U) != 0;
    battery.dischargeMos = (mosStatus & 0x02U) != 0;

    battery.hasTemperature = false;
    battery.temperature = 0.0f;

    int tempCountIndex = -1;
    if (battery.packetLen > PACKET03_TEMP_COUNT_IDX_A) {
        tempCountIndex = PACKET03_TEMP_COUNT_IDX_A;
    } else if (battery.packetLen > PACKET03_TEMP_COUNT_IDX_B) {
        tempCountIndex = PACKET03_TEMP_COUNT_IDX_B;
    }
    if (tempCountIndex >= 0) {
        uint8_t tempCount = battery.packetBuf[tempCountIndex];
        int tempStart = tempCountIndex + 1;
        if (tempCount > 0 && tempStart + 1 < battery.packetLen) {
            uint16_t rawTemp = ((uint16_t)battery.packetBuf[tempStart] << 8) | battery.packetBuf[tempStart + 1];
            battery.temperature = (rawTemp / 10.0f) - 273.15f;
            battery.hasTemperature = true;
        }
    }

    battery.hasData = true;
    battery.gotPacket03 = true;
    battery.lastGoodDataMs = millis();
}

static void parsePacket04(BatteryState& battery) {
    if (battery.packetLen < 7) {
        battery.packetError = true;
        return;
    }

    int payloadLen = battery.packetBuf[3];
    if (payloadLen <= 0 || (payloadLen % 2) != 0) {
        battery.packetError = true;
        return;
    }

    int count = payloadLen / 2;
    if (count > MAX_CELLS) count = MAX_CELLS;

    battery.cellCount = (uint8_t)count;
    for (int i = 0; i < count; i++) {
        int offset = 4 + (i * 2);
        battery.cellMv[i] = ((uint16_t)battery.packetBuf[offset] << 8) |
                            battery.packetBuf[offset + 1];
    }

    for (int i = count; i < MAX_CELLS; i++) {
        battery.cellMv[i] = 0;
    }

    battery.hasCellData = count > 0;
    battery.lastCellDataMs = battery.hasCellData ? millis() : 0;
    battery.gotPacket04 = true;
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
        Serial.printf("[DBG] [%s] notify start chunkLen=%u expected=%d status=0x%02X\n",
                      batteryConfigs[index].name,
                      (unsigned)length,
                      battery.expectedLen,
                      length > 2 ? pData[2] : 0xFF);
    }

    if (battery.packetLen + (int)length > MAX_PACKET_LEN) {
        Serial.printf("[DBG] [%s] notify overflow packetLen=%d add=%u\n",
                      batteryConfigs[index].name,
                      battery.packetLen,
                      (unsigned)length);
        battery.packetError = true;
        return;
    }

    memcpy(battery.packetBuf + battery.packetLen, pData, length);
    battery.packetLen += (int)length;

    if (battery.packetError || battery.expectedLen <= 0 || battery.packetLen != battery.expectedLen) {
        return;
    }

    if (!checksumValid(battery.packetBuf, battery.packetLen)) {
        Serial.printf("[DBG] [%s] checksum fail type=0x%02X len=%d\n",
                      batteryConfigs[index].name,
                      battery.packetBuf[1],
                      battery.packetLen);
        battery.packetError = true;
        return;
    }

    uint8_t packetType = battery.packetBuf[1];
    bool expect03 = battery.requestStage == REQUEST_STAGE_WAIT_03;
    bool expect04 = battery.requestStage == REQUEST_STAGE_WAIT_04;
    if ((packetType == 0x03 && !expect03) ||
        (packetType == 0x04 && !expect04) ||
        (packetType != 0x03 && packetType != 0x04)) {
        Serial.printf("[DBG] [%s] unexpected packet type=0x%02X stage=%u len=%d\n",
                      batteryConfigs[index].name,
                      packetType,
                      (unsigned)battery.requestStage,
                      battery.packetLen);
        battery.packetError = true;
        return;
    }

    if (packetType == 0x03) {
        parsePacket03(battery);
    } else {
        parsePacket04(battery);
    }

    if (battery.packetError) return;

    if (battery.requestInFlight) {
        battery.requestInFlight = false;
        battery.requestDeadlineMs = 0;
        battery.lastRequestCompletedMs = millis();
        battery.requestStage = (packetType == 0x03) ? REQUEST_STAGE_READY_04 : REQUEST_STAGE_IDLE;

        if (packetType == 0x04) {
            // Count a successful full 0x03 -> 0x04 polling cycle once.
            battery.okReads++;
        }

        if (packetType == 0x03) {
            Serial.printf("[DBG] [%s] packet03 complete V=%.2f I=%.2f SoC=%u\n",
                          batteryConfigs[index].name,
                          battery.voltage,
                          battery.current,
                          battery.soc);
        } else {
            Serial.printf("[DBG] [%s] packet04 complete cells=%u\n",
                          batteryConfigs[index].name,
                          battery.cellCount);
        }
    }
}

class ClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* client) override {
        int index = findBatteryByClient(client);
        if (index < 0) return;

        BatteryState& battery = batteries[index];
        battery.connected = true;
        battery.connectedAtMs = millis();
        battery.nextReconnectMs = 0;
        Serial.printf("[%s] connected\n", batteryConfigs[index].name);
        logBatteryDebugState(index, "[DBG onConnect]");
    }

    void onDisconnect(BLEClient* client) override {
        int index = findBatteryByClient(client);
        if (index < 0) return;

        BatteryState& battery = batteries[index];
        battery.connected = false;
        battery.service = nullptr;
        battery.rx = nullptr;
        battery.tx = nullptr;
        battery.requestInFlight = false;
        battery.requestDeadlineMs = 0;
        battery.requestStage = REQUEST_STAGE_IDLE;
        clearCellData(battery);
        battery.lastDisconnectMs = millis();
        battery.nextReconnectMs = battery.lastDisconnectMs + RECONNECT_INTERVAL_MS;
        battery.disconnectCount++;

        Serial.printf("[%s] disconnected (drops=%lu)\n",
                      batteryConfigs[index].name,
                      battery.disconnectCount);
        logBatteryDebugState(index, "[DBG onDisconnect]");
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

        // During user-triggered scan: collect BMS candidates not already configured
        if (userScanActive && scanResultCount < MAX_SCAN_RESULTS) {
            bool alreadyKnown = false;
            for (int i = 0; i < batteryCount; i++) {
                String cfgMac = batteryConfigs[i].mac;
                cfgMac.toLowerCase();
                if (seenMac == cfgMac) { alreadyKnown = true; break; }
            }
            if (!alreadyKnown) {
                bool alreadyInResults = false;
                for (int j = 0; j < scanResultCount; j++) {
                    String rMac = scanResults[j].mac;
                    rMac.toLowerCase();
                    if (seenMac == rMac) { alreadyInResults = true; break; }
                }
                if (!alreadyInResults) {
                    strncpy(scanResults[scanResultCount].mac, seenMac.c_str(), MAX_MAC_LEN - 1);
                    scanResults[scanResultCount].mac[MAX_MAC_LEN - 1] = '\0';
                    String advName = advertisedDevice.haveName()
                                   ? advertisedDevice.getName().c_str()
                                   : "";
                    strncpy(scanResults[scanResultCount].name, advName.c_str(), MAX_NAME_LEN - 1);
                    scanResults[scanResultCount].name[MAX_NAME_LEN - 1] = '\0';
                    scanResultCount++;
                    Serial.printf("[SCAN] BMS candidate: %s (%s)\n",
                                  seenMac.c_str(), advName.c_str());
                }
            }
        }

        // Normal operation: match against configured batteries
        for (int i = 0; i < batteryCount; i++) {
            if (!batteryConfigs[i].enabled) continue;

            String targetMac = batteryConfigs[i].mac;
            targetMac.toLowerCase();
            if (seenMac != targetMac) continue;

            BLEAdvertisedDevice* discoveredDevice = new (std::nothrow) BLEAdvertisedDevice(advertisedDevice);
            if (discoveredDevice == nullptr) {
                Serial.printf("[%s] discovery allocation failed\n", batteryConfigs[i].name);
                return;
            }

            batteries[i].seen = true;
            if (batteries[i].advertisedDevice != nullptr) {
                delete batteries[i].advertisedDevice;
            }
            batteries[i].advertisedDevice = discoveredDevice;

            Serial.printf("[%s] discovered at %s\n",
                          batteryConfigs[i].name,
                          batteryConfigs[i].mac);

            // Only stop early during normal (non-user) scans
            if (!userScanActive && seenBatteryCount() >= enabledBatteryCount()) {
                BLEDevice::getScan()->stop();
            }
            return;
        }
    }
};

static ClientCallbacks clientCallbacks;
static DiscoveryCallbacks discoveryCallbacks;

static void cleanupBatteryClient(int index) {
    BatteryState& battery = batteries[index];

    Serial.printf("[DBG] cleanupBatteryClient start [%s]\n", batteryConfigs[index].name);
    logBatteryDebugState(index, "[DBG before cleanup]");

    if (battery.client != nullptr) {
        if (battery.client->isConnected()) {
            Serial.printf("[DBG] [%s] before disconnect()\n", batteryConfigs[index].name);
            battery.client->disconnect();
            Serial.printf("[DBG] [%s] after disconnect()\n", batteryConfigs[index].name);
        }
        delete battery.client;
        battery.client = nullptr;
    }

    battery.connected = false;
    battery.service = nullptr;
    battery.rx = nullptr;
    battery.tx = nullptr;
    battery.requestInFlight = false;
    battery.requestDeadlineMs = 0;
    battery.requestStage = REQUEST_STAGE_IDLE;
    clearCellData(battery);
    resetPacketAssembly(battery);

    logBatteryDebugState(index, "[DBG after cleanup]");
}

static bool scanForAllEnabledBatteries(unsigned long timeoutMs) {
    for (int i = 0; i < batteryCount; i++) {
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
    while (!hasDeadlinePassed(deadlineMs) && seenBatteryCount() < enabledBatteryCount()) {
        int32_t remainingMs = (int32_t)(deadlineMs - millis());
        if (remainingMs < 0) remainingMs = 0;

        unsigned long sliceMs = ((unsigned long)remainingMs < SCAN_SLICE_MS)
                              ? (unsigned long)remainingMs
                              : SCAN_SLICE_MS;
        if (sliceMs < MIN_SCAN_SLICE_MS) sliceMs = MIN_SCAN_SLICE_MS;

        pBLEScan->start(scanDurationSeconds(sliceMs), false);
        pBLEScan->clearResults();

        if (wifiReady) server.handleClient();
    }

    return seenBatteryCount() == enabledBatteryCount();
}

static bool scanForBattery(int index, unsigned long timeoutMs) {
    BatteryState& battery = batteries[index];
    battery.seen = false;

    if (battery.advertisedDevice != nullptr) {
        delete battery.advertisedDevice;
        battery.advertisedDevice = nullptr;
    }

    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(BLE_SCAN_INTERVAL_UNITS);
    pBLEScan->setWindow(BLE_SCAN_WINDOW_UNITS);

    unsigned long deadlineMs = millis() + timeoutMs;
    while (!hasDeadlinePassed(deadlineMs) && !battery.seen) {
        int32_t remainingMs = (int32_t)(deadlineMs - millis());
        if (remainingMs < 0) remainingMs = 0;

        unsigned long sliceMs = ((unsigned long)remainingMs < SCAN_SLICE_MS)
                              ? (unsigned long)remainingMs
                              : SCAN_SLICE_MS;
        if (sliceMs < MIN_SCAN_SLICE_MS) sliceMs = MIN_SCAN_SLICE_MS;

        pBLEScan->start(scanDurationSeconds(sliceMs), false);
        pBLEScan->clearResults();

        if (wifiReady) server.handleClient();
    }

    return battery.seen;
}

static bool connectBattery(int index) {
    if (!isBatteryEnabled(index)) return false;
    if (isBatteryConnected(index)) return true;

    BatteryState& battery = batteries[index];
    if (battery.advertisedDevice == nullptr) {
        Serial.printf("[DBG] [%s] connect skipped: no advertisedDevice\n", batteryConfigs[index].name);
        battery.nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
        return false;
    }

    Serial.printf("[DBG] [%s] connectBattery start\n", batteryConfigs[index].name);
    logBatteryDebugState(index, "[DBG before connect]");

    cleanupBatteryClient(index);

    Serial.printf("[DBG] [%s] before createClient()\n", batteryConfigs[index].name);
    battery.client = BLEDevice::createClient();
    Serial.printf("[DBG] [%s] after createClient() client=%p\n", batteryConfigs[index].name, battery.client);

    if (battery.client == nullptr) {
        Serial.printf("[%s] failed to create BLE client\n", batteryConfigs[index].name);
        return false;
    }

    battery.client->setClientCallbacks(&clientCallbacks);

    delay(CONNECT_DELAY_MS);

    Serial.printf("[DBG] [%s] before connect()\n", batteryConfigs[index].name);
    bool connectOk = battery.client->connect(battery.advertisedDevice);
    Serial.printf("[DBG] [%s] after connect() => %s\n", batteryConfigs[index].name, connectOk ? "OK" : "FAIL");

    if (!connectOk) {
        Serial.printf("[%s] connect() failed\n", batteryConfigs[index].name);
        cleanupBatteryClient(index);
        return false;
    }

    Serial.printf("[DBG] [%s] before getService()\n", batteryConfigs[index].name);
    battery.service = battery.client->getService(serviceUUID);
    Serial.printf("[DBG] [%s] after getService() service=%p\n", batteryConfigs[index].name, battery.service);

    if (battery.service == nullptr) {
        Serial.printf("[%s] FF00 service not found\n", batteryConfigs[index].name);
        cleanupBatteryClient(index);
        return false;
    }

    Serial.printf("[DBG] [%s] before getCharacteristic(RX)\n", batteryConfigs[index].name);
    battery.rx = battery.service->getCharacteristic(charUUID_rx);
    Serial.printf("[DBG] [%s] after getCharacteristic(RX) rx=%p\n", batteryConfigs[index].name, battery.rx);

    if (battery.rx == nullptr || !battery.rx->canNotify()) {
        Serial.printf("[%s] FF01 notify characteristic not found\n", batteryConfigs[index].name);
        cleanupBatteryClient(index);
        return false;
    }

    Serial.printf("[DBG] [%s] before registerForNotify()\n", batteryConfigs[index].name);
    battery.rx->registerForNotify(notifyCallback);
    Serial.printf("[DBG] [%s] after registerForNotify()\n", batteryConfigs[index].name);

    Serial.printf("[DBG] [%s] before getCharacteristic(TX)\n", batteryConfigs[index].name);
    battery.tx = battery.service->getCharacteristic(charUUID_tx);
    Serial.printf("[DBG] [%s] after getCharacteristic(TX) tx=%p\n", batteryConfigs[index].name, battery.tx);

    if (battery.tx == nullptr || (!battery.tx->canWrite() && !battery.tx->canWriteNoResponse())) {
        Serial.printf("[%s] FF02 write characteristic not found\n", batteryConfigs[index].name);
        cleanupBatteryClient(index);
        return false;
    }

    delay(REQUEST_DELAY_MS);
    battery.connected = battery.client->isConnected();
    battery.connectedAtMs = millis();
    battery.requestInFlight = false;
    battery.requestDeadlineMs = 0;
    battery.lastRequestMs = millis() - REQUEST_INTERVAL_MS;

    if (battery.connected) {
        Serial.printf("[%s] ready: connected + notifications registered\n", batteryConfigs[index].name);
    }

    logBatteryDebugState(index, "[DBG after connect]");
    return battery.connected;
}

static void serviceBatteryPolling(int index, unsigned long nowMs) {
    static uint8_t cmd3[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
    static uint8_t cmd4[7] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

    if (!isBatteryConnected(index)) return;

    BatteryState& battery = batteries[index];

    if (battery.requestInFlight) {
        if (hasDeadlinePassed(battery.requestDeadlineMs)) {
            Serial.printf("[DBG] [%s] request timeout now=%lu deadline=%lu packetLen=%d expected=%d packetError=%s\n",
                          batteryConfigs[index].name,
                          nowMs,
                          battery.requestDeadlineMs,
                          battery.packetLen,
                          battery.expectedLen,
                          battery.packetError ? "yes" : "no");
            battery.requestInFlight = false;
            battery.requestDeadlineMs = 0;
            if (battery.requestStage == REQUEST_STAGE_WAIT_04) {
                clearCellData(battery);
            }
            battery.requestStage = REQUEST_STAGE_IDLE;
            battery.failedReads++;
            battery.requestTimeouts++;
            resetPacketAssembly(battery);
            logBatteryDebugState(index, "[DBG timeout]");
        }
        return;
    }

    uint8_t* cmd = cmd3;
    size_t cmdLen = sizeof(cmd3);
    BatteryRequestStage nextStage = REQUEST_STAGE_WAIT_03;
    bool partOfCurrentCycle = false;

    if (battery.requestStage == REQUEST_STAGE_READY_04) {
        cmd = cmd4;
        cmdLen = sizeof(cmd4);
        nextStage = REQUEST_STAGE_WAIT_04;
        partOfCurrentCycle = true;
    } else if ((nowMs - battery.lastRequestMs) < REQUEST_INTERVAL_MS) {
        return;
    } else {
        // Keep interval timing anchored to the start of the 0x03 -> 0x04 cycle.
        battery.lastRequestMs = nowMs;
    }

    resetPacketAssembly(battery);
    battery.lastRequestStartedMs = nowMs;
    battery.requestDeadlineMs = nowMs + RESPONSE_TIMEOUT_MS;
    battery.requestInFlight = true;
    battery.requestStage = nextStage;

    Serial.printf("[DBG] [%s] before writeValue cmd=0x%02X cycle=%s heap=%u\n",
                  batteryConfigs[index].name,
                  cmd[2],
                  partOfCurrentCycle ? "continue" : "start",
                  ESP.getFreeHeap());

    bool ok = battery.tx->writeValue(cmd, cmdLen, false);

    Serial.printf("[DBG] [%s] after writeValue cmd=0x%02X ok=%s\n",
                  batteryConfigs[index].name,
                  cmd[2],
                  ok ? "yes" : "no");

    if (!ok) {
        battery.requestInFlight = false;
        battery.requestDeadlineMs = 0;
        if (battery.requestStage == REQUEST_STAGE_WAIT_04) {
            clearCellData(battery);
        }
        battery.requestStage = REQUEST_STAGE_IDLE;
        battery.failedReads++;
        Serial.printf("[DBG] [%s] writeValue failed immediately for cmd=0x%02X\n",
                      batteryConfigs[index].name,
                      cmd[2]);
        logBatteryDebugState(index, "[DBG write fail]");
    }
}

static bool reconnectBattery(int index) {
    if (isBatteryConnected(index)) return true;

    Serial.printf("[%s] reconnect attempt\n", batteryConfigs[index].name);
    logBatteryDebugState(index, "[DBG before reconnect]");

    if (batteries[index].advertisedDevice == nullptr) {
        if (!scanForBattery(index, RECONNECT_SCAN_TIMEOUT_MS)) {
            Serial.printf("[%s] not seen during reconnect scan\n", batteryConfigs[index].name);
            batteries[index].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
            return false;
        }
    }

    if (connectBattery(index)) return true;

    if (!scanForBattery(index, RECONNECT_SCAN_TIMEOUT_MS)) {
        Serial.printf("[%s] rediscovery failed\n", batteryConfigs[index].name);
        batteries[index].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
        return false;
    }

    bool ok = connectBattery(index);
    if (!ok) batteries[index].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
    return ok;
}

static AggregateSnapshot buildAggregateSnapshot(unsigned long now) {
    AggregateSnapshot snap;

    float sumVoltage = 0.0f;
    float sumCurrent = 0.0f;
    float sumSoc = 0.0f;
    float sumTemp = 0.0f;
    uint8_t tempCount = 0;
    bool chargeAllowedInit = false;
    bool dischargeAllowedInit = false;

    for (int i = 0; i < batteryCount; i++) {
        if (!batteryConfigs[i].enabled) continue;

        const BatteryState& battery = batteries[i];
        if (!battery.hasData || battery.lastGoodDataMs == 0) continue;
        if (now - battery.lastGoodDataMs > DATA_FRESH_MS) continue;

        sumVoltage += battery.voltage;
        sumCurrent += battery.current;
        sumSoc += battery.soc;

        if (!chargeAllowedInit) {
            snap.chargeAllowed = battery.chargeMos;
            chargeAllowedInit = true;
        } else {
            snap.chargeAllowed = snap.chargeAllowed && battery.chargeMos;
        }

        if (!dischargeAllowedInit) {
            snap.dischargeAllowed = battery.dischargeMos;
            dischargeAllowedInit = true;
        } else {
            snap.dischargeAllowed = snap.dischargeAllowed && battery.dischargeMos;
        }

        if (battery.hasTemperature) {
            sumTemp += battery.temperature;
            tempCount++;
        }

        if (battery.lastGoodDataMs > snap.lastFreshMs) snap.lastFreshMs = battery.lastGoodDataMs;
        snap.contributingBatteries++;
    }

    if (snap.contributingBatteries > 0) {
        snap.valid = true;
        snap.voltage = sumVoltage / (float)snap.contributingBatteries;
        snap.current = sumCurrent;
        snap.soc = (uint8_t)roundf(sumSoc / (float)snap.contributingBatteries);
        if (tempCount > 0) {
            snap.hasTemperature = true;
            snap.temperature = sumTemp / (float)tempCount;
        }
    }

    return snap;
}

static String htmlEscape(const String& input) {
    String out;
    out.reserve(input.length() + 16);
    for (size_t i = 0; i < input.length(); i++) {
        char c = input.charAt(i);
        switch (c) {
            case '&': out += F("&amp;"); break;
            case '<': out += F("&lt;"); break;
            case '>': out += F("&gt;"); break;
            case '"': out += F("&quot;"); break;
            case '\'': out += F("&#39;"); break;
            default: out += c; break;
        }
    }
    return out;
}

static bool getCellMinMax(const BatteryState& battery,
                          uint8_t& minIndex,
                          uint16_t& minMv,
                          uint8_t& maxIndex,
                          uint16_t& maxMv) {
    if (!battery.hasCellData || battery.cellCount == 0) return false;

    minIndex = 0;
    maxIndex = 0;
    minMv = battery.cellMv[0];
    maxMv = battery.cellMv[0];

    for (uint8_t i = 1; i < battery.cellCount; i++) {
        uint16_t mv = battery.cellMv[i];
        if (mv < minMv) {
            minMv = mv;
            minIndex = i;
        }
        if (mv > maxMv) {
            maxMv = mv;
            maxIndex = i;
        }
    }

    return true;
}

static void sendCard(const String& label, const String& value, const char* cls = nullptr) {
    String html = "<div class='card'><div class='label'>" + htmlEscape(label) + "</div><div class='value";
    if (cls != nullptr && cls[0] != '\0') {
        html += " ";
        html += cls;
    }
    html += "'>" + htmlEscape(value) + "</div></div>";
    server.sendContent(html);
}

static void handleRoot() {
    unsigned long now = millis();
    AggregateSnapshot snap = buildAggregateSnapshot(now);

    Serial.printf("[WEB] GET / heap=%u connected=%d/%d wifi=%s\n",
                  ESP.getFreeHeap(),
                  connectedBatteryCount(),
                  enabledBatteryCount(),
                  wifiStatusToString(WiFi.status()));

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    server.sendContent(F("<!DOCTYPE html><html><head>"
                         "<meta charset='UTF-8'>"
                         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                         "<meta http-equiv='refresh' content='5'>"
                         "<title>BMS Multi-Battery Monitor</title>"
                         "<style>"
                         "body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}"
                         "h1{color:#e0c97f;margin-bottom:4px}h2{color:#a0b4cc;margin-top:20px;margin-bottom:6px}"
                         "table{width:100%;border-collapse:collapse}th,td{padding:8px 10px;text-align:center;border-bottom:1px solid #2a2a4a}"
                         "th{background:#16213e;color:#a0b4cc}.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:10px;margin-bottom:16px}"
                         ".card{background:#16213e;border-radius:8px;padding:12px;text-align:center}.card .label{font-size:0.75em;color:#8899aa;margin-bottom:4px}"
                         ".card .value{font-size:1.4em;font-weight:bold}.green{color:#4caf50}.amber{color:#ff9800}.red{color:#f44336}.mono{font-family:monospace}"
                         ".btn-mgr{display:inline-block;padding:7px 18px;background:#2a6ea6;color:#fff;border-radius:5px;text-decoration:none;font-size:0.95em;margin-bottom:12px}"
                         "</style></head><body><h1>Battery BMS (Persistent BLE)</h1>"
                         "<p><a class='btn-mgr' href='/batteries'>&#9881; Manage Batteries</a></p>"));

    server.sendContent(F("<div class='grid'>"));
    sendCard("Enabled Batteries", String(enabledBatteryCount()));
    sendCard("Connected Batteries", String(connectedBatteryCount()));

    if (snap.valid) {
        sendCard("Aggregate Voltage", String(snap.voltage, 2) + " V");
        sendCard("Aggregate Current", String(snap.current, 2) + " A");
        sendCard("Aggregate SoC", String(snap.soc) + " %");
        sendCard("Aggregate Temp", snap.hasTemperature ? (String(snap.temperature, 1) + " C") : String("n/a"));
        sendCard("Fresh Batteries", String(snap.contributingBatteries));
        sendCard("Last Fresh Data", String((now - snap.lastFreshMs) / 1000UL) + " s ago");
    } else {
        sendCard("Aggregate", "No fresh data", "amber");
    }

    server.sendContent(F("</div>"));

    server.sendContent(F("<h2>Per-battery status</h2><table><tr>"
                         "<th>Name</th><th>MAC</th><th>Enabled</th><th>Connected</th><th>Voltage (V)</th><th>Current (A)</th><th>SoC (%)</th><th>Temp (C)</th><th>Min Cell (V)</th><th>Min Cell ID</th><th>Max Cell (V)</th><th>Max Cell ID</th><th>Data age (s)</th><th>Drops</th>"
                         "</tr>"));

    for (int i = 0; i < batteryCount; i++) {
        const BatteryConfig& cfg = batteryConfigs[i];
        const BatteryState& battery = batteries[i];

        String age = battery.lastGoodDataMs == 0
                   ? "-"
                   : String((now - battery.lastGoodDataMs) / 1000UL);
        uint8_t minCellIndex = 0;
        uint8_t maxCellIndex = 0;
        uint16_t minCellMv = 0;
        uint16_t maxCellMv = 0;
        bool hasCellStats = getCellMinMax(battery, minCellIndex, minCellMv, maxCellIndex, maxCellMv);
        String batteryLink = "<a href='/battery?index=" + String(i) + "'>" + htmlEscape(String(cfg.name)) + "</a>";

        String row;
        row.reserve(512);
        row = "<tr><td>" + batteryLink + "</td><td class='mono'>" + htmlEscape(String(cfg.mac)) + "</td><td>" +
                     String(cfg.enabled ? "yes" : "no") + "</td><td class='" + (isBatteryConnected(i) ? "green'>yes" : "red'>no") +
                      "</td><td>" + (battery.hasData ? String(battery.voltage, 2) : String("-")) +
                      "</td><td>" + (battery.hasData ? String(battery.current, 2) : String("-")) +
                      "</td><td>" + (battery.hasData ? String(battery.soc) : String("-")) +
                      "</td><td>" + (battery.hasTemperature ? String(battery.temperature, 1) : String("-")) +
                      "</td><td>" + (hasCellStats ? String(minCellMv / 1000.0f, 3) : String("-")) +
                      "</td><td>" + (hasCellStats ? String(minCellIndex + 1) : String("-")) +
                      "</td><td>" + (hasCellStats ? String(maxCellMv / 1000.0f, 3) : String("-")) +
                      "</td><td>" + (hasCellStats ? String(maxCellIndex + 1) : String("-")) +
                      "</td><td>" + age +
                      "</td><td>" + String(battery.disconnectCount) + "</td></tr>";

        server.sendContent(row);
    }

    server.sendContent(F("</table><p>Auto-refreshes every 5s. Aggregate values use fresh enabled battery data only.</p></body></html>"));
    server.sendContent("");
}

static void handleBatteryDetail() {
    if (!server.hasArg("index")) {
        server.send(400, "text/plain", "Missing battery index");
        return;
    }

    String indexArg = server.arg("index");
    if (!isAllDigits(indexArg)) {
        server.send(400, "text/plain", "Invalid battery index");
        return;
    }

    int index = indexArg.toInt();
    if (index < 0 || index >= batteryCount) {
        server.send(404, "text/plain", "Battery not found");
        return;
    }

    const BatteryConfig& cfg = batteryConfigs[index];
    const BatteryState& battery = batteries[index];
    unsigned long now = millis();
    String cellDataAge = (battery.hasCellData && battery.lastCellDataMs != 0)
                       ? (String((now - battery.lastCellDataMs) / 1000UL) + " s")
                       : String("n/a");

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    server.sendContent(F("<!DOCTYPE html><html><head>"
                         "<meta charset='UTF-8'>"
                         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                         "<meta http-equiv='refresh' content='5'>"
                         "<title>Battery Cell Detail</title>"
                         "<style>"
                         "body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}"
                         "h1{color:#e0c97f;margin-bottom:4px}a{color:#8ec5ff}table{width:100%;border-collapse:collapse;margin-top:16px}"
                         "th,td{padding:8px 10px;text-align:center;border-bottom:1px solid #2a2a4a}"
                         "th{background:#16213e;color:#a0b4cc}.card{background:#16213e;border-radius:8px;padding:12px;margin:12px 0}"
                         ".mono{font-family:monospace}.muted{color:#a0b4cc}"
                         "</style></head><body>"));

    server.sendContent(F("<p><a href='/'>Back to summary</a></p>"));
    server.sendContent(String("<h1>Battery detail: ") + htmlEscape(String(cfg.name)) + "</h1>");
    server.sendContent(String("<div class='card'><div>MAC: <span class='mono'>") +
                       htmlEscape(String(cfg.mac)) +
                       "</span></div>");
    server.sendContent(String("<div>Connected: ") + (isBatteryConnected(index) ? "yes" : "no") + "</div>");
    server.sendContent(String("<div>Cell data age: ") + cellDataAge + "</div></div>");

    if (!battery.hasCellData || battery.cellCount == 0) {
        server.sendContent(F("<p class='muted'>No valid cell data available yet.</p></body></html>"));
        server.sendContent("");
        return;
    }

    server.sendContent("<h2>Cell voltages (" + String(battery.cellCount) + ")</h2>");
    server.sendContent(F("<table><tr><th>Cell</th><th>Voltage (V)</th></tr>"));
    for (uint8_t i = 0; i < battery.cellCount; i++) {
        String row = "<tr><td>" + String(i + 1) + "</td><td>" + String(battery.cellMv[i] / 1000.0f, 3) + "</td></tr>";
        server.sendContent(row);
    }
    server.sendContent(F("</table></body></html>"));
    server.sendContent("");
}

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

static void logBMSData() {
    unsigned long now = millis();
    AggregateSnapshot snap = buildAggregateSnapshot(now);

    Serial.println("\n========== BMS STATUS ==========");
    Serial.printf("Connected batteries: %d/%d\n", connectedBatteryCount(), enabledBatteryCount());

    if (snap.valid) {
        Serial.printf("Aggregate: %.2f V  %.2f A  SoC %u%%",
                      snap.voltage,
                      snap.current,
                      snap.soc);
        if (snap.hasTemperature) {
            Serial.printf("  Temp %.1f C", snap.temperature);
        }
        Serial.printf("  Fresh=%u\n", snap.contributingBatteries);
    } else {
        Serial.println("Aggregate: no fresh data");
    }

    for (int i = 0; i < batteryCount; i++) {
        if (!batteryConfigs[i].enabled) continue;

        BatteryState& battery = batteries[i];
        unsigned long ageSec = battery.lastGoodDataMs == 0
                             ? 0
                             : (now - battery.lastGoodDataMs) / 1000UL;

        Serial.printf("[%s] connected=%s seen=%s ok=%lu fail=%lu timeouts=%lu drops=%lu age=%lus reqInFlight=%s pkt=%d/%d",
                      batteryConfigs[i].name,
                      isBatteryConnected(i) ? "yes" : "no",
                      battery.seen ? "yes" : "no",
                      battery.okReads,
                      battery.failedReads,
                      battery.requestTimeouts,
                      battery.disconnectCount,
                      ageSec,
                      battery.requestInFlight ? "yes" : "no",
                      battery.packetLen,
                      battery.expectedLen);

        if (battery.hasData) {
            Serial.printf("  %.2f V %.2f A SoC %u%%",
                          battery.voltage,
                          battery.current,
                          battery.soc);
            if (battery.hasTemperature) Serial.printf(" %.1f C", battery.temperature);
        }
        Serial.println();
    }

    logSystemDebugSummary("[DBG summary]");
    Serial.println("=================================");
}

// -----------------------------------------------------------------------
// NVS persistence (Preferences)
// -----------------------------------------------------------------------

static Preferences prefs;

static void saveNVSConfig() {
    prefs.begin("bms_cfg", false);
    prefs.putInt("count", batteryCount);
    for (int i = 0; i < batteryCount; i++) {
        char key[12];
        snprintf(key, sizeof(key), "name_%d", i);
        prefs.putString(key, batteryConfigs[i].name);
        snprintf(key, sizeof(key), "mac_%d", i);
        prefs.putString(key, batteryConfigs[i].mac);
    }
    prefs.end();
    Serial.printf("[NVS] saved %d batteries\n", batteryCount);
}

static void loadNVSConfig() {
    memset(batteryConfigs, 0, sizeof(batteryConfigs));
    batteryCount = 0;

    prefs.begin("bms_cfg", true);
    int count = prefs.getInt("count", 0);
    if (count < 0) count = 0;
    if (count > MAX_BATTERIES) count = MAX_BATTERIES;

    for (int i = 0; i < count; i++) {
        char key[12];
        snprintf(key, sizeof(key), "name_%d", i);
        String name = prefs.getString(key, "");
        snprintf(key, sizeof(key), "mac_%d", i);
        String mac = prefs.getString(key, "");
        if (mac.length() == 0) continue;
        if (name.length() == 0) name = mac;
        strncpy(batteryConfigs[batteryCount].name, name.c_str(), MAX_NAME_LEN - 1);
        batteryConfigs[batteryCount].name[MAX_NAME_LEN - 1] = '\0';
        strncpy(batteryConfigs[batteryCount].mac, mac.c_str(), MAX_MAC_LEN - 1);
        batteryConfigs[batteryCount].mac[MAX_MAC_LEN - 1] = '\0';
        batteryConfigs[batteryCount].enabled = true;
        batteryCount++;
    }
    prefs.end();
    Serial.printf("[NVS] loaded %d batteries\n", batteryCount);
}

// -----------------------------------------------------------------------
// Battery management helpers
// -----------------------------------------------------------------------

// Returns true if s is a non-empty string of decimal digits.
static bool isAllDigits(const String& s) {
    if (s.length() == 0) return false;
    for (size_t i = 0; i < s.length(); i++) {
        if (!isDigit(s.charAt(i))) return false;
    }
    return true;
}

static bool isValidMac(const char* mac) {
    if (mac == nullptr || strlen(mac) != 17) return false;
    for (int i = 0; i < 17; i++) {
        if (i % 3 == 2) {
            if (mac[i] != ':') return false;
        } else {
            if (!isxdigit((unsigned char)mac[i])) return false;
        }
    }
    return true;
}

static void addBattery(const char* mac, const char* name) {
    if (batteryCount >= MAX_BATTERIES) return;
    int idx = batteryCount;
    strncpy(batteryConfigs[idx].name, name, MAX_NAME_LEN - 1);
    batteryConfigs[idx].name[MAX_NAME_LEN - 1] = '\0';
    strncpy(batteryConfigs[idx].mac, mac, MAX_MAC_LEN - 1);
    batteryConfigs[idx].mac[MAX_MAC_LEN - 1] = '\0';
    batteryConfigs[idx].enabled = true;
    batteries[idx] = BatteryState();
    batteries[idx].nextReconnectMs = millis() + 2000UL;
    batteryCount++;
    saveNVSConfig();
    Serial.printf("[MGR] added [%s] %s total=%d\n", name, mac, batteryCount);
}

static void removeBattery(int index) {
    if (index < 0 || index >= batteryCount) return;
    Serial.printf("[MGR] removing [%s] %s\n",
                  batteryConfigs[index].name, batteryConfigs[index].mac);

    cleanupBatteryClient(index);
    if (batteries[index].advertisedDevice != nullptr) {
        delete batteries[index].advertisedDevice;
        batteries[index].advertisedDevice = nullptr;
    }
    batteries[index].seen = false;
    batteries[index].nextReconnectMs = 0;

    // Shift entries down to fill the gap
    for (int i = index; i < batteryCount - 1; i++) {
        batteryConfigs[i] = batteryConfigs[i + 1];
        batteries[i] = batteries[i + 1];
    }

    // Clear the vacated last slot
    memset(&batteryConfigs[batteryCount - 1], 0, sizeof(BatteryConfig));
    batteries[batteryCount - 1] = BatteryState();

    batteryCount--;
    saveNVSConfig();
    Serial.printf("[MGR] removed. total=%d\n", batteryCount);
}

// -----------------------------------------------------------------------
// Web handlers: /batteries
// -----------------------------------------------------------------------

static const char BATTERIES_CSS[] PROGMEM =
    "body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}"
    "h1{color:#e0c97f;margin-bottom:6px}"
    "h2{color:#a0b4cc;margin-top:20px;margin-bottom:6px;font-size:1.05em}"
    "table{width:100%;border-collapse:collapse;margin-bottom:12px}"
    "th,td{padding:8px 10px;text-align:left;border-bottom:1px solid #2a2a4a}"
    "th{background:#16213e;color:#a0b4cc}"
    "a,button{text-decoration:none}"
    ".mono{font-family:monospace;font-size:0.9em}"
    ".btn{display:inline-block;padding:6px 16px;border-radius:4px;border:none;"
         "cursor:pointer;font-size:0.9em;color:#fff}"
    ".btn-back{background:#37474f}"
    ".btn-scan{background:#2a6ea6}"
    ".btn-add{background:#2e7d32;font-weight:bold;padding:8px 22px;font-size:1em}"
    ".btn-remove{background:#c62828}"
    ".note{color:#8899aa;font-size:0.85em;margin:4px 0}"
    "input[type=radio]{accent-color:#2a6ea6;width:18px;height:18px;cursor:pointer}"
    "input[type=text]{background:#16213e;color:#eee;border:1px solid #444;"
                     "border-radius:3px;padding:3px 6px;width:130px}"
    "tr.selected{background:#1e3a5f}";

static void handleBatteriesPage() {
    Serial.printf("[WEB] GET /batteries heap=%u\n", ESP.getFreeHeap());

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    server.sendContent(F("<!DOCTYPE html><html><head>"
                         "<meta charset='UTF-8'>"
                         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                         "<title>Manage Batteries</title><style>"));
    server.sendContent(BATTERIES_CSS);
    server.sendContent(F("</style></head><body>"
                         "<h1>&#9889; Manage Batteries</h1>"
                         "<p><a class='btn btn-back' href='/'>&#8592; Back to Summary</a></p>"));

    // ---- Configured / active batteries ----
    server.sendContent("<h2>Active batteries (" + String(batteryCount) +
                       "/" + String(MAX_BATTERIES) + ")</h2>");

    if (batteryCount == 0) {
        server.sendContent(F("<p class='note'>No batteries configured. "
                             "Use Scan below to discover and add batteries.</p>"));
    } else {
        server.sendContent(F("<table><tr><th>Name</th><th>MAC</th>"
                             "<th>Connected</th><th>Remove</th></tr>"));
        for (int i = 0; i < batteryCount; i++) {
            String row = "<tr><td>" + htmlEscape(String(batteryConfigs[i].name)) +
                         "</td><td class='mono'>" + htmlEscape(String(batteryConfigs[i].mac)) +
                         "</td><td>" + (isBatteryConnected(i) ? "<span style='color:#4caf50'>yes</span>"
                                                               : "<span style='color:#f44336'>no</span>") +
                         "</td><td><a class='btn btn-remove' href='/batteries/remove?index=" + String(i) +
                         "' onclick=\"return confirm('Remove " +
                         htmlEscape(String(batteryConfigs[i].name)) + "?')\">Remove</a></td></tr>";
            server.sendContent(row);
        }
        server.sendContent(F("</table>"));
    }

    // ---- Scan button ----
    server.sendContent(F("<h2>Discover new batteries</h2>"
                         "<form method='get' action='/batteries/scan'>"
                         "<button class='btn btn-scan' type='submit'>"
                         "&#128269; Scan for BMS Devices</button>"
                         "</form>"
                         "<p class='note'>Scan takes ~10 seconds. "
                         "Only BLE devices advertising the BMS service UUID are listed.</p>"));

    // ---- Candidate list with radio-select + single ADD button ----
    if (scanDone) {
        if (scanResultCount == 0) {
            server.sendContent(F("<p class='note'>No new BMS devices found. "
                                 "Try scanning again.</p>"));
        } else {
            server.sendContent("<h2>Candidates found (" + String(scanResultCount) + ")</h2>");
            server.sendContent(F("<form method='get' action='/batteries/add'>"
                                 "<table>"
                                 "<tr><th>Select</th><th>MAC</th>"
                                 "<th>Advertised Name</th><th>Display Name</th></tr>"));
            for (int i = 0; i < scanResultCount; i++) {
                String defaultName = (strlen(scanResults[i].name) > 0)
                                   ? String(scanResults[i].name)
                                   : String(scanResults[i].mac);
                String row =
                    "<tr>"
                    "<td><input type='radio' name='idx' value='" + String(i) + "'"
                    + (i == 0 ? " checked" : "") + "></td>"
                    "<td class='mono'>" + htmlEscape(String(scanResults[i].mac)) + "</td>"
                    "<td>" + htmlEscape(String(scanResults[i].name)) + "</td>"
                    "<td><input type='text' name='name_" + String(i) +
                    "' value='" + htmlEscape(defaultName) + "' maxlength='19'></td>"
                    "</tr>";
                server.sendContent(row);
            }
            server.sendContent(F("</table>"
                                 "<p><button class='btn btn-add' type='submit'>"
                                 "&#43; ADD TO ACTIVE LIST</button></p>"
                                 "</form>"));
        }
    }

    server.sendContent(F("</body></html>"));
    server.sendContent("");
}

static void handleBatteriesScan() {
    // Guard against re-entrant scan
    if (userScanActive) {
        server.sendHeader("Location", "/batteries");
        server.send(302, "text/plain", "");
        return;
    }

    Serial.printf("[WEB] /batteries/scan starting\n");

    scanResultCount = 0;
    memset(scanResults, 0, sizeof(scanResults));
    scanDone = false;
    userScanActive = true;

    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(BLE_SCAN_INTERVAL_UNITS);
    pBLEScan->setWindow(BLE_SCAN_WINDOW_UNITS);

    unsigned long deadline = millis() + USER_SCAN_TIMEOUT_MS;
    // Cast to int32_t so the comparison handles millis() wraparound (~49-day rollover)
    while ((int32_t)(deadline - millis()) > 0) {
        int32_t remaining = (int32_t)(deadline - millis());
        if (remaining <= 0) break;
        unsigned long sliceMs = ((unsigned long)remaining < SCAN_SLICE_MS)
                              ? (unsigned long)remaining : SCAN_SLICE_MS;
        if (sliceMs < MIN_SCAN_SLICE_MS) sliceMs = MIN_SCAN_SLICE_MS;
        pBLEScan->start(scanDurationSeconds(sliceMs), false);
        pBLEScan->clearResults();
        if (wifiReady) server.handleClient();
    }

    userScanActive = false;
    scanDone = true;
    Serial.printf("[WEB] scan complete: %d candidate(s) found\n", scanResultCount);

    server.sendHeader("Location", "/batteries");
    server.send(302, "text/plain", "");
}

static void handleBatteriesAdd() {
    if (!server.hasArg("idx")) {
        server.send(400, "text/plain", "No candidate selected");
        return;
    }

    String idxArg = server.arg("idx");
    if (!isAllDigits(idxArg)) {
        server.send(400, "text/plain", "Invalid idx");
        return;
    }
    int idx = idxArg.toInt();
    if (idx < 0 || idx >= scanResultCount) {
        server.send(400, "text/plain", "Invalid candidate index");
        return;
    }

    String mac = String(scanResults[idx].mac);
    mac.toLowerCase();
    mac.trim();

    if (!isValidMac(mac.c_str())) {
        server.send(400, "text/plain", "Invalid MAC address");
        return;
    }

    // Duplicate check
    for (int i = 0; i < batteryCount; i++) {
        String cfgMac = String(batteryConfigs[i].mac);
        cfgMac.toLowerCase();
        if (cfgMac == mac) {
            server.sendHeader("Location", "/batteries");
            server.send(302, "text/plain", "");
            return;
        }
    }

    if (batteryCount >= MAX_BATTERIES) {
        server.send(400, "text/plain", "Maximum battery count reached");
        return;
    }

    // Retrieve the user-edited display name for this candidate
    String nameKey = "name_" + String(idx);
    String name = server.hasArg(nameKey) ? server.arg(nameKey) : mac;
    name.trim();
    if (name.length() == 0) name = mac;
    if (name.length() >= MAX_NAME_LEN) name = name.substring(0, MAX_NAME_LEN - 1);

    addBattery(mac.c_str(), name.c_str());

    server.sendHeader("Location", "/batteries");
    server.send(302, "text/plain", "");
}

static void handleBatteriesRemove() {
    if (!server.hasArg("index")) {
        server.send(400, "text/plain", "Missing index");
        return;
    }

    String indexArg = server.arg("index");
    if (!isAllDigits(indexArg)) {
        server.send(400, "text/plain", "Invalid index");
        return;
    }

    int index = indexArg.toInt();
    if (index < 0 || index >= batteryCount) {
        server.send(400, "text/plain", "Index out of range");
        return;
    }

    removeBattery(index);

    server.sendHeader("Location", "/batteries");
    server.send(302, "text/plain", "");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== JBD BMS -> Solis CAN Bridge (persistent classic BLE multi-battery) ===");

    setupCAN();

    // Load active battery list from NVS before anything else
    loadNVSConfig();

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
        server.on("/battery", handleBatteryDetail);
        server.on("/batteries", handleBatteriesPage);
        server.on("/batteries/scan", handleBatteriesScan);
        server.on("/batteries/add", handleBatteriesAdd);
        server.on("/batteries/remove", handleBatteriesRemove);
        server.begin();
        Serial.println("Web server started on port 80");
        wifiReady = true;
    } else {
        Serial.println("WiFi failed - continuing without web server");
    }

    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(&discoveryCallbacks);

    Serial.printf("Configured batteries: %d\n", batteryCount);
    for (int i = 0; i < batteryCount; i++) {
        Serial.printf("  [%d] %s %s enabled=%s\n",
                      i,
                      batteryConfigs[i].name,
                      batteryConfigs[i].mac,
                      batteryConfigs[i].enabled ? "true" : "false");
    }

    bool allSeen = scanForAllEnabledBatteries(STARTUP_SCAN_TIMEOUT_MS);
    Serial.printf("Startup discovery: seen=%d/%d\n", seenBatteryCount(), enabledBatteryCount());
    if (!allSeen) {
        Serial.println("Some enabled batteries were not discovered at startup.");
    }

    for (int i = 0; i < batteryCount; i++) {
        if (!batteryConfigs[i].enabled) continue;

        if (!batteries[i].seen) {
            Serial.printf("[%s] startup connect skipped (not discovered)\n", batteryConfigs[i].name);
            batteries[i].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
            continue;
        }

        bool ok = connectBattery(i);
        Serial.printf("[%s] startup connect %s\n", batteryConfigs[i].name, ok ? "OK" : "FAIL");
        if (!ok) batteries[i].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
    }

    aggregate = buildAggregateSnapshot(millis());
    logSystemDebugSummary("[DBG setup complete]");
}

static unsigned long lastCAN = 0;
static unsigned long lastLog = 0;
static unsigned long lastHeartbeat = 0;

void loop() {
    unsigned long now = millis();

    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        Serial.printf("[loop] heartbeat now=%lu heap=%u wifi=%s connected=%d/%d\n",
                      now,
                      ESP.getFreeHeap(),
                      wifiStatusToString(WiFi.status()),
                      connectedBatteryCount(),
                      enabledBatteryCount());
    }

    if (wifiReady) server.handleClient();

    for (int i = 0; i < batteryCount; i++) {
        if (!batteryConfigs[i].enabled) continue;

        if (isBatteryConnected(i)) {
            serviceBatteryPolling(i, now);
        } else if (shouldAttemptReconnect(i, now)) {
            bool ok = reconnectBattery(i);
            Serial.printf("[%s] reconnect %s\n", batteryConfigs[i].name, ok ? "OK" : "FAIL");
        }

        if (wifiReady) server.handleClient();
    }

    aggregate = buildAggregateSnapshot(now);

    if (now - lastCAN >= CAN_INTERVAL_MS) {
        lastCAN = now;
        if (isAggregateUsable(aggregate, now)) {
            sendCANFrames(aggregate.voltage,
                          aggregate.current,
                          aggregate.soc,
                          aggregate.temperature,
                          aggregate.chargeAllowed,
                          aggregate.dischargeAllowed);
        }
    }

    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        logBMSData();
    }

    delay(5);
}
