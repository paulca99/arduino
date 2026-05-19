#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>

static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_rx("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_tx("0000ff02-0000-1000-8000-00805f9b34fb");

#define BMS1_NAME "Growatt"
#define BMS1_MAC  "a5:c2:37:49:c7:a2"

#define BMS2_NAME "Solax"
#define BMS2_MAC  "a4:c1:37:20:4e:3b"

#define BMS3_NAME "SP14S004P14S40A"
#define BMS3_MAC  "a5:c2:37:51:85:7f"

// Timing
#define PER_BATTERY_TIMEOUT_MS 5000
#define SCAN_SLICE_MS          1200
#define CONNECT_DELAY_MS        100
#define REQUEST_DELAY_MS        500
#define RESPONSE_TIMEOUT_MS    2500
#define BETWEEN_BATTERIES_MS    200
#define BETWEEN_CYCLES_MS      1000

#define MAX_PACKET_LEN          128

struct BatteryTarget {
    const char* name;
    const char* mac;
    bool enabled;
};

BatteryTarget batteries[] = {
    { BMS1_NAME, BMS1_MAC, true },
    { BMS2_NAME, BMS2_MAC, true },
    { BMS3_NAME, BMS3_MAC, true }
};

static const int BATTERY_COUNT = sizeof(batteries) / sizeof(batteries[0]);

static BLEScan* pBLEScan = nullptr;
static BLEClient* pClient = nullptr;
static BLEAdvertisedDevice* pRemoteDevice = nullptr;
static BLERemoteService* pRemoteService = nullptr;
static BLERemoteCharacteristic* pRx = nullptr;
static BLERemoteCharacteristic* pTx = nullptr;

static const char* currentTargetName = nullptr;
static const char* currentTargetMac = nullptr;
static bool targetSeen = false;
static bool gotPacket03 = false;
static float lastVoltage = 0.0f;
static float lastCurrent = 0.0f;
static uint8_t lastSoc = 0;

static uint8_t packetBuf[MAX_PACKET_LEN];
static int packetLen = 0;
static int expectedLen = 0;
static bool packetError = false;

static uint32_t cycleCount = 0;
static uint32_t okCount = 0;
static uint32_t failCount = 0;

void resetPacketAssembly() {
    packetLen = 0;
    expectedLen = 0;
    packetError = false;
    gotPacket03 = false;
    lastVoltage = 0.0f;
    lastCurrent = 0.0f;
    lastSoc = 0;
}

uint16_t calcChecksum(const uint8_t* data) {
    int checksum = 0x10000;
    int dataLength = data[3];
    for (int i = 0; i < dataLength + 1; i++) checksum -= data[i + 3];
    return (uint16_t)checksum;
}

bool checksumValid(const uint8_t* data) {
    int checksumIndex = data[3] + 4;
    uint16_t got = ((uint16_t)data[checksumIndex] << 8) | data[checksumIndex + 1];
    return calcChecksum(data) == got;
}

void cleanupConnection() {
    if (pClient != nullptr) {
        if (pClient->isConnected()) {
            pClient->disconnect();
            delay(200);
        }
        delete pClient;
        pClient = nullptr;
    }

    pRemoteService = nullptr;
    pRx = nullptr;
    pTx = nullptr;

    if (pRemoteDevice != nullptr) {
        delete pRemoteDevice;
        pRemoteDevice = nullptr;
    }
}

static void notifyCallback(
    BLERemoteCharacteristic* chr,
    uint8_t* pData,
    size_t length,
    bool isNotify
) {
    (void)chr;
    (void)isNotify;

    if (packetError) return;

    if (packetLen == 0) {
        if (length == 0 || pData[0] != 0xDD) return;
        packetError = (pData[2] != 0x00);
        expectedLen = pData[3] + 7;
    }

    if (packetLen + (int)length > MAX_PACKET_LEN) {
        packetError = true;
        return;
    }

    memcpy(packetBuf + packetLen, pData, length);
    packetLen += (int)length;

    if (!packetError && expectedLen > 0 && packetLen == expectedLen) {
        if (checksumValid(packetBuf) && packetBuf[1] == 0x03) {
            uint16_t rawVolts   = ((uint16_t)packetBuf[4] << 8) | packetBuf[5];
            int16_t  rawCurrent = ((int16_t)packetBuf[6] << 8) | packetBuf[7];
            uint8_t  rawSoc     = packetBuf[23];

            lastVoltage = rawVolts / 100.0f;
            lastCurrent = rawCurrent / 100.0f;
            lastSoc = rawSoc;
            gotPacket03 = true;
        }
    }
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (currentTargetMac == nullptr) return;

        String addr = advertisedDevice.getAddress().toString().c_str();
        addr.toLowerCase();

        String targetMac = currentTargetMac;
        targetMac.toLowerCase();

        bool addrOk = (addr == targetMac);
        bool svcOk  = advertisedDevice.haveServiceUUID() &&
                      advertisedDevice.isAdvertisingService(serviceUUID);

        if (addrOk && svcOk) {
            targetSeen = true;

            if (pRemoteDevice != nullptr) {
                delete pRemoteDevice;
                pRemoteDevice = nullptr;
            }
            pRemoteDevice = new BLEAdvertisedDevice(advertisedDevice);
            BLEDevice::getScan()->stop();
        }
    }
};

bool scanForTargetUntilDeadline(unsigned long deadlineMs) {
    targetSeen = false;

    if (pRemoteDevice != nullptr) {
        delete pRemoteDevice;
        pRemoteDevice = nullptr;
    }

    while (millis() < deadlineMs) {
        unsigned long now = millis();
        unsigned long remain = deadlineMs - now;
        unsigned long sliceMs = (remain < SCAN_SLICE_MS) ? remain : SCAN_SLICE_MS;
        if (sliceMs < 200) sliceMs = 200;

        pBLEScan->setActiveScan(true);
        pBLEScan->setInterval(1349);
        pBLEScan->setWindow(449);
        pBLEScan->start((uint32_t)((sliceMs + 999) / 1000), false);

        if (targetSeen && pRemoteDevice != nullptr) {
            return true;
        }
    }

    return false;
}

bool connectToServerBeforeDeadline(unsigned long deadlineMs) {
    if (pRemoteDevice == nullptr) return false;
    if (millis() >= deadlineMs) return false;

    pClient = BLEDevice::createClient();
    if (pClient == nullptr) return false;

    delay(CONNECT_DELAY_MS);
    if (millis() >= deadlineMs) return false;

    pClient->connect(pRemoteDevice);
    if (!pClient->isConnected()) return false;

    if (millis() >= deadlineMs) return false;
    pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        pClient->disconnect();
        return false;
    }

    if (millis() >= deadlineMs) return false;
    pRx = pRemoteService->getCharacteristic(charUUID_rx);
    if (pRx == nullptr || !pRx->canNotify()) {
        pClient->disconnect();
        return false;
    }
    pRx->registerForNotify(notifyCallback);

    if (millis() >= deadlineMs) return false;
    pTx = pRemoteService->getCharacteristic(charUUID_tx);
    if (pTx == nullptr || (!pTx->canWrite() && !pTx->canWriteNoResponse())) {
        pClient->disconnect();
        return false;
    }

    if (REQUEST_DELAY_MS > 0) {
        unsigned long waitUntil = millis() + REQUEST_DELAY_MS;
        while (millis() < waitUntil) {
            if (millis() >= deadlineMs) return false;
            delay(10);
        }
    }

    return millis() < deadlineMs;
}

bool requestOneVoltageReadBeforeDeadline(unsigned long deadlineMs) {
    static const uint8_t cmd3[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};

    resetPacketAssembly();

    if (pClient == nullptr || !pClient->isConnected() || pTx == nullptr) return false;
    if (millis() >= deadlineMs) return false;

    pTx->writeValue((uint8_t*)cmd3, sizeof(cmd3), false);

    unsigned long localDeadline = deadlineMs;
    unsigned long responseDeadline = millis() + RESPONSE_TIMEOUT_MS;
    if (responseDeadline < localDeadline) localDeadline = responseDeadline;

    while (millis() < localDeadline) {
        if (gotPacket03) return true;
        delay(20);
    }

    return false;
}

bool tryReadBattery(const char* name, const char* mac,
                    float& voltageOut, float& currentOut, uint8_t& socOut,
                    unsigned long& elapsedOut) {
    unsigned long startMs = millis();
    unsigned long deadlineMs = startMs + PER_BATTERY_TIMEOUT_MS;

    currentTargetName = name;
    currentTargetMac = mac;
    voltageOut = 0.0f;
    currentOut = 0.0f;
    socOut = 0;

    cleanupConnection();

    bool ok = false;

    if (scanForTargetUntilDeadline(deadlineMs)) {
        if (connectToServerBeforeDeadline(deadlineMs)) {
            ok = requestOneVoltageReadBeforeDeadline(deadlineMs);
        }
    }

    if (ok) {
        voltageOut = lastVoltage;
        currentOut = lastCurrent;
        socOut = lastSoc;
    }

    cleanupConnection();
    elapsedOut = millis() - startMs;
    return ok;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== 3-battery round-robin classic BLE test ===");

    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());

    Serial.printf("Configured entries: %d\n", BATTERY_COUNT);
    for (int i = 0; i < BATTERY_COUNT; i++) {
        Serial.printf("  [%d] %s  %s  enabled=%s\n",
                      i,
                      batteries[i].name,
                      batteries[i].mac,
                      batteries[i].enabled ? "true" : "false");
    }
}

void loop() {
    cycleCount++;
    int cycleOk = 0;
    int cycleFail = 0;

    Serial.printf("\n--- Cycle %lu ---\n", (unsigned long)cycleCount);

    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (!batteries[i].enabled) continue;

        float volts = 0.0f;
        float amps = 0.0f;
        uint8_t soc = 0;
        unsigned long elapsed = 0;

        bool ok = tryReadBattery(
            batteries[i].name,
            batteries[i].mac,
            volts,
            amps,
            soc,
            elapsed
        );

        if (ok) {
            okCount++;
            cycleOk++;
            Serial.printf("%s  OK   %.2f V  %.2f A  SoC %u%%  %lums\n",
                          batteries[i].name, volts, amps, soc, elapsed);
        } else {
            failCount++;
            cycleFail++;
            Serial.printf("%s  FAIL %lums\n", batteries[i].name, elapsed);
        }

        delay(BETWEEN_BATTERIES_MS);
    }

    Serial.printf("Cycle summary: ok=%d fail=%d totals ok=%lu fail=%lu\n",
                  cycleOk, cycleFail,
                  (unsigned long)okCount,
                  (unsigned long)failCount);

    delay(BETWEEN_CYCLES_MS);
}