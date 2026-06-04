#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "driver/twai.h"
#include <math.h>

static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_rx("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_tx("0000ff02-0000-1000-8000-00805f9b34fb");

#define BMS1_NAME "Growatt"
#define BMS1_MAC  "a5:c2:37:49:c7:a2"
#define BMS1_ENABLED false

#define BMS2_NAME "Growatt2"
#define BMS2_MAC  "a5:c2:37:51:85:89"
#define BMS2_ENABLED true

#define BMS3_NAME "SP14S004P14S40A"
#define BMS3_MAC  "a5:c2:37:51:85:7f"
#define BMS3_ENABLED true

#define CAN_TX_PIN     GPIO_NUM_21
#define CAN_RX_PIN     GPIO_NUM_19
#define CAN_INTERVAL_MS            500
#define DATA_FRESH_MS            15000UL
#define BLE_TIMEOUT_MS  (3UL * 60UL * 1000UL)
#define CAN_TASK_STACK_SIZE        4096
#define CAN_TASK_PRIORITY             1
#define CAN_MUTEX_TIMEOUT_MS        100
#define RS485_RX_PIN                 16
#define RS485_TX_PIN                 17
#ifndef WIFI_SSID
#define WIFI_SSID "TP-LINK_73F3"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "DEADBEEF"
#endif
#define WIFI_CONNECT_TIMEOUT_MS   5000
#define SOLIS_POLL_INTERVAL_MS     3000
#define SOLIS_BLOCK_READ_TIMEOUT_MS 300
#define SOLIS_JSON_RESERVE_BYTES 1700
#define SOLIS_BLOCK_START_REG     33050
#define SOLIS_BLOCK_END_REG       33142
#define SOLIS_BLOCK_REG_COUNT        93
#define SOLIS_MODBUS_HEADER_BYTES     3
#define SOLIS_MODBUS_CRC_BYTES        2
#define SOLIS_MODBUS_BYTES_PER_REG    2
#define SOLIS_MUTEX_TIMEOUT_MS      100
#define SOLIS_SLAVE_ID                1
#define SOLIS_DIVISOR_ZERO_GUARD 1.0e-6f
#if CONFIG_FREERTOS_UNICORE
#define CAN_TASK_CORE                0
#else
#ifndef CONFIG_ARDUINO_RUNNING_CORE
#define CONFIG_ARDUINO_RUNNING_CORE 1
#endif
#define CAN_TASK_CORE  (1 - CONFIG_ARDUINO_RUNNING_CORE)
#endif

#define STARTUP_SCAN_TIMEOUT_MS   15000
#define RECONNECT_SCAN_TIMEOUT_MS  6000
#define SCAN_SLICE_MS              1200
#define MIN_SCAN_SLICE_MS           200
#define BLE_SCAN_INTERVAL_UNITS    1349
#define BLE_SCAN_WINDOW_UNITS       449
#define MS_PER_SECOND              1000
#define CONNECT_DELAY_MS            100
#define DISCONNECT_CLEANUP_DELAY_MS  50
#define REQUEST_DELAY_MS            300
#define RESPONSE_TIMEOUT_MS        2500
#define RESPONSE_POLL_MS             20
#define BETWEEN_BATTERIES_MS        50
#define BETWEEN_CYCLES_MS          100
#define LOG_INTERVAL_MS            5000
#define RECONNECT_INTERVAL_MS     30000
#define MAX_PACKET_LEN             128
#define MAX_CELLS                   24
#define PACKET03_SOC_INDEX          23
#define PACKET03_FET_INDEX          24
#define PACKET03_TEMP_COUNT_IDX_A   26
#define PACKET03_TEMP_COUNT_IDX_B   25

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
    bool hasData = false;
    bool hasTemperature = false;
    bool chargeMos = false;
    bool dischargeMos = false;

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

struct SolisRegisterValue {
    uint16_t raw = 0;
    bool valid = false;
};

struct SolisSnapshot {
    SolisRegisterValue pv1Voltage;
    SolisRegisterValue pv1Current;
    SolisRegisterValue pv2Voltage;
    SolisRegisterValue pv2Current;
    SolisRegisterValue gridPower;
    SolisRegisterValue batteryCurrent;
    SolisRegisterValue batteryDirection;
    SolisRegisterValue batterySoc;
    SolisRegisterValue batteryVoltage;
    uint32_t lastPollMs = 0;
    uint32_t lastSuccessMs = 0;
    uint32_t pollCount = 0;
    uint32_t readErrors = 0;
    uint32_t lockTimeouts = 0;
};

static const uint16_t SOLIS_REG_PV1_VOLTAGE = 33050;
static const uint16_t SOLIS_REG_PV1_CURRENT = 33051;
static const uint16_t SOLIS_REG_PV2_VOLTAGE = 33052;
static const uint16_t SOLIS_REG_PV2_CURRENT = 33053;
static const uint16_t SOLIS_REG_GRID_POWER = 33132;
static const uint16_t SOLIS_REG_BATTERY_CURRENT = 33135;
static const uint16_t SOLIS_REG_BATTERY_DIRECTION = 33136;
static const uint16_t SOLIS_REG_BATTERY_SOC = 33140;
static const uint16_t SOLIS_REG_BATTERY_VOLTAGE = 33142;

struct RegisterSpec {
    uint16_t reg;
};

static const RegisterSpec SOLIS_REGISTER_SPECS[] = {
    {33050}, {33051}, {33052}, {33053}, {33059}, {33072}, {33074},
    {33080}, {33081}, {33085}, {33095}, {33129}, {33130}, {33131},
    {33132}, {33134}, {33135}, {33136}, {33137}, {33140}, {33142},
};
static const size_t SOLIS_REGISTER_COUNT = sizeof(SOLIS_REGISTER_SPECS) / sizeof(SOLIS_REGISTER_SPECS[0]);

static BatteryState batteries[] = {
    {BMS1_NAME, BMS1_MAC, BMS1_ENABLED},
    {BMS2_NAME, BMS2_MAC, BMS2_ENABLED},
    {BMS3_NAME, BMS3_MAC, BMS3_ENABLED},
};

static const int BATTERY_COUNT = sizeof(batteries) / sizeof(batteries[0]);

static BLEScan* pBLEScan = nullptr;
static unsigned long cycleCount = 0;
static unsigned long lastSummaryMs = 0;
static unsigned long lastSolisPollMs = 0;
static AggregateSnapshot aggregate;
static AggregateSnapshot canAggregate;
static SemaphoreHandle_t canAggregateMutex = nullptr;
static SolisSnapshot solisState = {};
static SemaphoreHandle_t solisMutex = nullptr;
static HardwareSerial RS485(2);
static AsyncWebServer server(80);

static bool isAggregateUsable(const AggregateSnapshot& snap, unsigned long nowMs);
static AggregateSnapshot buildAggregateSnapshot(unsigned long now);
static void updateCanAggregateSnapshot(const AggregateSnapshot& snap);
static AggregateSnapshot copyCanAggregateSnapshot();
static void canTxTask(void* pv);
static bool tryGetSolisScaledValue(const SolisRegisterValue& regValue,
                                   float divisor,
                                   bool signedValue,
                                   float& value);
static bool isSolisBatteryDischarging(const SolisRegisterValue& regValue);
static bool tryBuildSolisPowerW(const SolisRegisterValue& voltageReg,
                                const SolisRegisterValue& currentReg,
                                float voltageDivisor,
                                float currentDivisor,
                                float& powerW);
static bool tryBuildSignedSolisBatteryPowerW(const SolisSnapshot& snapshot, float& powerW);
static void setSolisRegisterFromBlock(uint16_t docReg,
                                      const uint16_t* blockValues,
                                      SolisRegisterValue& regValue);
static bool copySolisSnapshot(SolisSnapshot& snapshot);
static bool readSolisBlockU16(uint8_t slave, uint16_t startDocReg, uint16_t count, uint16_t* outValues);
static void pollSolisOnce(unsigned long nowMs);
static void maybePollSolis(unsigned long nowMs);
static void printSolisSummary(unsigned long nowMs);
static bool tryConnectWiFi(const char* ssid, const char* password);
static bool tryGetSolisRegisterValue(const SolisSnapshot& snapshot, uint16_t docReg, SolisRegisterValue& regValue);
static String buildSolisJson();
uint8_t socFromVoltageTable(float packVoltage);

void setupCAN();
void sendCANFrames(float voltage,
                   float current,
                   uint8_t soc,
                   float temperature,
                   bool chargeAllowed,
                   bool dischargeAllowed);

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
    return (int32_t)(millis() - deadlineMs) >= 0;
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

static bool tryGetSolisScaledValue(const SolisRegisterValue& regValue,
                                   float divisor,
                                   bool signedValue,
                                   float& value) {
    if (!regValue.valid || fabsf(divisor) < SOLIS_DIVISOR_ZERO_GUARD) return false;

    value = signedValue ? static_cast<float>(static_cast<int16_t>(regValue.raw)) / divisor
                        : static_cast<float>(regValue.raw) / divisor;
    return true;
}

static bool isSolisBatteryDischarging(const SolisRegisterValue& regValue) {
    return regValue.valid && regValue.raw == 1;
}

static bool tryBuildSolisPowerW(const SolisRegisterValue& voltageReg,
                                const SolisRegisterValue& currentReg,
                                float voltageDivisor,
                                float currentDivisor,
                                float& powerW) {
    float voltageVolts = 0.0f;
    float currentAmps = 0.0f;
    if (!tryGetSolisScaledValue(voltageReg, voltageDivisor, false, voltageVolts) ||
        !tryGetSolisScaledValue(currentReg, currentDivisor, false, currentAmps)) {
        return false;
    }

    powerW = voltageVolts * currentAmps;
    return true;
}

static bool tryBuildSignedSolisBatteryPowerW(const SolisSnapshot& snapshot, float& powerW) {
    if (!tryBuildSolisPowerW(snapshot.batteryVoltage, snapshot.batteryCurrent, 100.0f, 10.0f, powerW)) {
        return false;
    }

    if (isSolisBatteryDischarging(snapshot.batteryDirection)) {
        powerW = -powerW;
    }
    return true;
}

static uint16_t modbusCRC(const uint8_t* buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 0; i < 8; i++) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xA001U : (crc >> 1);
        }
    }
    return crc;
}

static void flushRS485Input() {
    while (RS485.available()) {
        RS485.read();
    }
}

static int readRS485Reply(uint8_t* buf, int maxLen, uint32_t timeoutMs) {
    int len = 0;
    uint32_t last = millis();
    while (millis() - last < timeoutMs) {
        while (RS485.available()) {
            uint8_t byteValue = RS485.read();
            if (len < maxLen) buf[len++] = byteValue;
            last = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return len;
}

static bool validModbusCRC(const uint8_t* buf, int len) {
    if (len < 4) return false;
    uint16_t rxCRC = buf[len - 2] | (static_cast<uint16_t>(buf[len - 1]) << 8);
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
    RS485.write(frame, sizeof(frame));
    RS485.flush();
}

static void setSolisRegisterFromBlock(uint16_t docReg,
                                      const uint16_t* blockValues,
                                      SolisRegisterValue& regValue) {
    if (docReg < SOLIS_BLOCK_START_REG || docReg > SOLIS_BLOCK_END_REG) return;

    regValue.raw = blockValues[docReg - SOLIS_BLOCK_START_REG];
    regValue.valid = true;
}

static bool readSolisBlockU16(uint8_t slave, uint16_t startDocReg, uint16_t count, uint16_t* outValues) {
    static uint8_t buf[SOLIS_MODBUS_HEADER_BYTES +
                       (SOLIS_BLOCK_REG_COUNT * SOLIS_MODBUS_BYTES_PER_REG) +
                       SOLIS_MODBUS_CRC_BYTES];

    const int expectedLen = SOLIS_MODBUS_HEADER_BYTES +
                            (count * SOLIS_MODBUS_BYTES_PER_REG) +
                            SOLIS_MODBUS_CRC_BYTES;
    if (expectedLen > static_cast<int>(sizeof(buf))) return false;

    const uint16_t rawAddr = startDocReg - 1;

    flushRS485Input();
    sendReadInput(slave, rawAddr, count);

    const int len = readRS485Reply(buf, sizeof(buf), SOLIS_BLOCK_READ_TIMEOUT_MS);
    if (len < expectedLen) return false;
    if (!validModbusCRC(buf, len)) return false;
    if (buf[0] != slave || buf[1] != 0x04) return false;
    if (buf[2] != static_cast<uint8_t>(count * SOLIS_MODBUS_BYTES_PER_REG)) return false;

    for (uint16_t i = 0; i < count; i++) {
        const int offset = SOLIS_MODBUS_HEADER_BYTES + (i * SOLIS_MODBUS_BYTES_PER_REG);
        outValues[i] = (static_cast<uint16_t>(buf[offset]) << 8) | buf[offset + 1];
    }
    return true;
}

static void pollSolisOnce(unsigned long nowMs) {
    if (solisMutex == nullptr) return;

    uint16_t blockValues[SOLIS_BLOCK_REG_COUNT];
    uint32_t lockTimeoutsThisPass = 0;
    bool blockOk = readSolisBlockU16(SOLIS_SLAVE_ID,
                                     SOLIS_BLOCK_START_REG,
                                     SOLIS_BLOCK_REG_COUNT,
                                     blockValues);

    if (blockOk) {
        if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(SOLIS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            setSolisRegisterFromBlock(SOLIS_REG_PV1_VOLTAGE, blockValues, solisState.pv1Voltage);
            setSolisRegisterFromBlock(SOLIS_REG_PV1_CURRENT, blockValues, solisState.pv1Current);
            setSolisRegisterFromBlock(SOLIS_REG_PV2_VOLTAGE, blockValues, solisState.pv2Voltage);
            setSolisRegisterFromBlock(SOLIS_REG_PV2_CURRENT, blockValues, solisState.pv2Current);
            setSolisRegisterFromBlock(SOLIS_REG_GRID_POWER, blockValues, solisState.gridPower);
            setSolisRegisterFromBlock(SOLIS_REG_BATTERY_CURRENT, blockValues, solisState.batteryCurrent);
            setSolisRegisterFromBlock(SOLIS_REG_BATTERY_DIRECTION, blockValues, solisState.batteryDirection);
            setSolisRegisterFromBlock(SOLIS_REG_BATTERY_SOC, blockValues, solisState.batterySoc);
            setSolisRegisterFromBlock(SOLIS_REG_BATTERY_VOLTAGE, blockValues, solisState.batteryVoltage);
            solisState.lastSuccessMs = nowMs;
            xSemaphoreGive(solisMutex);
        } else {
            lockTimeoutsThisPass++;
        }
    }

    if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(SOLIS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        solisState.lastPollMs = nowMs;
        solisState.pollCount++;
        if (!blockOk) solisState.readErrors++;
        solisState.lockTimeouts += lockTimeoutsThisPass;
        xSemaphoreGive(solisMutex);
    }

    if (!blockOk) {
        Serial.println("[Solis] poll pass had no successful reads");
    }
}

static void maybePollSolis(unsigned long nowMs) {
    if (solisMutex == nullptr) return;
    if (lastSolisPollMs != 0 && (nowMs - lastSolisPollMs) < SOLIS_POLL_INTERVAL_MS) return;

    lastSolisPollMs = nowMs;
    pollSolisOnce(nowMs);
}

static bool copySolisSnapshot(SolisSnapshot& snapshot) {
    if (solisMutex == nullptr) {
        snapshot = {};
        return false;
    }

    if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(SOLIS_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        snapshot = {};
        return false;
    }

    snapshot = solisState;
    xSemaphoreGive(solisMutex);
    return true;
}

static void printSolisSummary(unsigned long nowMs) {
    SolisSnapshot snapshot = {};
    if (!copySolisSnapshot(snapshot) || snapshot.pollCount == 0) {
        Serial.println("[Solis] no inverter polls yet");
        return;
    }

    unsigned long ageSec = snapshot.lastSuccessMs == 0 ? 0 : (nowMs - snapshot.lastSuccessMs) / 1000UL;
    float batteryVoltage = 0.0f;
    float batteryCurrent = 0.0f;
    float batterySoc = 0.0f;
    float gridPower = 0.0f;
    float batteryPower = 0.0f;
    float pv1Power = 0.0f;
    float pv2Power = 0.0f;

    bool haveBatteryVoltage = tryGetSolisScaledValue(snapshot.batteryVoltage, 100.0f, false, batteryVoltage);
    bool haveBatteryCurrent = tryGetSolisScaledValue(snapshot.batteryCurrent, 10.0f, false, batteryCurrent);
    bool haveBatterySoc = tryGetSolisScaledValue(snapshot.batterySoc, 1.0f, false, batterySoc);
    bool haveGridPower = tryGetSolisScaledValue(snapshot.gridPower, 1.0f, true, gridPower);
    bool haveBatteryPower = tryBuildSignedSolisBatteryPowerW(snapshot, batteryPower);
    bool havePv1Power = tryBuildSolisPowerW(snapshot.pv1Voltage, snapshot.pv1Current, 10.0f, 10.0f, pv1Power);
    bool havePv2Power = tryBuildSolisPowerW(snapshot.pv2Voltage, snapshot.pv2Current, 10.0f, 10.0f, pv2Power);

    Serial.printf("[Solis] age=%lus polls=%lu errors=%lu locks=%lu",
                  ageSec,
                  (unsigned long)snapshot.pollCount,
                  (unsigned long)snapshot.readErrors,
                  (unsigned long)snapshot.lockTimeouts);
    if (haveBatteryVoltage) Serial.printf(" batt=%.2fV", batteryVoltage);
    if (haveBatteryCurrent) Serial.printf(" %.1fA", batteryCurrent);
    if (haveBatterySoc) Serial.printf(" soc=%.0f%%", batterySoc);
    if (haveBatteryPower) Serial.printf(" battP=%.0fW", batteryPower);
    if (haveGridPower) Serial.printf(" grid=%.0fW", gridPower);
    if (havePv1Power) Serial.printf(" pv1=%.0fW", pv1Power);
    if (havePv2Power) Serial.printf(" pv2=%.0fW", pv2Power);
    if (snapshot.batteryDirection.valid) {
        Serial.printf(" dir=%s", isSolisBatteryDischarging(snapshot.batteryDirection) ? "discharging" : "charging");
    }
    Serial.println();
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

static bool tryGetSolisRegisterValue(const SolisSnapshot& snapshot, uint16_t docReg, SolisRegisterValue& regValue) {
    switch (docReg) {
        case SOLIS_REG_PV1_VOLTAGE: regValue = snapshot.pv1Voltage; return true;
        case SOLIS_REG_PV1_CURRENT: regValue = snapshot.pv1Current; return true;
        case SOLIS_REG_PV2_VOLTAGE: regValue = snapshot.pv2Voltage; return true;
        case SOLIS_REG_PV2_CURRENT: regValue = snapshot.pv2Current; return true;
        case SOLIS_REG_GRID_POWER: regValue = snapshot.gridPower; return true;
        case SOLIS_REG_BATTERY_CURRENT: regValue = snapshot.batteryCurrent; return true;
        case SOLIS_REG_BATTERY_DIRECTION: regValue = snapshot.batteryDirection; return true;
        case SOLIS_REG_BATTERY_SOC: regValue = snapshot.batterySoc; return true;
        case SOLIS_REG_BATTERY_VOLTAGE: regValue = snapshot.batteryVoltage; return true;
        default:
            regValue = {};
            return false;
    }
}

static String buildSolisJson() {
    SolisSnapshot snapshot = {};
    copySolisSnapshot(snapshot);

    String json;
    json.reserve(SOLIS_JSON_RESERVE_BYTES);
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

    json += ",\"batteryDirection\":";
    if (snapshot.batteryDirection.valid) {
        json += "\"";
        json += isSolisBatteryDischarging(snapshot.batteryDirection) ? "discharging" : "charging";
        json += "\"";
    } else {
        json += "null";
    }

    float derivedValue = 0.0f;
    json += ",\"batteryPowerW\":";
    json += tryBuildSignedSolisBatteryPowerW(snapshot, derivedValue) ? String(derivedValue, 1) : String("0");
    json += ",\"pv1PowerW\":";
    json += tryBuildSolisPowerW(snapshot.pv1Voltage, snapshot.pv1Current, 10.0f, 10.0f, derivedValue) ? String(derivedValue, 1) : String("0");
    json += ",\"pv2PowerW\":";
    json += tryBuildSolisPowerW(snapshot.pv2Voltage, snapshot.pv2Current, 10.0f, 10.0f, derivedValue) ? String(derivedValue, 1) : String("0");

    float pv1Power = 0.0f;
    float pv2Power = 0.0f;
    json += ",\"pvTotalPowerW\":";
    if (tryBuildSolisPowerW(snapshot.pv1Voltage, snapshot.pv1Current, 10.0f, 10.0f, pv1Power) &&
        tryBuildSolisPowerW(snapshot.pv2Voltage, snapshot.pv2Current, 10.0f, 10.0f, pv2Power)) {
        json += String(pv1Power + pv2Power, 1);
    } else {
        json += "0";
    }

    for (size_t i = 0; i < SOLIS_REGISTER_COUNT; i++) {
        SolisRegisterValue regValue = {};
        tryGetSolisRegisterValue(snapshot, SOLIS_REGISTER_SPECS[i].reg, regValue);

        json += ",\"";
        json += String(SOLIS_REGISTER_SPECS[i].reg);
        json += "\":{";
        json += "\"valid\":";
        json += regValue.valid ? "true" : "false";
        json += ",\"raw\":";
        json += String(regValue.raw);
        json += ",\"signed\":";
        json += String(static_cast<int16_t>(regValue.raw));
        json += "}";
    }

    json += "}";
    return json;
}

static bool isAggregateUsable(const AggregateSnapshot& snap, unsigned long nowMs) {
    if (snap.valid) return true;
    return snap.lastFreshMs != 0 && (nowMs - snap.lastFreshMs < BLE_TIMEOUT_MS);
}

static AggregateSnapshot buildAggregateSnapshot(unsigned long now) {
    AggregateSnapshot snap;

    float sumVoltage = 0.0f;
    float sumCurrent = 0.0f;
    float sumTemp = 0.0f;
    uint8_t tempCount = 0;
    bool chargeAllowedInit = false;
    bool dischargeAllowedInit = false;

    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (!batteries[i].enabled) continue;

        const BatteryState& battery = batteries[i];
        if (!battery.hasData || battery.lastGoodDataMs == 0) continue;
        if (now - battery.lastGoodDataMs > DATA_FRESH_MS) continue;

        sumVoltage += battery.voltage;
        sumCurrent += battery.current;

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
        snap.soc = socFromVoltageTable(snap.voltage);
        if (tempCount > 0) {
            snap.hasTemperature = true;
            snap.temperature = sumTemp / (float)tempCount;
        }
    }

    return snap;
}

static void resetPacketAssembly(BatteryState& battery) {
    battery.packetLen = 0;
    battery.expectedLen = 0;
    battery.packetError = false;
    battery.gotPacket03 = false;
    battery.gotPacket04 = false;
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

static bool parsePacket03(BatteryState& battery) {
    if (battery.packetLen <= PACKET03_SOC_INDEX) {
        battery.packetError = true;
        return false;
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
    return true;
}

static bool parsePacket04(BatteryState& battery) {
    if (battery.packetLen < 7) {
        battery.packetError = true;
        return false;
    }

    int payloadLen = battery.packetBuf[3];
    if (payloadLen <= 0 || (payloadLen % 2) != 0) {
        battery.packetError = true;
        return false;
    }

    int count = payloadLen / 2;
    if (count > MAX_CELLS) count = MAX_CELLS;

    battery.cellCount = count;
    for (int i = 0; i < count; i++) {
        int offset = 4 + (i * 2);
        battery.cellMv[i] = ((uint16_t)battery.packetBuf[offset] << 8) |
                             battery.packetBuf[offset + 1];
    }

    battery.hasCellData = true;
    battery.lastCellDataMs = millis();
    battery.gotPacket04 = true;
    return true;
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

    if (!checksumValid(battery.packetBuf, battery.packetLen)) {
        battery.packetError = true;
        return;
    }

    uint8_t packetType = battery.packetBuf[1];

    if (packetType == 0x03) {
        if (!parsePacket03(battery)) {
            battery.packetError = true;
        }
        return;
    }

    if (packetType == 0x04) {
        if (!parsePacket04(battery)) {
            battery.packetError = true;
        }
        return;
    }

    battery.packetError = true;
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

            BLEAdvertisedDevice* discoveredDevice = new BLEAdvertisedDevice(advertisedDevice);
            if (discoveredDevice == nullptr) {
                Serial.printf("[%s] discovery allocation failed\n", batteries[i].name);
                return;
            }

            batteries[i].seen = true;
            if (batteries[i].advertisedDevice != nullptr) {
                delete batteries[i].advertisedDevice;
            }
            batteries[i].advertisedDevice = discoveredDevice;

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
static DiscoveryCallbacks discoveryCallbacks;

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
    battery.hasCellData = false;
    battery.cellCount = 0;
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
    while (!hasDeadlinePassed(deadlineMs) && seenBatteryCount() < enabledBatteryCount()) {
        int32_t remainingMs = (int32_t)(deadlineMs - millis());
        if (remainingMs < 0) remainingMs = 0;
        unsigned long sliceMs = ((unsigned long)remainingMs < SCAN_SLICE_MS)
                              ? (unsigned long)remainingMs
                              : SCAN_SLICE_MS;
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
    while (!hasDeadlinePassed(deadlineMs) && !battery.seen) {
        int32_t remainingMs = (int32_t)(deadlineMs - millis());
        if (remainingMs < 0) remainingMs = 0;
        unsigned long sliceMs = ((unsigned long)remainingMs < SCAN_SLICE_MS)
                              ? (unsigned long)remainingMs
                              : SCAN_SLICE_MS;
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
    while (!hasDeadlinePassed(deadlineMs)) {
        if (!isBatteryConnected(battery)) break;
        if (battery.gotPacket03) {
            return true;
        }
        delay(RESPONSE_POLL_MS);
    }

    return false;
}

static bool requestCellVoltages(BatteryState& battery) {
    static uint8_t cmd4[7] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

    if (!isBatteryConnected(battery)) return false;

    resetPacketAssembly(battery);
    battery.lastRequestMs = millis();
    battery.tx->writeValue(cmd4, sizeof(cmd4), false);

    unsigned long deadlineMs = millis() + RESPONSE_TIMEOUT_MS;
    while (!hasDeadlinePassed(deadlineMs)) {
        if (!isBatteryConnected(battery)) break;
        if (battery.gotPacket04) {
            return true;
        }
        delay(RESPONSE_POLL_MS);
    }

    return false;
}

static void printCellVoltages(const BatteryState& battery) {
    Serial.printf("[%s] cells (%u): ", battery.name, battery.cellCount);
    for (uint8_t i = 0; i < battery.cellCount; i++) {
        Serial.printf("%u=%.3fV", i + 1, battery.cellMv[i] / 1000.0f);
        if (i + 1 < battery.cellCount) Serial.print("  ");
    }
    Serial.println();
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

static void updateCanAggregateSnapshot(const AggregateSnapshot& snap) {
    SemaphoreHandle_t mutex = canAggregateMutex;
    if (mutex == nullptr) return;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(CAN_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        return;
    }
    canAggregate = snap;
    xSemaphoreGive(mutex);
}

static AggregateSnapshot copyCanAggregateSnapshot() {
    AggregateSnapshot snap = {};
    SemaphoreHandle_t mutex = canAggregateMutex;
    if (mutex == nullptr) return snap;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(CAN_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        snap.valid = false;
        snap.lastFreshMs = 0;
        return snap;
    }
    snap = canAggregate;
    xSemaphoreGive(mutex);
    return snap;
}

static void canTxTask(void* pv) {
    (void)pv;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        unsigned long nowMs = millis();
        AggregateSnapshot snap = copyCanAggregateSnapshot();
        if (isAggregateUsable(snap, nowMs)) {
            sendCANFrames(snap.voltage,
                          snap.current,
                          snap.soc,
                          snap.temperature,
                          snap.chargeAllowed,
                          snap.dischargeAllowed);
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CAN_INTERVAL_MS));
    }
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
        if (battery.cellCount > 0) {
            Serial.printf("  cells=%u", battery.cellCount);
        }
        Serial.println();
    }

    printSolisSummary(now);
    Serial.printf("Connected %d/%d enabled batteries\n",
                  connectedCount,
                  enabledBatteryCount());
    Serial.println("========================================");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== BLE_BMS_MARK2 (persistent BLE baseline + CAN) ===");

    setupCAN();
    canAggregateMutex = xSemaphoreCreateMutex();
    if (canAggregateMutex == nullptr) {
        Serial.println("Failed to create CAN aggregate mutex");
        Serial.println("CAN TX task disabled (missing aggregate mutex)");
    } else {
        BaseType_t canTaskOk = xTaskCreatePinnedToCore(
            canTxTask,
            "CanTx",
            CAN_TASK_STACK_SIZE,
            nullptr,
            CAN_TASK_PRIORITY,
            nullptr,
            CAN_TASK_CORE);
        if (canTaskOk != pdPASS) {
            Serial.println("Failed to start CAN TX task");
        } else {
            Serial.printf("CAN TX task started on core %d\n", CAN_TASK_CORE);
        }
    }

    solisMutex = xSemaphoreCreateMutex();
    if (solisMutex == nullptr) {
        Serial.println("Failed to create Solis mutex; inverter polling disabled");
    } else {
        RS485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
        Serial.printf("Solis RS485 polling enabled on RX=%d TX=%d every %lu ms\n",
                      RS485_RX_PIN,
                      RS485_TX_PIN,
                      (unsigned long)SOLIS_POLL_INTERVAL_MS);
    }

    if (WIFI_SSID[0] != '\0') {
        WiFi.mode(WIFI_STA);
        bool wifiOk = tryConnectWiFi(WIFI_SSID, WIFI_PASSWORD);
        if (wifiOk) {
            Serial.printf("WiFi connected - http://%s\n", WiFi.localIP().toString().c_str());
            server.on("/api/solis", HTTP_GET, [](AsyncWebServerRequest* request) {
                request->send(200, "application/json", buildSolisJson());
            });
            server.begin();
            Serial.println("Async web server started on port 80 (/api/solis)");
        } else {
            Serial.println("WiFi failed - continuing without web server");
        }
    } else {
        Serial.println("WiFi disabled - set WIFI_SSID/WIFI_PASSWORD to enable /api/solis");
    }

    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(&discoveryCallbacks);

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

    aggregate = buildAggregateSnapshot(millis());
    updateCanAggregateSnapshot(aggregate);
    printSummary();
}

void loop() {
    cycleCount++;
    Serial.printf("\n--- Persistent cycle %lu ---\n", cycleCount);
    maybePollSolis(millis());

    for (int i = 0; i < BATTERY_COUNT; i++) {
        BatteryState& battery = batteries[i];
        if (!battery.enabled) continue;

        if (isBatteryConnected(battery)) {
            bool ok03 = requestBatterySnapshot(battery);
            if (ok03) {
                bool ok04 = requestCellVoltages(battery);
                if (ok04) {
                    battery.okReads++;
                    Serial.printf("[%s] OK   %.2f V  %.2f A  SoC %u%%\n",
                                  battery.name,
                                  battery.voltage,
                                  battery.current,
                                  battery.soc);
                    printCellVoltages(battery);
                } else {
                    battery.failedReads++;
                    Serial.printf("[%s] FAIL waiting for 0x04 cell-voltage response\n",
                                  battery.name);
                }
            } else {
                battery.failedReads++;
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
        maybePollSolis(millis());
    }

    if (millis() - lastSummaryMs >= LOG_INTERVAL_MS) {
        lastSummaryMs = millis();
        printSummary();
    }

    aggregate = buildAggregateSnapshot(millis());
    updateCanAggregateSnapshot(aggregate);

    delay(BETWEEN_CYCLES_MS);
}