#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <math.h>
#include <new>
#include <stdarg.h>
#include <string.h>
#include <esp_system.h>
#include "driver/twai.h"

// -----------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------
static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_rx("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_tx("0000ff02-0000-1000-8000-00805f9b34fb");

#define CAN_TX_PIN     GPIO_NUM_21
#define CAN_RX_PIN     GPIO_NUM_19

#define CAN_INTERVAL_MS            500
#define LOG_INTERVAL_MS           5000
#define HEARTBEAT_INTERVAL_MS     1000
#define BLE_TIMEOUT_MS  (3UL * 60UL * 1000UL)
#define DATA_FRESH_MS            15000UL
// Operational safety valve: reboot every 4 hours to recover from long-lived
// BLE/Wi-Fi stack wedging. Adjust this single constant in code if needed.
#define PERIODIC_REBOOT_INTERVAL_MS (4UL * 60UL * 60UL * 1000UL)
#define LOG_HISTORY_LINES             120
#define LOG_LINE_MAX_CHARS            160
#define LOG_PRINTF_BUFFER             320
#define CONFIG_NAMESPACE           "bms_cfg"
#define CONFIG_KEY_ENABLED_MASK "enabled_mask"
#define CONFIG_KEY_REBOOT_REASON "reboot_reason"
#define CONFIG_KEY_REBOOT_UPTIME "reboot_uptime"
#define CONFIG_KEY_REBOOT_AT_MS  "reboot_at_ms"

#define WIFI_SSID           "TP-LINK_73F3"
#define WIFI_PASSWORD_UPPER "DEADBEEF"
#define WIFI_PASSWORD_LOWER "deadbeef"
#define WIFI_CONNECT_TIMEOUT_MS  10000

#define RS485_RX_PIN                 16
#define RS485_TX_PIN                 17
#define SOLIS_MODBUS_TIMEOUT_MS     180
// Single full-span block read once every 2 seconds (previously 5 s with 21 individual reads).
#define SOLIS_POLL_INTERVAL_MS     2000
// SOLIS_INTER_REGISTER_DELAY_MS removed: no per-register delays with block read.
// Conservative inter-byte timeout for the 93-register block response.
// readRS485Reply() resets this timer on every byte received, so 300 ms gives
// ample headroom beyond the ~200 ms wire time for 191 bytes at 9600 baud.
#define SOLIS_BLOCK_READ_TIMEOUT_MS 300
// Full-span block: doc registers 33050..33142 inclusive (33142 - 33050 + 1 = 93).
#define SOLIS_BLOCK_START_REG     33050
#define SOLIS_BLOCK_END_REG       33142
#define SOLIS_BLOCK_REG_COUNT        93
#define SOLIS_MIN_POLL_WAIT_MS      100
#define SOLIS_STALE_POLL_MULTIPLIER   2
#define SOLIS_DIVISOR_EPSILON   1.0e-6f
#define SOLIS_MUTEX_TIMEOUT_MS      100
#define CAN_MUTEX_TIMEOUT_MS        100
#define SOLIS_SLAVE_ID                1
#define CAN_TASK_STACK_SIZE        4096
#define CAN_TASK_PRIORITY             1
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

struct BatteryConfig {
    const char* name;
    const char* mac;
    bool enabled;
};

static BatteryConfig batteryConfigs[] = {
    {"Growatt", "a5:c2:37:49:c7:a2", false},
    {"Solax", "a4:c1:37:20:4e:3b", false},
    {"SP14S004P14S40A", "a5:c2:37:51:85:7f", false},
    {"Growatt2", "a5:c2:37:51:85:89", true}
};

static const int BATTERY_COUNT = sizeof(batteryConfigs) / sizeof(batteryConfigs[0]);
static_assert(BATTERY_COUNT <= 32, "Battery enabled mask supports up to 32 batteries");

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

struct RegisterSpec {
    uint16_t reg;
};

static const RegisterSpec SOLIS_REGISTER_SPECS[] = {
    {33050}, {33051}, {33052}, {33053}, {33059}, {33072}, {33074},
    {33080}, {33081}, {33085}, {33095}, {33129}, {33130}, {33131},
    {33132}, {33134}, {33135}, {33136}, {33137}, {33140}, {33142},
};

static const size_t SOLIS_REGISTER_COUNT = sizeof(SOLIS_REGISTER_SPECS) / sizeof(SOLIS_REGISTER_SPECS[0]);
static const uint16_t SOLIS_REG_BATTERY_DIRECTION = 33136;
static const uint16_t SOLIS_REG_BATTERY_CURRENT = 33135;
static const uint16_t SOLIS_REG_BATTERY_SOC = 33140;
static const uint16_t SOLIS_REG_BATTERY_VOLTAGE = 33142;
static const uint16_t SOLIS_REG_GRID_POWER = 33132;
static const uint16_t SOLIS_REG_GRID_VOLTAGE = 33074;
static const uint16_t SOLIS_REG_GRID_FREQUENCY = 33095;
static const uint16_t SOLIS_REG_PV1_VOLTAGE = 33050;
static const uint16_t SOLIS_REG_PV1_CURRENT = 33051;
static const uint16_t SOLIS_REG_PV2_VOLTAGE = 33052;
static const uint16_t SOLIS_REG_PV2_CURRENT = 33053;

struct RegisterValue {
    uint16_t raw;
    bool valid;
};

struct SolisState {
    RegisterValue values[SOLIS_REGISTER_COUNT];
    uint32_t lastPollMs;
    uint32_t lastSuccessMs;
    uint32_t pollCount;
    uint32_t readErrors;
    uint32_t lockTimeouts;
};

struct SolisRegisterInfo {
    uint16_t reg;
    const char* label;
    float divisor;
    uint8_t decimals;
    const char* unit;
    bool signedValue;
    bool rawOnly;
    const char* note;
};

static const SolisRegisterInfo SOLIS_REGISTER_INFOS[] = {
    {33050, "PV string 1 voltage", 10.0f, 1, "V", false, false, "Confirmed. 0.1 V scale."},
    {33051, "PV string 1 current", 10.0f, 1, "A", false, false, "Confirmed. 0.1 A scale."},
    {33052, "PV string 2 voltage", 10.0f, 1, "V", false, false, "Confirmed. 0.1 V scale."},
    {33053, "PV string 2 current", 10.0f, 1, "A", false, false, "Confirmed. 0.1 A scale."},
    {33059, "Battery / power-related candidate", 1.0f, 0, "", false, true, "Raw candidate register."},
    {33072, "Register 33072", 1.0f, 0, "", false, true, "Unknown candidate register."},
    {33074, "Grid voltage", 10.0f, 1, "V", false, false, "Confirmed. 0.1 V scale."},
    {33080, "Register 33080", 1.0f, 0, "", false, true, "Unknown candidate register."},
    {33081, "Register 33081", 1.0f, 0, "", false, true, "Unknown candidate register."},
    {33085, "Register 33085", 1.0f, 0, "", false, true, "Unknown candidate register."},
    {33095, "Grid frequency", 100.0f, 2, "Hz", false, false, "Confirmed. 0.01 Hz scale."},
    {33129, "Register 33129", 1.0f, 0, "", false, true, "AC-side candidate register."},
    {33130, "Register 33130", 1.0f, 0, "", false, true, "Unknown AC-side candidate."},
    {33131, "Register 33131", 1.0f, 0, "", false, true, "Unknown mode-related candidate."},
    {33132, "Grid power", 1.0f, 0, "W", true, false, "Signed watts. Negative=import, positive=export."},
    {33134, "Register 33134", 1.0f, 0, "", false, true, "Battery / power-related candidate."},
    {33135, "Battery current", 10.0f, 1, "A", false, false, "Strong candidate. 0.1 A scale."},
    {33136, "Battery direction", 1.0f, 0, "", false, true, "0=charging, 1=discharging (candidate)."},
    {33137, "Register 33137", 1.0f, 0, "", false, true, "PV-related candidate register."},
    {33140, "Battery SOC", 1.0f, 0, "%", false, false, "Confirmed. SOC percent."},
    {33142, "Battery voltage", 100.0f, 2, "V", false, false, "Confirmed. 0.01 V scale."},
};
static const size_t SOLIS_REGISTER_INFO_COUNT = sizeof(SOLIS_REGISTER_INFOS) / sizeof(SOLIS_REGISTER_INFOS[0]);

static SolisState solisState = {};
static SemaphoreHandle_t solisMutex = nullptr;

static BatteryState batteries[BATTERY_COUNT];
static AggregateSnapshot aggregate;
static AggregateSnapshot canAggregate;
static SemaphoreHandle_t canAggregateMutex = nullptr;
static volatile uint32_t canAggregateLockTimeouts = 0;
static Preferences configPrefs;
static bool prefsReady = false;

static char logLines[LOG_HISTORY_LINES][LOG_LINE_MAX_CHARS];
static uint16_t logHead = 0;
static uint16_t logCount = 0;
static char logCurrentLine[LOG_LINE_MAX_CHARS];
static uint16_t logCurrentLen = 0;
static portMUX_TYPE logBufferMux = portMUX_INITIALIZER_UNLOCKED;

static char lastPersistedRebootReason[32] = "";
static unsigned long lastPersistedRebootUptimeMs = 0;
static unsigned long lastPersistedRebootAtMs = 0;
static bool lastPersistedRebootPresent = false;
static unsigned long bootStartMs = 0;

static BLEScan* pBLEScan = nullptr;
static bool wifiReady = false;

WebServer server(80);
HardwareSerial RS485(2);
static HardwareSerial& BaseSerial = Serial;

static void appendLogText(const char* text);

class LoggedSerialProxy {
public:
    void begin(unsigned long baud) { BaseSerial.begin(baud); }
    void flush() { BaseSerial.flush(); }

    size_t print(const char* value) {
        if (value == nullptr) return 0;
        size_t written = BaseSerial.print(value);
        appendLogText(value);
        return written;
    }

    size_t print(const String& value) {
        size_t written = BaseSerial.print(value);
        appendLogText(value.c_str());
        return written;
    }

    template <typename T>
    size_t print(const T& value) {
        String text(value);
        size_t written = BaseSerial.print(text);
        appendLogText(text.c_str());
        return written;
    }

    size_t println() {
        size_t written = BaseSerial.println();
        appendLogText("\n");
        return written;
    }

    size_t println(const char* value) {
        size_t written = BaseSerial.println(value);
        if (value != nullptr) appendLogText(value);
        appendLogText("\n");
        return written;
    }

    size_t println(const String& value) {
        size_t written = BaseSerial.println(value);
        appendLogText(value.c_str());
        appendLogText("\n");
        return written;
    }

    template <typename T>
    size_t println(const T& value) {
        String text(value);
        size_t written = BaseSerial.println(text);
        appendLogText(text.c_str());
        appendLogText("\n");
        return written;
    }

    size_t printf(const char* format, ...) {
        if (format == nullptr) return 0;

        char buffer[LOG_PRINTF_BUFFER];
        va_list args;
        va_start(args, format);
        int len = vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        if (len < 0) return 0;

        if (len >= (int)sizeof(buffer)) len = sizeof(buffer) - 1;

        BaseSerial.print(buffer);
        appendLogText(buffer);
        return static_cast<size_t>(len);
    }
};

static LoggedSerialProxy LogSerial;

// Explicit prototypes avoid Arduino auto-generated prototypes being emitted
// before the custom type definitions above.
static RegisterValue getSolisRegisterValue(const SolisState& state, uint16_t docReg);
static const SolisRegisterInfo* findSolisRegisterInfo(uint16_t docReg);
static bool isSolisBatteryDischarging(bool valid, uint16_t raw);
static bool tryGetSolisScaledValue(const SolisState& state, uint16_t docReg, float divisor, bool signedValue, float& value);
static bool tryBuildSignedSolisBatteryPowerW(const SolisState& state, float& powerW);
static bool tryBuildSolisPvPowerW(const SolisState& state, uint16_t voltageReg, uint16_t currentReg, float& powerW);
static bool copySolisSnapshot(SolisState& snapshot);
static String buildInverterJson();
static String buildStatusJson();
static String htmlEscape(const String& input);
static String jsonEscape(const String& input);
static const char* resetReasonToString(esp_reset_reason_t reason);
static void loadBatteryEnabledConfig();
static bool persistBatteryEnabledConfig();
static bool setBatteryEnabled(int index, bool enabled);
static void loadPersistedRebootInfo();
static void rememberPendingReboot(const char* reason, unsigned long nowMs);
static void clearPendingRebootInfo();
static void servicePeriodicReboot(unsigned long nowMs);
static bool isBatteryContributing(int index, unsigned long nowMs);
static bool tryParseBatteryIndex(const String& input, int& index);
static void handleBatteryToggle();
static void handleLogs();
static void handleLogsApi();
static void handleStatusApi();
static void handleInverter();
static void handleInverterApi();
static void cleanupBatteryClient(int index);
static bool readSolisBlockU16(uint8_t slave, uint16_t startDocReg, uint16_t count, uint16_t* outValues);
static void pollSolisOnce(unsigned long nowMs);
static void canTxTask(void* pv);

// CAN API from CAN_Pylontech.ino
void setupCAN();
void sendCANFrames(float voltage,
                   float current,
                   uint8_t soc,
                   float temperature,
                   bool chargeAllowed,
                   bool dischargeAllowed);

static void finishCurrentLogLineLocked() {
    if (logCurrentLen == 0) return;

    if (logCurrentLen >= LOG_LINE_MAX_CHARS) logCurrentLen = LOG_LINE_MAX_CHARS - 1;
    logCurrentLine[logCurrentLen] = '\0';
    strncpy(logLines[logHead], logCurrentLine, LOG_LINE_MAX_CHARS);
    logLines[logHead][LOG_LINE_MAX_CHARS - 1] = '\0';
    logHead = (logHead + 1) % LOG_HISTORY_LINES;
    if (logCount < LOG_HISTORY_LINES) logCount++;
    logCurrentLen = 0;
    logCurrentLine[0] = '\0';
}

static void appendLogCharLocked(char c, unsigned long timestampMs) {
    if (c == '\r') return;

    if (c == '\n') {
        finishCurrentLogLineLocked();
        return;
    }

    if (logCurrentLen == 0) {
        int prefixLen = snprintf(logCurrentLine, sizeof(logCurrentLine), "[%10lu] ", timestampMs);
        if (prefixLen < 0) prefixLen = 0;
        if (prefixLen >= LOG_LINE_MAX_CHARS) prefixLen = LOG_LINE_MAX_CHARS - 1;
        logCurrentLen = (uint16_t)prefixLen;
    }

    if (logCurrentLen < (LOG_LINE_MAX_CHARS - 1)) {
        logCurrentLine[logCurrentLen++] = c;
    }
}

static void appendLogText(const char* text) {
    if (text == nullptr) return;

    unsigned long timestampMs = millis();
    portENTER_CRITICAL(&logBufferMux);
    while (*text != '\0') {
        appendLogCharLocked(*text++, timestampMs);
    }
    portEXIT_CRITICAL(&logBufferMux);
}

static uint16_t copyLogLineAt(uint16_t orderedIndex, char* out, size_t outSize) {
    if (out == nullptr || outSize == 0) return 0;

    out[0] = '\0';

    portENTER_CRITICAL(&logBufferMux);
    uint16_t storedCount = logCount;
    uint16_t extraCurrent = logCurrentLen > 0 ? 1 : 0;
    uint16_t total = storedCount + extraCurrent;
    if (orderedIndex >= total) {
        portEXIT_CRITICAL(&logBufferMux);
        return total;
    }

    if (orderedIndex < storedCount) {
        uint16_t start = (logHead + LOG_HISTORY_LINES - storedCount) % LOG_HISTORY_LINES;
        uint16_t actualIndex = (start + orderedIndex) % LOG_HISTORY_LINES;
        strncpy(out, logLines[actualIndex], outSize);
        out[outSize - 1] = '\0';
    } else {
        size_t copyLen = logCurrentLen;
        if (copyLen >= outSize) copyLen = outSize - 1;
        memcpy(out, logCurrentLine, copyLen);
        out[copyLen] = '\0';
    }
    portEXIT_CRITICAL(&logBufferMux);
    return total;
}

static uint16_t getLogLineCount() {
    portENTER_CRITICAL(&logBufferMux);
    uint16_t total = logCount + (logCurrentLen > 0 ? 1 : 0);
    portEXIT_CRITICAL(&logBufferMux);
    return total;
}

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

static bool hasDeadlinePassed(unsigned long deadlineMs) {
    return (int32_t)(millis() - deadlineMs) >= 0;
}

static uint32_t scanDurationSeconds(unsigned long durationMs) {
    return (uint32_t)((durationMs + 999UL) / 1000UL);
}

static bool isBatteryEnabled(int index) {
    return index >= 0 && index < BATTERY_COUNT && batteryConfigs[index].enabled;
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
    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (batteryConfigs[i].enabled) count++;
    }
    return count;
}

static int seenBatteryCount() {
    int count = 0;
    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (batteryConfigs[i].enabled && batteries[i].seen) count++;
    }
    return count;
}

static int connectedBatteryCount() {
    int count = 0;
    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (isBatteryConnected(i)) count++;
    }
    return count;
}

static void logBatteryDebugState(int index, const char* prefix) {
    if (index < 0 || index >= BATTERY_COUNT) return;

    BatteryState& battery = batteries[index];
    unsigned long now = millis();
    unsigned long ageGood = battery.lastGoodDataMs == 0 ? 0 : (now - battery.lastGoodDataMs);
    unsigned long ageReq = battery.lastRequestMs == 0 ? 0 : (now - battery.lastRequestMs);
    long deadlineIn = battery.requestInFlight ? (long)(battery.requestDeadlineMs - now) : 0L;

    LogSerial.printf(
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
    LogSerial.printf(
        "%s now=%lu heap=%u wifi=%s connected=%d/%d enabled=%d\n",
        prefix,
        now,
        ESP.getFreeHeap(),
        wifiStatusToString(WiFi.status()),
        connectedBatteryCount(),
        BATTERY_COUNT,
        enabledBatteryCount()
    );

    for (int i = 0; i < BATTERY_COUNT; i++) {
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
        LogSerial.printf("[DBG] [%s] notify start chunkLen=%u expected=%d status=0x%02X\n",
                      batteryConfigs[index].name,
                      (unsigned)length,
                      battery.expectedLen,
                      length > 2 ? pData[2] : 0xFF);
    }

    if (battery.packetLen + (int)length > MAX_PACKET_LEN) {
        LogSerial.printf("[DBG] [%s] notify overflow packetLen=%d add=%u\n",
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
        LogSerial.printf("[DBG] [%s] checksum fail type=0x%02X len=%d\n",
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
        LogSerial.printf("[DBG] [%s] unexpected packet type=0x%02X stage=%u len=%d\n",
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
            LogSerial.printf("[DBG] [%s] packet03 complete V=%.2f I=%.2f SoC=%u\n",
                          batteryConfigs[index].name,
                          battery.voltage,
                          battery.current,
                          battery.soc);
        } else {
            LogSerial.printf("[DBG] [%s] packet04 complete cells=%u\n",
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
        LogSerial.printf("[%s] connected\n", batteryConfigs[index].name);
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

        LogSerial.printf("[%s] disconnected (drops=%lu)\n",
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

        for (int i = 0; i < BATTERY_COUNT; i++) {
            if (!batteryConfigs[i].enabled) continue;

            String targetMac = batteryConfigs[i].mac;
            targetMac.toLowerCase();
            if (seenMac != targetMac) continue;

            BLEAdvertisedDevice* discoveredDevice = new (std::nothrow) BLEAdvertisedDevice(advertisedDevice);
            if (discoveredDevice == nullptr) {
                LogSerial.printf("[%s] discovery allocation failed\n", batteryConfigs[i].name);
                return;
            }

            batteries[i].seen = true;
            if (batteries[i].advertisedDevice != nullptr) {
                delete batteries[i].advertisedDevice;
            }
            batteries[i].advertisedDevice = discoveredDevice;

            LogSerial.printf("[%s] discovered at %s\n",
                          batteryConfigs[i].name,
                          batteryConfigs[i].mac);

            if (seenBatteryCount() >= enabledBatteryCount()) {
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

    LogSerial.printf("[DBG] cleanupBatteryClient start [%s]\n", batteryConfigs[index].name);
    logBatteryDebugState(index, "[DBG before cleanup]");

    if (battery.client != nullptr) {
        if (battery.client->isConnected()) {
            LogSerial.printf("[DBG] [%s] before disconnect()\n", batteryConfigs[index].name);
            battery.client->disconnect();
            LogSerial.printf("[DBG] [%s] after disconnect()\n", batteryConfigs[index].name);
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
        LogSerial.printf("[DBG] [%s] connect skipped: no advertisedDevice\n", batteryConfigs[index].name);
        battery.nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
        return false;
    }

    LogSerial.printf("[DBG] [%s] connectBattery start\n", batteryConfigs[index].name);
    logBatteryDebugState(index, "[DBG before connect]");

    cleanupBatteryClient(index);

    LogSerial.printf("[DBG] [%s] before createClient()\n", batteryConfigs[index].name);
    battery.client = BLEDevice::createClient();
    LogSerial.printf("[DBG] [%s] after createClient() client=%p\n", batteryConfigs[index].name, battery.client);

    if (battery.client == nullptr) {
        LogSerial.printf("[%s] failed to create BLE client\n", batteryConfigs[index].name);
        return false;
    }

    battery.client->setClientCallbacks(&clientCallbacks);

    delay(CONNECT_DELAY_MS);

    LogSerial.printf("[DBG] [%s] before connect()\n", batteryConfigs[index].name);
    bool connectOk = battery.client->connect(battery.advertisedDevice);
    LogSerial.printf("[DBG] [%s] after connect() => %s\n", batteryConfigs[index].name, connectOk ? "OK" : "FAIL");

    if (!connectOk) {
        LogSerial.printf("[%s] connect() failed\n", batteryConfigs[index].name);
        cleanupBatteryClient(index);
        return false;
    }

    LogSerial.printf("[DBG] [%s] before getService()\n", batteryConfigs[index].name);
    battery.service = battery.client->getService(serviceUUID);
    LogSerial.printf("[DBG] [%s] after getService() service=%p\n", batteryConfigs[index].name, battery.service);

    if (battery.service == nullptr) {
        LogSerial.printf("[%s] FF00 service not found\n", batteryConfigs[index].name);
        cleanupBatteryClient(index);
        return false;
    }

    LogSerial.printf("[DBG] [%s] before getCharacteristic(RX)\n", batteryConfigs[index].name);
    battery.rx = battery.service->getCharacteristic(charUUID_rx);
    LogSerial.printf("[DBG] [%s] after getCharacteristic(RX) rx=%p\n", batteryConfigs[index].name, battery.rx);

    if (battery.rx == nullptr || !battery.rx->canNotify()) {
        LogSerial.printf("[%s] FF01 notify characteristic not found\n", batteryConfigs[index].name);
        cleanupBatteryClient(index);
        return false;
    }

    LogSerial.printf("[DBG] [%s] before registerForNotify()\n", batteryConfigs[index].name);
    battery.rx->registerForNotify(notifyCallback);
    LogSerial.printf("[DBG] [%s] after registerForNotify()\n", batteryConfigs[index].name);

    LogSerial.printf("[DBG] [%s] before getCharacteristic(TX)\n", batteryConfigs[index].name);
    battery.tx = battery.service->getCharacteristic(charUUID_tx);
    LogSerial.printf("[DBG] [%s] after getCharacteristic(TX) tx=%p\n", batteryConfigs[index].name, battery.tx);

    if (battery.tx == nullptr || (!battery.tx->canWrite() && !battery.tx->canWriteNoResponse())) {
        LogSerial.printf("[%s] FF02 write characteristic not found\n", batteryConfigs[index].name);
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
        LogSerial.printf("[%s] ready: connected + notifications registered\n", batteryConfigs[index].name);
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
            LogSerial.printf("[DBG] [%s] request timeout now=%lu deadline=%lu packetLen=%d expected=%d packetError=%s\n",
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

    LogSerial.printf("[DBG] [%s] before writeValue cmd=0x%02X cycle=%s heap=%u\n",
                  batteryConfigs[index].name,
                  cmd[2],
                  partOfCurrentCycle ? "continue" : "start",
                  ESP.getFreeHeap());

    bool ok = battery.tx->writeValue(cmd, cmdLen, false);

    LogSerial.printf("[DBG] [%s] after writeValue cmd=0x%02X ok=%s\n",
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
        LogSerial.printf("[DBG] [%s] writeValue failed immediately for cmd=0x%02X\n",
                      batteryConfigs[index].name,
                      cmd[2]);
        logBatteryDebugState(index, "[DBG write fail]");
    }
}

static bool reconnectBattery(int index) {
    if (isBatteryConnected(index)) return true;

    LogSerial.printf("[%s] reconnect attempt\n", batteryConfigs[index].name);
    logBatteryDebugState(index, "[DBG before reconnect]");

    if (batteries[index].advertisedDevice == nullptr) {
        if (!scanForBattery(index, RECONNECT_SCAN_TIMEOUT_MS)) {
            LogSerial.printf("[%s] not seen during reconnect scan\n", batteryConfigs[index].name);
            batteries[index].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
            return false;
        }
    }

    if (connectBattery(index)) return true;

    if (!scanForBattery(index, RECONNECT_SCAN_TIMEOUT_MS)) {
        LogSerial.printf("[%s] rediscovery failed\n", batteryConfigs[index].name);
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

    for (int i = 0; i < BATTERY_COUNT; i++) {
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

static String jsonBool(bool value) {
    return value ? "true" : "false";
}

static String jsonEscape(const String& input) {
    String out;
    out.reserve(input.length() + 16);
    for (size_t i = 0; i < input.length(); i++) {
        char c = input.charAt(i);
        switch (c) {
            case '\\': out += F("\\\\"); break;
            case '"': out += F("\\\""); break;
            case '\b': out += F("\\b"); break;
            case '\f': out += F("\\f"); break;
            case '\n': out += F("\\n"); break;
            case '\r': out += F("\\r"); break;
            case '\t': out += F("\\t"); break;
            default:
                if ((uint8_t)c < 0x20U) {
                    char escaped[7];
                    snprintf(escaped, sizeof(escaped), "\\u%04x", (unsigned char)c);
                    out += escaped;
                } else {
                    out += c;
                }
                break;
        }
    }
    return out;
}

static const char* resetReasonToString(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_UNKNOWN: return "unknown";
        case ESP_RST_POWERON: return "power_on";
        case ESP_RST_EXT: return "external_pin";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt_watchdog";
        case ESP_RST_TASK_WDT: return "task_watchdog";
        case ESP_RST_WDT: return "other_watchdog";
        case ESP_RST_DEEPSLEEP: return "deep_sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        default: return "other";
    }
}

static uint32_t buildEnabledBatteryMask() {
    uint32_t mask = 0;
    for (int i = 0; i < BATTERY_COUNT && i < 32; i++) {
        if (batteryConfigs[i].enabled) mask |= (1UL << i);
    }
    return mask;
}

static void applyEnabledBatteryMask(uint32_t mask) {
    for (int i = 0; i < BATTERY_COUNT && i < 32; i++) {
        batteryConfigs[i].enabled = (mask & (1UL << i)) != 0;
    }
}

static void loadBatteryEnabledConfig() {
    uint32_t defaultMask = buildEnabledBatteryMask();
    if (!prefsReady) {
        LogSerial.printf("Battery enable config using sketch defaults mask=0x%08lX\n",
                      static_cast<unsigned long>(defaultMask));
        return;
    }

    if (!configPrefs.isKey(CONFIG_KEY_ENABLED_MASK)) {
        configPrefs.putULong(CONFIG_KEY_ENABLED_MASK, defaultMask);
    }

    uint32_t storedMask = configPrefs.getULong(CONFIG_KEY_ENABLED_MASK, defaultMask);
    applyEnabledBatteryMask(storedMask);
    LogSerial.printf("Battery enable config loaded from NVS mask=0x%08lX\n",
                  static_cast<unsigned long>(storedMask));
}

static bool persistBatteryEnabledConfig() {
    if (!prefsReady) return false;

    uint32_t mask = buildEnabledBatteryMask();
    size_t written = configPrefs.putULong(CONFIG_KEY_ENABLED_MASK, mask);
    if (written == 0) return false;

    LogSerial.printf("Battery enable config saved to NVS mask=0x%08lX\n",
                  static_cast<unsigned long>(mask));
    return true;
}

static bool setBatteryEnabled(int index, bool enabled) {
    if (index < 0 || index >= BATTERY_COUNT) return false;
    if (batteryConfigs[index].enabled == enabled) return true;

    bool previous = batteryConfigs[index].enabled;
    batteryConfigs[index].enabled = enabled;
    if (!persistBatteryEnabledConfig()) {
        batteryConfigs[index].enabled = previous;
        return false;
    }

    BatteryState& battery = batteries[index];
    if (enabled) {
        battery.seen = false;
        battery.nextReconnectMs = millis();
    } else {
        battery.nextReconnectMs = 0;
        battery.seen = false;
        if (battery.advertisedDevice != nullptr) {
            delete battery.advertisedDevice;
            battery.advertisedDevice = nullptr;
        }
        cleanupBatteryClient(index);
    }

    LogSerial.printf("[%s] aggregate membership %s via web\n",
                  batteryConfigs[index].name,
                  enabled ? "enabled" : "disabled");
    return true;
}

static void loadPersistedRebootInfo() {
    lastPersistedRebootPresent = false;
    lastPersistedRebootReason[0] = '\0';
    lastPersistedRebootUptimeMs = 0;
    lastPersistedRebootAtMs = 0;

    if (!prefsReady) return;
    if (!configPrefs.isKey(CONFIG_KEY_REBOOT_REASON)) return;

    String reason = configPrefs.getString(CONFIG_KEY_REBOOT_REASON, "");
    if (reason.length() == 0) {
        clearPendingRebootInfo();
        return;
    }

    reason.toCharArray(lastPersistedRebootReason, sizeof(lastPersistedRebootReason));
    lastPersistedRebootUptimeMs = configPrefs.getULong(CONFIG_KEY_REBOOT_UPTIME, 0);
    lastPersistedRebootAtMs = configPrefs.getULong(CONFIG_KEY_REBOOT_AT_MS, 0);
    lastPersistedRebootPresent = true;
    clearPendingRebootInfo();
}

static void rememberPendingReboot(const char* reason, unsigned long nowMs) {
    if (!prefsReady || reason == nullptr) return;
    configPrefs.putString(CONFIG_KEY_REBOOT_REASON, reason);
    configPrefs.putULong(CONFIG_KEY_REBOOT_UPTIME, nowMs);
    configPrefs.putULong(CONFIG_KEY_REBOOT_AT_MS, nowMs);
}

static void clearPendingRebootInfo() {
    if (!prefsReady) return;
    configPrefs.remove(CONFIG_KEY_REBOOT_REASON);
    configPrefs.remove(CONFIG_KEY_REBOOT_UPTIME);
    configPrefs.remove(CONFIG_KEY_REBOOT_AT_MS);
}

static bool isBatteryContributing(int index, unsigned long nowMs) {
    if (!isBatteryEnabled(index)) return false;
    const BatteryState& battery = batteries[index];
    unsigned long ageMs = nowMs - battery.lastGoodDataMs;
    return battery.hasData &&
           battery.lastGoodDataMs != 0 &&
           (int32_t)(nowMs - battery.lastGoodDataMs) >= 0 &&
           ageMs <= DATA_FRESH_MS;
}

static bool tryParseBatteryIndex(const String& input, int& index) {
    if (input.length() == 0) return false;

    char* endPtr = nullptr;
    long parsed = strtol(input.c_str(), &endPtr, 10);
    if (endPtr == nullptr || *endPtr != '\0') return false;
    if (parsed < 0 || parsed >= BATTERY_COUNT) return false;

    index = (int)parsed;
    return true;
}

static void servicePeriodicReboot(unsigned long nowMs) {
    if (PERIODIC_REBOOT_INTERVAL_MS == 0) return;
    unsigned long uptimeMs = nowMs - bootStartMs;
    if (uptimeMs < PERIODIC_REBOOT_INTERVAL_MS) return;

    LogSerial.printf("[SYSTEM] periodic reboot due after %lu ms uptime (interval=%lu ms)\n",
                  uptimeMs,
                  (unsigned long)PERIODIC_REBOOT_INTERVAL_MS);
    rememberPendingReboot("periodic_4h", uptimeMs);
    LogSerial.flush();
    delay(50);
    ESP.restart();
}

static RegisterValue getSolisRegisterValue(const SolisState& state, uint16_t docReg) {
    for (size_t i = 0; i < SOLIS_REGISTER_COUNT; i++) {
        if (SOLIS_REGISTER_SPECS[i].reg == docReg) return state.values[i];
    }
    return {0, false};
}

static const SolisRegisterInfo* findSolisRegisterInfo(uint16_t docReg) {
    for (size_t i = 0; i < SOLIS_REGISTER_INFO_COUNT; i++) {
        if (SOLIS_REGISTER_INFOS[i].reg == docReg) return &SOLIS_REGISTER_INFOS[i];
    }
    return nullptr;
}

static bool isSolisBatteryDischarging(bool valid, uint16_t raw) {
    return valid && raw == 1;
}

static bool tryGetSolisScaledValue(const SolisState& state,
                                   uint16_t docReg,
                                   float divisor,
                                   bool signedValue,
                                   float& value) {
    RegisterValue regValue = getSolisRegisterValue(state, docReg);
    if (!regValue.valid || fabsf(divisor) < SOLIS_DIVISOR_EPSILON) return false;

    value = signedValue ? static_cast<float>(static_cast<int16_t>(regValue.raw)) / divisor
                        : static_cast<float>(regValue.raw) / divisor;
    return true;
}

static bool tryBuildSignedSolisBatteryPowerW(const SolisState& state, float& powerW) {
    float currentAmps = 0.0f;
    float voltageVolts = 0.0f;
    if (!tryGetSolisScaledValue(state, SOLIS_REG_BATTERY_CURRENT, 10.0f, false, currentAmps) ||
        !tryGetSolisScaledValue(state, SOLIS_REG_BATTERY_VOLTAGE, 100.0f, false, voltageVolts)) {
        return false;
    }

    RegisterValue direction = getSolisRegisterValue(state, SOLIS_REG_BATTERY_DIRECTION);
    powerW = voltageVolts * currentAmps;
    if (isSolisBatteryDischarging(direction.valid, direction.raw)) powerW = -powerW;
    return true;
}

static bool tryBuildSolisPvPowerW(const SolisState& state,
                                  uint16_t voltageReg,
                                  uint16_t currentReg,
                                  float& powerW) {
    float voltageVolts = 0.0f;
    float currentAmps = 0.0f;
    if (!tryGetSolisScaledValue(state, voltageReg, 10.0f, false, voltageVolts) ||
        !tryGetSolisScaledValue(state, currentReg, 10.0f, false, currentAmps)) {
        return false;
    }

    powerW = voltageVolts * currentAmps;
    return true;
}

static bool copySolisSnapshot(SolisState& snapshot) {
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

static bool readSolisDocRegU16(uint8_t slave, uint16_t docReg, uint16_t& value) {
    uint8_t buf[32];
    const uint16_t rawAddr = docReg - 1;

    flushRS485Input();
    sendReadInput(slave, rawAddr, 1);

    const int len = readRS485Reply(buf, sizeof(buf), SOLIS_MODBUS_TIMEOUT_MS);
    if (len < 7) return false;
    if (!validModbusCRC(buf, len)) return false;
    if (buf[0] != slave || buf[1] != 0x04 || buf[2] != 2) return false;

    value = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];
    return true;
}

// Reads a contiguous block of 'count' Modbus input registers (FC 0x04) starting
// at doc register 'startDocReg', using the same docReg-1 raw-address convention
// as readSolisDocRegU16().  On success the decoded 16-bit values are written to
// outValues[0..count-1] and true is returned.
//
// For the full 93-register span (33050..33142) the Modbus response is:
//   1 byte slave  +  1 byte func  +  1 byte byteCount  +  186 data bytes  +  2 CRC bytes  =  191 bytes.
// The receive buffer below is sized for exactly this maximum.
static bool readSolisBlockU16(uint8_t slave, uint16_t startDocReg, uint16_t count, uint16_t* outValues) {
    // Statically allocated to avoid a large stack frame.
    // Layout: 1 byte slave + 1 byte func + 1 byte byteCount + (count × 2) data bytes + 2 CRC bytes.
    // For 93 registers: 3 + 186 + 2 = 191 bytes.
    static uint8_t buf[3 + SOLIS_BLOCK_REG_COUNT * 2 + 2];  // 191 bytes for 93 registers

    const int expectedLen = 3 + count * 2 + 2;
    if (expectedLen > static_cast<int>(sizeof(buf))) return false;

    const uint16_t rawAddr = startDocReg - 1;  // docReg-1 convention, same as readSolisDocRegU16

    flushRS485Input();
    sendReadInput(slave, rawAddr, count);

    const int len = readRS485Reply(buf, sizeof(buf), SOLIS_BLOCK_READ_TIMEOUT_MS);
    if (len < expectedLen) return false;
    if (!validModbusCRC(buf, len)) return false;
    if (buf[0] != slave || buf[1] != 0x04) return false;
    if (buf[2] != static_cast<uint8_t>(count * 2)) return false;

    for (uint16_t i = 0; i < count; i++) {
        const int offset = 3 + (i * 2);
        outValues[i] = (static_cast<uint16_t>(buf[offset]) << 8) | buf[offset + 1];
    }
    return true;
}

static void pollSolisOnce(unsigned long nowMs) {
    // Snapshot buffer for the full 93-register block (33050..33142).
    uint16_t blockValues[SOLIS_BLOCK_REG_COUNT];
    uint32_t lockTimeoutsThisPass = 0;

    // Single Modbus FC04 read covering doc registers 33050..33142 inclusive.
    // This replaces the previous loop of 21 individual one-register reads,
    // reducing RS485 transaction overhead and improving responsiveness.
    // The full block is a consistent snapshot taken at a single point in time.
    bool blockOk = readSolisBlockU16(SOLIS_SLAVE_ID,
                                     SOLIS_BLOCK_START_REG,
                                     SOLIS_BLOCK_REG_COUNT,
                                     blockValues);

    if (blockOk) {
        if (xSemaphoreTake(solisMutex, pdMS_TO_TICKS(SOLIS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            // Extract only the registers the sketch currently cares about from the
            // full snapshot.  All entries in SOLIS_REGISTER_SPECS are expected to lie
            // within the block span; the bounds check below guards against future
            // additions that inadvertently fall outside 33050..33142.
            for (size_t i = 0; i < SOLIS_REGISTER_COUNT; i++) {
                const uint16_t docReg = SOLIS_REGISTER_SPECS[i].reg;
                if (docReg < SOLIS_BLOCK_START_REG || docReg > SOLIS_BLOCK_END_REG) {
                    // Register outside block span; skip (entry stays invalid).
                    continue;
                }
                const uint16_t offset = docReg - SOLIS_BLOCK_START_REG;
                solisState.values[i].raw   = blockValues[offset];
                solisState.values[i].valid = true;
            }
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
        LogSerial.println("Solis poll pass had no successful reads");
    }
}

static void updateCanAggregateSnapshot(const AggregateSnapshot& snap) {
    SemaphoreHandle_t mutex = canAggregateMutex;
    if (mutex == nullptr) return;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(CAN_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        canAggregateLockTimeouts++;
        return;
    }
    canAggregate = snap;
    if (mutex != nullptr) xSemaphoreGive(mutex);
}

static AggregateSnapshot copyCanAggregateSnapshot() {
    AggregateSnapshot snap = {};
    SemaphoreHandle_t mutex = canAggregateMutex;
    if (mutex == nullptr) return snap;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(CAN_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        canAggregateLockTimeouts++;
        snap.valid = false;
        snap.lastFreshMs = 0;
        return snap;
    }
    snap = canAggregate;
    if (mutex != nullptr) xSemaphoreGive(mutex);
    return snap;
}

static void canTxTask(void* pv) {
    (void)pv;
    // Baseline tick used by vTaskDelayUntil() for steady CAN cadence.
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

static String formatSolisRegisterDisplay(uint16_t docReg, const RegisterValue& regValue) {
    if (!regValue.valid) return String("--");

    const SolisRegisterInfo* info = findSolisRegisterInfo(docReg);
    if (info == nullptr || info->rawOnly) return String(regValue.raw);

    float scaled = info->signedValue ? static_cast<float>(static_cast<int16_t>(regValue.raw)) / info->divisor
                                     : static_cast<float>(regValue.raw) / info->divisor;
    String text = String(scaled, static_cast<unsigned int>(info->decimals));
    if (info->unit != nullptr && info->unit[0] != '\0') {
        text += " ";
        text += info->unit;
    }
    return text;
}

static String formatSolisCardValue(const SolisState& snapshot,
                                   uint16_t docReg,
                                   float divisor,
                                   bool signedValue,
                                   uint8_t decimals,
                                   const char* unit) {
    float value = 0.0f;
    if (!tryGetSolisScaledValue(snapshot, docReg, divisor, signedValue, value)) return String("--");

    String text = String(value, static_cast<unsigned int>(decimals));
    if (unit != nullptr && unit[0] != '\0') {
        text += " ";
        text += unit;
    }
    return text;
}

static String buildStatusJson() {
    unsigned long now = millis();
    unsigned long uptimeMs = now - bootStartMs;
    AggregateSnapshot snap = buildAggregateSnapshot(now);

    String json;
    json.reserve(2800);
    json += "{";
    json += "\"uptimeMs\":";
    json += String(uptimeMs);
    json += ",\"enabledBatteryCount\":";
    json += String(enabledBatteryCount());
    json += ",\"connectedBatteryCount\":";
    json += String(connectedBatteryCount());
    json += ",\"periodicRebootIntervalMs\":";
    json += String((unsigned long)PERIODIC_REBOOT_INTERVAL_MS);
    json += ",\"nextPeriodicRebootInMs\":";
    json += (PERIODIC_REBOOT_INTERVAL_MS == 0 || uptimeMs >= PERIODIC_REBOOT_INTERVAL_MS)
          ? String(0)
          : String((unsigned long)(PERIODIC_REBOOT_INTERVAL_MS - uptimeMs));
    json += ",\"resetReason\":\"";
    json += resetReasonToString(esp_reset_reason());
    json += "\"";
    json += ",\"lastRequestedReboot\":";
    if (lastPersistedRebootPresent) {
        json += "{";
        json += "\"reason\":\"";
        json += jsonEscape(String(lastPersistedRebootReason));
        json += "\",\"uptimeMs\":";
        json += String(lastPersistedRebootUptimeMs);
        json += ",\"recordedUptimeMs\":";
        json += String(lastPersistedRebootAtMs);
        json += "}";
    } else {
        json += "null";
    }
    json += ",\"aggregate\":{";
    json += "\"valid\":";
    json += jsonBool(snap.valid);
    json += ",\"contributingBatteries\":";
    json += String(snap.contributingBatteries);
    json += ",\"lastFreshMs\":";
    json += String(snap.lastFreshMs);
    json += ",\"voltage\":";
    json += snap.valid ? String(snap.voltage, 2) : String("null");
    json += ",\"current\":";
    json += snap.valid ? String(snap.current, 2) : String("null");
    json += ",\"soc\":";
    json += snap.valid ? String(snap.soc) : String("null");
    json += ",\"temperatureC\":";
    json += (snap.valid && snap.hasTemperature) ? String(snap.temperature, 1) : String("null");
    json += "},\"batteries\":[";

    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (i != 0) json += ",";
        const BatteryConfig& cfg = batteryConfigs[i];
        const BatteryState& battery = batteries[i];
        bool contributing = isBatteryContributing(i, now);

        json += "{";
        json += "\"index\":";
        json += String(i);
        json += ",\"name\":\"";
        json += jsonEscape(String(cfg.name));
        json += "\",\"mac\":\"";
        json += jsonEscape(String(cfg.mac));
        json += "\",\"enabled\":";
        json += jsonBool(cfg.enabled);
        json += ",\"connected\":";
        json += jsonBool(isBatteryConnected(i));
        json += ",\"seen\":";
        json += jsonBool(battery.seen);
        json += ",\"contributing\":";
        json += jsonBool(contributing);
        json += ",\"lastGoodDataMs\":";
        json += String(battery.lastGoodDataMs);
        json += ",\"dataAgeMs\":";
        json += battery.lastGoodDataMs == 0 ? String("null") : String(now - battery.lastGoodDataMs);
        json += ",\"voltage\":";
        json += battery.hasData ? String(battery.voltage, 2) : String("null");
        json += ",\"current\":";
        json += battery.hasData ? String(battery.current, 2) : String("null");
        json += ",\"soc\":";
        json += battery.hasData ? String(battery.soc) : String("null");
        json += ",\"temperatureC\":";
        json += battery.hasTemperature ? String(battery.temperature, 1) : String("null");
        json += ",\"disconnectCount\":";
        json += String(battery.disconnectCount);
        json += "}";
    }
    json += "]}";
    return json;
}

static String buildInverterJson() {
    SolisState snapshot = {};
    copySolisSnapshot(snapshot);

    String json;
    json.reserve(1700);
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

    RegisterValue direction = getSolisRegisterValue(snapshot, SOLIS_REG_BATTERY_DIRECTION);
    json += ",\"batteryDirection\":";
    if (direction.valid) {
        json += "\"";
        json += isSolisBatteryDischarging(direction.valid, direction.raw) ? "discharging" : "charging";
        json += "\"";
    } else {
        json += "null";
    }

    float derivedValue = 0.0f;
    json += ",\"batteryPowerW\":";
    json += tryBuildSignedSolisBatteryPowerW(snapshot, derivedValue) ? String(derivedValue, 1) : String("null");
    json += ",\"pv1PowerW\":";
    json += tryBuildSolisPvPowerW(snapshot, SOLIS_REG_PV1_VOLTAGE, SOLIS_REG_PV1_CURRENT, derivedValue) ? String(derivedValue, 1) : String("null");
    json += ",\"pv2PowerW\":";
    json += tryBuildSolisPvPowerW(snapshot, SOLIS_REG_PV2_VOLTAGE, SOLIS_REG_PV2_CURRENT, derivedValue) ? String(derivedValue, 1) : String("null");

    float pv1Power = 0.0f;
    float pv2Power = 0.0f;
    json += ",\"pvTotalPowerW\":";
    if (tryBuildSolisPvPowerW(snapshot, SOLIS_REG_PV1_VOLTAGE, SOLIS_REG_PV1_CURRENT, pv1Power) &&
        tryBuildSolisPvPowerW(snapshot, SOLIS_REG_PV2_VOLTAGE, SOLIS_REG_PV2_CURRENT, pv2Power)) {
        json += String(pv1Power + pv2Power, 1);
    } else {
        json += "null";
    }

    for (size_t i = 0; i < SOLIS_REGISTER_COUNT; i++) {
        json += ",\"";
        json += String(SOLIS_REGISTER_SPECS[i].reg);
        json += "\":{";
        json += "\"valid\":";
        json += jsonBool(snapshot.values[i].valid);
        json += ",\"raw\":";
        json += String(snapshot.values[i].raw);
        json += ",\"signed\":";
        json += String(static_cast<int16_t>(snapshot.values[i].raw));
        json += "}";
    }
    json += "}";
    return json;
}

static void handleInverterApi() {
    server.send(200, "application/json", buildInverterJson());
}

static void handleStatusApi() {
    server.send(200, "application/json", buildStatusJson());
}

static void handleInverter() {
    unsigned long now = millis();
    SolisState snapshot = {};
    copySolisSnapshot(snapshot);

    unsigned long ageMs = snapshot.lastSuccessMs == 0 ? 0 : (now - snapshot.lastSuccessMs);
    bool stale = snapshot.lastSuccessMs == 0 || ageMs > (SOLIS_POLL_INTERVAL_MS * SOLIS_STALE_POLL_MULTIPLIER);

    String batteryDirection = "unknown";
    RegisterValue direction = getSolisRegisterValue(snapshot, SOLIS_REG_BATTERY_DIRECTION);
    if (direction.valid) {
        batteryDirection = isSolisBatteryDischarging(direction.valid, direction.raw) ? "discharging" : "charging";
    }

    float batteryPowerW = 0.0f;
    float pv1PowerW = 0.0f;
    float pv2PowerW = 0.0f;
    bool haveBatteryPower = tryBuildSignedSolisBatteryPowerW(snapshot, batteryPowerW);
    bool havePv1Power = tryBuildSolisPvPowerW(snapshot, SOLIS_REG_PV1_VOLTAGE, SOLIS_REG_PV1_CURRENT, pv1PowerW);
    bool havePv2Power = tryBuildSolisPvPowerW(snapshot, SOLIS_REG_PV2_VOLTAGE, SOLIS_REG_PV2_CURRENT, pv2PowerW);

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    server.sendContent(F("<!DOCTYPE html><html><head>"
                         "<meta charset='UTF-8'>"
                         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                         "<meta http-equiv='refresh' content='5'>"
                         "<title>Solis inverter monitor</title>"
                         "<style>"
                         "body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}"
                         "h1{color:#e0c97f;margin-bottom:4px}h2{color:#a0b4cc;margin-top:20px;margin-bottom:6px}"
                         "a{color:#8ec5ff}table{width:100%;border-collapse:collapse;margin-top:10px}"
                         "th,td{padding:8px 10px;text-align:left;border-bottom:1px solid #2a2a4a;vertical-align:top}"
                         "th{background:#16213e;color:#a0b4cc}.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:10px;margin-bottom:16px}"
                         ".card{background:#16213e;border-radius:8px;padding:12px;text-align:center}.card .label{font-size:0.75em;color:#8899aa;margin-bottom:4px}"
                         ".card .value{font-size:1.25em;font-weight:bold}.green{color:#4caf50}.amber{color:#ff9800}.mono{font-family:monospace}"
                         "</style></head><body>"));
    server.sendContent(F("<p><a href='/'>Back to summary</a></p>"));
    server.sendContent(F("<h1>Solis inverter monitor</h1>"));
    server.sendContent(F("<p>Cached RS485 data only. Single 93-register Modbus block read (33050..33142) runs in a dedicated FreeRTOS task every ~2 seconds so BLE/CAN/web work stays responsive.</p>"));

    server.sendContent(F("<div class='grid'>"));
    sendCard(stale ? "Last good poll (stale)" : "Last good poll (fresh)",
             snapshot.lastSuccessMs == 0 ? String("Never") : String(ageMs / 1000UL) + " s ago",
             stale ? "amber" : "green");
    sendCard("Poll count", String(snapshot.pollCount));
    sendCard("Read errors", String(snapshot.readErrors));
    sendCard("Lock timeouts", String(snapshot.lockTimeouts));
    sendCard("Grid power", formatSolisCardValue(snapshot, SOLIS_REG_GRID_POWER, 1.0f, true, 0, "W"));
    sendCard("Grid voltage", formatSolisCardValue(snapshot, SOLIS_REG_GRID_VOLTAGE, 10.0f, false, 1, "V"));
    sendCard("Grid frequency", formatSolisCardValue(snapshot, SOLIS_REG_GRID_FREQUENCY, 100.0f, false, 2, "Hz"));
    sendCard("Battery SoC", formatSolisCardValue(snapshot, SOLIS_REG_BATTERY_SOC, 1.0f, false, 0, "%"));
    sendCard("Battery voltage", formatSolisCardValue(snapshot, SOLIS_REG_BATTERY_VOLTAGE, 100.0f, false, 2, "V"));
    sendCard("Battery current", formatSolisCardValue(snapshot, SOLIS_REG_BATTERY_CURRENT, 10.0f, false, 1, "A"));
    sendCard("Battery direction", batteryDirection);
    sendCard("Battery power", haveBatteryPower ? String(batteryPowerW, 1) + " W" : String("--"));
    sendCard("PV1 power", havePv1Power ? String(pv1PowerW, 1) + " W" : String("--"));
    sendCard("PV2 power", havePv2Power ? String(pv2PowerW, 1) + " W" : String("--"));
    sendCard("PV total power",
             (havePv1Power && havePv2Power) ? (String(pv1PowerW + pv2PowerW, 1) + " W") : String("--"));
    server.sendContent(F("</div>"));

    server.sendContent(F("<h2>Register snapshot</h2><table><tr>"
                         "<th>Label</th><th>Register</th><th>Display</th><th>Raw</th><th>Signed</th><th>Notes</th>"
                         "</tr>"));
    for (size_t i = 0; i < SOLIS_REGISTER_INFO_COUNT; i++) {
        const SolisRegisterInfo& info = SOLIS_REGISTER_INFOS[i];
        RegisterValue regValue = getSolisRegisterValue(snapshot, info.reg);

        String row;
        row.reserve(320);
        row = "<tr><td>" + htmlEscape(String(info.label)) +
              "</td><td class='mono'>" + String(info.reg) +
              "</td><td>" + htmlEscape(formatSolisRegisterDisplay(info.reg, regValue)) +
              "</td><td>" + (regValue.valid ? String(regValue.raw) : String("--")) +
              "</td><td>" + (regValue.valid ? String(static_cast<int16_t>(regValue.raw)) : String("--")) +
              "</td><td>" + htmlEscape(String(info.note)) + "</td></tr>";
        server.sendContent(row);
    }
    server.sendContent(F("</table>"));
    server.sendContent(F("<p>JSON endpoints: <a href='/api/inverter'>/api/inverter</a> · <a href='/api/solis'>/api/solis</a></p>"));
    server.sendContent(F("</body></html>"));
    server.sendContent("");
}

static void handleLogsApi() {
    unsigned long now = millis();
    unsigned long uptimeMs = now - bootStartMs;
    uint16_t lineCount = getLogLineCount();

    String json;
    json.reserve(1200 + lineCount * 96);
    json += "{";
    json += "\"uptimeMs\":";
    json += String(uptimeMs);
    json += ",\"lineCount\":";
    json += String(lineCount);
    json += ",\"resetReason\":\"";
    json += resetReasonToString(esp_reset_reason());
    json += "\"";
    json += ",\"lastRequestedReboot\":";
    if (lastPersistedRebootPresent) {
        json += "{";
        json += "\"reason\":\"";
        json += jsonEscape(String(lastPersistedRebootReason));
        json += "\",\"uptimeMs\":";
        json += String(lastPersistedRebootUptimeMs);
        json += ",\"recordedUptimeMs\":";
        json += String(lastPersistedRebootAtMs);
        json += "}";
    } else {
        json += "null";
    }
    json += ",\"lines\":[";
    for (uint16_t i = 0; i < lineCount; i++) {
        char line[LOG_LINE_MAX_CHARS];
        copyLogLineAt(i, line, sizeof(line));
        if (i != 0) json += ",";
        json += "\"";
        json += jsonEscape(String(line));
        json += "\"";
    }
    json += "]}";
    server.send(200, "application/json", json);
}

static void handleLogs() {
    unsigned long now = millis();
    unsigned long uptimeMs = now - bootStartMs;
    uint16_t lineCount = getLogLineCount();

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");
    server.sendContent(F("<!DOCTYPE html><html><head>"
                         "<meta charset='UTF-8'>"
                         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                         "<meta http-equiv='refresh' content='5'>"
                         "<title>Device logs</title>"
                         "<style>"
                         "body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}"
                         "h1{color:#e0c97f;margin-bottom:4px}a{color:#8ec5ff}"
                         ".card{background:#16213e;border-radius:8px;padding:12px;margin-bottom:16px}"
                         ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:10px;margin-bottom:16px}"
                         "pre{background:#0f172a;border-radius:8px;padding:12px;white-space:pre-wrap;word-break:break-word}"
                         ".mono{font-family:monospace}.muted{color:#a0b4cc}"
                         "</style></head><body>"));
    server.sendContent(F("<p><a href='/'>Back to summary</a></p>"));
    server.sendContent(F("<h1>Device logs</h1>"));
    server.sendContent(F("<p class='muted'>Recent in-memory log lines only; no live work is triggered by this page.</p>"));
    server.sendContent(F("<div class='grid'>"));
    sendCard("Captured lines", String(lineCount));
    sendCard("Reset reason", String(resetReasonToString(esp_reset_reason())));
    sendCard("Uptime", String(uptimeMs / 1000UL) + " s");
    if (lastPersistedRebootPresent) {
        sendCard("Previous requested reboot",
                 String(lastPersistedRebootReason) + " @ " + String(lastPersistedRebootUptimeMs / 1000UL) + " s");
    }
    server.sendContent(F("</div><pre class='mono'>"));
    for (uint16_t i = 0; i < lineCount; i++) {
        char line[LOG_LINE_MAX_CHARS];
        copyLogLineAt(i, line, sizeof(line));
        server.sendContent(htmlEscape(String(line)));
        server.sendContent("\n");
    }
    server.sendContent(F("</pre><p><a href='/api/logs'>/api/logs</a> · <a href='/api/status'>/api/status</a></p></body></html>"));
    server.sendContent("");
}

static void handleBatteryToggle() {
    if (!server.hasArg("index") || !server.hasArg("enabled")) {
        server.send(400, "text/plain", "Missing battery toggle arguments");
        return;
    }

    int index = -1;
    if (!tryParseBatteryIndex(server.arg("index"), index)) {
        server.send(400, "text/plain", "Invalid battery index");
        return;
    }

    String enabledArg = server.arg("enabled");
    enabledArg.toLowerCase();
    bool enabled = enabledArg == "1" || enabledArg == "true" || enabledArg == "on";
    if (!setBatteryEnabled(index, enabled)) {
        server.send(500, "text/plain", "Failed to persist battery config");
        return;
    }

    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "Battery config updated");
}

static void handleRoot() {
    unsigned long now = millis();
    unsigned long uptimeMs = now - bootStartMs;
    AggregateSnapshot snap = buildAggregateSnapshot(now);

    LogSerial.printf("[WEB] GET / heap=%u connected=%d/%d wifi=%s\n",
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
                         "button{background:#2d6cdf;color:#fff;border:none;border-radius:6px;padding:6px 10px;cursor:pointer}"
                         "button.warn{background:#8b322c}form{margin:0}"
                         "</style></head><body><h1>Battery BMS (Persistent BLE)</h1>"));

    server.sendContent(F("<div class='grid'>"));
    sendCard("Enabled Batteries", String(enabledBatteryCount()));
    sendCard("Connected Batteries", String(connectedBatteryCount()));
    sendCard("Periodic reboot", PERIODIC_REBOOT_INTERVAL_MS == 0 ? String("disabled")
                                                                : String(PERIODIC_REBOOT_INTERVAL_MS / 3600000UL) + " h");
    sendCard("Next reboot", PERIODIC_REBOOT_INTERVAL_MS == 0 ? String("n/a")
                                                            : (uptimeMs >= PERIODIC_REBOOT_INTERVAL_MS
                                                                   ? String("due")
                                                                   : String((PERIODIC_REBOOT_INTERVAL_MS - uptimeMs) / 60000UL) + " min"));

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
    if (lastPersistedRebootPresent) {
        sendCard("Prev reboot", String(lastPersistedRebootReason) + " @ " + String(lastPersistedRebootUptimeMs / 1000UL) + " s");
    }

    server.sendContent(F("</div>"));
    server.sendContent(F("<p>Configured known batteries are listed below. The enable/disable buttons are persisted in NVS and directly control which batteries are scanned, connected and included in the aggregate after reboot.</p>"));

    server.sendContent(F("<h2>Per-battery status</h2><table><tr>"
                         "<th>Name</th><th>MAC</th><th>Enabled</th><th>Connected</th><th>Contributing</th><th>Voltage (V)</th><th>Current (A)</th><th>SoC (%)</th><th>Temp (C)</th><th>Min Cell (V)</th><th>Min Cell ID</th><th>Max Cell (V)</th><th>Max Cell ID</th><th>Data age (s)</th><th>Drops</th><th>Aggregate membership</th>"
                         "</tr>"));

    for (int i = 0; i < BATTERY_COUNT; i++) {
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
        bool contributing = isBatteryContributing(i, now);
        String batteryLink = "<a href='/battery?index=" + String(i) + "'>" + htmlEscape(String(cfg.name)) + "</a>";
        String escapedIndex = htmlEscape(String(i));
        String escapedEnabledValue = htmlEscape(String(cfg.enabled ? 0 : 1));
        String action = "<form method='POST' action='/battery-toggle'>"
                       "<input type='hidden' name='index' value='" + escapedIndex + "'>"
                       "<input type='hidden' name='enabled' value='" + escapedEnabledValue + "'>"
                       "<button" + String(cfg.enabled ? " class='warn'" : "") + " type='submit'>" +
                       String(cfg.enabled ? "Disable" : "Enable") + "</button></form>";

        String row;
        row.reserve(640);
        row = "<tr><td>" + batteryLink + "</td><td class='mono'>" + htmlEscape(String(cfg.mac)) + "</td><td>" +
                     String(cfg.enabled ? "yes" : "no") + "</td><td class='" + (isBatteryConnected(i) ? "green'>yes" : "red'>no") +
                      "</td><td class='" + (contributing ? "green'>yes" : "amber'>no") +
                      "</td><td>" + (battery.hasData ? String(battery.voltage, 2) : String("-")) +
                      "</td><td>" + (battery.hasData ? String(battery.current, 2) : String("-")) +
                      "</td><td>" + (battery.hasData ? String(battery.soc) : String("-")) +
                      "</td><td>" + (battery.hasTemperature ? String(battery.temperature, 1) : String("-")) +
                      "</td><td>" + (hasCellStats ? String(minCellMv / 1000.0f, 3) : String("-")) +
                      "</td><td>" + (hasCellStats ? String(minCellIndex + 1) : String("-")) +
                      "</td><td>" + (hasCellStats ? String(maxCellMv / 1000.0f, 3) : String("-")) +
                      "</td><td>" + (hasCellStats ? String(maxCellIndex + 1) : String("-")) +
                      "</td><td>" + age +
                      "</td><td>" + String(battery.disconnectCount) +
                      "</td><td>" + action + "</td></tr>";

        server.sendContent(row);
    }

    server.sendContent(F("</table><p><a href='/logs'>Device logs</a> · <a href='/api/logs'>Log JSON</a> · <a href='/api/status'>Status JSON</a> · <a href='/inverter'>Solis inverter monitor</a> · <a href='/api/inverter'>Inverter JSON</a></p>"
                        "<p>Auto-refreshes every 5s. Aggregate values use fresh enabled battery data only.</p></body></html>"));
    server.sendContent("");
}

static void handleBatteryDetail() {
    if (!server.hasArg("index")) {
        server.send(400, "text/plain", "Missing battery index");
        return;
    }

    int index = -1;
    if (!tryParseBatteryIndex(server.arg("index"), index)) {
        server.send(400, "text/plain", "Invalid battery index");
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
    LogSerial.printf("Trying WiFi: %s\n", ssid);
    WiFi.begin(ssid, password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
        delay(250);
        LogSerial.print(".");
    }
    LogSerial.println();
    return WiFi.status() == WL_CONNECTED;
}

static void logBMSData() {
    unsigned long now = millis();
    AggregateSnapshot snap = buildAggregateSnapshot(now);

    LogSerial.println("\n========== BMS STATUS ==========");
    LogSerial.printf("Connected batteries: %d/%d\n", connectedBatteryCount(), enabledBatteryCount());

    if (snap.valid) {
        LogSerial.printf("Aggregate: %.2f V  %.2f A  SoC %u%%",
                      snap.voltage,
                      snap.current,
                      snap.soc);
        if (snap.hasTemperature) {
            LogSerial.printf("  Temp %.1f C", snap.temperature);
        }
        LogSerial.printf("  Fresh=%u\n", snap.contributingBatteries);
    } else {
        LogSerial.println("Aggregate: no fresh data");
    }

    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (!batteryConfigs[i].enabled) continue;

        BatteryState& battery = batteries[i];
        unsigned long ageSec = battery.lastGoodDataMs == 0
                             ? 0
                             : (now - battery.lastGoodDataMs) / 1000UL;

        LogSerial.printf("[%s] connected=%s seen=%s ok=%lu fail=%lu timeouts=%lu drops=%lu age=%lus reqInFlight=%s pkt=%d/%d",
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
            LogSerial.printf("  %.2f V %.2f A SoC %u%%",
                          battery.voltage,
                          battery.current,
                          battery.soc);
            if (battery.hasTemperature) LogSerial.printf(" %.1f C", battery.temperature);
        }
        LogSerial.println();
    }

    LogSerial.printf("CAN aggregate lock timeouts: %lu\n", static_cast<unsigned long>(canAggregateLockTimeouts));
    logSystemDebugSummary("[DBG summary]");
    LogSerial.println("=================================");
}

void setup() {
    LogSerial.begin(115200);
    bootStartMs = millis();
    delay(500);
    LogSerial.println();
    LogSerial.println("=== JBD BMS -> Solis CAN Bridge (persistent classic BLE multi-battery) ===");

    prefsReady = configPrefs.begin(CONFIG_NAMESPACE, false);
    if (prefsReady) {
        LogSerial.printf("Preferences namespace '%s' ready\n", CONFIG_NAMESPACE);
        loadBatteryEnabledConfig();
        loadPersistedRebootInfo();
    } else {
        LogSerial.printf("Failed to open Preferences namespace '%s'; using sketch defaults only\n", CONFIG_NAMESPACE);
    }

    LogSerial.printf("Reset reason: %s\n", resetReasonToString(esp_reset_reason()));
    if (lastPersistedRebootPresent) {
        LogSerial.printf("Previous requested reboot: %s after %lu ms uptime (recorded at %lu ms)\n",
                      lastPersistedRebootReason,
                      lastPersistedRebootUptimeMs,
                      lastPersistedRebootAtMs);
    }

    setupCAN();

    solisMutex = xSemaphoreCreateMutex();
    if (solisMutex == nullptr) {
        LogSerial.println("Failed to create Solis mutex; inverter polling disabled");
    } else {
        RS485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
        LogSerial.printf("Solis RS485 polling will run in loop() on RX=%d TX=%d\n", RS485_RX_PIN, RS485_TX_PIN);
    }

    canAggregateMutex = xSemaphoreCreateMutex();
    if (canAggregateMutex == nullptr) {
        LogSerial.println("Failed to create CAN aggregate mutex");
        LogSerial.println("CAN TX task disabled (missing aggregate mutex)");
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
            LogSerial.println("Failed to start CAN TX task");
        } else {
            LogSerial.printf("CAN TX task started on core %d\n", CAN_TASK_CORE);
        }
    }

    WiFi.mode(WIFI_STA);
    bool wifiOk = tryConnectWiFi(WIFI_SSID, WIFI_PASSWORD_UPPER);
    if (!wifiOk) {
        WiFi.disconnect(true);
        delay(500);
        wifiOk = tryConnectWiFi(WIFI_SSID, WIFI_PASSWORD_LOWER);
    }
    if (wifiOk) {
        LogSerial.printf("WiFi connected - http://%s\n", WiFi.localIP().toString().c_str());
        server.on("/", handleRoot);
        server.on("/battery", handleBatteryDetail);
        server.on("/battery-toggle", HTTP_POST, handleBatteryToggle);
        server.on("/logs", handleLogs);
        server.on("/inverter", handleInverter);
        server.on("/api/status", handleStatusApi);
        server.on("/api/logs", handleLogsApi);
        server.on("/api/inverter", handleInverterApi);
        server.on("/api/solis", handleInverterApi);
        server.begin();
        LogSerial.println("Web server started on port 80");
        wifiReady = true;
    } else {
        LogSerial.println("WiFi failed - continuing without web server");
    }

    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(&discoveryCallbacks);

    LogSerial.printf("Configured batteries: %d\n", BATTERY_COUNT);
    for (int i = 0; i < BATTERY_COUNT; i++) {
        LogSerial.printf("  [%d] %s %s enabled=%s\n",
                      i,
                      batteryConfigs[i].name,
                      batteryConfigs[i].mac,
                      batteryConfigs[i].enabled ? "true" : "false");
    }

    bool allSeen = scanForAllEnabledBatteries(STARTUP_SCAN_TIMEOUT_MS);
    LogSerial.printf("Startup discovery: seen=%d/%d\n", seenBatteryCount(), enabledBatteryCount());
    if (!allSeen) {
        LogSerial.println("Some enabled batteries were not discovered at startup.");
    }

    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (!batteryConfigs[i].enabled) continue;

        if (!batteries[i].seen) {
            LogSerial.printf("[%s] startup connect skipped (not discovered)\n", batteryConfigs[i].name);
            batteries[i].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
            continue;
        }

        bool ok = connectBattery(i);
        LogSerial.printf("[%s] startup connect %s\n", batteryConfigs[i].name, ok ? "OK" : "FAIL");
        if (!ok) batteries[i].nextReconnectMs = millis() + RECONNECT_INTERVAL_MS;
    }

    aggregate = buildAggregateSnapshot(millis());
    updateCanAggregateSnapshot(aggregate);
    logSystemDebugSummary("[DBG setup complete]");
}

static unsigned long lastSolisPoll = 0;
static unsigned long lastLog = 0;
static unsigned long lastHeartbeat = 0;

void loop() {
    unsigned long now = millis();

    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        LogSerial.printf("[loop] heartbeat now=%lu heap=%u wifi=%s connected=%d/%d\n",
                      now,
                      ESP.getFreeHeap(),
                      wifiStatusToString(WiFi.status()),
                      connectedBatteryCount(),
                      enabledBatteryCount());
    }

    if (wifiReady) server.handleClient();

    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (!batteryConfigs[i].enabled) continue;

        if (isBatteryConnected(i)) {
            serviceBatteryPolling(i, now);
        } else if (shouldAttemptReconnect(i, now)) {
            bool ok = reconnectBattery(i);
            LogSerial.printf("[%s] reconnect %s\n", batteryConfigs[i].name, ok ? "OK" : "FAIL");
        }

        if (wifiReady) server.handleClient();
    }

    if (solisMutex != nullptr && (now - lastSolisPoll >= SOLIS_POLL_INTERVAL_MS)) {
        pollSolisOnce(now);
        lastSolisPoll = millis();
    }

    aggregate = buildAggregateSnapshot(now);
    updateCanAggregateSnapshot(aggregate);

    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        logBMSData();
    }

    servicePeriodicReboot(now);

    delay(5);
}
