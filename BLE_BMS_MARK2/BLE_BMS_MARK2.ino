#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <HardwareSerial.h>
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
#define RS485_TASK_STACK_SIZE      4096
#define RS485_TASK_PRIORITY           2
#define CAN_MUTEX_TIMEOUT_MS        100
#define RS485_RX_PIN                 16
#define RS485_TX_PIN                 17
#if CONFIG_FREERTOS_UNICORE
#define CAN_TASK_CORE                0
#define RS485_TASK_CORE              0
#else
#ifndef CONFIG_ARDUINO_RUNNING_CORE
#define CONFIG_ARDUINO_RUNNING_CORE 1
#endif
#define CAN_TASK_CORE  (1 - CONFIG_ARDUINO_RUNNING_CORE)
#define RS485_TASK_CORE CONFIG_ARDUINO_RUNNING_CORE
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
#define PER_BATTERY_CURRENT_LIMIT_AX10 400U
#define DERATE_CURRENT_CAP_AX10       100U
#define DISCHARGE_DERATE_HALF_MV     3250U
#define DISCHARGE_DERATE_CAP_MV      3200U
#define DISCHARGE_CUTOFF_MV          3150U
#define CHARGE_DERATE_HALF_MV        4050U
#define CHARGE_DERATE_CAP_MV         4100U
#define CHARGE_CUTOFF_MV             4150U
#define DERATE_HALF_DIVISOR             2U

// -----------------------------------------------------------------------
// RS485 Modbus slave configuration
//
// The ESP32 acts as Modbus slave ID RS485_SLAVE_ID on the RS485 bus.
// The Raspberry Pi master polls this slave using FC03 (read holding
// registers) to retrieve battery telemetry.
//
// Set RS485_DE_PIN to the GPIO used for DE/RE direction control (active-
// high), or -1 to disable if the RS485 transceiver handles direction
// automatically (e.g. auto-direction ICs or full-duplex wiring).
// -----------------------------------------------------------------------
#define RS485_SLAVE_ID               5
#define RS485_DE_PIN                -1
#define RS485_BAUD                9600
#define RS485_FC_READ_HOLDING      0x03
// Stale-frame timeout: if no new byte arrives within this many ms after
// the first byte, discard the partial frame and start fresh.
#define RS485_FRAME_STALE_MS        50

// -----------------------------------------------------------------------
// Modbus register map (holding registers, FC03)
//
// The Pi reads one contiguous block starting at address 0.
// All values are unsigned 16-bit; signed fields use two's complement
// (int16_t cast to uint16_t).
//
// Aggregate block  — addresses 0 … 7  (8 registers)
// -----------------------------------------------------------------------
//  Addr  Name                 Scale   Notes
//  ----  -------------------  ------  ---------------------------------
//   0    agg_valid            1       1 = aggregate is usable
//   1    agg_contributing     1       number of contributing batteries
//   2    agg_voltage          ×100    pack voltage in V (e.g. 5490 = 54.90 V)
//   3    agg_current          ×100    signed; positive = charging (A)
//   4    agg_soc              1       state of charge 0-100 %
//   5    agg_temperature      ×10     signed °C; 0 if temperature unavailable
//   6    agg_charge_allowed   1       1 if charge MOS OK across all contributing
//   7    agg_discharge_allowed 1      1 if discharge MOS OK across all contributing
//
// Per-battery block — 24 registers each
//   Battery 0: addresses  8 … 31
//   Battery 1: addresses 32 … 55
//   Battery 2: addresses 56 … 79
// -----------------------------------------------------------------------
//  +0   enabled              1       1 if this battery slot is configured
//  +1   has_data             1       1 if fresh BMS data is available
//  +2   voltage              ×100    pack voltage in V
//  +3   current              ×100    signed A; positive = charging
//  +4   soc                  1       0-100 %
//  +5   charge_mos           1       1 if charge MOS on
//  +6   discharge_mos        1       1 if discharge MOS on
//  +7   temperature          ×10     signed °C; 0 if unavailable
//  +8   cell_count           1       number of cells (0 if no cell data)
//  +9   cell_mv_1            1       cell 1 millivolts
//  +10  cell_mv_2            1       cell 2 millivolts
//   …   …                   …       …
//  +22  cell_mv_14           1       cell 14 millivolts
//  +23  reserved             0       always 0
// -----------------------------------------------------------------------
#define REG_AGG_VALID               0
#define REG_AGG_CONTRIBUTING        1
#define REG_AGG_VOLTAGE_V100        2
#define REG_AGG_CURRENT_A100        3
#define REG_AGG_SOC                 4
#define REG_AGG_TEMP_C10            5
#define REG_AGG_CHARGE_ALLOWED      6
#define REG_AGG_DISCHARGE_ALLOWED   7
#define REG_BATT_BASE               8

#define SLAVE_CELLS_PER_BATTERY    14
#define SLAVE_REGS_PER_BATTERY     24   // 9 header + 14 cells + 1 reserved

#define REG_BATT_ENABLED            0
#define REG_BATT_HAS_DATA           1
#define REG_BATT_VOLTAGE_V100       2
#define REG_BATT_CURRENT_A100       3
#define REG_BATT_SOC                4
#define REG_BATT_CHARGE_MOS         5
#define REG_BATT_DISCHARGE_MOS      6
#define REG_BATT_TEMP_C10           7
#define REG_BATT_CELL_COUNT         8
#define REG_BATT_CELL_MV_FIRST      9

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
    uint8_t chargeContributingBatteries = 0;
    uint8_t dischargeContributingBatteries = 0;
    float voltage = 0.0f;
    float current = 0.0f;
    uint8_t soc = 0;
    float temperature = 0.0f;
    bool hasTemperature = false;
    bool chargeAllowed = false;
    bool dischargeAllowed = false;
    uint16_t chargeCurrentLimitAx10 = 0;
    uint16_t dischargeCurrentLimitAx10 = 0;
    uint16_t minCellMv = 0;
    uint16_t maxCellMv = 0;
    bool hasFreshCellExtremes = false;
    unsigned long lastFreshMs = 0;
};

static BatteryState batteries[] = {
    {BMS1_NAME, BMS1_MAC, BMS1_ENABLED},
    {BMS2_NAME, BMS2_MAC, BMS2_ENABLED},
    {BMS3_NAME, BMS3_MAC, BMS3_ENABLED},
};

static const int BATTERY_COUNT = sizeof(batteries) / sizeof(batteries[0]);

// Total holding registers exposed over RS485
#define SLAVE_REG_COUNT  (REG_BATT_BASE + BATTERY_COUNT * SLAVE_REGS_PER_BATTERY)

static BLEScan* pBLEScan = nullptr;
static unsigned long cycleCount = 0;
static unsigned long lastSummaryMs = 0;
static AggregateSnapshot aggregate;
static AggregateSnapshot canAggregate;
static SemaphoreHandle_t canAggregateMutex = nullptr;
static HardwareSerial RS485(2);

// RS485 slave receive state
static uint8_t slaveRxBuf[8];
static int slaveRxLen = 0;
static unsigned long slaveRxFirstByteMs = 0;

static bool isAggregateUsable(const AggregateSnapshot& snap, unsigned long nowMs);
static AggregateSnapshot buildAggregateSnapshot(unsigned long now);
static void updateCanAggregateSnapshot(const AggregateSnapshot& snap);
static AggregateSnapshot copyCanAggregateSnapshot();
static void canTxTask(void* pv);
static void rs485Task(void* pv);
static void buildRegisterMap(uint16_t* regs, int count, const AggregateSnapshot& agg);
static void sendModbusResponse(uint16_t startReg, uint16_t count,
                               const uint16_t* regs, int regCount);
static void processRS485Slave(const AggregateSnapshot& agg);

void setupCAN();
void sendCANFrames(float voltage,
                   float current,
                   uint8_t soc,
                   float temperature,
                   uint16_t chargeCurrentLimitAx10,
                   uint16_t dischargeCurrentLimitAx10,
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

static bool validModbusCRC(const uint8_t* buf, int len) {
    if (len < 4) return false;
    uint16_t rxCRC = buf[len - 2] | (static_cast<uint16_t>(buf[len - 1]) << 8);
    return rxCRC == modbusCRC(buf, len - 2);
}

// -----------------------------------------------------------------------
// buildRegisterMap — populate the Modbus holding register array from the
// current aggregate snapshot and per-battery state.
// -----------------------------------------------------------------------
static void buildRegisterMap(uint16_t* regs, int count, const AggregateSnapshot& agg) {
    for (int i = 0; i < count; i++) regs[i] = 0;

    // Aggregate block
    regs[REG_AGG_VALID]             = agg.valid ? 1U : 0U;
    regs[REG_AGG_CONTRIBUTING]      = agg.contributingBatteries;
    regs[REG_AGG_VOLTAGE_V100]      = (uint16_t)(agg.voltage * 100.0f + 0.5f);
    regs[REG_AGG_CURRENT_A100]      = (uint16_t)(int16_t)(agg.current * 100.0f);
    regs[REG_AGG_SOC]               = agg.soc;
    regs[REG_AGG_TEMP_C10]          = agg.hasTemperature
                                      ? (uint16_t)(int16_t)(agg.temperature * 10.0f)
                                      : 0U;
    regs[REG_AGG_CHARGE_ALLOWED]    = agg.chargeAllowed ? 1U : 0U;
    regs[REG_AGG_DISCHARGE_ALLOWED] = agg.dischargeAllowed ? 1U : 0U;

    // Per-battery blocks
    for (int b = 0; b < BATTERY_COUNT; b++) {
        const BatteryState& bat = batteries[b];
        int base = REG_BATT_BASE + b * SLAVE_REGS_PER_BATTERY;
        if (base + SLAVE_REGS_PER_BATTERY > count) break;

        regs[base + REG_BATT_ENABLED]        = bat.enabled ? 1U : 0U;
        regs[base + REG_BATT_HAS_DATA]       = bat.hasData ? 1U : 0U;
        regs[base + REG_BATT_VOLTAGE_V100]   = bat.hasData
                                               ? (uint16_t)(bat.voltage * 100.0f + 0.5f)
                                               : 0U;
        regs[base + REG_BATT_CURRENT_A100]   = bat.hasData
                                               ? (uint16_t)(int16_t)(bat.current * 100.0f)
                                               : 0U;
        regs[base + REG_BATT_SOC]            = bat.hasData ? bat.soc : 0U;
        regs[base + REG_BATT_CHARGE_MOS]     = bat.chargeMos ? 1U : 0U;
        regs[base + REG_BATT_DISCHARGE_MOS]  = bat.dischargeMos ? 1U : 0U;
        regs[base + REG_BATT_TEMP_C10]       = (bat.hasData && bat.hasTemperature)
                                               ? (uint16_t)(int16_t)(bat.temperature * 10.0f)
                                               : 0U;
        regs[base + REG_BATT_CELL_COUNT]     = bat.hasCellData ? bat.cellCount : 0U;

        for (int c = 0; c < SLAVE_CELLS_PER_BATTERY; c++) {
            regs[base + REG_BATT_CELL_MV_FIRST + c] =
                (bat.hasCellData && c < bat.cellCount) ? bat.cellMv[c] : 0U;
        }
        // reserved slot (+23) is already zero from the initial memset above
    }
}

// -----------------------------------------------------------------------
// sendModbusResponse — build and transmit an FC03 response frame.
// Clamps the requested register window to [0, regCount).
// -----------------------------------------------------------------------
static void sendModbusResponse(uint16_t startReg, uint16_t count,
                               const uint16_t* regs, int regCount) {
    if (startReg >= (uint16_t)regCount) count = 0;
    if ((uint32_t)startReg + count > (uint32_t)regCount) {
        count = (uint16_t)(regCount - startReg);
    }

    // Frame: slave(1) + func(1) + byteCount(1) + data(count×2) + CRC(2)
    const int dataBytes = count * 2;
    const int frameLen  = 3 + dataBytes + 2;
    static uint8_t frame[3 + SLAVE_REG_COUNT * 2 + 2];

    frame[0] = RS485_SLAVE_ID;
    frame[1] = RS485_FC_READ_HOLDING;
    frame[2] = (uint8_t)dataBytes;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t val = regs[startReg + i];
        frame[3 + i * 2]     = (val >> 8) & 0xFF;
        frame[3 + i * 2 + 1] = val & 0xFF;
    }
    uint16_t crc = modbusCRC(frame, 3 + dataBytes);
    frame[3 + dataBytes]     = crc & 0xFF;
    frame[3 + dataBytes + 1] = crc >> 8;

    if (RS485_DE_PIN >= 0) digitalWrite(RS485_DE_PIN, HIGH);
    RS485.write(frame, frameLen);
    RS485.flush();
    if (RS485_DE_PIN >= 0) digitalWrite(RS485_DE_PIN, LOW);
    // Discard any TX echo that may appear on the RX line (half-duplex bus)
    flushRS485Input();
}

// -----------------------------------------------------------------------
// processRS485Slave — accumulate incoming bytes and respond to valid
// FC03 requests addressed to RS485_SLAVE_ID.
//
// Called by the dedicated RS485 task. All non-matching or malformed frames
// are silently discarded; the Pi master will retry on timeout.
// -----------------------------------------------------------------------
static void processRS485Slave(const AggregateSnapshot& agg) {
    unsigned long nowMs = millis();

    // Reset a partial frame that has gone stale (no new byte for too long)
    if (slaveRxLen > 0 && (nowMs - slaveRxFirstByteMs) > RS485_FRAME_STALE_MS) {
        slaveRxLen = 0;
    }

    // Capture first-byte timestamp before draining so the stale-frame timer
    // always reflects when the frame started, not when a mid-frame byte arrived.
    if (slaveRxLen == 0 && RS485.available()) slaveRxFirstByteMs = nowMs;

    // Drain available bytes into the receive buffer (FC03 request = 8 bytes)
    while (RS485.available() && slaveRxLen < (int)sizeof(slaveRxBuf)) {
        slaveRxBuf[slaveRxLen++] = (uint8_t)RS485.read();
    }

    // Need all 8 bytes of a standard Modbus RTU request before processing
    if (slaveRxLen < 8) return;

    // Filter: only respond to requests addressed to this slave using FC03
    if (slaveRxBuf[0] != RS485_SLAVE_ID || slaveRxBuf[1] != RS485_FC_READ_HOLDING) {
        slaveRxLen = 0;
        return;
    }

    // Validate CRC
    if (!validModbusCRC(slaveRxBuf, 8)) {
        Serial.println("[RS485] bad CRC — frame discarded");
        slaveRxLen = 0;
        return;
    }

    uint16_t startReg = ((uint16_t)slaveRxBuf[2] << 8) | slaveRxBuf[3];
    uint16_t count    = ((uint16_t)slaveRxBuf[4] << 8) | slaveRxBuf[5];

    Serial.printf("[RS485] FC03 req start=%u count=%u\n", startReg, count);

    // Build and send response
    static uint16_t regs[SLAVE_REG_COUNT];
    buildRegisterMap(regs, SLAVE_REG_COUNT, agg);
    sendModbusResponse(startReg, count, regs, SLAVE_REG_COUNT);

    slaveRxLen = 0;
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
    uint8_t maxSoc = 0;
    bool hasChargeCellData = false;
    bool hasDischargeCellData = false;
    uint16_t maxCellMvForCharge = 0;
    uint16_t minCellMvForCharge = UINT16_MAX;
    uint16_t maxCellMvForDischarge = 0;
    uint16_t minCellMvForDischarge = UINT16_MAX;

    for (int i = 0; i < BATTERY_COUNT; i++) {
        if (!batteries[i].enabled) continue;

        const BatteryState& battery = batteries[i];
        if (!battery.hasData || battery.lastGoodDataMs == 0) continue;
        if (now - battery.lastGoodDataMs > DATA_FRESH_MS) continue;

        sumVoltage += battery.voltage;
        sumCurrent += battery.current;

        if (battery.soc > maxSoc) maxSoc = battery.soc;

        if (battery.chargeMos) snap.chargeContributingBatteries++;
        if (battery.dischargeMos) snap.dischargeContributingBatteries++;

        bool freshCellData = battery.hasCellData &&
                             battery.cellCount > 0 &&
                             battery.lastCellDataMs != 0 &&
                             (now - battery.lastCellDataMs <= DATA_FRESH_MS);
        if (freshCellData) {
            uint16_t localMin = UINT16_MAX;
            uint16_t localMax = 0;
            for (uint8_t c = 0; c < battery.cellCount && c < MAX_CELLS; c++) {
                uint16_t mv = battery.cellMv[c];
                if (mv < localMin) localMin = mv;
                if (mv > localMax) localMax = mv;
            }

            if (battery.chargeMos) {
                if (localMin < minCellMvForCharge) minCellMvForCharge = localMin;
                if (localMax > maxCellMvForCharge) maxCellMvForCharge = localMax;
                hasChargeCellData = true;
            }
            if (battery.dischargeMos) {
                if (localMin < minCellMvForDischarge) minCellMvForDischarge = localMin;
                if (localMax > maxCellMvForDischarge) maxCellMvForDischarge = localMax;
                hasDischargeCellData = true;
            }
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
        snap.soc = maxSoc > 100 ? 100 : maxSoc;
        if (tempCount > 0) {
            snap.hasTemperature = true;
            snap.temperature = sumTemp / (float)tempCount;
        }
    }

    uint32_t chargeBaseAx10 =
        (uint32_t)snap.chargeContributingBatteries * PER_BATTERY_CURRENT_LIMIT_AX10;
    uint32_t dischargeBaseAx10 =
        (uint32_t)snap.dischargeContributingBatteries * PER_BATTERY_CURRENT_LIMIT_AX10;
    snap.chargeCurrentLimitAx10 =
        chargeBaseAx10 > UINT16_MAX ? UINT16_MAX : (uint16_t)chargeBaseAx10;
    snap.dischargeCurrentLimitAx10 =
        dischargeBaseAx10 > UINT16_MAX ? UINT16_MAX : (uint16_t)dischargeBaseAx10;

    if (snap.dischargeCurrentLimitAx10 > 0) {
        if (hasDischargeCellData) {
            if (minCellMvForDischarge < DISCHARGE_CUTOFF_MV) {
                snap.dischargeCurrentLimitAx10 = 0;
            } else if (minCellMvForDischarge < DISCHARGE_DERATE_CAP_MV) {
                if (snap.dischargeCurrentLimitAx10 > DERATE_CURRENT_CAP_AX10) {
                    snap.dischargeCurrentLimitAx10 = DERATE_CURRENT_CAP_AX10;
                }
            } else if (minCellMvForDischarge < DISCHARGE_DERATE_HALF_MV) {
                snap.dischargeCurrentLimitAx10 /= DERATE_HALF_DIVISOR;
            }
        } else if (snap.dischargeCurrentLimitAx10 > PER_BATTERY_CURRENT_LIMIT_AX10) {
            // If cell telemetry is temporarily stale, keep operation predictable
            // but avoid advertising multi-pack current without cell-level safety.
            // Single-pack operation remains unchanged at the same 40A limit.
            snap.dischargeCurrentLimitAx10 = PER_BATTERY_CURRENT_LIMIT_AX10;
        }
    }

    if (snap.chargeCurrentLimitAx10 > 0) {
        if (hasChargeCellData) {
            if (maxCellMvForCharge >= CHARGE_CUTOFF_MV) {
                snap.chargeCurrentLimitAx10 = 0;
            } else if (maxCellMvForCharge >= CHARGE_DERATE_CAP_MV) {
                if (snap.chargeCurrentLimitAx10 > DERATE_CURRENT_CAP_AX10) {
                    snap.chargeCurrentLimitAx10 = DERATE_CURRENT_CAP_AX10;
                }
            } else if (maxCellMvForCharge >= CHARGE_DERATE_HALF_MV) {
                snap.chargeCurrentLimitAx10 /= DERATE_HALF_DIVISOR;
            }
        } else if (snap.chargeCurrentLimitAx10 > PER_BATTERY_CURRENT_LIMIT_AX10) {
            // If cell telemetry is temporarily stale, keep operation predictable
            // but avoid advertising multi-pack current without cell-level safety.
            // Single-pack operation remains unchanged at the same 40A limit.
            snap.chargeCurrentLimitAx10 = PER_BATTERY_CURRENT_LIMIT_AX10;
        }
    }

    snap.chargeAllowed = snap.chargeCurrentLimitAx10 > 0;
    snap.dischargeAllowed = snap.dischargeCurrentLimitAx10 > 0;

    if (hasChargeCellData || hasDischargeCellData) {
        snap.hasFreshCellExtremes = true;
        if (hasChargeCellData && hasDischargeCellData) {
            snap.minCellMv = min(minCellMvForCharge, minCellMvForDischarge);
            snap.maxCellMv = max(maxCellMvForCharge, maxCellMvForDischarge);
        } else if (hasChargeCellData) {
            snap.minCellMv = minCellMvForCharge;
            snap.maxCellMv = maxCellMvForCharge;
        } else {
            snap.minCellMv = minCellMvForDischarge;
            snap.maxCellMv = maxCellMvForDischarge;
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
                          snap.chargeCurrentLimitAx10,
                          snap.dischargeCurrentLimitAx10,
                          snap.chargeAllowed,
                          snap.dischargeAllowed);
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CAN_INTERVAL_MS));
    }
}

static void rs485Task(void* pv) {
    (void)pv;

    for (;;) {
        AggregateSnapshot snap = copyCanAggregateSnapshot();
        if (!snap.valid && canAggregateMutex == nullptr) {
            snap = aggregate;
        }
        processRS485Slave(snap);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void printSummary() {
    unsigned long now = millis();
    int connectedCount = 0;

    Serial.printf("\n=== BLE BMS summary @ %lus ===\n", now / 1000UL);
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

    Serial.printf("Connected %d/%d enabled batteries\n",
                  connectedCount,
                  enabledBatteryCount());
    AggregateSnapshot snap = copyCanAggregateSnapshot();
    if (!snap.valid && canAggregateMutex == nullptr) {
        snap = aggregate;
    }
    if (snap.hasFreshCellExtremes) {
        Serial.printf("[AGG] total=%u chg=%u dsg=%u lim=%.1fA/%.1fA cell=%u/%u mV req=%s/%s\n",
                      snap.contributingBatteries,
                      snap.chargeContributingBatteries,
                      snap.dischargeContributingBatteries,
                      snap.chargeCurrentLimitAx10 / 10.0f,
                      snap.dischargeCurrentLimitAx10 / 10.0f,
                      snap.minCellMv,
                      snap.maxCellMv,
                      snap.chargeAllowed ? "on" : "off",
                      snap.dischargeAllowed ? "on" : "off");
    } else {
        Serial.printf("[AGG] total=%u chg=%u dsg=%u lim=%.1fA/%.1fA cell=n/a req=%s/%s\n",
                      snap.contributingBatteries,
                      snap.chargeContributingBatteries,
                      snap.dischargeContributingBatteries,
                      snap.chargeCurrentLimitAx10 / 10.0f,
                      snap.dischargeCurrentLimitAx10 / 10.0f,
                      snap.chargeAllowed ? "on" : "off",
                      snap.dischargeAllowed ? "on" : "off");
    }
    Serial.printf("[RS485] slave id=%d regs=%d\n", RS485_SLAVE_ID, SLAVE_REG_COUNT);
    Serial.println("========================================");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== BLE_BMS_MARK2 (BLE battery bridge + RS485 Modbus slave) ===");

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

    // Initialise RS485 as a Modbus slave (receive by default)
    if (RS485_DE_PIN >= 0) {
        pinMode(RS485_DE_PIN, OUTPUT);
        digitalWrite(RS485_DE_PIN, LOW);  // receive mode
    }
    RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    flushRS485Input();
    Serial.printf("RS485 Modbus slave id=%d  baud=%d  RX=%d TX=%d  regs=%d\n",
                  RS485_SLAVE_ID, RS485_BAUD, RS485_RX_PIN, RS485_TX_PIN,
                  SLAVE_REG_COUNT);
    BaseType_t rs485TaskOk = xTaskCreatePinnedToCore(
        rs485Task,
        "RS485Slave",
        RS485_TASK_STACK_SIZE,
        nullptr,
        RS485_TASK_PRIORITY,
        nullptr,
        RS485_TASK_CORE);
    if (rs485TaskOk != pdPASS) {
        Serial.println("Failed to start RS485 task");
    } else {
        Serial.printf("RS485 task started on core %d\n", RS485_TASK_CORE);
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
    Serial.printf("\n--- Main cycle %lu ---\n", cycleCount);

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
    }

    if (millis() - lastSummaryMs >= LOG_INTERVAL_MS) {
        lastSummaryMs = millis();
        printSummary();
    }

    aggregate = buildAggregateSnapshot(millis());
    updateCanAggregateSnapshot(aggregate);

    delay(BETWEEN_CYCLES_MS);
}
