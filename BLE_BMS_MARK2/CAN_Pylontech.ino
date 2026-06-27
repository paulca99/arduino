#include "driver/twai.h"
#include <math.h>

// -----------------------------------------------------------------------
// Pylontech CAN protocol for Solis S5-EH1P3.6K-L
//
// Frame IDs:
//   0x351 — Charge voltage, charge current limit, discharge current limit
//   0x355 — SoC, SoH
//   0x356 — Battery voltage, current, temperature
//   0x359 — Protection & alarm flags
//   0x35C — Charge/discharge request flags
//   0x35E — Manufacturer name ("PYLONTEC")
//   0x305 — Network alive / keep-alive counter
//
// Sent at the cadence configured by the caller task (BLE_BMS_MARK2 sets 500 ms).
// Solis expects 500 kbps.
// Wiring: SN65HVD230 TX→CAN_TX_PIN, RX→CAN_RX_PIN
//         Solis RJ45 pin4=CAN-H, pin5=CAN-L
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Pack series cell count — change to 14 for a 14S pack, leave at 13 for 13S.
// All pack-voltage parameters are derived from this single constant.
// -----------------------------------------------------------------------
#ifndef PACK_SERIES_CELLS
#define PACK_SERIES_CELLS       14      // default: 14S (set to 13 for 13S packs)
#endif

// Pack max charge voltage in decivolts: PACK_SERIES_CELLS × 4.20 V × 10
// e.g. 13S → 546, 14S → 588
#define PACK_MAX_V10            ((uint16_t)((PACK_SERIES_CELLS) * 4.20f * 10.0f + 0.5f))

// SoC curve reference: authored for 14S pack voltages. Scale voltages by
// PACK_SERIES_CELLS / 14 so 13S packs work automatically.
#define SOC_REF_SERIES          14.0f
#define SOC_SCALE               ((float)(PACK_SERIES_CELLS) / SOC_REF_SERIES)

// Battery limits — Boston Power Swing 5300 NMC, 13S 18P (by default)
// Cell max: 4.20V, Cell min: 2.75V, Pack: 54.6V / 35.75V, Capacity: ~95Ah
#define MAX_CHARGE_VOLTAGE      PACK_MAX_V10   // PACK_SERIES_CELLS x 4.20V NMC (x10)
static uint32_t aliveCounter = 0;

// -----------------------------------------------------------------------
// SOC_EMA_TAU_S — EMA time constant for the voltage-based SoC filter.
//
// Using instantaneous pack voltage to derive SoC causes the reported SoC
// to rebound immediately when the inverter stops drawing current (voltage
// recovers). That can make the inverter think the battery is usable again,
// re-enable discharge, sag voltage, and oscillate near low SoC thresholds.
//
// We smooth the voltage with an exponential moving average before converting
// it to SoC. Set SOC_EMA_TAU_S to 0 to disable smoothing.
// -----------------------------------------------------------------------
#define SOC_EMA_TAU_S   120.0f   // EMA time constant ≈ 2-minute smoothing window

// -----------------------------------------------------------------------
// voltageToSoc — NMC discharge curve lookup + linear interpolation
// Input: pack voltage (V). Output: SoC 0–100%.
// Curve is authored for a 14S reference pack; voltages are scaled by
// SOC_SCALE (= PACK_SERIES_CELLS / 14) so 13S packs work automatically.
// Based on Boston Power Swing 5300 NMC cell characterisation.
// -----------------------------------------------------------------------
static uint8_t voltageToSoc(float packV) {
    // {14S reference pack voltage, SoC%} — must be in descending voltage order
    static const float curve[][2] = {
        {58.8f, 100},
        {56.0f,  80},
        {53.9f,  70},
        {52.5f,  60},
        {51.1f,  50},
        {49.7f,  40},
        {48.3f,  30},
        {46.2f,  20},
        {44.1f,  10},
        {38.5f,   0},
    };
    const uint8_t points = sizeof(curve) / sizeof(curve[0]);

    if (packV >= curve[0][0] * SOC_SCALE)          return 100;
    if (packV <= curve[points - 1][0] * SOC_SCALE) return 0;

    for (uint8_t i = 0; i < points - 1; i++) {
        float vHigh = curve[i][0]     * SOC_SCALE;
        float socHigh = curve[i][1];
        float vLow = curve[i + 1][0]  * SOC_SCALE;
        float socLow = curve[i + 1][1];
        if (packV <= vHigh && packV > vLow) {
            float t = (packV - vLow) / (vHigh - vLow);
            return (uint8_t)(socLow + t * (socHigh - socLow) + 0.5f);
        }
    }
    return 0;
}

// -----------------------------------------------------------------------
// Helper — transmit one CAN frame, print warning if it fails
// -----------------------------------------------------------------------
static bool canSend(twai_message_t& msg) {
    if (twai_transmit(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
        Serial.printf("⚠️  CAN TX failed for ID 0x%03X\n", msg.identifier);
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------
// Setup TWAI (CAN) driver
// -----------------------------------------------------------------------
void setupCAN() {
    twai_general_config_t general_config = {
        .mode           = TWAI_MODE_NORMAL,
        .tx_io          = CAN_TX_PIN,
        .rx_io          = CAN_RX_PIN,
        .clkout_io      = TWAI_IO_UNUSED,
        .bus_off_io     = TWAI_IO_UNUSED,
        .tx_queue_len   = 20,
        .rx_queue_len   = 10,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0
    };
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK) {
        Serial.println("❌ CAN driver install failed");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("❌ CAN driver start failed");
        return;
    }
    Serial.println("✅ CAN driver started (500 kbps)");
}

// -----------------------------------------------------------------------
// 0x351 — Charge voltage & current limits
// -----------------------------------------------------------------------
static void can_send_limits(uint16_t chargeCurrentLimitAx10,
                            uint16_t dischargeCurrentLimitAx10) {
    twai_message_t msg;
    msg.identifier      = 0x351;
    msg.flags           = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 6;
    msg.data[0] = (MAX_CHARGE_VOLTAGE    & 0xFF);
    msg.data[1] = (MAX_CHARGE_VOLTAGE    >> 8) & 0xFF;
    msg.data[2] = (chargeCurrentLimitAx10    & 0xFF);
    msg.data[3] = (chargeCurrentLimitAx10    >> 8) & 0xFF;
    msg.data[4] = (dischargeCurrentLimitAx10 & 0xFF);
    msg.data[5] = (dischargeCurrentLimitAx10 >> 8) & 0xFF;
    if (canSend(msg)) {
        Serial.printf("[CAN TX 0x351] maxChargeV=%.1fV chargeLimit=%.1fA dischargeLimit=%.1fA\n",
                      MAX_CHARGE_VOLTAGE / 10.0f,
                      chargeCurrentLimitAx10 / 10.0f,
                      dischargeCurrentLimitAx10 / 10.0f);
    }
}

// -----------------------------------------------------------------------
// 0x355 — SoC and SoH
//
// Reports SoC to the inverter from pack voltage rather than the BMS-reported
// coulomb-counter SoC, because the BMS SoC can drift over time. The incoming
// measuredSoc is deliberately unused here; RS485 telemetry can still expose
// raw BMS SoC separately.
// -----------------------------------------------------------------------
static void can_send_soc(float packVoltage, uint8_t measuredSoc) {
    (void)measuredSoc;

    static float filteredV = -1.0f;
    static unsigned long lastEmaMs = 0;

    float rawV = packVoltage;
    unsigned long nowMs = millis();

    if (rawV > 0.0f) {
        if (filteredV < 0.0f) {
            filteredV = rawV;
        } else if (SOC_EMA_TAU_S > 0.0f && lastEmaMs != 0) {
            float dtSec = (nowMs - lastEmaMs) * 1.0e-3f;
            float alpha = 1.0f - expf(-dtSec / SOC_EMA_TAU_S);
            filteredV += alpha * (rawV - filteredV);
        } else {
            filteredV = rawV;
        }
        lastEmaMs = nowMs;
    }

    uint8_t soc = voltageToSoc(filteredV);
    uint8_t soh = 100;

    twai_message_t msg;
    msg.identifier       = 0x355;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 4;
    msg.data[0] = soc;
    msg.data[1] = 0x00;
    msg.data[2] = soh;
    msg.data[3] = 0x00;
    if (canSend(msg)) {
        Serial.printf("[CAN TX 0x355] SoC=%u%% SoH=%u%% voltage=%.2fV filtered=%.2fV\n",
                      soc, soh, rawV, filteredV);
    }
}

// -----------------------------------------------------------------------
// 0x356 — Voltage, current, temperature
// -----------------------------------------------------------------------
static void can_send_measurements(float packVoltage, float packCurrent, float packTemp) {
    int16_t voltage = (int16_t)(packVoltage * 100.0f);  // e.g. 54.9V → 5490
    int16_t current = (int16_t)(packCurrent * 10.0f);  // e.g. 12.3A → 123 (signed)
    int16_t temp    = (int16_t)(packTemp * 10.0f);

    twai_message_t msg;
    msg.identifier       = 0x356;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 6;
    msg.data[0] =  voltage        & 0xFF;
    msg.data[1] = (voltage >> 8)  & 0xFF;
    msg.data[2] =  current        & 0xFF;
    msg.data[3] = (current >> 8)  & 0xFF;
    msg.data[4] =  temp           & 0xFF;
    msg.data[5] = (temp    >> 8)  & 0xFF;
    if (canSend(msg)) {
        Serial.printf("[CAN TX 0x356] voltage=%.2fV current=%.1fA temp=%.1fC\n",
                      packVoltage, packCurrent, packTemp);
    }
}

// -----------------------------------------------------------------------
// 0x359 — Protection & alarm flags
// -----------------------------------------------------------------------
static void can_send_alarms(float packVoltage, float packTemp) {
    const float maxPackV = (float)PACK_SERIES_CELLS * 4.20f;
    const float minPackV = (float)PACK_SERIES_CELLS * 2.75f;
    uint8_t protection = 0;
    uint8_t alarms = 0;
    if (packVoltage > maxPackV) bitSet(protection, 1);   // pack overvoltage
    if (packVoltage < minPackV) bitSet(protection, 2);   // pack undervoltage
    if (packTemp > 55.0f) bitSet(protection, 3);         // high temp
    if (packTemp < 0.0f) bitSet(protection, 4);          // low temp
    alarms = protection;

    twai_message_t msg;
    msg.identifier       = 0x359;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 7;
    msg.data[0] = protection;  // protection flags
    msg.data[1] = 0x00;
    msg.data[2] = alarms;      // alarm flags (mirror)
    msg.data[3] = 0x00;
    msg.data[4] = 0x01;
    msg.data[5] = 0x50;        // 'P'
    msg.data[6] = 0x4E;        // 'N'
    if (canSend(msg)) {
        Serial.printf("[CAN TX 0x359] protection=0x%02X alarms=0x%02X\n", protection, alarms);
    }
}

// -----------------------------------------------------------------------
// 0x35C — Charge/discharge request flags
// -----------------------------------------------------------------------
static void can_send_request(bool chargeAllowed, bool dischargeAllowed) {
    uint8_t flags = 0;
    if (chargeAllowed) flags |= 0x80;
    if (dischargeAllowed) flags |= 0x40;

    twai_message_t msg;
    msg.identifier       = 0x35C;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 2;
    msg.data[0] = flags;
    msg.data[1] = 0x00;
    if (canSend(msg)) {
        Serial.printf("[CAN TX 0x35C] chargeAllowed=%s dischargeAllowed=%s\n",
                      chargeAllowed ? "yes" : "no",
                      dischargeAllowed ? "yes" : "no");
    }
}

// -----------------------------------------------------------------------
// 0x35E — Manufacturer name "PYLONTEC"
// -----------------------------------------------------------------------
static void can_send_manufacturer() {
    twai_message_t msg;
    msg.identifier       = 0x35E;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 8;
    msg.data[0] = 'P';
    msg.data[1] = 'Y';
    msg.data[2] = 'L';
    msg.data[3] = 'O';
    msg.data[4] = 'N';
    msg.data[5] = 'T';
    msg.data[6] = 'E';
    msg.data[7] = 'C';
    if (canSend(msg)) {
        Serial.println("[CAN TX 0x35E] manufacturer=PYLONTEC");
    }
}

// -----------------------------------------------------------------------
// 0x305 — Network alive / keep-alive counter
// -----------------------------------------------------------------------
static void can_send_alive() {
    aliveCounter++;
    twai_message_t msg;
    msg.identifier       = 0x305;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 8;
    msg.data[0] =  aliveCounter        & 0xFF;
    msg.data[1] = (aliveCounter >>  8) & 0xFF;
    msg.data[2] = (aliveCounter >> 16) & 0xFF;
    msg.data[3] = (aliveCounter >> 24) & 0xFF;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    if (canSend(msg)) {
        Serial.printf("[CAN TX 0x305] aliveCounter=%lu\n", (unsigned long)aliveCounter);
    }
}

// -----------------------------------------------------------------------
// Send all Pylontech frames — call every 100ms from CAN scheduler
// -----------------------------------------------------------------------
void sendCANFrames(float voltage,
                   float current,
                   uint8_t soc,
                   float temperature,
                   bool chargeAllowed,
                   bool dischargeAllowed,
                   uint16_t chargeCurrentLimitAx10,
                   uint16_t dischargeCurrentLimitAx10) {
    can_send_limits(chargeCurrentLimitAx10, dischargeCurrentLimitAx10);
    can_send_soc(voltage, soc);
    can_send_measurements(voltage, current, temperature);
    can_send_alarms(voltage, temperature);
    can_send_request(chargeAllowed, dischargeAllowed);
    can_send_manufacturer();
    can_send_alive();
}
