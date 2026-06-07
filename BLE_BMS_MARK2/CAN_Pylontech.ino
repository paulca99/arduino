#include "driver/twai.h"

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

// Battery limits — Boston Power Swing 5300 NMC, 13S 18P (by default)
// Cell max: 4.20V, Cell min: 2.75V, Pack: 54.6V / 35.75V, Capacity: ~95Ah
#define MAX_CHARGE_VOLTAGE      PACK_MAX_V10   // PACK_SERIES_CELLS x 4.20V NMC (x10)
static uint32_t aliveCounter = 0;

// -----------------------------------------------------------------------
// Helper — transmit one CAN frame, print warning if it fails
// -----------------------------------------------------------------------
static void canSend(twai_message_t& msg) {
    if (twai_transmit(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
        Serial.printf("⚠️  CAN TX failed for ID 0x%03X\n", msg.identifier);
    }
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
    canSend(msg);
}

// -----------------------------------------------------------------------
// 0x355 — SoC and SoH
//
// Reports the highest SoC value received from any contributing BMS.
// SoH is assumed 100% as the JBD BMS does not report it.
// -----------------------------------------------------------------------
static void can_send_soc(uint8_t measuredSoc) {
    uint8_t soc = measuredSoc > 100 ? 100 : measuredSoc;
    uint8_t soh = 100;

    twai_message_t msg;
    msg.identifier       = 0x355;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 4;
    msg.data[0] = soc;
    msg.data[1] = 0x00;
    msg.data[2] = soh;
    msg.data[3] = 0x00;
    canSend(msg);
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
    canSend(msg);
}

// -----------------------------------------------------------------------
// 0x359 — Protection & alarm flags
// -----------------------------------------------------------------------
static void can_send_alarms(float packVoltage, float packTemp) {
    const float maxPackV = (float)PACK_SERIES_CELLS * 4.20f;
    const float minPackV = (float)PACK_SERIES_CELLS * 2.75f;
    uint8_t protection = 0;
    if (packVoltage > maxPackV) bitSet(protection, 1);   // pack overvoltage
    if (packVoltage < minPackV) bitSet(protection, 2);   // pack undervoltage
    if (packTemp > 55.0f) bitSet(protection, 3);         // high temp
    if (packTemp < 0.0f) bitSet(protection, 4);          // low temp

    twai_message_t msg;
    msg.identifier       = 0x359;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 7;
    msg.data[0] = protection;  // protection flags
    msg.data[1] = 0x00;
    msg.data[2] = protection;  // alarm flags (mirror)
    msg.data[3] = 0x00;
    msg.data[4] = 0x01;
    msg.data[5] = 0x50;        // 'P'
    msg.data[6] = 0x4E;        // 'N'
    canSend(msg);
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
    canSend(msg);
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
    canSend(msg);
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
    canSend(msg);
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
    can_send_soc(soc);
    can_send_measurements(voltage, current, temperature);
    can_send_alarms(voltage, temperature);
    can_send_request(chargeAllowed, dischargeAllowed);
    can_send_manufacturer();
    can_send_alive();
}
