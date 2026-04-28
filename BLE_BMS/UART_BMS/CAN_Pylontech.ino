#include "driver/twai.h"
#include <bms2.h>

// -----------------------------------------------------------------------
// Pylontech CAN protocol for Solis S5-EH1P3.6K-L
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Pack series cell count — change to 13 for a 13S pack, leave at 14 for 14S.
// All pack-voltage parameters are derived from this single constant.
// -----------------------------------------------------------------------
#ifndef PACK_SERIES_CELLS
#define PACK_SERIES_CELLS       14      // default: 14S; set to 13 for 13S
#endif

// Pack max charge voltage in decivolts: PACK_SERIES_CELLS × 4.20 V × 10
// e.g. 14S → 588, 13S → 546
#define PACK_MAX_V10            ((uint16_t)((PACK_SERIES_CELLS) * 4.20f * 10.0f + 0.5f))

#define MAX_CHARGE_VOLTAGE      PACK_MAX_V10   // PACK_SERIES_CELLS x 4.20V NMC (x10)
#define MAX_CHARGE_CURRENT      500   // 50.0A  (x10)
#define MAX_DISCHARGE_CURRENT   500   // 50.0A  (x10)

static uint32_t aliveCounter = 0;

static void canSend(twai_message_t& msg) {
    if (twai_transmit(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
        Serial.printf("⚠️  CAN TX failed for ID 0x%03X\n", msg.identifier);
    }
}

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

static void can_send_limits() {
    twai_message_t msg;
    msg.identifier       = 0x351;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 6;
    msg.data[0] = (MAX_CHARGE_VOLTAGE    & 0xFF);
    msg.data[1] = (MAX_CHARGE_VOLTAGE    >> 8) & 0xFF;
    msg.data[2] = (MAX_CHARGE_CURRENT    & 0xFF);
    msg.data[3] = (MAX_CHARGE_CURRENT    >> 8) & 0xFF;
    msg.data[4] = (MAX_DISCHARGE_CURRENT & 0xFF);
    msg.data[5] = (MAX_DISCHARGE_CURRENT >> 8) & 0xFF;
    canSend(msg);
}

static void can_send_soc(OverkillSolarBms2& bms) {
    twai_message_t msg;
    msg.identifier       = 0x355;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 4;
    msg.data[0] = bms.get_state_of_charge();
    msg.data[1] = 0x00;
    msg.data[2] = 100;   // SoH — JBD doesn't report it
    msg.data[3] = 0x00;
    canSend(msg);
}

static void can_send_measurements(OverkillSolarBms2& bms) {
    int16_t voltage = (int16_t)(bms.get_voltage() * 10.0f);
    int16_t current = (int16_t)(bms.get_current() * 10.0f);
    int16_t temp    = (int16_t)(bms.get_ntc_temperature(0) * 10.0f);

    twai_message_t msg;
    msg.identifier       = 0x356;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 6;
    msg.data[0] =  voltage       & 0xFF;
    msg.data[1] = (voltage >> 8) & 0xFF;
    msg.data[2] =  current       & 0xFF;
    msg.data[3] = (current >> 8) & 0xFF;
    msg.data[4] =  temp          & 0xFF;
    msg.data[5] = (temp    >> 8) & 0xFF;
    canSend(msg);
}

static void can_send_alarms(OverkillSolarBms2& bms) {
    uint8_t numCells = bms.get_num_cells();
    float minV = 9999, maxV = 0;
    for (uint8_t c = 0; c < numCells; c++) {
        float v = bms.get_cell_voltage(c);
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
    }

    uint8_t protection = 0;
    if (maxV > 3.65f)                       bitSet(protection, 1);
    if (minV < 2.80f)                       bitSet(protection, 2);
    if (bms.get_ntc_temperature(0) > 55.0f) bitSet(protection, 3);
    if (bms.get_ntc_temperature(0) <  0.0f) bitSet(protection, 4);

    twai_message_t msg;
    msg.identifier       = 0x359;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 7;
    msg.data[0] = protection;
    msg.data[1] = 0x00;
    msg.data[2] = protection;
    msg.data[3] = 0x00;
    msg.data[4] = 0x01;
    msg.data[5] = 0x50;
    msg.data[6] = 0x4E;
    canSend(msg);
}

static void can_send_request(OverkillSolarBms2& bms) {
    uint8_t flags = 0;
    if (bms.get_charge_mosfet_status())    flags |= 0x80;
    if (bms.get_discharge_mosfet_status()) flags |= 0x40;

    twai_message_t msg;
    msg.identifier       = 0x35C;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 2;
    msg.data[0] = flags;
    msg.data[1] = 0x00;
    canSend(msg);
}

static void can_send_manufacturer() {
    twai_message_t msg;
    msg.identifier       = 0x35E;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 8;
    msg.data[0] = 'P'; msg.data[1] = 'Y';
    msg.data[2] = 'L'; msg.data[3] = 'O';
    msg.data[4] = 'N'; msg.data[5] = 'T';
    msg.data[6] = 'E'; msg.data[7] = 'C';
    canSend(msg);
}

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
    msg.data[4] = 0x00; msg.data[5] = 0x00;
    msg.data[6] = 0x00; msg.data[7] = 0x00;
    canSend(msg);
}

void sendCANFrames(OverkillSolarBms2& bms) {
    can_send_limits();
    can_send_soc(bms);
    can_send_measurements(bms);
    can_send_alarms(bms);
    can_send_request(bms);
    can_send_manufacturer();
    can_send_alive();
}
