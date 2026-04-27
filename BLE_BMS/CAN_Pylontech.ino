#include "driver/twai.h"
#include <bms2.h>

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
// All sent every 100ms. Solis expects 500 kbps.
// Wiring: SN65HVD230 TX→GPIO11, RX→GPIO12
//         Solis RJ45 pin4=CAN-H, pin5=CAN-L
// -----------------------------------------------------------------------

// Battery limits — Boston Power Swing 5300 NMC, 14S 18P
// Cell max: 4.20V, Cell min: 2.75V, Pack: 58.8V / 38.5V, Capacity: ~95Ah
#define MAX_CHARGE_VOLTAGE      588   // 58.8V — 14 x 4.20V NMC (x10)
#define MAX_CHARGE_CURRENT      400   // 40.0A  (x10)
#define MAX_DISCHARGE_CURRENT   400   // 40.0A  (x10)

static uint32_t aliveCounter = 0;

// -----------------------------------------------------------------------
// voltageToSoc — NMC 14S discharge curve lookup + linear interpolation
// Input: pack voltage (V). Output: SoC 0–100%.
// Based on Boston Power Swing 5300 NMC cell characterisation.
// -----------------------------------------------------------------------
static uint8_t voltageToSoc(float packV) {
    // {pack voltage, SoC%} — must be in descending voltage order
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

    if (packV >= curve[0][0])             return 100;
    if (packV <= curve[points - 1][0])    return 0;

    for (uint8_t i = 0; i < points - 1; i++) {
        float vHigh = curve[i][0],   socHigh = curve[i][1];
        float vLow  = curve[i+1][0], socLow  = curve[i+1][1];
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
static void can_send_limits() {
    twai_message_t msg;
    msg.identifier      = 0x351;
    msg.flags           = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 6;
    msg.data[0] = (MAX_CHARGE_VOLTAGE    & 0xFF);
    msg.data[1] = (MAX_CHARGE_VOLTAGE    >> 8) & 0xFF;
    msg.data[2] = (MAX_CHARGE_CURRENT    & 0xFF);
    msg.data[3] = (MAX_CHARGE_CURRENT    >> 8) & 0xFF;
    msg.data[4] = (MAX_DISCHARGE_CURRENT & 0xFF);
    msg.data[5] = (MAX_DISCHARGE_CURRENT >> 8) & 0xFF;
    canSend(msg);
}

// -----------------------------------------------------------------------
// 0x355 — SoC and SoH
// -----------------------------------------------------------------------
static void can_send_soc(OverkillSolarBms2& bms) {
    // Use voltage-based SoC — immune to BMS coulomb counter resets on alarm.
    uint8_t soc = voltageToSoc(bms.get_voltage());

    // TEMPORARY: shift SoC -5% below 45% so Solis sees 40% when pack is
    // at 45% real SoC. This prevents Solis triggering emergency grid charge
    // when Cell 9 alarms early. Remove this block once pack 9 is rebuilt.
    if (soc < 45) {
        soc = (soc >= 5) ? soc - 5 : 0;
    }

    uint8_t soh = 100; // JBD BMS doesn't report SoH — assume 100%

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
static void can_send_measurements(OverkillSolarBms2& bms) {
    int16_t voltage = (int16_t)(bms.get_voltage() * 100.0f);  // e.g. 54.9V → 5490
    int16_t current = (int16_t)(bms.get_current() * 10.0f);  // e.g. 12.3A → 123 (signed)
    int16_t temp    = (int16_t)(bms.get_ntc_temperature(0) * 10.0f);

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
static void can_send_alarms(OverkillSolarBms2& bms) {
    // Calculate cell min/max for over/under voltage detection
    uint8_t  numCells = bms.get_num_cells();
    float minV = 9999, maxV = 0;
    for (uint8_t c = 0; c < numCells; c++) {
        float v = bms.get_cell_voltage(c);
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
    }

    uint8_t protection = 0;
    if (maxV > 4.20f)  bitSet(protection, 1);  // cell overvoltage  — NMC max 4.20V
    if (minV < 2.75f)  bitSet(protection, 2);  // cell undervoltage — NMC cutoff 2.75V
    if (bms.get_ntc_temperature(0) > 55.0f) bitSet(protection, 3);  // high temp
    if (bms.get_ntc_temperature(0) <  0.0f) bitSet(protection, 4);  // low temp

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
// Send all Pylontech frames — call every 100ms from main loop
// -----------------------------------------------------------------------
void sendCANFrames(OverkillSolarBms2& bms) {
    can_send_limits();
    can_send_soc(bms);
    can_send_measurements(bms);
    can_send_alarms(bms);
    can_send_request(bms);
    can_send_manufacturer();
    can_send_alive();
}
