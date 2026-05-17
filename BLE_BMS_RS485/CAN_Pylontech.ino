#include "driver/twai.h"
#include <bms2.h>
#include <float.h>

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
// Wiring (BLE_BMS_RS485 defaults): SN65HVD230 TX→GPIO21, RX→GPIO19
//                                  Solis RJ45 pin4=CAN-H, pin5=CAN-L
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Pack series cell count — change to 14 for a 14S pack, leave at 13 for 13S.
// All pack-voltage parameters are derived from this single constant.
// -----------------------------------------------------------------------
#ifndef PACK_SERIES_CELLS
#define PACK_SERIES_CELLS       13      // default: 13S; set to 14 for 14S
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
#define MAX_CHARGE_CURRENT      400   // 40.0A  (x10)
#define MAX_DISCHARGE_CURRENT   400   // 40.0A  (x10)

static uint32_t aliveCounter = 0;

// -----------------------------------------------------------------------
// SOC_EMA_TAU_S — EMA time constant for the voltage-based SoC filter.
//
// Using instantaneous pack voltage to derive SoC causes the reported SoC
// to rebound immediately when the inverter stops drawing current (voltage
// recovers).  That makes the inverter think the battery is usable again,
// which re-enables discharge, voltage sags, SoC drops again — the classic
// oscillation near the inverter's lower SoC threshold.
//
// We smooth the voltage with an exponential moving average (EMA) before
// passing it to voltageToSoc().  An EMA needs only two state variables
// (filtered value + last timestamp) — no ring buffer, negligible RAM.
//
// Time constant τ = 120 s ≈ 2 minutes.  That means a sudden 1 V voltage
// step takes ~2 minutes to be fully reflected in the reported SoC, so
// brief load/no-load rebounds are suppressed.
//
// Tuning:
//   Increase SOC_EMA_TAU_S → more smoothing (slower SoC response).
//   Decrease SOC_EMA_TAU_S → less smoothing (faster SoC response).
//   Set to 0 → disables smoothing (reverts to instantaneous behaviour).
//
// Alpha is computed per-call from elapsed wall-clock time so the
// behaviour is stable even if loop timing varies.
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

    if (packV >= curve[0][0] * SOC_SCALE)             return 100;
    if (packV <= curve[points - 1][0] * SOC_SCALE)    return 0;

    for (uint8_t i = 0; i < points - 1; i++) {
        float vHigh = curve[i][0]   * SOC_SCALE, socHigh = curve[i][1];
        float vLow  = curve[i+1][0] * SOC_SCALE, socLow  = curve[i+1][1];
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
//
// Voltage-based SoC override: uses the filtered (EMA) pack voltage rather
// than instantaneous voltage so that brief load/no-load voltage swings do
// not immediately flip the reported SoC.  The voltage-based override is
// retained because the JBD BMS can report 0% SoC whenever an alarm fires.
// See SOC_EMA_TAU_S above for smoothing details and tuning guidance.
// -----------------------------------------------------------------------
static void can_send_soc(OverkillSolarBms2& bms) {
    // EMA state — persists across calls; filteredV < 0 means "not yet seeded".
    static float         filteredV = -1.0f;
    static unsigned long lastEmaMs = 0;

    float        rawV  = bms.get_voltage();
    unsigned long nowMs = millis();

    if (rawV > 0.0f) {
        if (filteredV < 0.0f) {
            // First valid reading: seed the filter so we start from a sensible
            // voltage immediately rather than ramping up from 0 V.
            filteredV = rawV;
            lastEmaMs = nowMs;
        } else if (SOC_EMA_TAU_S > 0.0f) {
            float dtSec = (nowMs - lastEmaMs) * 1.0e-3f;
            // alpha = 1 - exp(-dt / tau).  For dt << tau this ≈ dt/tau (small
            // step); for dt >> tau this → 1 (instant follow, e.g. after a long
            // gap with no BLE data).  expf() is part of the ESP32 Arduino core.
            float alpha = 1.0f - expf(-dtSec / SOC_EMA_TAU_S);
            filteredV  += alpha * (rawV - filteredV);
        } else {
            filteredV = rawV;  // tau == 0: smoothing disabled, use raw voltage
        }
        lastEmaMs = nowMs;
    }
    // rawV <= 0 means no valid BMS data yet — leave filteredV unchanged so
    // the last known good voltage (or the initial -1 sentinel) is preserved.

    // Convert smoothed voltage to SoC — immune to BMS coulomb-counter resets.
    // voltageToSoc() clamps out-of-range inputs, so the -1 sentinel returns 0%.
    uint8_t soc = voltageToSoc(filteredV);
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
    float minV = FLT_MAX, maxV = 0;
    for (uint8_t c = 0; c < numCells; c++) {
        float v = bms.get_cell_voltage(c);
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
    }

    uint8_t protection = 0;
    const float ntcTemp = bms.get_ntc_temperature(0);
    if (maxV > 4.20f)  bitSet(protection, 1);  // cell overvoltage  — NMC max 4.20V
    if (minV < 2.75f)  bitSet(protection, 2);  // cell undervoltage — NMC cutoff 2.75V
    if (ntcTemp > 55.0f) bitSet(protection, 3);  // high temp
    if (ntcTemp <  0.0f) bitSet(protection, 4);  // low temp

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
// 0x35E — Manufacturer name "PYLONTEC" (8-byte CAN payload field)
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
