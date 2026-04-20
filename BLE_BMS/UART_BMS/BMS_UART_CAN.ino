#include <bms2.h>
#include "driver/twai.h"

// -----------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------
#define BMS_RX_PIN      16    // ESP32 RX2 ← JBD TX
#define BMS_TX_PIN      17    // ESP32 TX2 → JBD RX
#define BMS_BAUD        9600

#define CAN_TX_PIN      GPIO_NUM_21
#define CAN_RX_PIN      GPIO_NUM_19

#define CAN_INTERVAL_MS   100
#define LOG_INTERVAL_MS   5000

// -----------------------------------------------------------------------
// BMS
// -----------------------------------------------------------------------
OverkillSolarBms2 bms;

// -----------------------------------------------------------------------
// Serial log — every 5 seconds
// -----------------------------------------------------------------------
void logBMSData() {
    uint8_t numCells = bms.get_num_cells();
    float   voltage  = bms.get_voltage();

    if (numCells == 0 || voltage == 0.0f) {
        Serial.println("⏳ Waiting for valid BMS data...");
        return;
    }

    float minV = 9999, maxV = 0, sum = 0;
    for (uint8_t c = 0; c < numCells; c++) {
        float v = bms.get_cell_voltage(c);
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sum += v;
    }
    float avgV   = sum / numCells;
    float deltaV = maxV - minV;

    Serial.println("\n========== BMS STATUS ==========");
    Serial.printf("Voltage:   %.2f V\n",   voltage);
    Serial.printf("Current:   %.2f A\n",   bms.get_current());
    Serial.printf("SoC:       %d %%\n",    bms.get_state_of_charge());
    Serial.printf("Capacity:  %.1f Ah\n",  bms.get_balance_capacity());
    Serial.printf("Temp:      %.1f C\n",   bms.get_ntc_temperature(0));
    Serial.printf("CHG: %s  DSG: %s\n",
        bms.get_charge_mosfet_status()    ? "ON" : "OFF",
        bms.get_discharge_mosfet_status() ? "ON" : "OFF");
    Serial.println("--- Cells ---");
    Serial.printf("Min: %.3fV  Max: %.3fV  Avg: %.3fV  Delta: %.0fmV\n",
        minV, maxV, avgV, deltaV * 1000.0f);
    for (uint8_t c = 0; c < numCells; c++) {
        Serial.printf("  Cell %2d: %.3f V%s\n", c + 1,
            bms.get_cell_voltage(c),
            bms.get_balance_status(c) ? " *bal*" : "");
    }
    Serial.println("=================================");
}

// -----------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== JBD BMS (UART) → Solis CAN Bridge ===");

    // Start CAN
    setupCAN();

    // Start UART to BMS
    Serial2.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
    bms.begin(&Serial2);
    Serial.println("✅ BMS UART started");
}

// -----------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------
static unsigned long lastCAN = 0;
static unsigned long lastLog = 0;

void loop() {
    // Poll BMS
    bms.main_task(true);

    unsigned long now = millis();

    // Send CAN frames every 100ms
    if (now - lastCAN >= CAN_INTERVAL_MS) {
        lastCAN = now;
        sendCANFrames(bms);
    }

    // Log to serial every 5 seconds
    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        logBMSData();
    }
}
