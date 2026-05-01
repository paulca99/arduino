/*
 * charger2.ino — Refactored ESP32 charger firmware
 *
 * Features:
 *  - EmonLib-based local AC measurements (grid / charger / GTI)
 *  - ADS1115 local battery voltage over I2C
 *  - Async Solis RS485/Modbus polling on Core 1 (FreeRTOS task)
 *  - AsyncWebServer status UI on port 80 + JSON API on port 8080
 *  - NTP time-based GTI scheduling
 *  - Same PWM/relay charger control algorithm as charger/
 *
 * See README.md for wiring, library dependencies, and configuration.
 *
 * WiFi credentials:
 *   Copy secrets_template.h → secrets.h and fill in your SSID/password.
 *   secrets.h is git-ignored so credentials stay local.
 */

#include "solis_modbus.h"
#include "measurements.h"
#include "control.h"
#include "web.h"
#include "timestuff.h"

static int loopCount = 0;
static int checkTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n[charger2] Starting up...");

  solisModbusSetup();   // init RS485 UART + mutex
  measurementsSetup();  // init EmonLib + ADS1115 battery ADC
  controlSetup();       // init PWM channels and relay pins
  wifiSetup();          // connect WiFi, start web servers (uses secrets.h)
  timeSetup();          // configure NTP (needs WiFi)

  // Start Solis polling on Core 1
  xTaskCreatePinnedToCore(
    solisModbusTask,  // task function
    "SolisPoller",    // task name
    4096,             // stack size (bytes)
    NULL,             // parameter
    1,                // priority
    NULL,             // task handle
    1                 // core 1
  );

  Serial.println("[charger2] Ready.");
}

void loop() {
  // AC measurements — called every iteration
  readGrid();
  readCharger();
  readLocalBattery();

  // Time/NTP check — every 50 iterations (~few seconds)
  checkTime++;
  if (checkTime >= 50) {
    timeLoop();
    checkTime = 0;
  }

  // GTI reading + charger adjustment.
  // First 15 iterations: just accumulate measurements.
  // From iteration 16 onward: run on every loop (same as charger/).
  if (loopCount > 15) {
    readGti();
    adjustCharger();
    loopCount = 16;  // stays at 16 so the block runs every subsequent loop
  }
  loopCount++;
}
