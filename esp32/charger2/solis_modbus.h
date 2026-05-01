/*
 * solis_modbus.h - Solis S5-EH1P RS485/Modbus poller
 *
 * Runs as a FreeRTOS task on Core 1.
 * Shared data is protected by solisMutex.
 * Last-good values are kept with a millisecond timestamp so callers
 * can check data freshness.
 *
 * RS485 wiring:
 *   ESP32 GPIO 25 → RS485 RO  (RX)
 *   ESP32 GPIO 26 → RS485 DI  (TX)
 *   RS485 DE/RE tied together → GPIO 27 (optional flow-control pin,
 *   set RS485_DE_PIN to -1 if using auto-direction module)
 */
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ── Pin configuration ──────────────────────────────────────────────
#define RS485_RX_PIN  25   // UART2 RX — free from PSU PWM pins
#define RS485_TX_PIN  26   // UART2 TX
#define RS485_DE_PIN  -1   // DE/RE driver-enable pin; -1 = not used
#define RS485_BAUD    9600

// ── Shared data structure ──────────────────────────────────────────
struct SolisData {
  // PV strings
  float pv1Voltage;    // V
  float pv1Current;    // A
  float pv2Voltage;    // V
  float pv2Current;    // A

  // Solis battery (NOT the local ADC battery)
  float batteryVoltage;  // V
  float batteryCurrent;  // A  (+ve = charging, −ve = discharging)
  float batteryPower;    // W  (+ve = charging)

  // Grid (as seen by the inverter)
  float gridVoltage;    // V
  float gridCurrent;    // A
  float gridPower;      // W  (−ve = export, +ve = import)

  // AC quality
  float frequency;      // Hz
  float powerFactor;

  // Inverter health
  float inverterTemp;   // °C
  float totalPower;     // W  (inverter AC output)

  // Energy totals
  float dailyEnergy;    // kWh
  float totalEnergy;    // kWh

  // Metadata
  unsigned long lastUpdateMs; // millis() at last successful decode
  bool          valid;        // true once at least one full poll done
};

extern SemaphoreHandle_t solisMutex;

// ── Public API ─────────────────────────────────────────────────────

/** Call once in setup() before starting the task. */
void solisModbusSetup();

/** FreeRTOS task — pin to Core 1 via xTaskCreatePinnedToCore(). */
void solisModbusTask(void* pv);

/** Thread-safe copy of the shared SolisData snapshot. */
SolisData getSolisSnapshot();

/** Milliseconds since the last successful Solis poll (0 if never). */
unsigned long solisDataAgeMs();
