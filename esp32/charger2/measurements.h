/*
 * measurements.h — EmonLib-based AC energy measurements + ADS1115 battery voltage
 *
 * Wiring (same as charger/):
 *   GPIO 34 — AC voltage sensing (grid voltage transformer)
 *   GPIO 32 — Grid current CT
 *   GPIO 33 — Charger/GTI current CT
 *   I2C SDA/SCL — ADS1115 at 0x48 (battery voltage divider on AIN0)
 */
#pragma once

#include <Arduino.h>
#include "espEmonLib.h"

// ── Calibration constants (adjustable via web UI) ──────────────────
extern float gridVoltageCalibration;   // default 297
extern float gridPhaseOffset;          // default 1.4
extern float gridCurrentCalibration;   // default 52.6
extern float chargerCurrentCalibration; // default 30

// ── Live readings ──────────────────────────────────────────────────
extern float chargerPower;       // W — charger AC power
extern float gtiPower;           // W — GTI AC power (measured on same CT when charger is off)
extern float localBatteryVoltage; // V — local ADC battery via ADS1115

// ── EmonLib instances (needed by control and web) ─────────────────
extern EnergyMonitor emonGrid;
extern EnergyMonitor emonCharger;

// ── Public API ─────────────────────────────────────────────────────

/** Initialise EmonLib and ADS1115.  Call once in setup(). */
void measurementsSetup();

/** Read grid power/voltage.
 *  Uses Solis Modbus values when available and fresh (< 5 s old).
 *  Falls back to the local CT + voltage transformer (EmonLib) if Solis
 *  data is stale or has not yet been received.
 *  Populates emonGrid.Vrms, .Irms, .realPower, .powerFactor.
 */
void readGrid();

/**
 * Read charger AC current and compute charger power.
 * Returns chargerPower.  Returns 0 if charger is off.
 */
float readCharger();

/**
 * Read GTI power via the charger CT (only valid when charger is off).
 * Also updates lowerChargerLimit based on GTI power.
 * Returns gtiPower.  Returns 0 if charger is on.
 */
float readGti();

/** Read local battery voltage via ADS1115 + rolling average. */
void readLocalBattery();

/** Reconfigure EmonLib after changing calibration constants. */
void reconfigureEmon();
