/*
 * control.h — PWM/relay charger control
 *
 * Controls five series-connected PSUs via ESP32 LEDC PWM channels and a
 * power relay.  Implements the same charge-control algorithm as charger/.
 *
 * Pin assignments:
 *   Power relay  : GPIO 13 (LOW = ON, HIGH = OFF)
 *   Afterburner  : GPIO 27 (LOW = ON, HIGH = OFF)
 *   GTI relay    : GPIO 23 (HIGH = ON, LOW = OFF)
 *   PSU PWM pins : GPIO 19, 18, 5, 14, 16  (LEDC channels 0–4)
 */
#pragma once

#include <Arduino.h>

// ── Control parameters (adjustable via web UI) ────────────────────
extern int   upperChargerLimit;   // W — max grid import before reducing power
extern int   lowerChargerLimit;   // W — min grid export before increasing power
extern float voltageLimit;        // V — battery voltage ceiling (with hysteresis)
extern int   chargerPLimit;       // W — absolute charger watt cap
extern int   chargerPowerCreep;   // 0/1 — allow chargerPLimit to auto-creep

// ── Runtime state ─────────────────────────────────────────────────
extern bool    powerOn;        // true when charger relay is on
extern bool    afterburnerOn;  // true when PSU5 (afterburner) relay is on
extern bool    GTIenabled;     // true when GTI relay is allowed by schedule
extern int     psu_resistance_values[]; // current PWM resistance value per PSU
extern const int PSU_COUNT;

// ── Public API ─────────────────────────────────────────────────────

/** Initialise GPIO pins and PWM channels.  Call once in setup(). */
void controlSetup();

/**
 * Main control loop — call each iteration after readGrid()/readCharger().
 * Decides whether to turn power on/off and how much to increase/reduce.
 */
void adjustCharger();

// Ramp helpers (used by testLoop)
bool rampUp();
void rampDown();

// Power state
void turnPowerOn();
void turnPowerOff();
void turnGTIOn();
void turnGTIOff();
void turnAfterburnerOn();
void turnAfterburnerOff();

// Queries
bool isAtMinPower();
bool isAtMaxPower();
int  getTotalResistance();

// Low-level
void writePowerValuesToPSUs();
void goBottom();
void goMid();
void goTop();

// Test mode
void setupTest();
