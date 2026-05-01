/*
 * control.cpp — PWM/relay charger control algorithm
 *
 * Ported directly from charger/pwmFunctions.cpp with minor clean-up.
 * Logic is intentionally preserved to maintain parity with the working
 * charger/ firmware.
 */
#include "control.h"
#include "measurements.h"

// ── Pin assignments ────────────────────────────────────────────────
static const int POWER_PIN        = 13;
static const int AFTERBURNER_PIN  = 27;
static const int GTI_PIN          = 23;

static const int PSU_VOLTAGE_PINS[] = {19, 18, 5, 14, 16};
static const int PWM_CHANNELS[]     = {0,  1,  2,  4,  3};
static const int PWM_FREQ           = 200;   // Hz
static const int PWM_RESOLUTION     = 9;     // bits → range = 511

const int PSU_COUNT = sizeof(PSU_VOLTAGE_PINS) / sizeof(PSU_VOLTAGE_PINS[0]);

// ── Control parameters ────────────────────────────────────────────
int   upperChargerLimit = 100;
int   lowerChargerLimit = 0;
float voltageLimit      = 58.4f;
int   chargerPLimit     = 2900;
int   chargerPowerCreep = 0;

// ── Runtime state ─────────────────────────────────────────────────
bool powerOn       = false;
bool afterburnerOn = false;
bool GTIenabled    = true;

static bool VOLTAGE_HIGH = false;
static int  SOC          = 90;  // TODO: derive from Solis data

static const int RANGE = (1 << PWM_RESOLUTION) - 1;  // 511
int psu_resistance_values[5] = {RANGE, RANGE, RANGE, RANGE, RANGE};
static int psu_pointer = 0;

// ── Helpers ───────────────────────────────────────────────────────

int getTotalResistance() {
  int total = 0;
  for (int i = 0; i < PSU_COUNT; i++) total += psu_resistance_values[i];
  return total;
}

void writePowerValuesToPSUs() {
  for (int i = 0; i < PSU_COUNT; i++)
    ledcWrite(PWM_CHANNELS[i], psu_resistance_values[i]);
}

static bool voltageLimitReached() {
  float v = localBatteryVoltage;
  if (VOLTAGE_HIGH && (v < (voltageLimit - 1.0f))) VOLTAGE_HIGH = false;
  if (v >= voltageLimit || VOLTAGE_HIGH) {
    Serial.printf("[Ctrl] VOLTAGE LIMIT %.2f V\n", v);
    VOLTAGE_HIGH = true;
    return true;
  }
  return false;
}

bool isAtMinPower() {
  for (int i = 0; i < PSU_COUNT; i++) {
    if (psu_resistance_values[i] < RANGE) return false;
  }
  if (afterburnerOn) { turnAfterburnerOff(); return false; }
  return true;
}

bool isAtMaxPower() {
  for (int i = 0; i < PSU_COUNT; i++) {
    if (psu_resistance_values[i] > 0) return false;
  }
  Serial.println("[Ctrl] MAX POWER");
  for (int i = 0; i < PSU_COUNT; i++) psu_resistance_values[i] = RANGE - 1;
  writePowerValuesToPSUs();
  if (afterburnerOn) {
    turnPowerOff();
    delay(3000);
    turnPowerOn();
    turnAfterburnerOn();
  } else {
    turnAfterburnerOn();
    delay(6000);
    return false;
  }
  return true;
}

static void setMinPower() {
  for (int i = 0; i < PSU_COUNT; i++) psu_resistance_values[i] = RANGE - 1;
  psu_pointer = 0;
}

static void incrementPower(int stepAmount) {
  if (localBatteryVoltage >= voltageLimit) return;
  for (int i = 0; i < stepAmount; i++) {
    psu_resistance_values[psu_pointer]--;
    if (psu_resistance_values[psu_pointer] < 0) psu_resistance_values[psu_pointer] = 0;
    if (++psu_pointer >= PSU_COUNT) psu_pointer = 0;
  }
  writePowerValuesToPSUs();
}

static void decrementPower(int stepAmount) {
  if (isAtMinPower()) return;
  for (int i = 0; i < stepAmount; i++) {
    if (--psu_pointer < 0) psu_pointer = PSU_COUNT - 1;
    psu_resistance_values[psu_pointer]++;
    if (psu_resistance_values[psu_pointer] > RANGE) psu_resistance_values[psu_pointer] = RANGE;
  }
  writePowerValuesToPSUs();
}

// ── Power state transitions ────────────────────────────────────────

void turnGTIOff()  { digitalWrite(GTI_PIN, LOW); }
void turnGTIOn()   { VOLTAGE_HIGH = false; if (GTIenabled) digitalWrite(GTI_PIN, HIGH); }

void turnAfterburnerOff() {
  digitalWrite(AFTERBURNER_PIN, HIGH);
  afterburnerOn = false;
  Serial.println("[Ctrl] Afterburner OFF");
}

void turnAfterburnerOn() {
  digitalWrite(AFTERBURNER_PIN, LOW);
  afterburnerOn = true;
  Serial.println("[Ctrl] Afterburner ON");
}

void turnPowerOff() {
  turnAfterburnerOff();
  digitalWrite(POWER_PIN, HIGH);
  powerOn = false;
  turnGTIOn();
  upperChargerLimit = 0;
  lowerChargerLimit = -100;
  Serial.println("[Ctrl] Power OFF");
}

void turnPowerOn() {
  setMinPower();
  writePowerValuesToPSUs();
  digitalWrite(POWER_PIN, LOW);
  powerOn = true;
  turnGTIOff();
  upperChargerLimit = 100;
  lowerChargerLimit = 0;
  Serial.println("[Ctrl] Power ON");
}

// ── Main setup ────────────────────────────────────────────────────

void controlSetup() {
  pinMode(POWER_PIN,       OUTPUT);
  pinMode(AFTERBURNER_PIN, OUTPUT);
  pinMode(GTI_PIN,         OUTPUT);
  turnPowerOff();

  for (int i = 0; i < PSU_COUNT; i++) {
    pinMode(PSU_VOLTAGE_PINS[i], OUTPUT);
    ledcSetup(PWM_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PSU_VOLTAGE_PINS[i], PWM_CHANNELS[i]);
    ledcWrite(PWM_CHANNELS[i], psu_resistance_values[i]);
  }
  writePowerValuesToPSUs();
}

// ── Ramp helpers (test mode) ──────────────────────────────────────

void setupTest() { turnPowerOn(); turnAfterburnerOn(); }

bool rampUp() {
  for (int step = 0; step < (RANGE * PSU_COUNT); step++) {
    incrementPower(1);
    readLocalBattery();
    float cp = readCharger();
    Serial.printf("[Ctrl] RAMP UP step=%d chargerP=%.0f batt=%.2f\n",
                  step, cp, localBatteryVoltage);
    if (cp >= chargerPLimit)            return true;
    if (voltageLimitReached())          return true;
    delay(6);
  }
  return false;
}

void rampDown() {
  while (!isAtMinPower()) {
    decrementPower(1);
    readLocalBattery();
    float cp = readCharger();
    Serial.printf("[Ctrl] RAMP DOWN chargerP=%.0f batt=%.2f\n",
                  cp, localBatteryVoltage);
    delay(6);
  }
  Serial.println("[Ctrl] RAMP DOWN complete");
}

// ── Go-to positions ───────────────────────────────────────────────

void goBottom() { for (int i = 0; i < PSU_COUNT; i++) psu_resistance_values[i] = 0;      writePowerValuesToPSUs(); }
void goMid()    { for (int i = 0; i < PSU_COUNT; i++) psu_resistance_values[i] = RANGE/2; writePowerValuesToPSUs(); }
void goTop()    { for (int i = 0; i < PSU_COUNT; i++) psu_resistance_values[i] = RANGE;   writePowerValuesToPSUs(); }

// ── Core control algorithm ────────────────────────────────────────

static void increaseChargerPower(float startCp) {
  float vbatt      = localBatteryVoltage;
  float fakeGridp  = emonGrid.realPower    + 10000.0f;
  float fakeLower  = (float)lowerChargerLimit + 10000.0f;
  float increase   = fakeLower - fakeGridp;  // always positive

  Serial.printf("[Ctrl] increase: start=%.0f target+%.0f lower=%d grid=%.0f\n",
                startCp, increase, lowerChargerLimit, emonGrid.realPower);

  if (afterburnerOn)  increase *= 0.7f;
  if (vbatt > 56.0f && increase > 50.0f) increase = 50.0f;

  int   stepAmount = (int)(increase / (12800.0f / RANGE)) + 1;
  float target     = startCp + increase;
  if (target > chargerPLimit) {
    if (chargerPowerCreep) chargerPLimit += 5;
    target = chargerPLimit;
  }

  float cp  = startCp;
  int   cnt = 0;
  while (cp < target
         && emonCharger.Irms < 18.0f
         && !voltageLimitReached()
         && !isAtMaxPower()
         && cp < chargerPLimit
         && localBatteryVoltage < voltageLimit
         && cnt < 50) {
    incrementPower(stepAmount);
    cp     = readCharger();
    readLocalBattery();
    cnt++;
  }
}

static void reduceChargerPower(float startCp) {
  readLocalBattery();
  decrementPower(1);  // always at least one step down
  float fakeGridp = emonGrid.realPower    + 10000.0f;
  float fakeUpper = (float)upperChargerLimit + 10000.0f;
  float reduction = (fakeGridp - fakeUpper) * 0.7f;

  if (localBatteryVoltage > 55.0f && reduction > 200.0f) reduction = 200.0f;

  if (reduction > 0.0f) {
    int   stepAmount = (int)(reduction / (12800.0f / RANGE)) + 1;
    float target     = startCp - reduction;
    float cp         = startCp;
    Serial.printf("[Ctrl] reduce: by=%.0f target=%.0f\n", reduction, target);
    while (cp > target && !isAtMinPower()) {
      cp = readCharger();
      decrementPower(stepAmount);
    }
  }
}

void adjustCharger() {
  readLocalBattery();
  bool vLimit = voltageLimitReached();

  if (localBatteryVoltage > voltageLimit) decrementPower(30);

  float presentCp = readCharger();

  if (emonGrid.realPower > upperChargerLimit
      || localBatteryVoltage > voltageLimit
      || presentCp > chargerPLimit) {
    if (isAtMinPower() && powerOn) {
      turnPowerOff();
    } else {
      reduceChargerPower(presentCp);
    }
  } else if (SOC < 99 && emonGrid.realPower < lowerChargerLimit) {
    if (!powerOn) {
      turnPowerOn();
      if (localBatteryVoltage > 54.0f) turnAfterburnerOn();
      readGrid();
      for (int i = 0; i < 10; i++) readCharger();  // absorb turn-on spike
      presentCp = readCharger();
    }
    increaseChargerPower(presentCp);
  }
}
