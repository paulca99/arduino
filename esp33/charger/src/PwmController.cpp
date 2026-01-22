#include "Arduino.h"
#include "PwmController.h"
#include "Config.h"
#include "Emon.h"
#include "Battery.h"

// ============================================================================
// PWM Controller Module Implementation
// ============================================================================

// Pin and channel arrays for 5 series PSUs
static const int psuVoltagePins[] = {PSU_PIN_1, PSU_PIN_2, PSU_PIN_3, PSU_PIN_4, PSU_PIN_5};
static const int pwmChannels[] = {PWM_CHANNEL_0, PWM_CHANNEL_1, PWM_CHANNEL_2, PWM_CHANNEL_3, PWM_CHANNEL_4};
static const int psuCount = 5;

// PWM range (0 to this value)
static const int pwmRange = (1 << PWM_RESOLUTION) - 1; // 2^8 - 1 = 255

// PSU resistance values (PWM duty cycle, higher = less power)
int psuResistanceValues[5] = {pwmRange, pwmRange, pwmRange, pwmRange, pwmRange};
static int psuPointer = 0;

// State variables
bool powerOn = false;
static bool voltageHighLatch = false;

// Configurable limits (can be updated at runtime)
int upperChargerLimit = DEFAULT_UPPER_CHARGER_LIMIT;
int lowerChargerLimit = DEFAULT_LOWER_CHARGER_LIMIT;
bool GTIenabled = true;
int SOC = 90; // TODO: needs calculating

// ============================================================================
// Internal Helper Functions
// ============================================================================

static void writePowerValuesToPSUs()
{
  for (int i = 0; i < psuCount; i++)
  {
    ledcWrite(pwmChannels[i], psuResistanceValues[i]);
  }
}

static bool voltageLimitReached()
{
  float presentVoltage = readBattery();
  
  // Hysteresis: once high, must drop below (limit - hysteresis) to clear
  if (voltageHighLatch && (presentVoltage < (VOLTAGE_LIMIT - VOLTAGE_HYSTERESIS)))
  {
    voltageHighLatch = false;
  }

  if (presentVoltage >= VOLTAGE_LIMIT || voltageHighLatch)
  {
    Serial.println("VOLTAGE LIMIT TRIPPED: " + String(presentVoltage));
    voltageHighLatch = true;
    return true;
  }
  return false;
}

static void setMinPower()
{
  for (int i = 0; i < psuCount; i++)
  {
    psuResistanceValues[i] = pwmRange - 1;
  }
  psuPointer = 0;
}

static void incrementPowerInternal(bool write, int stepAmount)
{
  // Means reducing resistance = more power
  // 255,255,255,255,255 -> 254,255,255,255,255 -> 254,254,255,255,255

  psuResistanceValues[psuPointer] = psuResistanceValues[psuPointer] - stepAmount;
  if (psuResistanceValues[psuPointer] < 0)
  {
    psuResistanceValues[psuPointer] = 0;
  }
  psuPointer++;
  if (psuPointer == psuCount)
  {
    psuPointer = 0;
  }

  if (write)
  {
    writePowerValuesToPSUs();
  }
}

static void decrementPower(bool write, int stepAmount)
{
  // Means increasing resistance = less power
  // 00000, 00001, 00011, 00111, 01111, 11111, 11112
  if (!isAtMinPower())
  {
    psuPointer--;
    if (psuPointer == -1)
    {
      psuPointer = psuCount - 1;
    }
    psuResistanceValues[psuPointer] = psuResistanceValues[psuPointer] + stepAmount;
    if (psuResistanceValues[psuPointer] > pwmRange)
    {
      psuResistanceValues[psuPointer] = pwmRange;
    }
  }
  if (write)
  {
    writePowerValuesToPSUs();
  }
}

static void increaseChargerPower(float startingChargerPower)
{
  // Grid power is lower than lowerChargerLimit (exporting more than threshold)
  
  float fakeGridp = grid.realPower + 10000.0f; // Cancel out negative values
  float fakeLowerLimit = lowerChargerLimit + 10000.0f;
  float increaseAmount = fakeLowerLimit - fakeGridp; // Will always be positive
  
  Serial.println("startingCpower=" + String(startingChargerPower) + " " + 
                 String(lowerChargerLimit) + " - " + String(grid.realPower) + 
                 " = " + String(increaseAmount));
  
  increaseAmount = increaseAmount * POWER_ADJUST_DAMPING; // Don't overshoot
  int stepAmount = (increaseAmount / POWER_STEP_DIVISOR) + 1; // React faster to large changes
  float target = startingChargerPower + increaseAmount;

  if (target > CHARGER_POWER_LIMIT)
    target = CHARGER_POWER_LIMIT; // Pin it at max

  Serial.println("increase amount = " + String(increaseAmount));
  float chargerPowerNow = startingChargerPower;
  
  while (chargerPowerNow < target && 
         (charger.Irms < CURRENT_LIMIT) && 
         !voltageLimitReached() && 
         !isAtMaxPower() && 
         chargerPowerNow < CHARGER_POWER_LIMIT)
  {
    incrementPowerInternal(true, stepAmount);
    chargerPowerNow = readCharger();
  }
}

static void reduceChargerPower(float startingChargerPower)
{
  // Grid power is higher than upper limit (importing too much)
  decrementPower(true, 1); // Always reduce by 1 in case vbatt is too high
  
  float fakeGridp = grid.realPower + 10000.0f; // Cancel out negative values
  float fakeUpperLimit = upperChargerLimit + 10000.0f;
  float reductionAmount = fakeGridp - fakeUpperLimit; // Will always be positive unless voltage too high
  reductionAmount = reductionAmount * POWER_ADJUST_DAMPING; // Don't overshoot
  int stepAmount = (reductionAmount / POWER_STEP_DIVISOR) + 1; // React faster to large changes
  float target = startingChargerPower - reductionAmount;
  
  if (reductionAmount > 0.0f) // Don't bother if it's negative
  {
    float chargerPowerNow = startingChargerPower;
    Serial.println("decrease amount = " + String(reductionAmount));
    
    while (chargerPowerNow > target && !isAtMinPower())
    {
      chargerPowerNow = readCharger();
      decrementPower(true, stepAmount);
    }
  }
}

// ============================================================================
// Public Functions
// ============================================================================

bool isAtMaxPower()
{
  for (int i = 0; i < psuCount; i++)
  {
    if (psuResistanceValues[i] > 0)
    {
      return false;
    }
  }
  Serial.println("MAX POWER");
  return true;
}

int getTotalResistance()
{
  int retval = 0; // Initialize to zero (fix for uninitialized variable bug)
  for (int i = 0; i < psuCount; i++)
  {
    retval += psuResistanceValues[i];
  }
  return retval;
}

bool isAtMinPower()
{
  for (int i = 0; i < psuCount; i++)
  {
    if (psuResistanceValues[i] < pwmRange)
    {
      return false;
    }
  }
  Serial.println("IsAtMinPower");
  return true;
}

void turnGTIOn()
{
  voltageHighLatch = false;
  if (GTIenabled)
  {
    digitalWrite(GTI_RELAY_PIN, HIGH);
  }
}

void turnGTIOff()
{
  digitalWrite(GTI_RELAY_PIN, LOW);
}

void turnPowerOff()
{
  digitalWrite(CHARGER_RELAY_PIN, HIGH);
  powerOn = false;
  turnGTIOn(); // GTI can be on when charger is off (subject to time schedule)
}

void turnPowerOn()
{
  digitalWrite(CHARGER_RELAY_PIN, LOW);
  powerOn = true;
  turnGTIOff(); // GTI must be off when charger is on (interlock)
}

void pwmSetup()
{
  pinMode(CHARGER_RELAY_PIN, OUTPUT);
  pinMode(GTI_RELAY_PIN, OUTPUT);
  turnPowerOff();
  
  for (int i = 0; i < psuCount; i++)
  {
    pinMode(psuVoltagePins[i], OUTPUT);
    ledcSetup(pwmChannels[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(psuVoltagePins[i], pwmChannels[i]);
    ledcWrite(pwmChannels[i], psuResistanceValues[i]);
  }
  writePowerValuesToPSUs();
}

void incrementPower()
{
  incrementPowerInternal(true, 1);
}

void rampUp()
{
  turnPowerOn();
  for (int dutyCycle = 0; dutyCycle <= (pwmRange * 5); dutyCycle++)
  {
    incrementPowerInternal(true, 1);
    delay(10);
  }
}

void rampDown()
{
  turnPowerOn();
  for (int dutyCycle = (pwmRange * 5); dutyCycle >= 0; dutyCycle--)
  {
    decrementPower(true, 1);
  }
}

void adjustCharger()
{
  float vbatt = readBattery();
  bool vLimitHit = voltageLimitReached();

  // Fast voltage limit response
  if (vLimitHit)
  {
    decrementPower(true, 30);
  }

  float presentChargerPower = readCharger();
  if (presentChargerPower > CHARGER_POWER_LIMIT)
  {
    Serial.println("CHARGE POWER LIMIT REACHED " + String(presentChargerPower));
  }
  
  // Condition to reduce or stop charging
  if (grid.realPower > upperChargerLimit || 
      vbatt > VOLTAGE_LIMIT || 
      presentChargerPower > CHARGER_POWER_LIMIT)
  {
    if (isAtMinPower() && powerOn)
    {
      Serial.println("turning off , setting pin HIGH");
      turnPowerOff(); // Turn off
    }
    else
    {
      reduceChargerPower(presentChargerPower);
    }
  }
  // Condition to increase or start charging
  else if (SOC < SOC_FULL_THRESHOLD && 
           grid.realPower < lowerChargerLimit && 
           !vLimitHit)
  {
    if (!powerOn)
    {
      Serial.println("turning on , setting pin LOW");
      turnPowerOn(); // Turn on
      
      // Allow spike to settle
      readGrid();
      for (int i = 0; i < 10; i++) // Reading charger a few times stops the power-on spike
      {
        readCharger();
      }
      presentChargerPower = readCharger();
    }
    increaseChargerPower(presentChargerPower);
  }
}
