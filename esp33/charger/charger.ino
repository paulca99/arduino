// ============================================================================
// ESP32 Charger Diverter - Main Sketch
// ============================================================================
// Reorganized version of esp32/charger with improved structure.
// This is a thin entry point that delegates to organized modules in src/.

#include "src/Config.h"
#include "src/WiFiUI.h"
#include "src/Emon.h"
#include "src/PwmController.h"
#include "src/Battery.h"
#include "src/TimeSchedule.h"

// ============================================================================
// Global State
// ============================================================================

static int loopCount = 0;
static hw_timer_t *timer = NULL;

// Timer interrupt handler
void ARDUINO_ISR_ATTR onTimer()
{
  // ESP.restart(); // Uncomment to restart on timer
  Serial.println("Timer fired");
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(2000);
  Serial.println("setupStart");
  
  // Configure watchdog timer (1 hour timeout)
  timer = timerBegin(0, TIMER_PRESCALER, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_ALARM_US, true);
  timerAlarmEnable(timer);
  
  Serial.println("wifisetupStart");
  wifiSetup();
  
  Serial.println("timesetupStart");
  timeSetup();
  
  setupEmon();
  setupBattery();
  pwmSetup();
}

// ============================================================================
// Main Loop
// ============================================================================

void loop()
{
  autoLoop();
  // testLoop(); // Uncomment for testing
}

// Test loop (for manual testing)
void testLoop()
{
  rampDown();
  delay(5000);
  // rampUp();
  // wifiLoop();
}

// Main automatic control loop
void autoLoop()
{
  // Read all sensors
  readCharger(false); // Pass false initially, will be updated by PwmController
  readGrid();
  readBattery();
  wifiLoop();
  timeLoop();
  
  // Adjust charger power after stabilization period
  if (loopCount > CHARGER_ADJUST_INTERVAL)
  {
    readGti(false); // Pass false initially
    adjustCharger();
    loopCount = CHARGER_ADJUST_INTERVAL + 1; // Prevent overflow
  }
  loopCount++;
}
