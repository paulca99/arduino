#include <avr/sleep.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void setup()
{
  Serial.begin(115200);

  /* Setup the interrupt pin */
  attachInterrupt(1, onPulse, FALLING);

  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode
}

void loop()
{
  //-------------------------------------------------------------
  // 1) Enter sleep mode
  //-------------------------------------------------------------
  //cbi(ADCSRA,ADEN);    // switch Analog to Digital converter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  // The arduino is now sleeping...

  //-------------------------------------------------------------
  // 2) Program will resume from here on interrupt
  //-------------------------------------------------------------
  sleep_disable();
  sbi(ADCSRA,ADEN);    // switch Analog to Digitalconverter ON

  Serial.print('P');

  delay(10);
}

void onPulse()
{
  // It continues in the main loop
}
