#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define TEST_PIN PIN_PA1 // Pin 4

void setup() {
  wdt_disable(); // Disable watchdog timer immediately on boot

  // Set clock to full speed (disable bootloader division)
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;

  pinMode(TEST_PIN, OUTPUT);
}

void loop() {
  digitalWrite(TEST_PIN, HIGH);
  _delay_ms(500); // 500ms ON
  digitalWrite(TEST_PIN, LOW);
  _delay_ms(500); // 500ms OFF
}
