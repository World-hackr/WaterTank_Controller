#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

// All usable GPIO pins on the ATtiny402 (excluding PA0/UPDI Pin 6)
const uint8_t testPins[] = {
  PIN_PA6, // Pin 2 (Sensor Ladder)
  PIN_PA7, // Pin 3 (Probe Drive)
  PIN_PA1, // Pin 4 (RF Data / Serial TX)
  PIN_PA2, // Pin 5 (Relay Output)
  PIN_PA3  // Pin 7 (Button Input / Status LED)
};
const uint8_t pinCount = sizeof(testPins) / sizeof(testPins[0]);

void setup() {
  wdt_disable(); // Disable watchdog timer immediately on boot

  // Set clock to 20MHz (division 1) - matches working blink speed
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;

  // Configure all test pins as OUTPUT
  for (uint8_t i = 0; i < pinCount; i++) {
    pinMode(testPins[i], OUTPUT);
  }
}

void loop() {
  // Set all pins HIGH
  for (uint8_t i = 0; i < pinCount; i++) {
    digitalWrite(testPins[i], HIGH);
  }
  _delay_ms(1000); // Wait 1 second (1000ms)

  // Set all pins LOW
  for (uint8_t i = 0; i < pinCount; i++) {
    digitalWrite(testPins[i], LOW);
  }
  _delay_ms(1000); // Wait 1 second (1000ms)
}
