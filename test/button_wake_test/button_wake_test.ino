#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define BTN_PIN PIN_PA3 // Pin 7 (Button)

void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;
}

ISR(PORTA_PORT_vect) {
  // Clear the interrupt flag on Pin 7 (PA3)
  PORTA.INTFLAGS = PIN3_bm; 
}

void setup() {
  wdt_disable();
  set_clock_full_speed();

  Serial.swap(1);
  Serial.begin(57600); // 115200 real-world baud at 20MHz clock
  
  pinMode(BTN_PIN, INPUT_PULLUP);
  
  _delay_ms(1000);
  Serial.println("=== Button Wake Test Booted ===");
  Serial.println("Entering Deep Sleep. Press the button on Pin 7 (PA3) to wake me!");
}

void loop() {
  // Ensure serial transmissions are fully sent before sleeping
  Serial.flush();
  _delay_ms(10);

  // 1. Configure Pin 7 (PA3) to generate an interrupt when pulled LOW (button press)
  PORTA.PIN3CTRL = PORT_ISC_LEVEL_gc; 

  // 2. Put CPU into deep power-down sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sei(); // Enable global interrupts
  sleep_mode(); // CPU sleeps here
  
  // --- WOKE UP HERE ---
  cli(); // Disable interrupts during wake routine
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc; // Disable pin interrupt

  Serial.println("Woke up from sleep!");

  // Measure how long the button is held
  uint32_t pressStartMs = millis();
  while (digitalRead(BTN_PIN) == LOW) {
    _delay_ms(10);
  }
  uint32_t pressDuration = millis() - pressStartMs;

  Serial.print("Button released. Hold duration: ");
  Serial.print(pressDuration);
  Serial.println(" ms");
  Serial.println("Going back to sleep...");
  Serial.println();
}
