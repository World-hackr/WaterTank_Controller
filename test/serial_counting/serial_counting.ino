#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

void setup() {
  wdt_disable(); // Disable watchdog immediately to prevent resets

  // Set clock to 20MHz (division 1) - matches working blink test
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;

  // Swap Serial pins to alternate mapping (TX: PA1/Pin 4, RX: PA2/Pin 5)
  Serial.swap(1);
  
  // Initialize at 57600 baud. At 20MHz clock (with 10MHz compile settings), 
  // this will output at exactly 115200 baud on Pin 4 (PA1).
  Serial.begin(57600);
  
  _delay_ms(1000);
  Serial.println("=== Serial Counting Test Booted ===");
}

uint32_t count = 0;

void loop() {
  Serial.print("Count: ");
  Serial.println(count++);
  _delay_ms(1000); // Wait 1 second (uses accurate utility delay)
}
