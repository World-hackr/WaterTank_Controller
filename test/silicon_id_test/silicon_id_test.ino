#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

void setup() {
  wdt_disable(); // Disable watchdog immediately to prevent resets

  // Set clock to 20MHz (division 1) - matches working blink test
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;

  // Swap Serial pins to alternate mapping (TX: PA1/Pin 4)
  Serial.swap(1);
  Serial.begin(57600); // 115200 real-world baud at 20MHz clock
  
  _delay_ms(1000);
  Serial.println("=== Silicon ID Test Booted ===");
}

void loop() {
  Serial.print("Silicon ID (10 bytes): ");
  
  // Read the 10-byte unique serial number from memory addresses 0x1103 - 0x110C
  for (uint8_t offset = 0; offset < 10; offset++) {
    uint8_t val = *(volatile uint8_t*)(0x1103 + offset);
    
    // Print leading zero if single hex character
    if (val < 0x10) Serial.print("0");
    Serial.print(val, HEX);
    Serial.print(" ");
  }
  Serial.println();

  _delay_ms(2000); // Print every 2 seconds
}
