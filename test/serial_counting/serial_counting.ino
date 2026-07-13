#include <avr/io.h>
#include <util/delay.h>

void setup() {
  // Set clock to full speed (10MHz or 20MHz internal osc)
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;

  // Initialize hardware serial at 115200 baud on Pin 4 (PA1)
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Serial Counting Test Booted ===");
}

uint32_t count = 0;

void loop() {
  Serial.print("Count: ");
  Serial.println(count++);
  delay(1000); // Count once per second
}
