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
  Serial.begin(4800); // 9600 real-world baud at 20MHz clock
  
  _delay_ms(1000);
  Serial.println("=== Battery Voltage Test Booted (9600 Baud) ===");
}

void loop() {
  // 1. Configure VREF to use internal 1.1V bandgap reference
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  delayMicroseconds(500);

  // 2. Configure ADC to measure VDD against internal reference
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc; 
  ADC0.CTRLA = ADC_ENABLE_bm;

  // Discard first conversion for accuracy
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  // Perform actual conversion
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  uint16_t adc = ADC0.RES;
  ADC0.CTRLA = 0; // Disable ADC

  if (adc != 0) {
    // Vcc = 1.1V * 1023 / ADC
    uint32_t vcc_mv = (1125300UL / adc);
    Serial.print("Vcc: ");
    Serial.print((float)vcc_mv / 1000.0, 3);
    Serial.println(" V");
  } else {
    Serial.println("Vcc: ADC Error");
  }

  _delay_ms(2000); // Repeat every 2 seconds
}
