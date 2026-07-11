#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define BTN_PIN PIN_PA3 // Pin 7 (Your remapped button pin)

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;
}

// Reads the 10-byte unique silicon serial number from address 0x1103 to 0x110C
static void print_silicon_id() {
  Serial.print("Unique Silicon ID (10 bytes): ");
  for (uint8_t offset = 0; offset < 10; offset++) {
    uint8_t val = *(volatile uint8_t*)(0x1103 + offset);
    if (val < 0x10) Serial.print("0");
    Serial.print(val, HEX);
    Serial.print(" ");
  }
  Serial.println();
}

static void print_battery_voltage() {
  // Configure VREF for internal 1.1V reference
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  delayMicroseconds(500);

  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc; // Read internal 1.1V reference
  ADC0.CTRLA = ADC_ENABLE_bm;

  // Discard first reading
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  uint16_t adc = ADC0.RES;
  ADC0.CTRLA = 0; // Disable ADC

  if (adc == 0) {
    Serial.println("Vcc Reading: Error (ADC=0)");
    return;
  }
  
  uint32_t vcc_mv = (1125300UL / adc);
  Serial.print("Internal Vcc: ");
  Serial.print((float)vcc_mv / 1000.0, 3);
  Serial.println(" V");
}

static void print_internal_temperature() {
  // Configure VREF for 1.1V reference
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  delayMicroseconds(500);

  ADC0.CTRLC = ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV16_gc; // Use 1.1V Vref
  ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc; // Measure internal temp
  ADC0.CTRLA = ADC_ENABLE_bm;

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  uint16_t adc = ADC0.RES;
  ADC0.CTRLA = 0; // Disable ADC

  // Read temperature calibration parameters from signature row
  int8_t sigrow_offset = *(volatile int8_t*)(0x1121); // TEMPSENSE0 Offset
  uint8_t sigrow_gain = *(volatile uint8_t*)(0x1122);  // TEMPSENSE1 Gain

  // Calculate temperature in Kelvin and Celsius
  // Temp = (ADC - Offset) * Gain / 256
  int32_t temp_k = ((int32_t)adc - sigrow_offset) * sigrow_gain;
  temp_k = temp_k / 256;
  float temp_c = (float)temp_k - 273.15;

  Serial.print("Internal Temperature: ");
  Serial.print(temp_c, 1);
  Serial.println(" C");
}

void setup() {
  wdt_disable();
  set_clock_full_speed();

  // Initialize Hardware Serial on Pin 4 (PA1)
  // Pin 4 is the hardware USART0 TX pin. Make sure your USB-to-TTL Rx pin is connected here!
  Serial.begin(115200);
  
  pinMode(BTN_PIN, INPUT_PULLUP);

  delay(1000);
  Serial.println();
  Serial.println("=== ATtiny402 Hardware Diagnostics Mode ===");
  Serial.println("Serial baud rate: 115200");
}

void loop() {
  Serial.println("----------------------------------------");
  
  // 1. Print unique silicon ID
  print_silicon_id();

  // 2. Print internal Vcc voltage
  print_battery_voltage();

  // 3. Print internal temperature sensor value
  print_internal_temperature();

  // 4. Print button state on Pin 7 (PA3)
  bool buttonState = (digitalRead(BTN_PIN) == LOW);
  Serial.print("Button Pin 7 (PA3) State: ");
  if (buttonState) {
    Serial.println("PRESSED (LOW)");
  } else {
    Serial.println("RELEASED (HIGH)");
  }

  delay(2000); // Repeat every 2 seconds
}
