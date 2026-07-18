#include <avr/io.h>
#include <util/delay.h>

// --- Port-mapped Pin Configurations (User PCB Layout) ---
#define TX_DATA_PIN_bm  PIN4_bm // PA4 (Pin 2)  - RF Data Out
#define DRIVE_PIN_bm    PIN6_bm // PA6 (Pin 4)  - Common Probe Drive (Active HIGH)
#define BTN_PIN_bm      PIN7_bm // PA7 (Pin 5)  - Manual Button Input (Active LOW)
#define STATUS_LED_bm   PIN0_bm // PB0 (Pin 9)  - Status/Pairing LED (Active HIGH)

// Level Probes (with external 1M pull-downs and 6.7k series resistors)
#define PROBE_L1_bm     PIN1_bm // PA1 (Pin 11) - Probe Level 1
#define PROBE_L2_bm     PIN2_bm // PA2 (Pin 12) - Probe Level 2

#define PROBE_L3_bm     PIN3_bm // PA3 (Pin 13) - Probe Level 3
#define PROBE_L4_bm     PIN5_bm // PA5 (Pin 3)  - Probe Level 4

// --- Protocol Constants ---
#define BIT_US 1000
#define REPEATS_PER_PACKET 4
#define MAX_PACKET_LEN 16
#define PACKET_GAP_MS 35

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00; // Prescaler divisor 1 (20MHz internal)
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
  return crc;
}

static void tx_write(bool value) {
  if (value) {
    PORTA.OUTSET = TX_DATA_PIN_bm;
  } else {
    PORTA.OUTCLR = TX_DATA_PIN_bm;
  }
}

static void send_symbol(uint8_t symbol) {
  for (uint8_t bit = 0; bit < 6; bit++) {
    tx_write((symbol & (1 << bit)) != 0);
    _delay_us(BIT_US);
  }
}

static void send_encoded_byte(uint8_t value) {
  send_symbol(symbols[value >> 4]);
  send_symbol(symbols[value & 0x0f]);
}

static void radio_send(const uint8_t *payload, uint8_t len) {
  if (len == 0 || len > (MAX_PACKET_LEN - 3)) return;

  uint8_t count = 1 + len + 2;
  uint16_t crc = 0xffff;
  crc = crc16_update(crc, count);
  for (uint8_t i = 0; i < len; i++) crc = crc16_update(crc, payload[i]);
  crc = ~crc;

  for (uint8_t i = 0; i < 6; i++) send_symbol(0x2a);
  send_symbol(0x38);
  send_symbol(0x2c);
  send_encoded_byte(count);
  for (uint8_t i = 0; i < len; i++) send_encoded_byte(payload[i]);
  send_encoded_byte(crc & 0xff);
  send_encoded_byte(crc >> 8);
  tx_write(false);
}

static void send_packet_repeats(const uint8_t *payload, uint8_t repeats) {
  for (uint8_t r = 0; r < repeats; r++) {
    radio_send(payload, 8);
    _delay_ms(PACKET_GAP_MS);
  }
}

// Reads 4 digital probe pins to determine water level (0 to 4) - Active HIGH
static uint8_t read_digital_level() {
  uint8_t level = 0;
  
  // Probes are Active HIGH (pulled to 3.3V by water)
  if (PORTA.IN & PROBE_L1_bm) level = 1; 
  if (PORTA.IN & PROBE_L2_bm) level = 2; 
  if (PORTA.IN & PROBE_L3_bm) level = 3; 
  if (PORTA.IN & PROBE_L4_bm) level = 4; 

  return level;
}

// Read Vcc directly against internal reference
static uint8_t read_battery_code() {
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  _delay_us(500); // Wait for Vref to settle fully

  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.CTRLA = ADC_ENABLE_bm;

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  uint16_t adc = ADC0.RES;
  ADC0.CTRLA = 0; // Shut down ADC

  return (adc >= 341) ? 150 : 180;
}

void setup() {
  set_clock_full_speed();

  // Configure Data and Drive pins as outputs
  PORTA.DIRSET = TX_DATA_PIN_bm | DRIVE_PIN_bm;
  PORTA.OUTCLR = TX_DATA_PIN_bm;
  PORTA.OUTSET = DRIVE_PIN_bm; // Keep Drive Pin HIGH continuously for testing

  // Configure Status LED on Port B as output
  PORTB.DIRSET = STATUS_LED_bm;
  PORTB.OUTCLR = STATUS_LED_bm;

  // Configure Button (PA7) with Pull-up enabled.
  PORTA.PIN7CTRL = PORT_PULLUPEN_bm;

  // Disable internal pull-ups on Probes (PA1, PA2, PA3, PA5) to rely on external 1M pull-downs
  PORTA.PIN1CTRL = 0;
  PORTA.PIN2CTRL = 0;
  PORTA.PIN3CTRL = 0;
  PORTA.PIN5CTRL = 0;

  // Disable digital input buffers on other unused Port B pins for power saving
  PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
}

void loop() {
  // Read Inputs
  bool buttonPressed = !(PORTA.IN & BTN_PIN_bm); // PA7 is LOW when pressed
  uint8_t level = read_digital_level();
  uint8_t battery = read_battery_code();

  // Local LED feedback: LED turns ON if button is pressed
  if (buttonPressed) {
    PORTB.OUTSET = STATUS_LED_bm;
  } else {
    PORTB.OUTCLR = STATUS_LED_bm;
  }

  // Create simple raw diagnostic packet
  uint8_t packet[8];
  packet[0] = 0xAA;                   // Diagnostic Header
  packet[1] = buttonPressed ? 0 : 1;  // Button state (0 = pressed, 1 = released)
  packet[2] = level;                  // Level (0 to 4)
  packet[3] = battery;                // Battery code
  packet[4] = 0;
  packet[5] = 0;
  packet[6] = 0;
  packet[7] = 0;

  // Send packet repeated 4 times
  send_packet_repeats(packet, REPEATS_PER_PACKET);

  // Sleep/Wait 1 second before next transmission
  _delay_ms(1000);
}

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}
