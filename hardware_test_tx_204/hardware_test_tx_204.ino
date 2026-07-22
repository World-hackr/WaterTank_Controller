#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define TX_DATA_PIN_bm  PIN4_bm // PA4 - RF data out
#define DRIVE_PIN_bm    PIN6_bm // PA6 - probe drive, active HIGH
#define BTN_PIN_bm      PIN7_bm // PA7 - button, active LOW
#define STATUS_LED_bm   PIN0_bm // PB0 - LED, active HIGH

#define PROBE_L1_bm     PIN1_bm // PA1
#define PROBE_L2_bm     PIN2_bm // PA2
#define PROBE_L3_bm     PIN3_bm // PA3
#define PROBE_L4_bm     PIN5_bm // PA5

#define BIT_US 1000
#define PACKET_GAP_MS 35
#define TEST_REPEATS 3
#define MAX_PACKET_LEN 16

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

uint8_t packetCounter = 0;

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
  return crc;
}

static void tx_write(bool value) {
  if (value) PORTA.OUTSET = TX_DATA_PIN_bm;
  else PORTA.OUTCLR = TX_DATA_PIN_bm;
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

static uint8_t read_probe_mask() {
  PORTA.OUTSET = DRIVE_PIN_bm;
  PORTA.DIRSET = DRIVE_PIN_bm;
  _delay_us(20);

  uint8_t mask = 0;
  if (PORTA.IN & PROBE_L1_bm) mask |= 0x01;
  if (PORTA.IN & PROBE_L2_bm) mask |= 0x02;
  if (PORTA.IN & PROBE_L3_bm) mask |= 0x04;
  if (PORTA.IN & PROBE_L4_bm) mask |= 0x08;

  PORTA.DIRCLR = DRIVE_PIN_bm;
  PORTA.OUTCLR = DRIVE_PIN_bm;
  return mask;
}

static uint8_t read_battery_code() {
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  _delay_us(500);
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.CTRLA = ADC_ENABLE_bm;
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  uint8_t code = ADC0.RES >> 2;
  ADC0.CTRLA = 0;
  return code;
}

static void send_test_packet() {
  uint8_t payload[8];
  payload[0] = 0xA7;
  payload[1] = 0x54;
  payload[2] = packetCounter++;
  payload[3] = read_probe_mask();
  payload[4] = (PORTA.IN & BTN_PIN_bm) ? 0 : 1;
  payload[5] = read_battery_code();
  payload[6] = 0x5A;
  payload[7] = 0xC3;

  PORTB.OUTSET = STATUS_LED_bm;
  for (uint8_t i = 0; i < TEST_REPEATS; i++) {
    radio_send(payload, 8);
    _delay_ms(PACKET_GAP_MS);
  }
  PORTB.OUTCLR = STATUS_LED_bm;
}

void setup() {
  wdt_disable();
  PORTA.DIRSET = TX_DATA_PIN_bm | DRIVE_PIN_bm;
  PORTA.OUTCLR = TX_DATA_PIN_bm | DRIVE_PIN_bm;
  PORTB.DIRSET = STATUS_LED_bm;
  PORTB.OUTCLR = STATUS_LED_bm;

  PORTA.PIN7CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN1CTRL = 0;
  PORTA.PIN2CTRL = 0;
  PORTA.PIN3CTRL = 0;
  PORTA.PIN5CTRL = 0;
}

void loop() {
  send_test_packet();
  if (!(PORTA.IN & BTN_PIN_bm)) {
    for (uint8_t i = 0; i < 4; i++) {
      PORTB.OUTTGL = STATUS_LED_bm;
      _delay_ms(80);
    }
    PORTB.OUTCLR = STATUS_LED_bm;
  }
  _delay_ms(700);
}

int main(void) {
  setup();
  while (1) loop();
  return 0;
}
