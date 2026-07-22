#include <avr/io.h>
#include <util/delay.h>

// --- Pin Settings (ATtiny204) ---
#define TX_DATA_PIN_bm  PIN4_bm // PA4 (Pin 2) - RF Data Out
#define STATUS_LED_bm   PIN0_bm // PB0 (Pin 9) - Local Status LED

#define BIT_US 1000
#define REPEATS_PER_PACKET 4
#define PACKET_GAP_MS 35

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00; // 20MHz clock
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
  }
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
  uint8_t count = 1 + len + 2;
  uint16_t crc = 0xffff;
  crc = crc16_update(crc, count);
  for (uint8_t i = 0; i < len; i++) crc = crc16_update(crc, payload[i]);
  crc = ~crc;

  // Preamble & Sync
  for (uint8_t i = 0; i < 6; i++) send_symbol(0x2a);
  send_symbol(0x38);
  send_symbol(0x2c);

  // Send Data
  send_encoded_byte(count);
  for (uint8_t i = 0; i < len; i++) send_encoded_byte(payload[i]);

  // Send CRC
  send_encoded_byte(crc & 0xff);
  send_encoded_byte(crc >> 8);
  tx_write(false);
}

void setup() {
  set_clock_full_speed();
  PORTA.DIRSET = TX_DATA_PIN_bm;
  PORTA.OUTCLR = TX_DATA_PIN_bm;

  PORTB.DIRSET = STATUS_LED_bm;
  PORTB.OUTCLR = STATUS_LED_bm;
}

uint8_t counter = 0;

void loop() {
  // Toggle status LED to show activity
  PORTB.OUTSET = STATUS_LED_bm;

  // Prepare simple packet: Header (0x99) + Counter (0 to 4)
  uint8_t packet[8];
  packet[0] = 0x99;
  packet[1] = counter;
  for (uint8_t i = 2; i < 8; i++) packet[i] = 0x00;

  // Send 4 times
  for (uint8_t r = 0; r < REPEATS_PER_PACKET; r++) {
    radio_send(packet, 8);
    _delay_ms(PACKET_GAP_MS);
  }

  PORTB.OUTCLR = STATUS_LED_bm;

  // Rotate counter 0 to 15
  counter++;
  if (counter > 15) counter = 0;

  // Wait 1 second before sending again
  _delay_ms(1000);
}

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}
