#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define TX_DATA  PIN_PA1
#define TEST_LED PIN_PA3

#define BIT_US 1000
#define PACKET_INTERVAL_MS 800
#define REPEATS_PER_PACKET 4
#define MAX_PACKET_LEN 16

static const uint64_t AUTH_KEY = 0x7b3a91d04c8e25f1ULL;

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

uint8_t sequenceId = 0;

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;
}

static uint64_t rotl64(uint64_t value, uint8_t shift) {
  return (value << shift) | (value >> (64 - shift));
}

static uint64_t read64le(const uint8_t *p) {
  uint64_t value = 0;
  for (uint8_t i = 0; i < 8; i++) {
    value |= ((uint64_t)p[i]) << (8 * i);
  }
  return value;
}

static void write64le(uint8_t *out, uint64_t value) {
  for (uint8_t i = 0; i < 8; i++) {
    out[i] = (uint8_t)(value >> (8 * i));
  }
}

static void sip_round(uint64_t &v0, uint64_t &v1, uint64_t &v2, uint64_t &v3) {
  v0 += v1;
  v1 = rotl64(v1, 13);
  v1 ^= v0;
  v0 = rotl64(v0, 32);
  v2 += v3;
  v3 = rotl64(v3, 16);
  v3 ^= v2;
  v0 += v3;
  v3 = rotl64(v3, 21);
  v3 ^= v0;
  v2 += v1;
  v1 = rotl64(v1, 17);
  v1 ^= v2;
  v2 = rotl64(v2, 32);
}

static uint64_t auth64_tag(const uint8_t *message, uint8_t length, uint64_t key) {
  uint64_t k0 = key;
  uint64_t k1 = rotl64(key ^ 0xa5a5a5a55a5a5a5aULL, 17);
  uint64_t v0 = 0x736f6d6570736575ULL ^ k0;
  uint64_t v1 = 0x646f72616e646f6dULL ^ k1;
  uint64_t v2 = 0x6c7967656e657261ULL ^ k0;
  uint64_t v3 = 0x7465646279746573ULL ^ k1;

  uint8_t offset = 0;
  while ((length - offset) >= 8) {
    uint64_t m = read64le(message + offset);
    v3 ^= m;
    sip_round(v0, v1, v2, v3);
    sip_round(v0, v1, v2, v3);
    v0 ^= m;
    offset += 8;
  }

  uint64_t b = ((uint64_t)length) << 56;
  for (uint8_t i = 0; i < (length - offset); i++) {
    b |= ((uint64_t)message[offset + i]) << (8 * i);
  }

  v3 ^= b;
  sip_round(v0, v1, v2, v3);
  sip_round(v0, v1, v2, v3);
  v0 ^= b;
  v2 ^= 0xff;
  sip_round(v0, v1, v2, v3);
  sip_round(v0, v1, v2, v3);
  sip_round(v0, v1, v2, v3);
  sip_round(v0, v1, v2, v3);
  return v0 ^ v1 ^ v2 ^ v3;
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
  }
  return crc;
}

static void tx_write(bool value) {
  digitalWrite(TX_DATA, value ? HIGH : LOW);
  digitalWrite(TEST_LED, value ? HIGH : LOW);
}

static void send_symbol(uint8_t symbol) {
  for (uint8_t bit = 0; bit < 6; bit++) {
    tx_write((symbol & (1 << bit)) != 0);
    delayMicroseconds(BIT_US);
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

static void build_auth_packet(uint8_t *payload, uint8_t sequence) {
  payload[0] = 'A';
  payload[1] = 'U';
  payload[2] = 'T';
  payload[3] = sequence;
  write64le(payload + 4, auth64_tag(payload, 4, AUTH_KEY));
}

void setup() {
  wdt_disable();
  set_clock_full_speed();
  pinMode(TX_DATA, OUTPUT);
  pinMode(TEST_LED, OUTPUT);
  tx_write(false);
}

void loop() {
  uint8_t payload[12];
  build_auth_packet(payload, sequenceId);

  for (uint8_t repeat = 0; repeat < REPEATS_PER_PACKET; repeat++) {
    radio_send(payload, sizeof(payload));
    delay(35);
  }

  sequenceId++;
  delay(PACKET_INTERVAL_MS);
}
