#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define PROBE_SENSE   PIN_PA6
#define PROBE_DRIVE   PIN_PA7
#define TX_DATA       PIN_PA1
#define MANUAL_BUTTON PIN_PA2
#define TEST_LED      PIN_PA3

#define BIT_US 1000
#define PACKET_GAP_MS 35
#define REPEATS_PER_PACKET 4
#define MAX_PACKET_LEN 16

#define WAKE_TICK_MS 8000UL
#define DRY_REPORT_WAKE_COUNT 5
#define DRY_RETRY_WAKE_INTERVAL 150
#define OVERFLOW_REPORT_WAKE_COUNT 10
#define MANUAL_ENTER_HOLD_MS 5000UL
#define FIRST_POWER_HOLD_MS 10000UL
#define SHORT_PRESS_MAX_MS 1500UL
#define REQUIRE_FIRST_POWER_ACTIVATION 1

#define FLAG_MOTOR_SHOULD_RUN  0x01
#define FLAG_MOTOR_SHOULD_STOP 0x02
#define FLAG_MANUAL_MODE       0x04
#define FLAG_MANUAL_TOGGLE     0x08
#define FLAG_MANUAL_KEEPALIVE  0x10
#define FLAG_BOOT              0x20
#define FLAG_LEVEL_CHANGED     0x40
#define FLAG_LOW_BATTERY       0x80

static const uint64_t AUTH_KEY = 0x7b3a91d04c8e25f1ULL;

#define ADC_LEVEL_1 160
#define ADC_LEVEL_2 340
#define ADC_LEVEL_3 520
#define ADC_LEVEL_4 700
#define ADC_LEVEL_5 850
#define ADC_SAMPLES 9
#define STABLE_REQUIRED_READS 2

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

enum TxMode : uint8_t { MODE_AUTO, MODE_MANUAL_SLEEP, MODE_MANUAL_ON };

uint8_t sequenceId = 0;
uint8_t reportedLevel = 255;
uint8_t candidateLevel = 255;
uint8_t candidateCount = 0;
uint8_t dryReportRemaining = 0;
uint8_t overflowReportRemaining = 0;
uint16_t dryRetryCounter = 0;
bool activated = (REQUIRE_FIRST_POWER_ACTIVATION == 0);
bool bootFlagPending = true;
bool autoFilling = false;
TxMode txMode = MODE_AUTO;

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;
}

static bool button_pressed() {
  return digitalRead(MANUAL_BUTTON) == LOW;
}

static bool button_held(uint32_t holdMs) {
  if (!button_pressed()) return false;
  uint32_t start = millis();
  while (button_pressed()) {
    if ((millis() - start) >= holdMs) return true;
    delay(20);
  }
  return false;
}

static bool button_short_press() {
  if (!button_pressed()) return false;
  uint32_t start = millis();
  while (button_pressed()) {
    if ((millis() - start) > SHORT_PRESS_MAX_MS) return false;
    delay(20);
  }
  return true;
}

static uint64_t rotl64(uint64_t value, uint8_t shift) {
  return (value << shift) | (value >> (64 - shift));
}

static uint64_t read64le(const uint8_t *p) {
  uint64_t value = 0;
  for (uint8_t i = 0; i < 8; i++) value |= ((uint64_t)p[i]) << (8 * i);
  return value;
}

static void write64le(uint8_t *out, uint64_t value) {
  for (uint8_t i = 0; i < 8; i++) out[i] = (uint8_t)(value >> (8 * i));
}

static void sip_round(uint64_t &v0, uint64_t &v1, uint64_t &v2, uint64_t &v3) {
  v0 += v1; v1 = rotl64(v1, 13); v1 ^= v0; v0 = rotl64(v0, 32);
  v2 += v3; v3 = rotl64(v3, 16); v3 ^= v2;
  v0 += v3; v3 = rotl64(v3, 21); v3 ^= v0;
  v2 += v1; v1 = rotl64(v1, 17); v1 ^= v2; v2 = rotl64(v2, 32);
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
    v3 ^= m; sip_round(v0, v1, v2, v3); sip_round(v0, v1, v2, v3); v0 ^= m;
    offset += 8;
  }
  uint64_t b = ((uint64_t)length) << 56;
  for (uint8_t i = 0; i < (length - offset); i++) b |= ((uint64_t)message[offset + i]) << (8 * i);
  v3 ^= b; sip_round(v0, v1, v2, v3); sip_round(v0, v1, v2, v3); v0 ^= b;
  v2 ^= 0xff;
  sip_round(v0, v1, v2, v3); sip_round(v0, v1, v2, v3);
  sip_round(v0, v1, v2, v3); sip_round(v0, v1, v2, v3);
  return v0 ^ v1 ^ v2 ^ v3;
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
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

static uint16_t read_probe_adc() {
  digitalWrite(PROBE_DRIVE, HIGH);
  delayMicroseconds(80);
  uint16_t adc = analogRead(PROBE_SENSE);
  digitalWrite(PROBE_DRIVE, LOW);
  return adc;
}

static uint8_t classify_adc(uint16_t adc) {
  if (adc >= ADC_LEVEL_5) return 5;
  if (adc >= ADC_LEVEL_4) return 4;
  if (adc >= ADC_LEVEL_3) return 3;
  if (adc >= ADC_LEVEL_2) return 2;
  if (adc >= ADC_LEVEL_1) return 1;
  return 0;
}

static uint16_t median_adc() {
  uint16_t values[ADC_SAMPLES];
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    values[i] = read_probe_adc();
    delayMicroseconds(500);
  }
  for (uint8_t i = 1; i < ADC_SAMPLES; i++) {
    uint16_t v = values[i];
    int8_t j = i - 1;
    while (j >= 0 && values[j] > v) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = v;
  }
  return values[ADC_SAMPLES / 2];
}

static uint8_t read_filtered_level(bool *changed) {
  uint8_t instantLevel = classify_adc(median_adc());
  if (reportedLevel == 255) {
    reportedLevel = instantLevel;
    candidateLevel = instantLevel;
    candidateCount = STABLE_REQUIRED_READS;
    *changed = true;
    return reportedLevel;
  }
  if (instantLevel != candidateLevel) {
    candidateLevel = instantLevel;
    candidateCount = 1;
  } else if (candidateCount < STABLE_REQUIRED_READS) {
    candidateCount++;
  }
  if (candidateCount >= STABLE_REQUIRED_READS && reportedLevel != candidateLevel) {
    reportedLevel = candidateLevel;
    *changed = true;
  } else {
    *changed = false;
  }
  return reportedLevel;
}

static uint8_t read_battery_code() {
  return 0;
}

static void send_payload(uint8_t level, uint8_t flags) {
  uint8_t payload[13];
  uint8_t battery = read_battery_code();
  if (battery != 0 && battery < 60) flags |= FLAG_LOW_BATTERY;
  if (bootFlagPending) flags |= FLAG_BOOT;
  payload[0] = 'W';
  payload[1] = sequenceId++;
  payload[2] = level;
  payload[3] = battery;
  payload[4] = flags;
  write64le(payload + 5, auth64_tag(payload, 5, AUTH_KEY));
  for (uint8_t repeat = 0; repeat < REPEATS_PER_PACKET; repeat++) {
    radio_send(payload, sizeof(payload));
    delay(PACKET_GAP_MS);
  }
  bootFlagPending = false;
}

static uint8_t auto_flags_for_level(uint8_t level, bool levelChanged) {
  uint8_t flags = levelChanged ? FLAG_LEVEL_CHANGED : 0;
  if (level <= 1) {
    flags |= FLAG_MOTOR_SHOULD_RUN;
    autoFilling = (level == 1);
  } else if (level >= 4) {
    flags |= FLAG_MOTOR_SHOULD_STOP;
    autoFilling = false;
  } else if (autoFilling) {
    flags |= FLAG_MOTOR_SHOULD_RUN;
  }
  return flags;
}

static void service_auto() {
  if (button_held(MANUAL_ENTER_HOLD_MS)) {
    txMode = MODE_MANUAL_SLEEP;
    digitalWrite(PROBE_DRIVE, LOW);
    return;
  }
  bool levelChanged = false;
  uint8_t level = read_filtered_level(&levelChanged);
  if (levelChanged) {
    dryRetryCounter = 0;
    dryReportRemaining = (level == 0) ? DRY_REPORT_WAKE_COUNT : 0;
    overflowReportRemaining = (level == 5) ? OVERFLOW_REPORT_WAKE_COUNT : 0;
  }
  bool shouldSend = levelChanged || bootFlagPending;
  if (level == 0 && dryReportRemaining > 0) {
    dryReportRemaining--;
    shouldSend = true;
  } else if (level == 0 && ++dryRetryCounter >= DRY_RETRY_WAKE_INTERVAL) {
    dryRetryCounter = 0;
    shouldSend = true;
  }
  if (level == 5 && overflowReportRemaining > 0) {
    overflowReportRemaining--;
    shouldSend = true;
  }
  if (autoFilling && level >= 1 && level <= 3) shouldSend = true;
  if (shouldSend) send_payload(level, auto_flags_for_level(level, levelChanged));
  delay(WAKE_TICK_MS);
}

static void service_manual_sleep() {
  digitalWrite(PROBE_DRIVE, LOW);
  if (button_held(MANUAL_ENTER_HOLD_MS)) {
    reportedLevel = 255;
    bootFlagPending = true;
    txMode = MODE_AUTO;
    delay(300);
  } else if (button_short_press()) {
    send_payload(0, FLAG_MANUAL_MODE | FLAG_MANUAL_TOGGLE);
    txMode = MODE_MANUAL_ON;
    delay(300);
  } else {
    delay(50);
  }
}

static void service_manual_on() {
  digitalWrite(PROBE_DRIVE, LOW);
  if (button_short_press()) {
    send_payload(0, FLAG_MANUAL_MODE | FLAG_MANUAL_TOGGLE);
    txMode = MODE_MANUAL_SLEEP;
    delay(300);
    return;
  }
  if (button_held(MANUAL_ENTER_HOLD_MS)) {
    send_payload(0, FLAG_MANUAL_MODE | FLAG_MANUAL_TOGGLE);
    txMode = MODE_AUTO;
    reportedLevel = 255;
    bootFlagPending = true;
    delay(300);
    return;
  }
  send_payload(0, FLAG_MANUAL_MODE | FLAG_MANUAL_KEEPALIVE);
  delay(WAKE_TICK_MS);
}

void setup() {
  wdt_disable();
  set_clock_full_speed();
  pinMode(PROBE_DRIVE, OUTPUT);
  digitalWrite(PROBE_DRIVE, LOW);
  pinMode(PROBE_SENSE, INPUT);
  pinMode(TX_DATA, OUTPUT);
  pinMode(TEST_LED, OUTPUT);
  pinMode(MANUAL_BUTTON, INPUT_PULLUP);
  tx_write(false);
}

void loop() {
  if (!activated) {
    digitalWrite(PROBE_DRIVE, LOW);
    if (button_held(FIRST_POWER_HOLD_MS)) {
      activated = true;
      reportedLevel = 255;
      bootFlagPending = true;
      delay(300);
    } else {
      delay(100);
    }
    return;
  }
  if (txMode == MODE_MANUAL_SLEEP) service_manual_sleep();
  else if (txMode == MODE_MANUAL_ON) service_manual_on();
  else service_auto();
}
