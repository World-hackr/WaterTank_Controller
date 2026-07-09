#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>

#define RX_DATA   PIN_PA1
#define RELAY_PIN PIN_PA2
#define LINE_A    PIN_PA6
#define LINE_B    PIN_PA7
#define LINE_C    PIN_PA3

#define SAMPLE_RATE_HZ 8000UL
#define RAMP_LEN 160
#define RAMP_INC 20
#define RAMP_TRANSITION 80
#define RAMP_INC_RETARD 11
#define RAMP_INC_ADVANCE 29
#define START_SYMBOL 0x0b38
#define MAX_PACKET_LEN 16
#define RF_MISSED_FILL_MS 24000UL
#define MANUAL_MAX_RUN_MS 600000UL

#define FLAG_MOTOR_SHOULD_RUN  0x01
#define FLAG_MOTOR_SHOULD_STOP 0x02
#define FLAG_MANUAL_MODE       0x04
#define FLAG_MANUAL_TOGGLE     0x08
#define FLAG_MANUAL_KEEPALIVE  0x10
#define FLAG_BOOT              0x20
#define FLAG_LEVEL_CHANGED     0x40
#define FLAG_LOW_BATTERY       0x80

static const uint64_t AUTH_KEY = 0x7b3a91d04c8e25f1ULL;

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

volatile uint8_t integrator = 0, pllRamp = 0, rxBitCount = 0, rxCount = 0, rxLen = 0;
volatile bool lastSample = false, rxActive = false, packetReady = false;
volatile uint16_t rxBits = 0;
volatile uint8_t rxBuf[MAX_PACKET_LEN];

uint8_t currentLevel = 0;
uint8_t lastSequence = 0;
uint8_t packetFlags = 0;
bool relayOn = false;
bool havePacket = false;
bool autoFilling = false;
bool manualActive = false;
bool rfLostWhileFilling = false;
uint32_t lastGoodPacketMs = 0;
uint32_t lastSequenceMs = 0;
uint32_t manualStartedMs = 0;

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;
}

static uint64_t rotl64(uint64_t value, uint8_t shift) {
  return (value << shift) | (value >> (64 - shift));
}

static uint64_t read64le(const uint8_t *p) {
  uint64_t value = 0;
  for (uint8_t i = 0; i < 8; i++) value |= ((uint64_t)p[i]) << (8 * i);
  return value;
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

static uint8_t symbol_6to4(uint8_t symbol) {
  for (uint8_t i = 0; i < 16; i++) if (symbol == symbols[i]) return i;
  return 0xff;
}

static void reset_rx_message() {
  rxActive = false;
  rxBitCount = 0;
  rxCount = 0;
  rxLen = 0;
}

static void timer_setup() {
  uint16_t compareValue = (uint16_t)((F_CPU / SAMPLE_RATE_HZ) - 1);
  TCB0.CTRLA = 0;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CCMP = compareValue;
  TCB0.CNT = 0;
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

static void leds_off() {
  PORTA.DIRCLR = PIN6_bm | PIN7_bm | PIN3_bm;
  PORTA.OUTCLR = PIN6_bm | PIN7_bm | PIN3_bm;
}

static void drive_led(uint8_t led) {
  leds_off();
  switch (led) {
    case 1: PORTA.DIRSET = PIN6_bm | PIN7_bm; PORTA.OUTSET = PIN6_bm; PORTA.OUTCLR = PIN7_bm; break;
    case 2: PORTA.DIRSET = PIN6_bm | PIN7_bm; PORTA.OUTSET = PIN7_bm; PORTA.OUTCLR = PIN6_bm; break;
    case 3: PORTA.DIRSET = PIN7_bm | PIN3_bm; PORTA.OUTSET = PIN7_bm; PORTA.OUTCLR = PIN3_bm; break;
    case 4: PORTA.DIRSET = PIN7_bm | PIN3_bm; PORTA.OUTSET = PIN3_bm; PORTA.OUTCLR = PIN7_bm; break;
    case 5: PORTA.DIRSET = PIN6_bm | PIN3_bm; PORTA.OUTSET = PIN6_bm; PORTA.OUTCLR = PIN3_bm; break;
    case 6: PORTA.DIRSET = PIN6_bm | PIN3_bm; PORTA.OUTSET = PIN3_bm; PORTA.OUTCLR = PIN6_bm; break;
  }
}

ISR(TCB0_INT_vect) {
  bool sample = (PORTA.IN & PIN1_bm) != 0;
  if (sample) integrator++;
  if (sample != lastSample) {
    pllRamp += (pllRamp < RAMP_TRANSITION) ? RAMP_INC_RETARD : RAMP_INC_ADVANCE;
    lastSample = sample;
  } else {
    pllRamp += RAMP_INC;
  }
  if (pllRamp >= RAMP_LEN) {
    rxBits >>= 1;
    if (integrator >= 5) rxBits |= 0x0800;
    pllRamp -= RAMP_LEN;
    integrator = 0;
    if (packetReady) {
    } else if (rxActive) {
      if (++rxBitCount >= 12) {
        uint8_t hi = symbol_6to4(rxBits & 0x3f);
        uint8_t lo = symbol_6to4(rxBits >> 6);
        if (hi == 0xff || lo == 0xff) {
          reset_rx_message();
        } else {
          uint8_t value = (hi << 4) | lo;
          if (rxLen == 0) {
            rxCount = value;
            if (rxCount < 4 || rxCount > MAX_PACKET_LEN) reset_rx_message();
          }
          if (rxActive) {
            rxBuf[rxLen++] = value;
            if (rxLen >= rxCount) {
              packetReady = true;
              reset_rx_message();
            }
          }
        }
        rxBitCount = 0;
      }
    } else if (rxBits == START_SYMBOL) {
      rxActive = true;
      rxBitCount = 0;
      rxLen = 0;
      rxCount = 0;
    }
  }
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

static bool radio_recv(uint8_t *payload, uint8_t *len) {
  uint8_t packet[MAX_PACKET_LEN];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (!packetReady) return false;
    for (uint8_t i = 0; i < MAX_PACKET_LEN; i++) packet[i] = rxBuf[i];
    packetReady = false;
  }
  uint8_t count = packet[0];
  if (count < 4 || count > MAX_PACKET_LEN) return false;
  uint16_t crc = 0xffff;
  for (uint8_t i = 0; i < count; i++) crc = crc16_update(crc, packet[i]);
  if (crc != 0xf0b8) return false;
  uint8_t payloadLen = count - 3;
  if (*len > payloadLen) *len = payloadLen;
  for (uint8_t i = 0; i < *len; i++) payload[i] = packet[i + 1];
  return true;
}

static bool verify_payload(const uint8_t *payload, uint8_t len) {
  if (len != 13 || payload[0] != 'W') return false;
  return auth64_tag(payload, 5, AUTH_KEY) == read64le(payload + 5);
}

static bool sequence_allowed(uint8_t seq, uint8_t flags) {
  uint32_t now = millis();
  if ((flags & FLAG_BOOT) != 0) return true;
  if (!havePacket) return true;
  if (seq == lastSequence) return (now - lastSequenceMs) <= 2000UL;
  uint8_t diff = seq - lastSequence;
  return diff > 0 && diff < 128;
}

static void set_relay(bool on) {
  relayOn = on;
  if (relayOn) PORTA.OUTSET = PIN2_bm;
  else PORTA.OUTCLR = PIN2_bm;
}

static void apply_packet(const uint8_t *payload) {
  uint8_t seq = payload[1];
  uint8_t level = payload[2];
  uint8_t flags = payload[4];
  uint32_t now = millis();
  havePacket = true;
  lastSequence = seq;
  lastSequenceMs = now;
  lastGoodPacketMs = now;
  currentLevel = level > 5 ? 5 : level;
  packetFlags = flags;
  rfLostWhileFilling = false;

  if ((flags & FLAG_MANUAL_MODE) != 0) {
    if ((flags & FLAG_MANUAL_TOGGLE) != 0) {
      manualActive = true;
      if (relayOn) {
        set_relay(false);
        manualActive = false;
      } else {
        set_relay(true);
        manualStartedMs = now;
      }
    } else if ((flags & FLAG_MANUAL_KEEPALIVE) != 0 && manualActive && relayOn) {
      lastGoodPacketMs = now;
    }
    return;
  }

  manualActive = false;
  if ((flags & FLAG_MOTOR_SHOULD_STOP) != 0 || currentLevel >= 4) {
    autoFilling = false;
    set_relay(false);
  } else if ((flags & FLAG_MOTOR_SHOULD_RUN) != 0 && currentLevel <= 3) {
    autoFilling = true;
    set_relay(true);
  }
}

static void service_safety() {
  uint32_t now = millis();
  if (manualActive && relayOn && (now - manualStartedMs) > MANUAL_MAX_RUN_MS) {
    set_relay(false);
    manualActive = false;
  }
  if (!manualActive && autoFilling && relayOn && (now - lastGoodPacketMs) > RF_MISSED_FILL_MS) {
    set_relay(false);
    autoFilling = false;
    rfLostWhileFilling = true;
  }
}

static bool low_battery_overlay_on() {
  if ((packetFlags & FLAG_LOW_BATTERY) == 0) return true;
  return ((millis() / 500UL) & 1) == 0;
}

static void update_display() {
  if (rfLostWhileFilling) {
    leds_off();
    _delay_ms(16);
    return;
  }
  if (!low_battery_overlay_on()) {
    leds_off();
    _delay_ms(16);
    return;
  }

  if (currentLevel == 0) {
    if (((millis() / 250UL) & 1) == 0) drive_led(1);
    else leds_off();
    _delay_ms(3);
    leds_off();
    _delay_ms(13);
    return;
  }

  uint8_t level = currentLevel > 5 ? 5 : currentLevel;
  for (uint8_t led = 1; led <= 6; led++) {
    bool on = (led <= level);
    if (currentLevel == 5 && led == 6) on = (((millis() / 250UL) & 1) != 0);
    if (on) drive_led(led);
    else leds_off();
    _delay_ms(2);
  }
  leds_off();
}

void setup() {
  wdt_disable();
  set_clock_full_speed();
  PORTA.DIRSET = PIN2_bm;
  PORTA.OUTCLR = PIN2_bm;
  leds_off();
  pinMode(RX_DATA, INPUT);
  timer_setup();
  sei();
}

void loop() {
  uint8_t payload[13];
  uint8_t len = sizeof(payload);
  if (radio_recv(payload, &len) && verify_payload(payload, len) && sequence_allowed(payload[1], payload[4])) {
    apply_packet(payload);
  }
  service_safety();
  update_display();
}
