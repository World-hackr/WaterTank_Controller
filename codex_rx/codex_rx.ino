#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/delay.h>

// --- Register Pin Definitions (ATtiny402) ---
#define RX_DATA_PIN_bm   PIN1_bm // PA1 (Pin 4) - RF input
#define PAIR_BTN_PIN_bm  PIN0_bm // PA0 / UPDI - Pairing button input, active LOW
#define RELAY_PIN_bm     PIN2_bm // PA2 (Pin 5) - Pump Relay Control

// Charlieplexed LED matrix pins
#define LINE_A_bm        PIN6_bm // PA6 (Pin 2)
#define LINE_B_bm        PIN7_bm // PA7 (Pin 3)
#define LINE_C_bm        PIN3_bm // PA3 (Pin 7)

// --- Protocol Constants ---
#define SAMPLE_RATE_HZ 8000UL
#define RAMP_LEN 160
#define RAMP_INC 20
#define RAMP_TRANSITION 80
#define RAMP_INC_RETARD 11
#define RAMP_INC_ADVANCE 29
#define START_SYMBOL 0x0b38
#define MAX_PACKET_LEN 16

#define MSG_AUTO_REPORT   0x01
#define MSG_MANUAL_ON     0x02
#define MSG_MANUAL_OFF    0x03
#define MSG_MANUAL_KEEP   0x05

#define FLAG_PUMP_DESIRED      0x01
#define FLAG_DRY_WARNING       0x40
#define FLAG_OVERFLOW_WARNING  0x80

// Timeouts
#define FILL_KEEP_TIMEOUT_MS       24000UL   // 3 missed packets (24 seconds)
#define PAIRING_HOLD_MS            5000UL
#define SEQUENCE_RESYNC_MS         16000UL   // Debug value; use 120000UL for production
#define SEQUENCE_WINDOW            100
#define BATTERY_LOW_CODE           100       // Threshold adjusted to ~2.8V VDD

#define PAIR_MAGIC_0 0xC3
#define PAIR_MAGIC_1 0x3C

#define EE_ID_ADDR  0x00 // 3 bytes for paired Transmitter ID
#define EE_KEY_ADDR 0x03 // 16 bytes for derived Unique Key
#define EE_VALID_ADDR 0x13
#define EE_VALID_VALUE 0xA5

// XTEA 128-bit Master Key (used only during pairing)
static const uint32_t MASTER_KEY[4] = {0x7b3a91d0, 0x4c8e25f1, 0x12345678, 0x9abcdef0};

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

volatile uint8_t integrator = 0, pllRamp = 0, rxBitCount = 0, rxCount = 0, rxLen = 0;
volatile bool lastSample = false, rxActive = false, packetReady = false;
volatile uint16_t rxBits = 0;
volatile uint8_t rxBuf[MAX_PACKET_LEN];

uint8_t currentProbeMask = 0;
uint8_t lastSequence = 0;
uint8_t batteryCode = 0;

// Pairing ID
uint8_t pairedId[3] = {0xFF, 0xFF, 0xFF};

// Active XTEA Key (loaded from EEPROM or Master Key)
uint32_t activeKey[4];

// State flags
bool relayOn = false;
bool havePacket = false;
bool inPairingMode = false;
uint8_t pairingCandidate[8];
uint8_t pairingCandidateCount = 0;

// Timers
uint32_t lastGoodPacketMs = 0;

// Button tracker
uint32_t buttonLowStartMs = 0;
bool buttonHeldActive = false;

// XTEA decryption algorithm
static void xtea_decrypt(uint32_t num_rounds, uint32_t v[2], uint32_t const k[4]) {
  uint32_t i;
  uint32_t v0 = v[0], v1 = v[1], delta = 0x9E3779B9, sum = delta * num_rounds;
  for (i = 0; i < num_rounds; i++) {
    v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum >> 11) & 3]);
    sum -= delta;
    v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
  }
  v[0] = v0; v[1] = v1;
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
  PORTA.DIRCLR = LINE_A_bm | LINE_B_bm | LINE_C_bm;
  PORTA.OUTCLR = LINE_A_bm | LINE_B_bm | LINE_C_bm;
}

static void drive_led(uint8_t led) {
  leds_off();
  switch (led) {
    case 1: PORTA.DIRSET = LINE_A_bm | LINE_B_bm; PORTA.OUTSET = LINE_A_bm; PORTA.OUTCLR = LINE_B_bm; break;
    case 2: PORTA.DIRSET = LINE_A_bm | LINE_B_bm; PORTA.OUTSET = LINE_B_bm; PORTA.OUTCLR = LINE_A_bm; break;
    case 3: PORTA.DIRSET = LINE_B_bm | LINE_C_bm; PORTA.OUTSET = LINE_B_bm; PORTA.OUTCLR = LINE_C_bm; break;
    case 4: PORTA.DIRSET = LINE_B_bm | LINE_C_bm; PORTA.OUTSET = LINE_C_bm; PORTA.OUTCLR = LINE_B_bm; break;
    case 5: PORTA.DIRSET = LINE_A_bm | LINE_C_bm; PORTA.OUTSET = LINE_A_bm; PORTA.OUTCLR = LINE_C_bm; break;
    case 6: PORTA.DIRSET = LINE_A_bm | LINE_C_bm; PORTA.OUTSET = LINE_C_bm; PORTA.OUTCLR = LINE_A_bm; break;
  }
}

ISR(TCB0_INT_vect) {
  bool sample = (PORTA.IN & RX_DATA_PIN_bm) != 0;
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

static void update_relay() {
  if (inPairingMode) {
    PORTA.OUTCLR = RELAY_PIN_bm;
    return;
  }

  if (relayOn) {
    PORTA.OUTSET = RELAY_PIN_bm;
  } else {
    PORTA.OUTCLR = RELAY_PIN_bm;
  }
}

static void trigger_pairing_confirmation_blinks() {
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t led = 1; led <= 6; led++) drive_led(led);
    _delay_ms(150);
    leds_off();
    _delay_ms(150);
  }
}

static void derive_unique_key_from_pair_identity(uint8_t *sn, uint32_t *destKey) {
  uint8_t *dk = (uint8_t*)destKey;
  const uint8_t *mk = (const uint8_t*)MASTER_KEY;

  dk[0] = mk[0] ^ sn[3]; dk[1] = mk[1] ^ sn[2]; dk[2] = mk[2] ^ sn[1]; dk[3] = mk[3] ^ sn[0];
  dk[4] = mk[4] ^ sn[7]; dk[5] = mk[5] ^ sn[6]; dk[6] = mk[6] ^ sn[5]; dk[7] = mk[7] ^ sn[4];
  dk[8] = mk[8] ^ (sn[3] ^ sn[7]); dk[9] = mk[9] ^ (sn[2] ^ sn[6]);
  dk[10] = mk[10] ^ (sn[1] ^ sn[5]); dk[11] = mk[11] ^ (sn[0] ^ sn[4]);

  uint16_t sum0 = (uint16_t)sn[3] + sn[7];
  dk[12] = mk[12] ^ (uint8_t)sum0;
  uint16_t sum1 = (uint16_t)sn[2] + sn[6] + (sum0 >> 8);
  dk[13] = mk[13] ^ (uint8_t)sum1;
  uint16_t sum2 = (uint16_t)sn[1] + sn[5] + (sum1 >> 8);
  dk[14] = mk[14] ^ (uint8_t)sum2;
  uint16_t sum3 = (uint16_t)sn[0] + sn[4] + (sum2 >> 8);
  dk[15] = mk[15] ^ (uint8_t)sum3;
}

static void update_display() {
  uint32_t now = millis();

  // 1. Circular animation during pairing
  if (inPairingMode) {
    uint8_t led = 1 + ((now / 150) % 6);
    drive_led(led);
    _delay_ms(15);
    return;
  }

  // 2. Keep display completely dark if no packet has been successfully received yet
  if (!havePacket) {
    if ((now / 700) % 2 == 0) drive_led(6);
    else leds_off();
    _delay_ms(15);
    return;
  }

  bool batteryLow = batteryCode >= BATTERY_LOW_CODE;

  if (currentProbeMask == 0) {
    bool dryBlinkOn = (now / 150) % 2 == 0;
    if (dryBlinkOn) {
      drive_led(1);
      _delay_ms(2);
    }
  } else {
    if (currentProbeMask & 0x01) {
      drive_led(2);
      _delay_ms(2);
    }
    if (currentProbeMask & 0x02) {
      drive_led(3);
      _delay_ms(2);
    }
    if (currentProbeMask & 0x04) {
      drive_led(4);
      _delay_ms(2);
    }
    if ((currentProbeMask & 0x08) && ((now / 150) % 2 == 0)) {
      drive_led(5);
      _delay_ms(2);
    }
  }

  if (!batteryLow || ((now / 700) % 2 == 0)) {
    drive_led(6);
    _delay_ms(2);
  }

  leds_off();
}

static bool sequence_allowed(uint8_t seq, uint8_t msgType, uint32_t now) {
  if (!havePacket) return true;
  uint8_t diff = seq - lastSequence;
  if (diff == 0) return false;
  if (msgType == MSG_MANUAL_ON || msgType == MSG_MANUAL_OFF || msgType == MSG_MANUAL_KEEP) return true;
  if (now - lastGoodPacketMs > SEQUENCE_RESYNC_MS) return true;
  return diff <= SEQUENCE_WINDOW;
}

void setup() {
  wdt_disable();
  
  // Configure PA2 (Relay) as output
  PORTA.DIRSET = RELAY_PIN_bm;
  PORTA.OUTCLR = RELAY_PIN_bm;
  
  leds_off();
  
  // Configure PA1 (RF Data / button) as input
  PORTA.DIRCLR = RX_DATA_PIN_bm;
  PORTA.DIRCLR = PAIR_BTN_PIN_bm;
  PORTA.PIN0CTRL = PORT_PULLUPEN_bm;
  
  // EEPROM reads
  uint8_t pairingValid = eeprom_read_byte((const uint8_t*)EE_VALID_ADDR);
  pairedId[0] = eeprom_read_byte((const uint8_t*)(EE_ID_ADDR + 0));
  pairedId[1] = eeprom_read_byte((const uint8_t*)(EE_ID_ADDR + 1));
  pairedId[2] = eeprom_read_byte((const uint8_t*)(EE_ID_ADDR + 2));

  // If EEPROM is invalid or partly erased, enter pairing mode immediately.
  if (pairingValid != EE_VALID_VALUE || pairedId[0] == 0xFF || pairedId[1] == 0xFF || pairedId[2] == 0xFF) {
    inPairingMode = true;
    for (uint8_t i = 0; i < 4; i++) activeKey[i] = MASTER_KEY[i];
  } else {
    // Load derived Unique Key from EEPROM
    eeprom_read_block((void*)activeKey, (const void*)EE_KEY_ADDR, 16);
  }

  timer_setup();
  sei();
}

void loop() {
  uint32_t now = millis();

  // 1. Check pairing button on PA0 / UPDI. Input only, active LOW.
  if (!(PORTA.IN & PAIR_BTN_PIN_bm)) {
    if (buttonLowStartMs == 0) {
      buttonLowStartMs = now;
    } else if ((now - buttonLowStartMs > PAIRING_HOLD_MS) && !buttonHeldActive) {
      buttonHeldActive = true;
      
      eeprom_write_byte((uint8_t*)EE_VALID_ADDR, 0x00);
      eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 0), 0xFF);
      eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 1), 0xFF);
      eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 2), 0xFF);
      pairedId[0] = 0xFF; pairedId[1] = 0xFF; pairedId[2] = 0xFF;
      relayOn = false;

      for (uint8_t i = 0; i < 4; i++) activeKey[i] = MASTER_KEY[i];

      leds_off();
      _delay_ms(200);
      inPairingMode = true;
    }
  } else {
    buttonLowStartMs = 0;
    buttonHeldActive = false;
  }

  // 2. Receive and process radio packets
  uint8_t ciphertext[8];
  uint8_t len = sizeof(ciphertext);

  if (radio_recv(ciphertext, &len)) {
    if (len == 8) {
      // Decrypt the XTEA block in-place using current activeKey
      xtea_decrypt(32, (uint32_t*)ciphertext, activeKey);
      
      if (inPairingMode) {
        bool validPairing = (ciphertext[0] == PAIR_MAGIC_0 && ciphertext[1] == PAIR_MAGIC_1);
        bool sameCandidate = validPairing;
        if (validPairing) {
          for (uint8_t i = 0; i < 8; i++) {
            if (pairingCandidate[i] != ciphertext[i]) sameCandidate = false;
          }
          if (!sameCandidate) {
            for (uint8_t i = 0; i < 8; i++) pairingCandidate[i] = ciphertext[i];
            pairingCandidateCount = 1;
          } else if (pairingCandidateCount < 2) {
            pairingCandidateCount++;
          }
        }

        if (validPairing && pairingCandidateCount >= 2) {
          // 1. Calculate the same Unique Key byte-for-byte as the TX.
          derive_unique_key_from_pair_identity(ciphertext, activeKey);

          // 2. Save pairing data. Valid marker is written last so partial writes are rejected after reset.
          eeprom_write_byte((uint8_t*)EE_VALID_ADDR, 0x00);
          eeprom_write_block((const void*)activeKey, (void*)EE_KEY_ADDR, 16);

          // 3. Extract the 3-byte binding ID (last 3 bytes of Silicon ID)
          uint8_t rxId[3] = {ciphertext[5], ciphertext[6], ciphertext[7]};

          // 4. Save Binding ID to EEPROM
          eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 0), rxId[0]);
          eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 1), rxId[1]);
          eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 2), rxId[2]);
          eeprom_write_byte((uint8_t*)EE_VALID_ADDR, EE_VALID_VALUE);
          pairedId[0] = rxId[0]; pairedId[1] = rxId[1]; pairedId[2] = rxId[2];
          
          inPairingMode = false;
          pairingCandidateCount = 0;
          trigger_pairing_confirmation_blinks();
        }
      } else {
        // Normal Operation: verify transmitter ID and sequence
        uint8_t msgType = ciphertext[0];
        uint8_t seq = ciphertext[1];
        uint8_t probeMask = ciphertext[2] & 0x0f;
        uint8_t flags = ciphertext[4];
        uint8_t rxId[3] = {ciphertext[5], ciphertext[6], ciphertext[7]};

        if (rxId[0] == pairedId[0] && rxId[1] == pairedId[1] && rxId[2] == pairedId[2]) {
          
          if (sequence_allowed(seq, msgType, now)) {
            // Accept packet
            havePacket = true;
            lastSequence = seq;
            lastGoodPacketMs = now;
            currentProbeMask = probeMask;
            batteryCode = ciphertext[3];

            // Process commands
            if (msgType == MSG_MANUAL_ON) {
              relayOn = true;
            } else if (msgType == MSG_MANUAL_OFF) {
              relayOn = false;
            } else if (msgType == MSG_MANUAL_KEEP) {
              relayOn = true;
            } else if (msgType == MSG_AUTO_REPORT) {
              relayOn = (flags & FLAG_PUMP_DESIRED) != 0;
            }
          }
        }
      }
    }
  }

  // 3. Safety timers
  if (relayOn && (now - lastGoodPacketMs > FILL_KEEP_TIMEOUT_MS)) {
    relayOn = false;
  }

  update_relay();
  update_display();
}
