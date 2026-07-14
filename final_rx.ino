#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/delay.h>

#define RX_DATA   PIN_PA1 // Pin 4 (Shared RF input and pairing button)
#define RELAY_PIN PIN_PA2 // Pin 5
#define LINE_A    PIN_PA6 // Pin 2
#define LINE_B    PIN_PA7 // Pin 3
#define LINE_C    PIN_PA3 // Pin 7

#define SAMPLE_RATE_HZ 8000UL
#define RAMP_LEN 160
#define RAMP_INC 20
#define RAMP_TRANSITION 80
#define RAMP_INC_RETARD 11
#define RAMP_INC_ADVANCE 29
#define START_SYMBOL 0x0b38
#define MAX_PACKET_LEN 16

// Protocol constants
#define MSG_AUTO_REPORT 0x01
#define MSG_MANUAL_CMD  0x02
#define MSG_PAIRING     0x03

// Flags
#define FLAG_LEVEL_CHANGED 0x01
#define FLAG_LOW_BATTERY   0x02
#define FLAG_FILL_TIMEOUT  0x04
#define FLAG_MANUAL_MODE   0x08
#define FLAG_MANUAL_TOGGLE 0x10
#define FLAG_MANUAL_KEEP   0x20
#define FLAG_DRY_WARNING   0x40
#define FLAG_OVERFLOW_WARNING 0x80

// Timeouts
#define RX_FILL_TIMEOUT_DEFAULT_MS 1800000UL // 30 minutes
#define RX_FILL_TIMEOUT_MIN_MS     900000UL  // 15 minutes
#define RX_FILL_TIMEOUT_MAX_MS     3600000UL // 60 minutes
#define MANUAL_MAX_RUNTIME_MS      600000UL  // 10 minutes
#define FILL_KEEP_TIMEOUT_MS       24000UL   // 3 missed packets (24 seconds)

// EEPROM addresses
#define EE_ID_ADDR  0x00 // 3 bytes for paired Transmitter ID (0x00 - 0x02)
#define EE_KEY_ADDR 0x03 // 16 bytes for derived Unique Key (0x03 - 0x12)

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

uint8_t currentLevel = 0;
uint8_t lastSequence = 0;
uint8_t batteryCode = 0;
uint8_t packetFlags = 0;
uint8_t messageType = 0;

// Pairing ID
uint8_t pairedId[3] = {0xFF, 0xFF, 0xFF};

// Active XTEA Key (loaded from EEPROM or Master Key)
uint32_t activeKey[4];

// State flags
bool autoRelayOn = false;
bool manualRelayOn = false;
bool systemFault = false;
bool havePacket = false;
bool inPairingMode = false;

// Timers
uint32_t lastGoodPacketMs = 0;
uint32_t fillStartTimeMs = 0;
uint32_t manualStartTimeMs = 0;
uint32_t allowedFillTimeMs = RX_FILL_TIMEOUT_DEFAULT_MS;
uint32_t learnedFillTimeMs = 0;

// Dual-Purpose Button tracker
uint32_t buttonLowStartMs = 0;
bool buttonHeldActive = false;

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;
}

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

static void update_relay() {
  if (systemFault || inPairingMode) {
    PORTA.OUTCLR = PIN2_bm;
    return;
  }

  if (manualRelayOn || autoRelayOn) {
    PORTA.OUTSET = PIN2_bm;
  } else {
    PORTA.OUTCLR = PIN2_bm;
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

static void update_display() {
  uint32_t now = millis();

  // Pairing Mode circular animation
  if (inPairingMode) {
    uint8_t led = 1 + ((now / 150) % 6);
    drive_led(led);
    _delay_ms(15);
    return;
  }

  if (systemFault) {
    // Flash LED 6 only
    bool blinkState = (now / 150) % 2 == 0;
    if (blinkState) drive_led(6);
    else leds_off();
    _delay_ms(15);
    return;
  }

  bool filling = autoRelayOn || manualRelayOn;
  if (filling && (now - lastGoodPacketMs > FILL_KEEP_TIMEOUT_MS)) {
    // Link Lost mid-filling -> turn OFF pump and clear display
    autoRelayOn = false;
    manualRelayOn = false;
    leds_off();
    return;
  }

  // Low battery background overlay blink
  bool batteryWarning = (packetFlags & FLAG_LOW_BATTERY) != 0;
  bool batteryBlinkOn = true;
  if (batteryWarning) {
    batteryBlinkOn = (now / 500) % 2 == 0;
  }

  if (!batteryBlinkOn) {
    leds_off();
    _delay_ms(15);
    return;
  }

  uint8_t level = currentLevel > 5 ? 5 : currentLevel;

  if (level == 0) {
    // LED 1 blinks slowly
    bool dryBlinkOn = (now / 800) % 2 == 0;
    if (dryBlinkOn) drive_led(1);
    else leds_off();
    _delay_ms(15);
  } else if (level == 5) {
    // LED 1-5 solid, LED 6 blinks out-of-phase
    bool overflowBlinkOn = (now / 800) % 2 == 1;
    for (uint8_t led = 1; led <= 6; led++) {
      if (led == 6) {
        if (overflowBlinkOn) drive_led(6);
        else leds_off();
      } else {
        drive_led(led);
      }
      _delay_ms(2);
    }
    leds_off();
  } else {
    // Normal solid levels
    for (uint8_t led = 1; led <= 5; led++) {
      if (led <= level) drive_led(led);
      else leds_off();
      _delay_ms(2);
    }
    leds_off();
  }
}

void setup() {
  wdt_disable();
  set_clock_full_speed();
  
  PORTA.DIRSET = PIN2_bm;
  PORTA.OUTCLR = PIN2_bm; // Relay start OFF
  
  leds_off();
  pinMode(RX_DATA, INPUT);
  
  // Read Paired ID from EEPROM
  pairedId[0] = eeprom_read_byte((const uint8_t*)(EE_ID_ADDR + 0));
  pairedId[1] = eeprom_read_byte((const uint8_t*)(EE_ID_ADDR + 1));
  pairedId[2] = eeprom_read_byte((const uint8_t*)(EE_ID_ADDR + 2));

  // If EEPROM empty, enter pairing mode immediately
  if (pairedId[0] == 0xFF && pairedId[1] == 0xFF && pairedId[2] == 0xFF) {
    inPairingMode = true;
    // Use factory Master Key for pairing
    for (uint8_t i = 0; i < 4; i++) activeKey[i] = MASTER_KEY[i];
  } else {
    // Load derived Unique Key from EEPROM for normal operation
    eeprom_read_block((void*)activeKey, (const void*)EE_KEY_ADDR, 16);
  }

  timer_setup();
  sei();
}

void loop() {
  uint32_t now = millis();

  // 1. Check for Manual Pairing Button Press on RX_DATA line (PA1)
  if (digitalRead(RX_DATA) == LOW) {
    if (buttonLowStartMs == 0) {
      buttonLowStartMs = now;
    } else if ((now - buttonLowStartMs > 3000UL) && !buttonHeldActive) {
      buttonHeldActive = true;
      // Clear EEPROM Pairing ID
      eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 0), 0xFF);
      eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 1), 0xFF);
      eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 2), 0xFF);
      pairedId[0] = 0xFF; pairedId[1] = 0xFF; pairedId[2] = 0xFF;

      // Force active key to factory Master Key
      for (uint8_t i = 0; i < 4; i++) activeKey[i] = MASTER_KEY[i];

      // Trigger visual indicator and enter Pairing Mode
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
        // Verification of pairing package: contains 8-byte Silicon ID.
        // We verify that the first 2 bytes match the factory signature 0x3055
        if (ciphertext[0] == 0x30 && ciphertext[1] == 0x55) {
          // 1. Calculate the Unique Key dynamically on the RX
          uint32_t serial_high = ((uint32_t)ciphertext[0] << 24) | ((uint32_t)ciphertext[1] << 16) | ((uint32_t)ciphertext[2] << 8) | ciphertext[3];
          uint32_t serial_low  = ((uint32_t)ciphertext[4] << 24) | ((uint32_t)ciphertext[5] << 16) | ((uint32_t)ciphertext[6] << 8) | ciphertext[7];

          activeKey[0] = MASTER_KEY[0] ^ serial_high;
          activeKey[1] = MASTER_KEY[1] ^ serial_low;
          activeKey[2] = MASTER_KEY[2] ^ (serial_high ^ serial_low);
          activeKey[3] = MASTER_KEY[3] ^ (serial_high + serial_low);

          // 2. Save derived Unique Key to EEPROM
          eeprom_write_block((const void*)activeKey, (void*)EE_KEY_ADDR, 16);

          // 3. Extract the 3-byte binding ID (last 3 bytes of Silicon ID: index 5, 6, 7)
          uint8_t rxId[3] = {ciphertext[5], ciphertext[6], ciphertext[7]};

          // 4. Save Binding ID to EEPROM
          eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 0), rxId[0]);
          eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 1), rxId[1]);
          eeprom_write_byte((uint8_t*)(EE_ID_ADDR + 2), rxId[2]);
          pairedId[0] = rxId[0]; pairedId[1] = rxId[1]; pairedId[2] = rxId[2];
          
          inPairingMode = false;
          trigger_pairing_confirmation_blinks();
        }
      } else {
        // Normal Operation: verify transmitter ID and sequence
        uint8_t msgType = ciphertext[0];
        uint8_t seq = ciphertext[1];
        uint8_t level = ciphertext[2];
        uint8_t flags = ciphertext[4];
        uint8_t rxId[3] = {ciphertext[5], ciphertext[6], ciphertext[7]};

        if (rxId[0] == pairedId[0] && rxId[1] == pairedId[1] && rxId[2] == pairedId[2]) {
          
          // Replay check
          uint8_t diff = seq - lastSequence;
          if (havePacket && diff >= 128 && seq != lastSequence) {
            // Out of order packet, reject
          } else {
            // Accept packet
            havePacket = true;
            lastSequence = seq;
            lastGoodPacketMs = now;
            currentLevel = level > 5 ? 5 : level;
            packetFlags = flags;

            // Process Commands
            if (msgType == MSG_MANUAL_CMD) {
              if (flags & FLAG_MANUAL_TOGGLE) {
                manualRelayOn = !manualRelayOn;
                autoRelayOn = false;
                if (manualRelayOn) manualStartTimeMs = now;
              } else if (flags & FLAG_MANUAL_KEEP) {
                if (manualRelayOn) {
                  // Keepalive confirm, do NOT extend 10-minute timer limit
                }
              }
            } else if (msgType == MSG_AUTO_REPORT) {
              if (!manualRelayOn) {
                if (currentLevel >= 5 || (flags & FLAG_OVERFLOW_WARNING)) {
                  autoRelayOn = false;
                } else if (currentLevel <= 1 && !systemFault) {
                  if (!autoRelayOn) {
                    autoRelayOn = true;
                    fillStartTimeMs = now;
                  }
                } else if (currentLevel >= 4) {
                  if (autoRelayOn) {
                    autoRelayOn = false;
                    uint32_t duration = now - fillStartTimeMs;
                    if (learnedFillTimeMs == 0) {
                      learnedFillTimeMs = duration;
                    } else {
                      learnedFillTimeMs = (learnedFillTimeMs * 3 + duration) / 4;
                    }
                    allowedFillTimeMs = learnedFillTimeMs * 2;
                    if (allowedFillTimeMs < RX_FILL_TIMEOUT_MIN_MS) allowedFillTimeMs = RX_FILL_TIMEOUT_MIN_MS;
                    if (allowedFillTimeMs > RX_FILL_TIMEOUT_MAX_MS) allowedFillTimeMs = RX_FILL_TIMEOUT_MAX_MS;
                  }
                }

                if (flags & FLAG_FILL_TIMEOUT) {
                  autoRelayOn = false;
                  systemFault = true;
                }
              }
            }
          }
        }
      }
    }
  }

  // 3. Safety timers
  if (autoRelayOn && (now - fillStartTimeMs > allowedFillTimeMs)) {
    autoRelayOn = false;
    systemFault = true;
  }

  if (manualRelayOn && (now - manualStartTimeMs > MANUAL_MAX_RUNTIME_MS)) {
    manualRelayOn = false;
    systemFault = true;
  }

  // Reset packet history if link is offline for more than 60 seconds (allows remote reboot resync)
  if (havePacket && (now - lastGoodPacketMs > 60000UL)) {
    havePacket = false;
  }

  update_relay();
  update_display();
}
