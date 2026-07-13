#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#define PROBE_SENSE PIN_PA6 // Pin 2
#define PROBE_DRIVE PIN_PA7 // Pin 3
#define TX_DATA     PIN_PA1 // Pin 4
#define BTN_PIN     PIN_PA3 // Pin 7 (Remapped button pin)

#define BIT_US 1000
#define REPEATS_PER_PACKET 4
#define MAX_PACKET_LEN 16
#define ADC_SAMPLES 9
#define STABLE_REQUIRED_READS 2
#define PACKET_GAP_MS 35

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

// Ticks (8.2 second intervals)
#define DEFAULT_FILL_TIMEOUT_TICKS 220 // ~30 minutes
#define MIN_FILL_TIMEOUT_TICKS     110 // ~15 minutes
#define MAX_FILL_TIMEOUT_TICKS     440 // ~60 minutes
#define DRY_REPORT_WAKE_COUNT      5   // 40 seconds
#define DRY_RETRY_WAKE_INTERVAL    150 // 20 minutes
#define OVERFLOW_REPORT_WAKE_COUNT 10  // 80 seconds

// EEPROM addresses
#define EE_ACT_ADDR  0x00 // 0x55 if activated, 0xFF if first startup
#define EE_KEY_ADDR  0x01 // 16 bytes for derived Unique Key (0x01 - 0x10)

// XTEA 128-bit Master Key (used only during pairing)
static const uint32_t MASTER_KEY[4] = {0x7b3a91d0, 0x4c8e25f1, 0x12345678, 0x9abcdef0};

// Calibrated ADC Probe levels
#define ADC_LEVEL_1 160
#define ADC_LEVEL_2 340
#define ADC_LEVEL_3 520
#define ADC_LEVEL_4 700
#define ADC_LEVEL_5 850

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

uint8_t sequenceId = 0;
uint8_t reportedLevel = 0;
uint8_t candidateLevel = 0;
uint8_t candidateCount = 0;
bool firstReport = true;

// Active XTEA Key (loaded from EEPROM or Master Key)
uint32_t activeKey[4];

// State Tracking
bool manualMode = false;
bool manualOn = false;
bool activeFilling = false;
uint16_t fillTimerTicks = 0;
uint16_t allowedFillTicks = DEFAULT_FILL_TIMEOUT_TICKS;
uint16_t learnedFillTicks = 0;

// Watchdog/PIT Counter tracking
uint16_t dryWakeCount = 0;
uint16_t overflowWakeCount = 0;

// PIT 1.024-second wake counter
volatile uint8_t oneSecondWakes = 0;

// Reads 8 bytes from Silicon Serial Number starting at 0x1103
static void get_silicon_id_8(uint8_t *id) {
  for (uint8_t i = 0; i < 8; i++) {
    id[i] = *(volatile uint8_t*)(0x1103 + i);
  }
}

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;
}

static void xtea_encrypt(uint32_t num_rounds, uint32_t v[2], uint32_t const k[4]) {
  uint32_t i;
  uint32_t v0 = v[0], v1 = v[1], sum = 0, delta = 0x9E3779B9;
  for (i = 0; i < num_rounds; i++) {
    v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
    sum += delta;
    v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum >> 11) & 3]);
  }
  v[0] = v0; v[1] = v1;
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
  return crc;
}

static void tx_write(bool value) {
  digitalWrite(TX_DATA, value ? HIGH : LOW);
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
  
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc;
  ADC0.MUXPOS = ADC_MUXPOS_AIN6_gc; 
  ADC0.CTRLA = ADC_ENABLE_bm;

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  uint16_t adc = ADC0.RES;
  ADC0.CTRLA = 0; 
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

  if (firstReport) {
    reportedLevel = instantLevel;
    candidateLevel = instantLevel;
    candidateCount = STABLE_REQUIRED_READS;
    firstReport = false;
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
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  delayMicroseconds(500);

  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.CTRLA = ADC_ENABLE_bm;

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  uint16_t adc = ADC0.RES;
  ADC0.CTRLA = 0;

  if (adc == 0) return 0;
  uint32_t vcc_mv = (1125300UL / adc);
  return (uint8_t)(vcc_mv / 20); // 20mV scaling
}

static void encrypt_and_send(uint8_t level, bool levelChanged, uint8_t msgType, uint8_t extraFlags, const uint32_t* key) {
  uint8_t plaintext[8];
  uint8_t battery = read_battery_code();
  uint8_t flags = extraFlags;
  if (levelChanged) flags |= FLAG_LEVEL_CHANGED;
  if (battery != 0 && battery < 165) flags |= FLAG_LOW_BATTERY; // <3.3V

  plaintext[0] = msgType;
  plaintext[1] = sequenceId;
  plaintext[2] = level;
  plaintext[3] = battery;
  plaintext[4] = flags;
  
  // Bytes 5-7 contain the last 3 bytes of the Silicon ID
  plaintext[5] = *(volatile uint8_t*)(0x110A);
  plaintext[6] = *(volatile uint8_t*)(0x110B);
  plaintext[7] = *(volatile uint8_t*)(0x110C);

  // Encrypt in-place using the specified key
  xtea_encrypt(32, (uint32_t*)plaintext, key);

  for (uint8_t repeat = 0; repeat < REPEATS_PER_PACKET; repeat++) {
    radio_send(plaintext, 8);
    delay(PACKET_GAP_MS);
  }
  sequenceId++;
}

static void configure_pit_sleep() {
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm; 
}

static void configure_pit_off() {
  RTC.PITCTRLA = 0x00; 
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; 
  oneSecondWakes++;
}

ISR(PORTA_PORT_vect) {
  PORTA.INTFLAGS = PIN3_bm; 
}

static bool check_button_held(uint16_t ms) {
  uint16_t elapsed = 0;
  while (digitalRead(BTN_PIN) == LOW) {
    _delay_ms(10);
    elapsed += 10;
    if (elapsed >= ms) {
      // Wait for user to release the button before returning
      while (digitalRead(BTN_PIN) == LOW) {
        _delay_ms(10);
      }
      return true;
    }
  }
  return false;
}

// Generates the unique key dynamically using Silicon ID + Master Key
static void derive_unique_key(uint32_t *destKey) {
  uint8_t sn[8];
  get_silicon_id_8(sn);
  
  uint32_t serial_high = ((uint32_t)sn[0] << 24) | ((uint32_t)sn[1] << 16) | ((uint32_t)sn[2] << 8) | sn[3];
  uint32_t serial_low  = ((uint32_t)sn[4] << 24) | ((uint32_t)sn[5] << 16) | ((uint32_t)sn[6] << 8) | sn[7];

  destKey[0] = MASTER_KEY[0] ^ serial_high;
  destKey[1] = MASTER_KEY[1] ^ serial_low;
  destKey[2] = MASTER_KEY[2] ^ (serial_high ^ serial_low);
  destKey[3] = MASTER_KEY[3] ^ (serial_high + serial_low);
}

void setup() {
  wdt_disable();
  set_clock_full_speed();

  pinMode(PROBE_DRIVE, OUTPUT);
  digitalWrite(PROBE_DRIVE, LOW);
  pinMode(PROBE_SENSE, INPUT);
  pinMode(TX_DATA, OUTPUT);
  tx_write(false);

  pinMode(BTN_PIN, INPUT_PULLUP);
  
  uint8_t activationStatus = eeprom_read_byte((const uint8_t*)EE_ACT_ADDR);

  // 1. First Boot Activation & Pairing Window
  if (activationStatus != 0x55) {
    while (true) {
      if (digitalRead(BTN_PIN) == LOW) {
        if (check_button_held(10000)) {
          // Derive the Unique Key from Silicon ID and write it to EEPROM
          uint32_t derivedKey[4];
          derive_unique_key(derivedKey);
          
          eeprom_write_block((const void*)derivedKey, (void*)EE_KEY_ADDR, 16);
          eeprom_write_byte((uint8_t*)EE_ACT_ADDR, 0x55);
          
          // Generate pairing packet plaintext: contains the 8 bytes of Silicon ID
          uint8_t pairPayload[8];
          get_silicon_id_8(pairPayload);

          // Encrypt pairing packet with Factory Master Key
          xtea_encrypt(32, (uint32_t*)pairPayload, MASTER_KEY);
          
          // Broadcast pairing packet aggressively
          for (uint8_t r = 0; r < 10; r++) {
            radio_send(pairPayload, 8);
            _delay_ms(PACKET_GAP_MS);
          }
          break;
        }
      }
      PORTA.PIN3CTRL = PORT_ISC_LEVEL_gc; // Wake on button
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sei();
      sleep_mode();
      cli();
      PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
    }
  }

  // 2. Normal Boots: load the derived Unique Key from EEPROM
  eeprom_read_block((void*)activeKey, (const void*)EE_KEY_ADDR, 16);
  
  configure_pit_sleep();
}

void loop() {
  sei();
  
  // Debounced Button checks
  if (digitalRead(BTN_PIN) == LOW) {
    if (check_button_held(5000)) {
      // 5-second hold: Toggle AUTO/MANUAL mode
      manualMode = !manualMode;
      manualOn = false;
      activeFilling = false;
      
      if (manualMode) {
        configure_pit_off();
      } else {
        configure_pit_sleep();
      }
      _delay_ms(300);
    } else {
      // Short press: Toggle MANUAL State
      if (manualMode) {
        manualOn = !manualOn;
        encrypt_and_send(reportedLevel, false, MSG_MANUAL_CMD, FLAG_MANUAL_MODE | FLAG_MANUAL_TOGGLE, activeKey);
        
        if (manualOn) {
          configure_pit_sleep();
        } else {
          configure_pit_off();
        }
        _delay_ms(300);
      }
    }
  }

  // Wakes up logic every 8 wakes (8.2 seconds)
  if (oneSecondWakes >= 8 || (manualMode && !manualOn)) {
    oneSecondWakes = 0;

    if (manualMode) {
      if (manualOn) {
        // Send manual keepalive
        encrypt_and_send(reportedLevel, false, MSG_MANUAL_CMD, FLAG_MANUAL_MODE | FLAG_MANUAL_KEEP, activeKey);
      }
    } else {
      // Normal Auto water level checks
      bool levelChanged = false;
      uint8_t level = read_filtered_level(&levelChanged);

      if (level <= 1) {
        if (!activeFilling) {
          activeFilling = true;
          fillTimerTicks = 0;
        }
      }

      if (activeFilling) {
        fillTimerTicks++;
        if (level >= 4) {
          activeFilling = false;
          if (learnedFillTicks == 0) {
            learnedFillTicks = fillTimerTicks;
          } else {
            learnedFillTicks = (learnedFillTicks * 3 + fillTimerTicks) / 4;
          }
          allowedFillTicks = learnedFillTicks * 2;
          if (allowedFillTicks < MIN_FILL_TIMEOUT_TICKS) allowedFillTicks = MIN_FILL_TIMEOUT_TICKS;
          if (allowedFillTicks > MAX_FILL_TIMEOUT_TICKS) allowedFillTicks = MAX_FILL_TIMEOUT_TICKS;
        }
        
        if (fillTimerTicks >= allowedFillTicks) {
          activeFilling = false;
          encrypt_and_send(level, false, MSG_AUTO_REPORT, FLAG_FILL_TIMEOUT, activeKey);
        }
      }

      // Transmit requirement decision
      bool shouldSend = false;
      uint8_t extraFlags = 0;

      if (levelChanged) {
        shouldSend = true;
        if (level == 0) dryWakeCount = 0;
        if (level == 5) overflowWakeCount = 0;
      } else if (activeFilling) {
        shouldSend = true;
      } else if (level == 0) {
        if (dryWakeCount < DRY_REPORT_WAKE_COUNT) {
          shouldSend = true;
          dryWakeCount++;
        } else {
          dryWakeCount++;
          if (dryWakeCount >= DRY_RETRY_WAKE_INTERVAL) {
            shouldSend = true;
            dryWakeCount = DRY_REPORT_WAKE_COUNT;
          }
        }
        extraFlags |= FLAG_DRY_WARNING;
      } else if (level == 5) {
        if (overflowWakeCount < OVERFLOW_REPORT_WAKE_COUNT) {
          shouldSend = true;
          overflowWakeCount++;
        }
        extraFlags |= FLAG_OVERFLOW_WARNING;
      }

      if (shouldSend) {
        encrypt_and_send(level, levelChanged, MSG_AUTO_REPORT, extraFlags, activeKey);
      }
    }
  }

  // Configure sleep wakes
  if (manualMode && !manualOn) {
    PORTA.PIN3CTRL = PORT_ISC_LEVEL_gc; // Button wake only
  } else {
    PORTA.PIN3CTRL = PORT_ISC_LEVEL_gc; // Wake on button OR PIT tick
  }
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
}
