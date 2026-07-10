#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PROBE_SENSE PIN_PA6
#define PROBE_DRIVE PIN_PA7
#define TX_DATA     PIN_PA1
#define TEST_LED    PIN_PA3
#define BTN_PIN     PIN_PA2 // PA2 physical Pin 5 button

#define BIT_US 1000
#define REPEATS_PER_PACKET 4
#define MAX_PACKET_LEN 16
#define ADC_SAMPLES 9
#define STABLE_REQUIRED_READS 2
#define PACKET_GAP_MS 35

// Message types
#define MSG_AUTO_REPORT 0x01
#define MSG_MANUAL_CMD  0x02

// Flags
#define FLAG_LEVEL_CHANGED 0x01
#define FLAG_LOW_BATTERY   0x02
#define FLAG_FILL_TIMEOUT  0x04
#define FLAG_MANUAL_MODE   0x08
#define FLAG_MANUAL_TOGGLE 0x10
#define FLAG_MANUAL_KEEP   0x20
#define FLAG_DRY_WARNING   0x40
#define FLAG_OVERFLOW_WARNING 0x80

// Timing in Ticks (8 second intervals)
#define DEFAULT_FILL_TIMEOUT_TICKS 225 // 30 minutes
#define MIN_FILL_TIMEOUT_TICKS     112 // 15 minutes
#define MAX_FILL_TIMEOUT_TICKS     450 // 60 minutes
#define DRY_REPORT_WAKE_COUNT      5   // 40 seconds
#define DRY_RETRY_WAKE_INTERVAL    150 // 20 minutes
#define OVERFLOW_REPORT_WAKE_COUNT 10  // 80 seconds

static const uint64_t AUTH_KEY = 0x7b3a91d04c8e25f1ULL;

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
bool isActivated = false;

// State Tracking
bool manualMode = false;
bool manualOn = false;
bool activeFilling = false;
uint16_t fillTimerTicks = 0;
uint16_t allowedFillTicks = DEFAULT_FILL_TIMEOUT_TICKS;
uint16_t learnedFillTicks = 0; // Learned value in RAM

// Watchdog/PIT Counter tracking
uint16_t dryWakeCount = 0;
uint16_t overflowWakeCount = 0;

// PIT 1-second wake counter (8 wakes = 8 seconds)
volatile uint8_t oneSecondWakes = 0;

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

static void build_payload(uint8_t *payload, uint8_t level, bool levelChanged, uint8_t msgType, uint8_t extraFlags) {
  uint8_t battery = 0; // Battery measurement placeholder
  uint8_t flags = extraFlags;
  if (levelChanged) flags |= FLAG_LEVEL_CHANGED;

  payload[0] = msgType;
  payload[1] = sequenceId;
  payload[2] = level;
  payload[3] = battery;
  payload[4] = flags;
  write64le(payload + 5, auth64_tag(payload, 5, AUTH_KEY));
}

// RTC Periodic Interrupt Timer (PIT) configurations
static void configure_pit_sleep() {
  // Configure PIT to interrupt every 1.024 seconds (32768 cycles of internal OSCULP32K)
  RTC.PITINTCTRL = RTC_PI_bm; // Enable Periodic Interrupt
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm; // Enable PIT with 32768 cycles (~1s)
}

static void configure_pit_off() {
  RTC.PITCTRLA = 0x00; // Disable PIT
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; // Clear Periodic Interrupt flag
  oneSecondWakes++;
}

ISR(PORTA_PORT_vect) {
  // Wakeup on button Pin Change
  PORTA.INTFLAGS = PIN2_bm; // Clear flag
}

static bool check_button_held(uint16_t ms) {
  uint16_t elapsed = 0;
  while (digitalRead(BTN_PIN) == LOW) {
    _delay_ms(10);
    elapsed += 10;
    if (elapsed >= ms) return true;
  }
  return false;
}

void setup() {
  wdt_disable();
  set_clock_full_speed();

  pinMode(PROBE_DRIVE, OUTPUT);
  digitalWrite(PROBE_DRIVE, LOW);
  pinMode(PROBE_SENSE, INPUT);
  pinMode(TX_DATA, OUTPUT);
  pinMode(TEST_LED, OUTPUT);
  tx_write(false);

  pinMode(BTN_PIN, INPUT_PULLUP);
  
  // Power-up safety block: wait for 10-second activation hold
  while (!isActivated) {
    if (digitalRead(BTN_PIN) == LOW) {
      if (check_button_held(10000)) {
        isActivated = true;
        // Visual confirmation of activation
        for (uint8_t i = 0; i < 4; i++) {
          tx_write(true);
          _delay_ms(100);
          tx_write(false);
          _delay_ms(100);
        }
      }
    }
    // Deep Sleep until button press
    PORTA.PIN2CTRL = PORT_ISC_LEVEL_gc; // Low level interrupt to wake
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sei();
    sleep_mode();
    cli();
    PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  }
  
  configure_pit_sleep();
}

void loop() {
  sei();
  
  // debounced button hold checks
  if (digitalRead(BTN_PIN) == LOW) {
    if (check_button_held(5000)) {
      // 5-second hold: Toggle AUTO/MANUAL Mode
      manualMode = !manualMode;
      manualOn = false;
      activeFilling = false;
      
      // Flash LED: 2 blinks for AUTO, 4 blinks for MANUAL
      uint8_t blinks = manualMode ? 4 : 2;
      for (uint8_t i = 0; i < blinks; i++) {
        tx_write(true);
        _delay_ms(200);
        tx_write(false);
        _delay_ms(200);
      }
      
      if (manualMode) {
        configure_pit_off(); // Sleep without PIT in manual
      } else {
        configure_pit_sleep();
      }
    } else {
      // Short press: MANUAL Toggle Command
      if (manualMode) {
        manualOn = !manualOn;
        uint8_t flags = FLAG_MANUAL_MODE | FLAG_MANUAL_TOGGLE;
        uint8_t payload[13];
        build_payload(payload, reportedLevel, false, MSG_MANUAL_CMD, flags);
        
        // Transmit toggles aggressively for reliability
        for (uint8_t r = 0; r < 4; r++) { // 4 repeats per packet
          radio_send(payload, sizeof(payload));
          _delay_ms(PACKET_GAP_MS);
        }
        
        if (manualOn) {
          configure_pit_sleep(); // Enable PIT for 8s keepalive
        } else {
          configure_pit_off();
        }
      }
    }
  }

  // Only run main transmitter logic if we have reached the 8-second tick (8 wakes)
  // or if we are in Manual Sleep where WDT/PIT is OFF
  if (oneSecondWakes >= 8 || (manualMode && !manualOn)) {
    oneSecondWakes = 0; // Reset tick counter

    if (manualMode) {
      if (manualOn) {
        // Send manual keepalive
        uint8_t flags = FLAG_MANUAL_MODE | FLAG_MANUAL_KEEP;
        uint8_t payload[13];
        build_payload(payload, reportedLevel, false, MSG_MANUAL_CMD, flags);
        radio_send(payload, sizeof(payload));
      }
    } else {
      // Normal Auto Water Level Mode
      bool levelChanged = false;
      uint8_t level = read_filtered_level(&levelChanged);

      // Adaptive Filling Tracking
      if (level <= 1) {
        if (!activeFilling) {
          activeFilling = true;
          fillTimerTicks = 0;
        }
      }

      if (activeFilling) {
        fillTimerTicks++;
        if (levelChanged && level > 1) {
          // We are rising, update progress
        }
        if (level >= 4) {
          // Successful Fill! Update learned average
          activeFilling = false;
          if (learnedFillTicks == 0) {
            learnedFillTicks = fillTimerTicks;
          } else {
            learnedFillTicks = (learnedFillTicks * 3 + fillTimerTicks) / 4;
          }
          // Recalculate adaptive threshold (clamp between 15-60 mins)
          allowedFillTicks = learnedFillTicks * 2;
          if (allowedFillTicks < MIN_FILL_TIMEOUT_TICKS) allowedFillTicks = MIN_FILL_TIMEOUT_TICKS;
          if (allowedFillTicks > MAX_FILL_TIMEOUT_TICKS) allowedFillTicks = MAX_FILL_TIMEOUT_TICKS;
        }
        
        // Check for Timeout
        if (fillTimerTicks >= allowedFillTicks) {
          activeFilling = false;
          // Transmit fill timeout fault aggressively
          uint8_t payload[13];
          build_payload(payload, level, false, MSG_AUTO_REPORT, FLAG_FILL_TIMEOUT);
          for (uint8_t r = 0; r < 4; r++) {
            radio_send(payload, sizeof(payload));
            _delay_ms(PACKET_GAP_MS);
          }
        }
      }

      // Determine transmit requirement
      bool shouldSend = false;
      uint8_t extraFlags = 0;

      if (levelChanged) {
        shouldSend = true;
        if (level == 0) {
          dryWakeCount = 0;
        }
        if (level == 5) {
          overflowWakeCount = 0;
        }
      } else if (activeFilling) {
        shouldSend = true; // Send keepalive every 8 seconds during filling
      } else if (level == 0) {
        // Dry State: Send for 5 wakes, then retry once every 20 minutes
        if (dryWakeCount < DRY_REPORT_WAKE_COUNT) {
          shouldSend = true;
          dryWakeCount++;
        } else {
          dryWakeCount++;
          if (dryWakeCount >= DRY_RETRY_WAKE_INTERVAL) {
            shouldSend = true;
            dryWakeCount = DRY_REPORT_WAKE_COUNT; // Loop retry count
          }
        }
        extraFlags |= FLAG_DRY_WARNING;
      } else if (level == 5) {
        // Overflow State: Send for 10 wakes, then silent
        if (overflowWakeCount < OVERFLOW_REPORT_WAKE_COUNT) {
          shouldSend = true;
          overflowWakeCount++;
        }
        extraFlags |= FLAG_OVERFLOW_WARNING;
      }

      if (shouldSend) {
        uint8_t payload[13];
        build_payload(payload, level, levelChanged, MSG_AUTO_REPORT, extraFlags);
        for (uint8_t repeat = 0; repeat < REPEATS_PER_PACKET; repeat++) {
          radio_send(payload, sizeof(payload));
          _delay_ms(PACKET_GAP_MS);
        }
        sequenceId++;
      }
    }
  }

  // Sleep config
  if (manualMode && !manualOn) {
    PORTA.PIN2CTRL = PORT_ISC_LEVEL_gc; // Button wake only
  } else {
    PORTA.PIN2CTRL = PORT_ISC_LEVEL_gc; // Wake on button OR RTC/PIT interrupt
  }
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  
  // Wake Up -> Disable Pin interrupts
  PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
}
