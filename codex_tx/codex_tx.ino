#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TX_DATA_PIN_bm  PIN4_bm // PA4 - RF Data Out
#define DRIVE_PIN_bm    PIN6_bm // PA6 - Common Probe Drive, active HIGH
#define BTN_PIN_bm      PIN7_bm // PA7 - Button, active LOW
#define STATUS_LED_bm   PIN0_bm // PB0 - Status LED, active HIGH

#define PROBE_L1_bm     PIN1_bm // PA1 - motor ON level
#define PROBE_L2_bm     PIN2_bm // PA2 - display level
#define PROBE_L3_bm     PIN3_bm // PA3 - motor OFF level
#define PROBE_L4_bm     PIN5_bm // PA5 - overflow

#define BIT_US 1000
#define REPEATS_PER_PACKET 4
#define MAX_PACKET_LEN 16
#define PACKET_GAP_MS 35
#define MANUAL_HOLD_TICKS 150 // 150 * 20ms = 3 seconds
#define MANUAL_MAX_WAKES 75 // about 10 minutes at 8 seconds per wake
#define IDLE_START_WINDOW 5
#define IDLE_START_REQUIRED 3

#define MSG_AUTO_REPORT   0x01
#define MSG_MANUAL_ON     0x02
#define MSG_MANUAL_OFF    0x03
#define MSG_MANUAL_KEEP   0x05

#define FLAG_PUMP_DESIRED      0x01
#define FLAG_DRY_WARNING       0x40
#define FLAG_OVERFLOW_WARNING  0x80

#define PAIR_MAGIC_0 0xC3
#define PAIR_MAGIC_1 0x3C

static const uint32_t MASTER_KEY[4] = {0x7b3a91d0, 0x4c8e25f1, 0x12345678, 0x9abcdef0};
static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

uint8_t sequenceId = 0;
uint8_t lastProbeMask = 0;
uint8_t lastSentProbeMask = 0xff;
uint32_t activeKey[4];
bool desiredPump = false;
bool lastSentDesiredPump = true;
bool manualOn = false;
uint8_t manualWakeCount = 0;
uint8_t startupPairCount = 4;
uint8_t idleStartSamples = 0;
uint8_t idleStartVotes = 0;

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00; // Prescaler divisor 1 (20MHz internal)
}

static void get_pair_identity(uint8_t *id) {
  id[0] = PAIR_MAGIC_0;
  id[1] = PAIR_MAGIC_1;
  for (uint8_t i = 2; i < 8; i++) id[i] = *(volatile uint8_t*)(0x1103 + i);
}

static void xtea_encrypt(uint32_t v[2], uint32_t const k[4]) {
  uint32_t v0 = v[0], v1 = v[1], sum = 0, delta = 0x9E3779B9;
  for (uint8_t i = 0; i < 32; i++) {
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

static void send_packet_repeats(const uint8_t *payload) {
  for (uint8_t r = 0; r < REPEATS_PER_PACKET; r++) {
    radio_send(payload, 8);
    _delay_ms(PACKET_GAP_MS);
  }
}

static void send_pairing_packet() {
  uint8_t payload[8];
  get_pair_identity(payload);
  xtea_encrypt((uint32_t*)payload, MASTER_KEY);
  send_packet_repeats(payload);
}

static uint8_t read_probe_mask() {
  PORTA.OUTSET = DRIVE_PIN_bm;
  PORTA.DIRSET = DRIVE_PIN_bm;
  _delay_us(10);

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

static void encrypt_and_send(uint8_t probeMask, uint8_t msgType, bool pumpRequested) {
  uint8_t plaintext[8];
  uint8_t flags = 0;
  if (pumpRequested) flags |= FLAG_PUMP_DESIRED;
  if (probeMask == 0) flags |= FLAG_DRY_WARNING;
  if (probeMask & 0x08) flags |= FLAG_OVERFLOW_WARNING;

  plaintext[0] = msgType;
  plaintext[1] = sequenceId;
  plaintext[2] = probeMask;
  plaintext[3] = read_battery_code();
  plaintext[4] = flags;
  plaintext[5] = *(volatile uint8_t*)(0x1108);
  plaintext[6] = *(volatile uint8_t*)(0x1109);
  plaintext[7] = *(volatile uint8_t*)(0x110A);

  xtea_encrypt((uint32_t*)plaintext, activeKey);
  send_packet_repeats(plaintext);
  sequenceId++;
}

static void configure_pit_sleep() {
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
  while (RTC.STATUS & RTC_CTRLABUSY_bm);
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;
}

ISR(PORTA_PORT_vect) {
  PORTA.INTFLAGS = BTN_PIN_bm;
}

static bool button_held_3s() {
  uint8_t ticks = MANUAL_HOLD_TICKS;
  while (!(PORTA.IN & BTN_PIN_bm)) {
    _delay_ms(20);
    if (--ticks == 0) return true;
  }
  return false;
}

static void wait_button_release() {
  while (!(PORTA.IN & BTN_PIN_bm)) _delay_ms(10);
}

static void derive_unique_key(uint32_t *destKey) {
  uint8_t sn[8];
  get_pair_identity(sn);
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

static bool handle_button() {
  if (PORTA.IN & BTN_PIN_bm) return false;

  bool longPress = button_held_3s();
  startupPairCount = 0;

  if (longPress) {
    PORTB.OUTSET = STATUS_LED_bm;
    manualOn = !manualOn;
    manualWakeCount = 0;
    encrypt_and_send(lastProbeMask, manualOn ? MSG_MANUAL_ON : MSG_MANUAL_OFF, manualOn);
    wait_button_release();
    PORTB.OUTCLR = STATUS_LED_bm;
    return true;
  }
  return false;
}

static void update_desired_pump(uint8_t probeMask) {
  if (probeMask & 0x08) {
    desiredPump = false;
    idleStartSamples = 0;
    idleStartVotes = 0;
  } else if (probeMask & 0x04) {
    desiredPump = false;
    idleStartSamples = 0;
    idleStartVotes = 0;
  } else if (probeMask == 0 || probeMask == 0x01) { // Start pump only at Level 0 or Level 1
    if (desiredPump) return;
    idleStartVotes++;
    if (++idleStartSamples >= IDLE_START_WINDOW) {
      desiredPump = idleStartVotes >= IDLE_START_REQUIRED;
      idleStartSamples = 0;
      idleStartVotes = 0;
    }
  } else { // All other levels -> hold state
    if (desiredPump) return;
    if (++idleStartSamples >= IDLE_START_WINDOW) {
      idleStartSamples = 0;
      idleStartVotes = 0;
    }
  }
}

void setup() {
  wdt_disable();
  set_clock_full_speed();

  PORTA.DIRSET = TX_DATA_PIN_bm | DRIVE_PIN_bm;
  PORTA.OUTCLR = TX_DATA_PIN_bm | DRIVE_PIN_bm;
  PORTB.DIRSET = STATUS_LED_bm;
  PORTB.OUTCLR = STATUS_LED_bm;

  PORTA.PIN7CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN1CTRL = 0;
  PORTA.PIN2CTRL = 0;
  PORTA.PIN3CTRL = 0;
  PORTA.PIN5CTRL = 0;

  PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

  derive_unique_key(activeKey);
  configure_pit_sleep();
}

void loop() {
  sei();
  lastProbeMask = read_probe_mask();
  bool buttonCommandSent = handle_button();

  if (manualOn) {
    if (!buttonCommandSent) {
      if (manualWakeCount >= MANUAL_MAX_WAKES) {
        manualOn = false;
        encrypt_and_send(lastProbeMask, MSG_MANUAL_OFF, false);
      } else {
        manualWakeCount++;
        encrypt_and_send(lastProbeMask, MSG_MANUAL_KEEP, true);
      }
    }
  } else if (startupPairCount > 0) {
    startupPairCount--;
    PORTB.OUTSET = STATUS_LED_bm;
    send_pairing_packet();
    PORTB.OUTCLR = STATUS_LED_bm;
  } else {
    update_desired_pump(lastProbeMask);
    if (lastProbeMask != lastSentProbeMask || desiredPump != lastSentDesiredPump || desiredPump) {
      lastSentProbeMask = lastProbeMask;
      lastSentDesiredPump = desiredPump;
      encrypt_and_send(lastProbeMask, MSG_AUTO_REPORT, desiredPump);
    }
  }

  PORTA.PIN7CTRL = PORT_ISC_LEVEL_gc | PORT_PULLUPEN_bm;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  PORTA.PIN7CTRL = PORT_PULLUPEN_bm;
}

int main(void) {
  setup();
  while (1) loop();
  return 0;
}
