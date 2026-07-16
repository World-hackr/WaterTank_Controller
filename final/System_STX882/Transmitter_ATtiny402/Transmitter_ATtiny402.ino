#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TX_DATA_PIN  1 // PA1 (RF Data Out)
#define BTN_PIN      3 // PA3 (Button Input)
#define DRIVE_PIN    2 // PA2 (Common Probe Drive)

#define BIT_US 1000
#define REPEATS_PER_PACKET 4
#define MAX_PACKET_LEN 16
#define PACKET_GAP_MS 35
#define STABLE_REQUIRED_READS 2

// Protocol constants
#define MSG_AUTO_REPORT 0x01
#define MSG_MANUAL_CMD  0x02
#define MSG_PAIRING     0x03

// Flags
#define FLAG_LEVEL_CHANGED 0x01
#define FLAG_LOW_BATTERY   0x02
#define FLAG_MANUAL_MODE   0x08
#define FLAG_MANUAL_TOGGLE 0x10
#define FLAG_MANUAL_KEEP   0x20
#define FLAG_DRY_WARNING   0x40
#define FLAG_OVERFLOW_WARNING 0x80

// EEPROM addresses (Memory-mapped starting at 0x1400)
#define EEPROM_BASE  0x1400
#define EE_ACT_ADDR  0x00 // 0x55 if activated
#define EE_KEY_ADDR  0x01 // 16 bytes for derived Unique Key

// XTEA 128-bit Master Key (used only during pairing)
static const uint32_t MASTER_KEY[4] = {0x7b3a91d0, 0x4c8e25f1, 0x12345678, 0x9abcdef0};

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

uint8_t sequenceId = 0;
uint8_t reportedLevel = 0;

// Active XTEA Key (loaded from EEPROM or Master Key)
uint32_t activeKey[4];

// State Tracking
bool manualMode = false;
bool manualOn = false;

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

// XTEA encryption optimized for 8-bit AVR (loop count hardcoded to 32, counter is uint8_t)
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
  if (value) {
    PORTA.OUTSET = PIN1_bm;
  } else {
    PORTA.OUTCLR = PIN1_bm;
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

// Repeated packet transmission helper to prevent duplicate loop bloat
static void send_packet_repeats(const uint8_t *payload, uint8_t repeats) {
  for (uint8_t r = 0; r < repeats; r++) {
    radio_send(payload, 8);
    _delay_ms(PACKET_GAP_MS);
  }
}

// Reads 5 digital probe pins in order to determine water level
static uint8_t read_digital_level() {
  // Drive common line LOW
  PORTA.OUTCLR = PIN2_bm;
  PORTA.DIRSET = PIN2_bm; 
  _delay_us(10); // Let line settle

  uint8_t level = 0;
  
  // Probes are Active LOW (pulled to GND by water)
  if (!(PORTA.IN & PIN4_bm)) level = 1; // PA4 wet
  if (!(PORTA.IN & PIN5_bm)) level = 2; // PA5 wet
  if (!(PORTA.IN & PIN6_bm)) level = 3; // PA6 wet
  if (!(PORTA.IN & PIN7_bm)) level = 4; // PA7 wet
  if (!(PORTB.IN & PIN3_bm)) level = 5; // PB3 wet

  // Turn off drive pin (pull to High-Z input to prevent corrosion)
  PORTA.DIRCLR = PIN2_bm;
  PORTA.OUTSET = PIN2_bm;

  return level;
}

// Read Vcc directly against internal reference (single conversion after settling)
static uint8_t read_battery_code() {
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  _delay_us(500); // Wait for Vref to settle fully

  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.CTRLA = ADC_ENABLE_bm;

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  uint16_t adc = ADC0.RES;
  ADC0.CTRLA = 0;

  return (adc >= 341) ? 150 : 180;
}

static void encrypt_and_send(uint8_t level, uint8_t msgType, uint8_t flags) {
  uint8_t plaintext[8];
  uint8_t battery = read_battery_code();
  if (battery == 150) flags |= FLAG_LOW_BATTERY;

  plaintext[0] = msgType;
  plaintext[1] = sequenceId;
  plaintext[2] = level;
  plaintext[3] = battery;
  plaintext[4] = flags;
  
  // Bytes 5-7 contain the last 3 bytes of the Silicon ID
  plaintext[5] = *(volatile uint8_t*)(0x110A);
  plaintext[6] = *(volatile uint8_t*)(0x110B);
  plaintext[7] = *(volatile uint8_t*)(0x110C);

  // Encrypt in-place using the global activeKey
  xtea_encrypt((uint32_t*)plaintext, activeKey);

  send_packet_repeats(plaintext, REPEATS_PER_PACKET);
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

// 8-bit timer checks button state every 20ms (250 ticks = 5 seconds) to avoid 16-bit loop math
static bool check_button_held(uint8_t intervals) {
  while (!(PORTA.IN & PIN3_bm)) { // PA3 is LOW when pressed
    _delay_ms(20);
    intervals--;
    if (intervals == 0) {
      return true; // Threshold reached
    }
  }
  return false;
}

// Bypasses 32-bit compiler shift helpers by performing XOR and carry additions byte-by-byte
static void derive_unique_key(uint32_t *destKey) {
  uint8_t sn[8];
  get_silicon_id_8(sn);
  
  uint8_t *dk = (uint8_t*)destKey;
  const uint8_t *mk = (const uint8_t*)MASTER_KEY;

  // destKey[0] = MASTER_KEY[0] ^ serial_high
  dk[0] = mk[0] ^ sn[3];
  dk[1] = mk[1] ^ sn[2];
  dk[2] = mk[2] ^ sn[1];
  dk[3] = mk[3] ^ sn[0];

  // destKey[1] = MASTER_KEY[1] ^ serial_low
  dk[4] = mk[4] ^ sn[7];
  dk[5] = mk[5] ^ sn[6];
  dk[6] = mk[6] ^ sn[5];
  dk[7] = mk[7] ^ sn[4];

  // destKey[2] = MASTER_KEY[2] ^ (serial_high ^ serial_low)
  dk[8]  = mk[8]  ^ (sn[3] ^ sn[7]);
  dk[9]  = mk[9]  ^ (sn[2] ^ sn[6]);
  dk[10] = mk[10] ^ (sn[1] ^ sn[5]);
  dk[11] = mk[11] ^ (sn[0] ^ sn[4]);

  // destKey[3] = MASTER_KEY[3] ^ (serial_high + serial_low)
  uint16_t carry = 0;
  uint16_t sum0 = (uint16_t)sn[3] + sn[7];
  dk[12] = mk[12] ^ (uint8_t)sum0;
  carry = sum0 >> 8;

  uint16_t sum1 = (uint16_t)sn[2] + sn[6] + carry;
  dk[13] = mk[13] ^ (uint8_t)sum1;
  carry = sum1 >> 8;

  uint16_t sum2 = (uint16_t)sn[1] + sn[5] + carry;
  dk[14] = mk[14] ^ (uint8_t)sum2;
  carry = sum2 >> 8;

  uint16_t sum3 = (uint16_t)sn[0] + sn[4] + carry;
  dk[15] = mk[15] ^ (uint8_t)sum3;
}

// Custom register-level EEPROM write function to save libc library bloat
static void my_eeprom_write_byte(uint8_t offset, uint8_t value) {
  while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
  CPU_CCP = CCP_SPM_gc;
  NVMCTRL.CTRLA = NVMCTRL_CMD_NONE_gc;
  *(volatile uint8_t*)(EEPROM_BASE + offset) = value;
  CPU_CCP = CCP_SPM_gc;
  NVMCTRL.CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
}

void setup() {
  wdt_disable();
  set_clock_full_speed();

  // Configure PA1 (RF) and PA2 (Drive) as outputs
  PORTA.DIRSET = PIN1_bm | PIN2_bm;
  PORTA.OUTCLR = PIN1_bm | PIN2_bm;

  // Configure Button PA3 and Probes (PA4, PA5, PA6, PA7, PB3) with Pull-ups enabled in a small register loop
  volatile uint8_t *pinCtrl = &PORTA.PIN3CTRL;
  for (uint8_t i = 0; i < 5; i++) {
    pinCtrl[i] = PORT_PULLUPEN_bm;
  }
  PORTB.PIN3CTRL = PORT_PULLUPEN_bm;
  
  // Memory-mapped EEPROM read
  uint8_t activationStatus = *(volatile uint8_t*)(EEPROM_BASE + EE_ACT_ADDR);

  // 1. First Boot Activation & Pairing Window
  if (activationStatus != 0x55) {
    while (true) {
      if (!(PORTA.IN & PIN3_bm)) { // Button PA3 pressed
        if (check_button_held(250)) { // 250 * 20ms = 5-second pairing hold
          // Derive the Unique Key from Silicon ID and write it to EEPROM
          uint32_t derivedKey[4];
          derive_unique_key(derivedKey);
          
          for (uint8_t i = 0; i < 16; i++) {
            my_eeprom_write_byte(EE_KEY_ADDR + i, ((uint8_t*)derivedKey)[i]);
          }
          my_eeprom_write_byte(EE_ACT_ADDR, 0x55);
          
          // Generate pairing packet plaintext: contains the 8 bytes of Silicon ID
          uint8_t pairPayload[8];
          get_silicon_id_8(pairPayload);

          // Encrypt pairing packet with Factory Master Key
          xtea_encrypt((uint32_t*)pairPayload, MASTER_KEY);
          
          // Broadcast pairing packet aggressively
          send_packet_repeats(pairPayload, 10);

          // Now wait for user to release the button
          while (!(PORTA.IN & PIN3_bm)) {
            _delay_ms(10);
          }
          break;
        }
      }
      PORTA.PIN3CTRL = PORT_ISC_LEVEL_gc | PORT_PULLUPEN_bm; // Wake on button
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sei();
      sleep_mode();
      cli();
      PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
    }
  }

  // 2. Normal Boots: load the derived Unique Key from memory-mapped EEPROM
  for (uint8_t i = 0; i < 16; i++) {
    ((uint8_t*)activeKey)[i] = *(volatile uint8_t*)(EEPROM_BASE + EE_KEY_ADDR + i);
  }
  
  configure_pit_sleep();
}

void loop() {
  sei();
  
  // Debounced Button checks
  if (!(PORTA.IN & PIN3_bm)) {
    if (check_button_held(250)) { // 250 * 20ms = 5-second hold: Toggle AUTO/MANUAL mode
      manualMode = !manualMode;
      manualOn = false;
      
      if (manualMode) {
        configure_pit_off();
      } else {
        configure_pit_sleep();
      }
      
      // Wait for release before returning
      while (!(PORTA.IN & PIN3_bm)) {
        _delay_ms(10);
      }
    } else {
      // Short press: Toggle MANUAL State
      if (manualMode) {
        manualOn = !manualOn;
        encrypt_and_send(reportedLevel, MSG_MANUAL_CMD, FLAG_MANUAL_MODE | FLAG_MANUAL_TOGGLE);
        
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
        encrypt_and_send(reportedLevel, MSG_MANUAL_CMD, FLAG_MANUAL_MODE | FLAG_MANUAL_KEEP);
      }
    } else {
      // Normal Auto water level checks
      uint8_t level = read_digital_level();
      bool levelChanged = (level != reportedLevel);
      reportedLevel = level;

      // Transmit on change, or on every wake if water is not full (level != 4) to keep link alive
      if (levelChanged || level != 4) {
        uint8_t flags = 0;
        if (levelChanged) flags |= FLAG_LEVEL_CHANGED;
        if (level == 0) flags |= FLAG_DRY_WARNING;
        if (level == 5) flags |= FLAG_OVERFLOW_WARNING;
        encrypt_and_send(level, MSG_AUTO_REPORT, flags);
      }
    }
  }

  // Configure sleep wakes
  if (manualMode && !manualOn) {
    PORTA.PIN3CTRL = PORT_ISC_LEVEL_gc | PORT_PULLUPEN_bm; // Button wake only
  } else {
    PORTA.PIN3CTRL = PORT_ISC_LEVEL_gc | PORT_PULLUPEN_bm; // Wake on button OR PIT tick
  }
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
}

// Bypasses the Arduino core boilerplate setup/loop wrapper
int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}
