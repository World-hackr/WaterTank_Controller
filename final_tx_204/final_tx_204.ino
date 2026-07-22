#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// --- Port-mapped Pin Configurations (User PCB Layout) ---
#define TX_DATA_PIN_bm  PIN4_bm // PA4 (Pin 2)  - RF Data Out
#define DRIVE_PIN_bm    PIN6_bm // PA6 (Pin 4)  - Common Probe Drive (Active HIGH)
#define BTN_PIN_bm      PIN7_bm // PA7 (Pin 5)  - Manual Button Input (Active LOW)
#define STATUS_LED_bm   PIN0_bm // PB0 (Pin 9)  - Status/Pairing LED (Active HIGH)

// Level Probes (with external 1M pull-downs and 6.7k series resistors)
#define PROBE_L1_bm     PIN1_bm // PA1 (Pin 11) - Probe Level 1
#define PROBE_L2_bm     PIN2_bm // PA2 (Pin 12) - Probe Level 2
#define PROBE_L3_bm     PIN3_bm // PA3 (Pin 13) - Probe Level 3
#define PROBE_L4_bm     PIN5_bm // PA5 (Pin 3)  - Probe Level 4

// --- Protocol Constants ---
#define BIT_US 1000
#define REPEATS_PER_PACKET 4
#define MAX_PACKET_LEN 16
#define PACKET_GAP_MS 35

#define MSG_AUTO_REPORT 0x01
#define MSG_MANUAL_CMD  0x02
#define MSG_PAIRING     0x03

#define FLAG_LEVEL_CHANGED 0x01
#define FLAG_LOW_BATTERY   0x02
#define FLAG_MANUAL_MODE   0x08
#define FLAG_MANUAL_TOGGLE 0x10
#define FLAG_MANUAL_KEEP   0x20
#define FLAG_DRY_WARNING   0x40
#define FLAG_OVERFLOW_WARNING 0x80

// XTEA 128-bit Master Key (used only during pairing)
static const uint32_t MASTER_KEY[4] = {0x7b3a91d0, 0x4c8e25f1, 0x12345678, 0x9abcdef0};

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

uint8_t sequenceId = 0;
uint8_t reportedLevel = 0;

// Active XTEA Key (dynamically derived into RAM at boot)
uint32_t activeKey[4];

// State Tracking
bool manualMode = false;
bool manualOn = false;
uint8_t startupPairCount = 5; // Boot pairing checks (5 wakes = 40 seconds)

// Reads 8 bytes from Silicon Serial Number starting at 0x1103
static void get_silicon_id_8(uint8_t *id) {
  for (uint8_t i = 0; i < 8; i++) {
    id[i] = *(volatile uint8_t*)(0x1103 + i);
  }
}

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00; // Prescaler divisor 1 (20MHz internal)
}

// XTEA encryption optimized for 8-bit AVR
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

static void send_packet_repeats(const uint8_t *payload, uint8_t repeats) {
  for (uint8_t r = 0; r < repeats; r++) {
    radio_send(payload, 8);
    _delay_ms(PACKET_GAP_MS);
  }
}

static void send_pairing_packet() {
  uint8_t pairPayload[8];
  get_silicon_id_8(pairPayload);

  // Encrypt pairing packet with Factory Master Key
  xtea_encrypt((uint32_t*)pairPayload, MASTER_KEY);
  send_packet_repeats(pairPayload, REPEATS_PER_PACKET);
}

// Reads 4 digital probe pins to determine water level (0 to 4) - Active HIGH
static uint8_t read_digital_level() {
  // Drive common line HIGH
  PORTA.OUTSET = DRIVE_PIN_bm;
  PORTA.DIRSET = DRIVE_PIN_bm; 
  _delay_us(10); // Let line settle

  uint8_t level = 0;
  
  // Probes are Active HIGH (pulled to 3.3V by water)
  if (PORTA.IN & PROBE_L1_bm) level = 1; 
  if (PORTA.IN & PROBE_L2_bm) level = 2; 
  if (PORTA.IN & PROBE_L3_bm) level = 3; 
  if (PORTA.IN & PROBE_L4_bm) level = 4; 

  // Turn off drive pin (pull to High-Z input to prevent corrosion)
  PORTA.DIRCLR = DRIVE_PIN_bm;
  PORTA.OUTCLR = DRIVE_PIN_bm;

  return level;
}

// Read Vcc directly against internal reference
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
  ADC0.CTRLA = 0; // Shut down ADC

  return (adc >= 341) ? 150 : 180; // 150 = Low (<3.3V), 180 = Good (>3.3V)
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
  plaintext[5] = *(volatile uint8_t*)(0x1108);
  plaintext[6] = *(volatile uint8_t*)(0x1109);
  plaintext[7] = *(volatile uint8_t*)(0x110A);

  // Encrypt in-place using the global activeKey
  xtea_encrypt((uint32_t*)plaintext, activeKey);

  send_packet_repeats(plaintext, REPEATS_PER_PACKET);
  sequenceId++;
}

static void configure_pit_sleep() {
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; // Select 1.024 kHz ULP clock
  while (RTC.STATUS & RTC_CTRLABUSY_bm); // Wait for sync
  
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm; // 8,192 cycles = 8.0 seconds
}

static void configure_pit_off() {
  RTC.PITCTRLA = 0x00; 
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; 
}

ISR(PORTA_PORT_vect) {
  PORTA.INTFLAGS = BTN_PIN_bm; 
}

// 8-bit timer checks button state every 20ms (250 ticks = 5 seconds)
static bool check_button_held(uint8_t intervals) {
  while (!(PORTA.IN & BTN_PIN_bm)) {
    _delay_ms(20);
    intervals--;
    if (intervals == 0) {
      return true; // Hold detected
    }
  }
  return false;
}

// Computes the derived unique key dynamically in RAM
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

void setup() {
  wdt_disable();
  set_clock_full_speed();

  // Configure Data and Drive pins as outputs
  PORTA.DIRSET = TX_DATA_PIN_bm | DRIVE_PIN_bm;
  PORTA.OUTCLR = TX_DATA_PIN_bm | DRIVE_PIN_bm;

  // Configure Status LED on Port B as output
  PORTB.DIRSET = STATUS_LED_bm;
  PORTB.OUTCLR = STATUS_LED_bm;

  // Configure Button (PA7) with Pull-up enabled.
  PORTA.PIN7CTRL = PORT_PULLUPEN_bm;

  // Disable internal pull-ups on Probes (PA1, PA2, PA3, PA5) to rely on external 1M pull-downs
  PORTA.PIN1CTRL = 0;
  PORTA.PIN2CTRL = 0;
  PORTA.PIN3CTRL = 0;
  PORTA.PIN5CTRL = 0;

  // Disable digital input buffers on other unused Port B pins for power saving
  PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

  // Runtime Override: Disable BOD during Sleep to save battery leakage
  CPU_CCP = CCP_IOREG_gc;
  BOD.CTRLA = (BOD.CTRLA & ~BOD_SLEEP_gm) | BOD_SLEEP_DIS_gc;

  // Derive unique key directly into activeKey array in RAM
  derive_unique_key(activeKey);

  configure_pit_sleep();
}

void loop() {
  sei();
  
  // Debounced Button check
  if (!(PORTA.IN & BTN_PIN_bm)) {
    // Visual LED feedback immediately upon button press
    PORTB.OUTSET = STATUS_LED_bm;

    if (check_button_held(250)) { // 5-second hold: Toggle Mode or Pair
      if (manualMode) {
        // Toggle back to Auto
        manualMode = false;
        manualOn = false;
        PORTB.OUTCLR = STATUS_LED_bm;
        configure_pit_sleep();
      } else {
        // Continuous rapid pairing broadcast
        while (!(PORTA.IN & BTN_PIN_bm)) {
          PORTB.OUTTGL = STATUS_LED_bm;
          send_pairing_packet();
          _delay_ms(100);
        }
        PORTB.OUTCLR = STATUS_LED_bm;
      }
      
      while (!(PORTA.IN & BTN_PIN_bm)) {
        _delay_ms(10);
      }
    } else {
      // Short press: Toggle MANUAL pump state
      if (manualMode) {
        manualOn = !manualOn;
        encrypt_and_send(reportedLevel, MSG_MANUAL_CMD, FLAG_MANUAL_MODE | FLAG_MANUAL_TOGGLE);
        
        if (manualOn) {
          configure_pit_sleep();
        } else {
          configure_pit_off();
        }
      } else {
        // Short press in Auto: Force enter Manual Mode
        manualMode = true;
        manualOn = true;
        encrypt_and_send(reportedLevel, MSG_MANUAL_CMD, FLAG_MANUAL_MODE | FLAG_MANUAL_TOGGLE);
        configure_pit_sleep();
      }
      _delay_ms(300);
      PORTB.OUTCLR = STATUS_LED_bm;
    }
  }

  // Active Wake Logic (runs every 8.0 seconds or upon button interrupt)
  if (manualMode) {
    if (manualOn) {
      encrypt_and_send(reportedLevel, MSG_MANUAL_CMD, FLAG_MANUAL_MODE | FLAG_MANUAL_KEEP);
    }
  } else {
    // Normal Auto water level checks
    uint8_t level = read_digital_level();
    
    if (startupPairCount > 0) {
      // Broadcast slow pairing signals at boot (LED blinks during broadcast)
      startupPairCount--;
      PORTB.OUTSET = STATUS_LED_bm;
      send_pairing_packet();
      PORTB.OUTCLR = STATUS_LED_bm;
    } else {
      bool levelChanged = (level != reportedLevel);
      reportedLevel = level;

      // Transmit on level change, or on every wake when level is not full (level != 3) to keep link active
      if (levelChanged || level != 3) {
        uint8_t flags = 0;
        if (levelChanged) flags |= FLAG_LEVEL_CHANGED;
        if (level == 0) flags |= FLAG_DRY_WARNING;
        if (level == 4) flags |= FLAG_OVERFLOW_WARNING;
        
        encrypt_and_send(level, MSG_AUTO_REPORT, flags);
      }
    }
  }

  // Configure sleep interrupts
  if (manualMode && !manualOn) {
    PORTA.PIN7CTRL = PORT_ISC_LEVEL_gc | PORT_PULLUPEN_bm; // Wake on button only (PA7)
  } else {
    PORTA.PIN7CTRL = PORT_ISC_LEVEL_gc | PORT_PULLUPEN_bm; // Wake on button OR PIT timer (PA7)
  }
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  
  PORTA.PIN7CTRL = PORT_PULLUPEN_bm;
}

// Bypasses Arduino core setup/loop template to minimize vector and initialization size
int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}
