#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

// --- Pin Settings (ATtiny402) ---
#define RX_DATA_PIN_bm   PIN1_bm // PA1 (Pin 4) - RF Data Input
#define LINE_A_bm        PIN6_bm // PA6 (Pin 2) - Charlieplex Line A
#define LINE_B_bm        PIN7_bm // PA7 (Pin 3) - Charlieplex Line B
#define LINE_C_bm        PIN3_bm // PA3 (Pin 7) - Charlieplex Line C

#define SAMPLE_RATE_HZ 8000UL
#define RAMP_LEN 160
#define RAMP_INC 20
#define RAMP_TRANSITION 80
#define RAMP_INC_RETARD 11
#define RAMP_INC_ADVANCE 29
#define START_SYMBOL 0x0b38
#define MAX_PACKET_LEN 16

static const uint8_t symbols[16] = {
  0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

volatile uint8_t integrator = 0, pllRamp = 0, rxBitCount = 0, rxCount = 0, rxLen = 0;
volatile bool lastSample = false, rxActive = false, packetReady = false;
volatile uint16_t rxBits = 0;
volatile uint8_t rxBuf[MAX_PACKET_LEN];

uint8_t currentCount = 0;
uint32_t lastGoodPacketMs = 0;
uint32_t blinkEndMs = 0;
bool heartbeatOn = false;

extern "C" void init();
extern unsigned long millis();

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00; // 20MHz clock
}

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
  }
  return crc;
}

static uint8_t symbol_6to4(uint8_t symbol) {
  for (uint8_t i = 0; i < 16; i++) {
    if (symbol == symbols[i]) return i;
  }
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

void setup() {
  set_clock_full_speed();
  leds_off();
  PORTA.DIRCLR = RX_DATA_PIN_bm;

  timer_setup();
  sei();
}

void loop() {
  uint32_t now = millis();

  // Look for incoming packet
  uint8_t packet[8];
  uint8_t len = sizeof(packet);
  if (radio_recv(packet, &len)) {
    if (len == 8 && packet[0] == 0x99) {
      currentCount = packet[1]; // Get count (1 to 4)
      lastGoodPacketMs = now;
      heartbeatOn = true;
      blinkEndMs = now + 150; // Blink LED 6 for 150ms
    }
  }

  // Display scan loop
  if (heartbeatOn && (now < blinkEndMs)) {
    // LED 6 = Packet received indicator
    drive_led(6);
    _delay_ms(2);
  } else {
    heartbeatOn = false;
  }

  // Show count on LEDs 1 to 4
  if (now - lastGoodPacketMs < 3000) {
    uint8_t showLeds = currentCount > 4 ? 4 : currentCount;
    for (uint8_t led = 1; led <= 4; led++) {
      if (led <= showLeds) {
        drive_led(led);
        _delay_ms(2);
      }
    }
  }

  leds_off();
}

int main(void) {
  init(); // Arduino Core initialization
  setup();
  while (1) {
    loop();
  }
  return 0;
}
