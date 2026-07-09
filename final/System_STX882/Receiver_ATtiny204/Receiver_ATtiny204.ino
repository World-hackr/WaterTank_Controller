// ARCHIVED REFERENCE ONLY.
// This file is not part of the current final route.
// Current hardware target is ATtiny402 SOIC-8 with RX on PA1, relay on PA2,
// and 6-LED charlieplex display on PA6/PA7/PA3.
// Use RF_TEST_CODE/12_water_level_rx or final/System_STX882/Receiver_ATtiny402.

#if 0
// Target: ATtiny204 / ATtiny402 (megaTinyCore)
// Project: Water Tank Receiver (RXB12 / SRX882 433MHz ASK Upgrade)
//
// Features:
// - Software-based interrupt-driven pulse width decoder (no libraries needed).
// - Drive relay output with failsafe.
// - Failsafe: Turn relay OFF if no valid MOTOR_ON packet is received for 25 seconds.
// - Drive LED indicators for link status and packet validation.

#include <Arduino.h>

// Pin Configurations
#define PIN_RX_DATA      PIN_PA6 // Data Input from RXB12 / SRX882 (Must support interrupts)
#define PIN_RELAY        PIN_PA1 // Output to drive Relay transistor/MOSFET
#define PIN_LED_RELAY    PIN_PA2 // Relay state LED
#define PIN_LED_PACKET   PIN_PA7 // Packet validation LED (Blinks on good packet)
#define PIN_LED_LINK     PIN_PA3 // Link status LED (Solid if connected, blinking if lost)

// Timing tolerances (in microseconds)
const uint16_t SYNC_HIGH_MIN = 1000;
const uint16_t SYNC_HIGH_MAX = 1400;
const uint16_t SYNC_LOW_MIN  = 450;
const uint16_t SYNC_LOW_MAX  = 750;

const uint16_t BIT_SHORT_MIN = 250;
const uint16_t BIT_SHORT_MAX = 550;
const uint16_t BIT_LONG_MIN  = 650;
const uint16_t BIT_LONG_MAX  = 950;

const uint16_t GAP_MIN       = 400;
const uint16_t GAP_MAX       = 800;

// Device & Protocol Settings
const uint16_t EXPECTED_DEVICE_ID = 0x1201;
const unsigned long MOTOR_ON_TIMEOUT_MS = 25000UL; // 25 seconds failsafe
const unsigned long LINK_ALIVE_MS = 25000UL;

struct __attribute__((packed)) TransmitterPayload {
  uint16_t deviceId;
  uint8_t command;      // 0 = MOTOR_OFF, 1 = MOTOR_ON
  uint16_t batteryMV;   // Battery Millivolts
  uint16_t checksum;    // CRC-16 checksum
};

// State Variables
volatile bool newPacketAvailable = false;
volatile TransmitterPayload rxPacket;

// Failsafe & Blink timers
unsigned long lastValidPacketAt = 0;
unsigned long lastValidOnAt = 0;
unsigned long lastPacketLedAt = 0;
unsigned long nextLinkBlinkAt = 0;
bool relayOn = false;
bool linkLedState = false;
bool packetLedState = false;

// CRC-16 (CCITT-FALSE) calculation
uint16_t calculateCRC16(const uint8_t* data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Interrupt State Machine variables
volatile enum DecoderState {
  STATE_WAITING_SYNC_HIGH,
  STATE_WAITING_SYNC_LOW,
  STATE_READING_BITS
} decoderState = STATE_WAITING_SYNC_HIGH;

volatile uint32_t edgeTime = 0;
volatile uint16_t pulseWidthHigh = 0;
volatile uint16_t pulseWidthLow = 0;
volatile uint8_t bitCount = 0;
volatile uint8_t rxBuffer[sizeof(TransmitterPayload)];

// ISR for RX Data Pin Change
ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; // Clear flags
  
  if (!(flags & PIN6_bm)) return;
  
  uint32_t now = micros();
  bool pinState = ((PORTA.IN & PIN6_bm) != 0);
  uint32_t duration = now - edgeTime;
  edgeTime = now;
  
  if (pinState) {
    // Just went HIGH -> Rising edge (duration of the previous LOW pulse is 'duration')
    pulseWidthLow = duration;
    
    if (decoderState == STATE_WAITING_SYNC_LOW) {
      if (pulseWidthLow >= SYNC_LOW_MIN && pulseWidthLow <= SYNC_LOW_MAX) {
        decoderState = STATE_READING_BITS;
        bitCount = 0;
        memset((void*)rxBuffer, 0, sizeof(rxBuffer));
      } else {
        decoderState = STATE_WAITING_SYNC_HIGH;
      }
    } else if (decoderState == STATE_READING_BITS) {
      // Validate the gap between bits
      if (pulseWidthLow < GAP_MIN || pulseWidthLow > GAP_MAX) {
        decoderState = STATE_WAITING_SYNC_HIGH; // Invalid gap size, reset
      }
    }
  } else {
    // Just went LOW -> Falling edge (duration of the previous HIGH pulse is 'duration')
    pulseWidthHigh = duration;
    
    if (decoderState == STATE_WAITING_SYNC_HIGH) {
      if (pulseWidthHigh >= SYNC_HIGH_MIN && pulseWidthHigh <= SYNC_HIGH_MAX) {
        decoderState = STATE_WAITING_SYNC_LOW;
      }
    } else if (decoderState == STATE_READING_BITS) {
      bool bitVal = false;
      if (pulseWidthHigh >= BIT_LONG_MIN && pulseWidthHigh <= BIT_LONG_MAX) {
        bitVal = true;
      } else if (pulseWidthHigh >= BIT_SHORT_MIN && pulseWidthHigh <= BIT_SHORT_MAX) {
        bitVal = false;
      } else {
        decoderState = STATE_WAITING_SYNC_HIGH; // Invalid bit pulse, reset
        return;
      }
      
      // Store bit
      uint8_t byteIdx = bitCount / 8;
      uint8_t bitIdx = 7 - (bitCount % 8);
      if (bitVal) {
        rxBuffer[byteIdx] |= (1 << bitIdx);
      }
      
      bitCount++;
      if (bitCount >= sizeof(TransmitterPayload) * 8) {
        // Complete payload received!
        if (!newPacketAvailable) {
          memcpy((void*)&rxPacket, (const void*)rxBuffer, sizeof(TransmitterPayload));
          newPacketAvailable = true;
        }
        decoderState = STATE_WAITING_SYNC_HIGH;
      }
    }
  }
}

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, relayOn ? HIGH : LOW);
  digitalWrite(PIN_LED_RELAY, relayOn ? HIGH : LOW);
}

void serviceLinkLed() {
  unsigned long now = millis();
  bool linkAlive = (lastValidPacketAt != 0) && (now - lastValidPacketAt <= LINK_ALIVE_MS);
  
  if (linkAlive) {
    digitalWrite(PIN_LED_LINK, HIGH);
  } else {
    // Blink LED if connection is lost
    if (now >= nextLinkBlinkAt) {
      linkLedState = !linkLedState;
      digitalWrite(PIN_LED_LINK, linkLedState ? HIGH : LOW);
      nextLinkBlinkAt = now + 500;
    }
  }
}

void serviceFailsafe() {
  if (!relayOn) return;
  
  unsigned long now = millis();
  if (lastValidOnAt == 0 || (now - lastValidOnAt > MOTOR_ON_TIMEOUT_MS)) {
    // Timeout! No updates received, force motor OFF
    setRelay(false);
  }
}

void processIncomingPacket() {
  if (!newPacketAvailable) return;
  
  // Calculate checksum
  uint16_t calculatedCrc = calculateCRC16((const uint8_t*)&rxPacket, 5);
  
  if (rxPacket.checksum == calculatedCrc && rxPacket.deviceId == EXPECTED_DEVICE_ID) {
    // Valid packet!
    lastValidPacketAt = millis();
    
    // Blink packet validation LED
    digitalWrite(PIN_LED_PACKET, HIGH);
    packetLedState = true;
    lastPacketLedAt = millis();
    
    if (rxPacket.command == 1) { // MOTOR_ON
      lastValidOnAt = millis();
      setRelay(true);
    } else if (rxPacket.command == 0) { // MOTOR_OFF
      setRelay(false);
    }
  }
  
  newPacketAvailable = false;
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_LED_RELAY, OUTPUT);
  pinMode(PIN_LED_PACKET, OUTPUT);
  pinMode(PIN_LED_LINK, OUTPUT);
  
  setRelay(false);
  digitalWrite(PIN_LED_PACKET, LOW);
  digitalWrite(PIN_LED_LINK, LOW);
  
  // Configure RX Pin as input with interrupts enabled
  pinMode(PIN_RX_DATA, INPUT);
  PORTA.PIN6CTRL = PORT_ISC_BOTHEDGES_gc; // Trigger on both rising and falling edges
  
  sei(); // Enable global interrupts
}

void loop() {
  processIncomingPacket();
  serviceLinkLed();
  serviceFailsafe();
  
  // Turn off packet blink LED after 100ms
  if (packetLedState && (millis() - lastPacketLedAt >= 100)) {
    digitalWrite(PIN_LED_PACKET, LOW);
    packetLedState = false;
  }
}
#endif

#include <Arduino.h>

void setup() {
}

void loop() {
}
