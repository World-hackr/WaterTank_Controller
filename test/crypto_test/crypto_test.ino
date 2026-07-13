#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

// XTEA 128-bit Master Key
static const uint32_t MASTER_KEY[4] = {0x7b3a91d0, 0x4c8e25f1, 0x12345678, 0x9abcdef0};

static void set_clock_full_speed() {
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0x00;
}

// XTEA 32-round Encryption
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

// XTEA 32-round Decryption
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

// Helper to print 32-bit keys in hex
static void print_key(const char* label, const uint32_t k[4]) {
  Serial.print(label);
  for (uint8_t i = 0; i < 4; i++) {
    // Print in 8-digit hex format
    for (uint8_t shift = 28; shift <= 28; shift -= 4) {
      uint8_t digit = (k[i] >> shift) & 0x0F;
      Serial.print(digit, HEX);
      if (shift == 0) break;
    }
    Serial.print(" ");
  }
  Serial.println();
}

// Helper to print 8 bytes of hex
static void print_bytes(const uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  wdt_disable();
  set_clock_full_speed();

  Serial.swap(1);
  Serial.begin(57600); // 115200 real-world baud at 20MHz clock
  
  delay(1000);
  Serial.println();
  Serial.println("================================================");
  Serial.println("      ATtiny402 Cryptographic Pipeline Test      ");
  Serial.println("================================================");

  // 1. Show the Master Key
  print_key("1. Factory Master Key:   ", MASTER_KEY);

  // 2. Extract Silicon ID (10 bytes)
  uint8_t sn[10];
  Serial.print("2. Unique Silicon ID:    ");
  for (uint8_t i = 0; i < 10; i++) {
    sn[i] = *(volatile uint8_t*)(0x1103 + i);
    if (sn[i] < 0x10) Serial.print("0");
    Serial.print(sn[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // 3. Derive the Unique 128-bit Key
  uint32_t serial_high = ((uint32_t)sn[0] << 24) | ((uint32_t)sn[1] << 16) | ((uint32_t)sn[2] << 8) | sn[3];
  uint32_t serial_low  = ((uint32_t)sn[4] << 24) | ((uint32_t)sn[5] << 16) | ((uint32_t)sn[6] << 8) | sn[7];

  uint32_t derivedKey[4];
  derivedKey[0] = MASTER_KEY[0] ^ serial_high;
  derivedKey[1] = MASTER_KEY[1] ^ serial_low;
  derivedKey[2] = MASTER_KEY[2] ^ (serial_high ^ serial_low);
  derivedKey[3] = MASTER_KEY[3] ^ (serial_high + serial_low);

  print_key("3. Derived Unique Key:   ", derivedKey);
  Serial.println("------------------------------------------------");

  // 4. Simulate pairing handshake (using Master Key)
  Serial.println("4. Simulating Pairing Handshake:");
  uint8_t pairPacket[8];
  // Fill payload with first 8 bytes of Silicon ID
  for (uint8_t i = 0; i < 8; i++) pairPacket[i] = sn[i];

  Serial.print("   Original Pairing Plaintext:  ");
  print_bytes(pairPacket, 8);

  // Encrypt with Master Key
  xtea_encrypt(32, (uint32_t*)pairPacket, MASTER_KEY);
  Serial.print("   Encrypted Ciphertext:        ");
  print_bytes(pairPacket, 8);

  // Decrypt with Master Key
  xtea_decrypt(32, (uint32_t*)pairPacket, MASTER_KEY);
  Serial.print("   Decrypted Recovery:          ");
  print_bytes(pairPacket, 8);
  
  if (pairPacket[0] == sn[0] && pairPacket[7] == sn[7]) {
    Serial.println("   [RESULT] Pairing Handshake Verification: SUCCESS!");
  } else {
    Serial.println("   [RESULT] Pairing Handshake Verification: FAILED!");
  }
  Serial.println("------------------------------------------------");

  // 5. Simulate Daily level transmission (using Derived Unique Key)
  Serial.println("5. Simulating Daily Level Transmission:");
  
  uint8_t dailyPacket[8];
  dailyPacket[0] = 0x01; // MSG_AUTO_REPORT
  dailyPacket[1] = 0x42; // Sequence ID (66)
  dailyPacket[2] = 0x03; // Water level 3
  dailyPacket[3] = 0xDC; // Battery code (220)
  dailyPacket[4] = 0x01; // Flag Level Changed
  dailyPacket[5] = sn[8]; // Last 3 bytes of Silicon ID (Binding check)
  dailyPacket[6] = sn[9];
  dailyPacket[7] = 0xAA; // Padding/signature

  Serial.print("   Original Daily Plaintext:    ");
  print_bytes(dailyPacket, 8);

  // Encrypt with Derived Unique Key
  xtea_encrypt(32, (uint32_t*)dailyPacket, derivedKey);
  Serial.print("   Encrypted Ciphertext:        ");
  print_bytes(dailyPacket, 8);

  // Decrypt with Derived Unique Key
  xtea_decrypt(32, (uint32_t*)dailyPacket, derivedKey);
  Serial.print("   Decrypted Recovery:          ");
  print_bytes(dailyPacket, 8);

  if (dailyPacket[0] == 0x01 && dailyPacket[1] == 0x42 && dailyPacket[2] == 0x03) {
    Serial.println("   [RESULT] Daily Packet Verification: SUCCESS!");
  } else {
    Serial.println("   [RESULT] Daily Packet Verification: FAILED!");
  }
  Serial.println("================================================");
}

void loop() {
  // Pure math test runs once on boot, loop does nothing.
}
