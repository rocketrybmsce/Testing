#include <HardwareSerial.h>

// --- Pin Definitions for Ground Station Receiver ---
#define LORA_RX_PIN 16  // Receiver MCU RX -> E32 TX
#define LORA_TX_PIN 17  // Receiver MCU TX -> E32 RX

// Hardware Serial Instance for LoRa E32 Module
HardwareSerial LoRaSerial(2);

void setup() {
  // USB Serial connection to Ground Station Dashboard (Web Serial API @ 115200)
  Serial.begin(115200);

  // Hardware Serial connection to LoRa E32 Transceiver (Module pre-set to Mode 0: M0=GND, M1=GND)
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
}

void loop() {
  // Direction 1: ROCKET -> LORA -> RECEIVER -> USB -> DASHBOARD
  while (LoRaSerial.available() > 0) {
    char c = (char)LoRaSerial.read();
    Serial.write(c);
  }

  // Direction 2: DASHBOARD -> USB -> RECEIVER -> LORA -> ROCKET
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    LoRaSerial.write(c);
  }
}