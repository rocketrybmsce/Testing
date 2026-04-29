#include <Arduino.h>
#include <LoRa_E32.h>

/* ==============================
   PIN DEFINITIONS & SETTINGS
   ============================== */
#define LORA_M0_PIN   2
#define LORA_M1_PIN   15
#define LORA_AUX_PIN  4

HardwareSerial& LoRaSerial = Serial2; // RX2=16, TX2=17 on ESP32
LoRa_E32 e32module(&LoRaSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN, UART_BPS_RATE_9600);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- RX: LoRa Ground Station ---");

  // LoRa Setup
  LoRaSerial.begin(9600, SERIAL_8N1, 16, 17);
  if (e32module.begin()) {
    e32module.setMode(MODE_0_NORMAL);
    Serial.println("LoRa E32: OK. Listening for data...");
  } else {
    Serial.println("LoRa E32: INIT FAILED");
    while(1);
  }
}

void loop() {
  // Check if data is available from the LoRa module
  if (e32module.available() > 1) {
    
    // Read the incoming message
    ResponseContainer rs = e32module.receiveMessage();
    
    // If successful, print and parse it
    if (rs.status.code == E32_SUCCESS) {
      String receivedData = rs.data;
      
      // Print raw data (e.g., "12.5,-4.2,1")
      Serial.print("RX: ");
      Serial.print(receivedData);
      
      // Parse the data looking for the trigger signal at the end
      int firstComma = receivedData.indexOf(',');
      int secondComma = receivedData.indexOf(',', firstComma + 1);
      
      if (firstComma > 0 && secondComma > 0) {
        String tiltSignalStr = receivedData.substring(secondComma + 1);
        
        if (tiltSignalStr.toInt() == 1) {
          Serial.print("  -> ⚠️ TILT THRESHOLD TRIGGERED!");
        }
      }
      Serial.println(); // New line
    } else {
      Serial.println("RX Error: " + String(rs.status.getResponseDescription()));
    }
  }
}