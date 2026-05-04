#include <Arduino.h>
#include <Wire.h>
#include <LoRa_E32.h>
#include <Adafruit_BNO08x.h>

/* ==============================
   PIN DEFINITIONS & SETTINGS
   ============================== */
#define LORA_M0_PIN   2
#define LORA_M1_PIN   15
#define LORA_AUX_PIN  4

HardwareSerial& LoRaSerial = Serial2; // RX2=16, TX2=17 on ESP32
LoRa_E32 e32module(&LoRaSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN, UART_BPS_RATE_9600);

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// UPDATED: 45 Degree Cone of Safety
const float TILT_THRESHOLD_DEGREES = 45.0; 

/* ==============================
   GLOBALS
   ============================== */
bool tiltTriggered = false;
unsigned long lastTransmitTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- TX: BNO085 + LoRa Tilt Tester (45 Deg) ---");

  // LoRa Setup
  LoRaSerial.begin(9600, SERIAL_8N1, 16, 17);
  if (e32module.begin()) {
    e32module.setMode(MODE_0_NORMAL);
    Serial.println("LoRa E32: OK");
  } else {
    Serial.println("LoRa E32: INIT FAILED");
    while(1);
  }

  // BNO085 Setup
  Wire.begin();
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO085 NOT FOUND - CHECK WIRING");
    while(1);
  }

  // Enable Game Rotation Vector (No Magnetometer used) at 50ms interval (20Hz)
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 50000)) {
    Serial.println("Could not enable Game Rotation Vector");
  }
  
  Serial.println("BNO085: OK. System Ready.");
}

void loop() {
  // Check if BNO085 was reset
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 50000);
  }

  // Only run logic if new IMU data is available
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      
      // 1. Get Quaternions
      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      // 2. Convert Quaternions to Euler Angles (Roll and Pitch)
      // Roll (x-axis rotation)
      float sinr_cosp = 2 * (qr * qi + qj * qk);
      float cosr_cosp = 1 - 2 * (qi * qi + qj * qj);
      float roll = atan2(sinr_cosp, cosr_cosp) * 180.0 / PI;

      // Pitch (y-axis rotation)
      float sinp = 2 * (qr * qj - qk * qi);
      float pitch;
      if (abs(sinp) >= 1)
        pitch = copysign(90.0, sinp); // Out of range, use 90 degrees
      else
        pitch = asin(sinp) * 180.0 / PI;

      // 3. Check against Tilt Threshold
      // Assuming Z is pointing up through the rocket. If it tilts > 45deg on X or Y axis:
      if (!tiltTriggered && (abs(roll) > TILT_THRESHOLD_DEGREES || abs(pitch) > TILT_THRESHOLD_DEGREES)) {
        tiltTriggered = true;
        Serial.println("!!! TILT TRIGGERED (>45 DEGREES) !!!");
      }

      // 4. Transmit Telemetry at 5Hz (Every 200ms)
      if (millis() - lastTransmitTime >= 200) {
        lastTransmitTime = millis();
        
        // Build Telemetry String: "Pitch,Roll,TiltSignal"
        String telemetryData = String(pitch, 1) + "," + String(roll, 1) + "," + String(tiltTriggered ? 1 : 0);
        
        ResponseStatus resp = e32module.sendMessage(telemetryData);
        if(resp.code == E32_SUCCESS) {
          Serial.println("TX: " + telemetryData);
        }
      }
    }
  }
}