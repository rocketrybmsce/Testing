#include <Arduino.h>
#include <Wire.h>
#include <LoRa_E32.h>
#include <Adafruit_BMP3XX.h>

/* ==============================
   PIN DEFINITIONS & SETTINGS
   ============================== */
#define LORA_M0_PIN   2
#define LORA_M1_PIN   15
#define LORA_AUX_PIN  4

HardwareSerial& LoRaSerial = Serial2; // RX2=16, TX2=17 on ESP32
LoRa_E32 e32module(&LoRaSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN, UART_BPS_RATE_9600);

Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_HPA 1013.25
const float APOGEE_ALTITUDE_TOLERANCE = 3.0; // 3 meters drop to detect apogee

/* ==============================
   GLOBALS
   ============================== */
float groundAltitudeOffset = 0.0;
float maxAltitude = 0.0;
bool apogeeDetected = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- TX: BMP390 + LoRa Apogee Tester ---");

  // LoRa Setup
  LoRaSerial.begin(9600, SERIAL_8N1, 16, 17);
  if (e32module.begin()) {
    e32module.setMode(MODE_0_NORMAL);
    Serial.println("LoRa E32: OK");
  } else {
    Serial.println("LoRa E32: INIT FAILED");
    while(1);
  }

  // BMP390 Setup
  Wire.begin();
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 NOT FOUND - CHECK WIRING");
    while(1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Ground Calibration
  Serial.print("Calibrating ground altitude...");
  float sumAlt = 0.0;
  for (int i = 0; i < 10; i++) {
    bmp.performReading();
    sumAlt += bmp.readAltitude(SEALEVELPRESSURE_HPA);
    delay(50);
  }
  groundAltitudeOffset = sumAlt / 10.0;
  Serial.println(" DONE");
}

void loop() {
  if (!bmp.performReading()) return;

  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float relAltitude = altitude - groundAltitudeOffset;

  // Track max altitude
  if (altitude > maxAltitude) {
    maxAltitude = altitude;
  }

  // Detect Apogee (Altitude has dropped by the tolerance amount)
  if (!apogeeDetected && altitude < (maxAltitude - APOGEE_ALTITUDE_TOLERANCE)) { 
       apogeeDetected = true;
       Serial.println("!!! APOGEE DETECTED !!!");
  }

  // Build Telemetry String: "RelativeAltitude,ApogeeSignal"
  // ApogeeSignal is 1 if detected, 0 if not.
  String telemetryData = String(relAltitude, 2) + "," + String(apogeeDetected ? 1 : 0);

  // Transmit Data
  ResponseStatus resp = e32module.sendMessage(telemetryData);
  if(resp.code == E32_SUCCESS) {
    Serial.println("TX: " + telemetryData);
  } else {
    Serial.println("TX Failed");
  }

  delay(200); // 5Hz transmission rate
}