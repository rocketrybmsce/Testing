#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_BMP390.h>
#include <MPU9250_WE.h>       // Wolle's MPU9250_WE Library
#include <TinyGPS++.h>
#include <INA226_WE.h>       // INA226 library by Wolle Waldläufer

// --- Hardware Pin Definitions (ESP32-S3) ---
#define I2C_SDA          8   // PCB 1 <-> PCB 2 I2C SDA
#define I2C_SCL          9   // PCB 1 <-> PCB 2 I2C SCL

#define LORA_RX_PIN      10  // ESP32 RX -> E32 TX
#define LORA_TX_PIN      11  // ESP32 TX -> E32 RX

#define GPS_RX_PIN       24  // ESP32 RX -> GPS TX (U4 NEO-M8N)
#define GPS_TX_PIN       25  // ESP32 TX -> GPS RX (U4 NEO-M8N)

#define PYRO_PIN         2   // GPIO 2 -> TC4427 IN_A (Pyro Driver)

#define MPU9250_ADDR     0x68
#define SEALEVELPRESSURE_HPA (1013.25)

// --- Fixed State & System Flags ---
const int CURRENT_STATE = 1; // Default State 1 ("Motor Ignited" / Pad Ready)
bool isSystemReady = false;  // Lockout flag until $GTR,Ready command is received

// --- Peripherals & Communication Interfaces ---
HardwareSerial LoRaSerial(1); // UART1 for E32 LoRa module
HardwareSerial GpsSerial(2);  // UART2 for NEO-M8N GPS module

Adafruit_BMP390 bmp;
MPU9250_WE myMPU9250 = MPU9250_WE(&Wire, MPU9250_ADDR);
TinyGPSPlus gps;
INA226_WE ina226(0x40);

// --- Altitude Baseline ---
float baseAltitude = 0.0;

// Non-blocking telemetry timing
unsigned long lastPacket1Time = 0;
unsigned long lastPacket2Time = 0;
const unsigned long PACKET1_INTERVAL = 100; // 10 Hz for IMU/Baro
const unsigned long PACKET2_INTERVAL = 200; // 5 Hz for GPS/Power

void handleGroundCommands();
void sendPacket1();
void sendPacket2();

void setup() {
  Serial.begin(115200);

  // Pyro Driver Pin Setup (Kept LOW / Safe)
  pinMode(PYRO_PIN, OUTPUT);
  digitalWrite(PYRO_PIN, LOW);

  // Initialize Hardware Serials
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  GpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize I2C Bus on GPIO 8 & 9
  Wire.begin(I2C_SDA, I2C_SCL);

  // 1. Initialize BMP390 Barometer
  if (bmp.begin_I2C(0x77, &Wire) || bmp.begin_I2C(0x76, &Wire)) {
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Calibrate baseline ground altitude
    float sum = 0;
    for (int i = 0; i < 20; i++) {
      if (bmp.performReading()) {
        sum += bmp.readAltitude(SEALEVELPRESSURE_HPA);
      }
      delay(20);
    }
    baseAltitude = sum / 20.0;
  }

  // 2. Initialize MPU9250 IMU
  if (myMPU9250.init()) {
    myMPU9250.autoOffsets();                     // Zero sensor on flat launch pad
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_16G); // 16G range required for launch
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_2);        // 92 Hz bandwidth (~7.8 ms delay)
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_2000DPS);
  }

  // 3. Initialize INA226 Power Monitor
  if (ina226.init()) {
    ina226.setAverage(INA226_AVERAGES_16);
    ina226.setConversionTime(INA226_CONV_TIME_1100);
  }
}

void loop() {
  // Always ingest incoming GPS bytes
  while (GpsSerial.available() > 0) {
    gps.encode(GpsSerial.read());
  }

  // Handle Ground Station Commands ($GTR,Check & $GTR,Ready)
  handleGroundCommands();

  // Stream Telemetry Packets only after $GTR,Ready command is received
  if (isSystemReady) {
    if (millis() - lastPacket1Time >= PACKET1_INTERVAL) {
      lastPacket1Time = millis();
      sendPacket1();
    }

    if (millis() - lastPacket2Time >= PACKET2_INTERVAL) {
      lastPacket2Time = millis();
      sendPacket2();
    }
  }
}

// Process Incoming Pre-Launch Handshake Commands
void handleGroundCommands() {
  while (LoRaSerial.available() > 0) {
    String cmd = LoRaSerial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "$GTR,Check") {
      if (bmp.performReading()) {
        LoRaSerial.print("$RTG,1\n");
      }
      delay(50);

      xyzFloat testG = myMPU9250.getGValues();
      if (testG.x != 0.0 || testG.y != 0.0 || testG.z != 0.0) {
        LoRaSerial.print("$RTG,2\n");
      }
      delay(50);

      if (GpsSerial.available() >= 0) {
        LoRaSerial.print("$RTG,3\n");
      }
    } 
    else if (cmd == "$GTR,Ready") {
      isSystemReady = true; // Unlock telemetry broadcasting
    }
  }
}

// Telemetry Packet 1: $RTG,state,1,vx,vy,vz,ax,ay,az,roll,pitch,yaw,alt,pressure
void sendPacket1() {
  xyzFloat gVal = myMPU9250.getGValues();
  float ax = gVal.x * 9.81; // Acceleration in m/s²
  float ay = gVal.y * 9.81;
  float az = gVal.z * 9.81;

  xyzFloat angle = myMPU9250.getAngles();
  float roll  = angle.x;
  float pitch = angle.y;
  float yaw   = angle.z;

  float altitude = 0.0;
  float pressure = 0.0;

  if (bmp.performReading()) {
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - baseAltitude;
    pressure = bmp.pressure; // Pascals
  }

  float vx = 0.0, vy = 0.0, vz = 0.0;

  char packet1[128];
  snprintf(packet1, sizeof(packet1),
    "$RTG,%d,1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    CURRENT_STATE,
    vx, vy, vz,
    ax, ay, az,
    roll, pitch, yaw,
    altitude, pressure
  );

  LoRaSerial.print(packet1);
}

// Telemetry Packet 2: $RTG,state,2,lat,lon,vbat,current,t1,t2
void sendPacket2() {
  float lat = gps.location.isValid() ? gps.location.lat() : 0.000000;
  float lon = gps.location.isValid() ? gps.location.lng() : 0.000000;

  float vbat = ina226.getBusVoltage_V();
  float current = ina226.getCurrent_mA() / 1000.0; // Amperes

  float t1 = bmp.temperature;
  float t2 = 0.0;

  char packet2[128];
  snprintf(packet2, sizeof(packet2),
    "$RTG,%d,2,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f\n",
    CURRENT_STATE,
    lat, lon,
    vbat, current,
    t1, t2
  );

  LoRaSerial.print(packet2);
}