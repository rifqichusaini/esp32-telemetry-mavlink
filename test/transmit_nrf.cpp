#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050.h>
#include <kalman.h>

#define SDA_PIN 21
#define SCL_PIN 22

RF24 radio(4, 5);
const uint64_t txAddress = 0xF0F0F0F0E1LL;  // TX kirim ke sini
const uint64_t rxAddress = 0xF0F0F0F0D2LL;  // TX terima dari sini

MPU6050 mpu;
Kalman1D kalRoll;
Kalman1D kalPitch;

float maxRoll = 40.0;

struct TelemetryPacket {
  float roll;
  float pitch;
  float maxRoll;
};

float calcRollDeg(float ax, float ay, float az) {
  return atan2f(ay, az) * 180.0f / M_PI;
}

float calcPitchDeg(float ax, float ay, float az) {
  return atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  SPI.begin(18, 19, 23, 5);

  mpu.initMPU();
  
  radio.begin();
  radio.setPayloadSize(32);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  
  radio.openWritingPipe(txAddress);    // Kirim ke E1
  radio.openReadingPipe(1, rxAddress); // Terima dari D2
  
  radio.stopListening();

  Serial.println("TX Ready");
}

void loop() {
  // 1. Baca sensor
  mpu.read_raw_data();
  mpu.convert_data();
  
  float ax = mpu.get_accel_x();
  float ay = mpu.get_accel_y();
  float az = mpu.get_accel_z();
  
  float roll_acc = calcRollDeg(ax, ay, az);
  float pitch_acc = calcPitchDeg(ax, ay, az);

  float roll_deg = kalRoll.update(roll_acc);
  float pitch_deg = kalPitch.update(pitch_acc);

  // 2. Input lokal dari Serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') {
      maxRoll += 0.5;
      Serial.print("LOCAL maxRoll: "); Serial.println(maxRoll);
    }
    else if (c == 's') {
      maxRoll -= 0.5;
      Serial.print("LOCAL maxRoll: "); Serial.println(maxRoll);
    }
  }

  // 3. Kirim telemetry
  TelemetryPacket packet;
  packet.roll = roll_deg;
  packet.pitch = pitch_deg;
  packet.maxRoll = maxRoll;

  radio.stopListening();
  delayMicroseconds(130);
  
  bool ok = radio.write(&packet, sizeof(packet));

  Serial.print("TX | roll: ");
  Serial.print(packet.roll);
  Serial.print(" | pitch: ");
  Serial.print(packet.pitch);
  Serial.print(" | maxRoll: ");
  Serial.print(packet.maxRoll);
  Serial.print(" | status: ");
  Serial.println(ok ? "OK" : "FAIL");

  // 4. Dengarkan command dari RX
  radio.startListening();
  
  unsigned long timeout = millis() + 100; // timeout pendek
  
  while(millis() < timeout){
    if(radio.available()){
      char cmd;
      radio.read(&cmd, sizeof(cmd));

      if(cmd == 'w'){
        maxRoll += 0.5;
        Serial.print("REMOTE maxRoll: "); Serial.println(maxRoll);
        break;
      }
      else if(cmd == 's'){
        maxRoll -= 0.5;
        Serial.print("REMOTE maxRoll: "); Serial.println(maxRoll);
        break;
      }
    }
  }
  
  delay(50);
}