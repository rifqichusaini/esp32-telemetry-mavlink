#include <Wire.h>
#include <BMP280.h>
#include <MPU6050.h>
#include <kalman.h>

#define SDA_PIN 21
#define SCL_PIN 22

BMP280 bmp(0x76);
MPU6050 mpu;
Kalman1D kalRoll;
Kalman1D kalPitch;

float altitudeAwal = 0;
int isCalibrated = 0;

float maxRoll = 40.0;
float maxAlt = 100.0;

float calcRollDeg(float ax, float ay, float az) {
  return atan2f(ay, az) * 180.0f / M_PI;
}

float calcPitchDeg(float ax, float ay, float az) {
  return atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Initialized");
  delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);   // inisialisasi I2C
  delay(50);
  
  bmp.begin();
  bmp.setSeaLevel(1013.25f);  // optional
  delay(50);
  
  mpu.initMPU();
  delay(50);
  
  Serial2.begin(57600);
  Serial.println("Serial2 Initialized");
  Serial2.println("Serial2 Initialized");
  delay(10);
}

void loop() {
  bmp.read_raw_data();
  bmp.convert_data();

  if(isCalibrated < 2){
    altitudeAwal = bmp.get_altitude();
    isCalibrated++;
  }

  float altitude = bmp.get_altitude() - altitudeAwal;
  
  mpu.read_raw_data();
  mpu.convert_data();
  
  float ax = mpu.get_accel_x();
  float ay = mpu.get_accel_y();
  float az = mpu.get_accel_z();
  
  float roll_acc_deg = calcRollDeg(ax, ay, az);
  float pitch_acc_deg = calcPitchDeg(ax, ay, az);

  float roll_deg = kalRoll.update(roll_acc_deg);
  float pitch_deg = kalPitch.update(pitch_acc_deg);

  if (Serial2.available()) {
    char c = Serial2.read();
    if (c == 'w') {
      maxRoll += 0.5;
      Serial2.print("maxRoll naik ke : "); Serial2.println(maxRoll);
    }
    else if (c == 's') {
      maxRoll -= 0.5;
      if (maxRoll < 0) maxRoll = 0;
      Serial2.print("maxRoll turun ke : "); Serial2.println(maxRoll);
    }
    else if (c == 'q') {
      maxAlt += 0.5;
      Serial2.print("maxAlt naik ke : "); Serial2.println(maxAlt);
    }
    else if (c == 'a') {
      maxAlt -= 0.5;
      if (maxAlt < 0) maxAlt = 0;
      Serial2.print("maxAlt turun ke : "); Serial2.println(maxAlt);
    }
  }

  if(roll_deg > maxRoll){
    Serial2.print("ALERT: ROLL LIMIT ");
    Serial2.print(roll_deg);
    Serial2.println(" deg");
  }
  if(altitude > maxAlt){
    Serial2.print("ALERT: ALTITUDE LIMIT ");
    Serial2.print(altitude);
    Serial2.println(" m");  
  }
  
  Serial2.print("Roll: "); Serial2.print(roll_deg); Serial2.print(" deg | ");
  Serial2.print("Pitch: "); Serial2.print(pitch_deg); Serial2.print(" deg | ");
  Serial2.print("Altitude: "); Serial2.print(altitude); Serial2.println(" m");

  delay(100);
}