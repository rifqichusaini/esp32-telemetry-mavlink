// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Include your headers (you uploaded these)
#include "MPU6050.h"    // :contentReference[oaicite:2]{index=2}
#include "kalman.h"     // :contentReference[oaicite:3]{index=3}

// Include MAVLink common (ensure lib/mavlink exists in your project)
#include <common/mavlink.h>

// --- Configuration ---
#define SDA_PIN 21
#define SCL_PIN 22

// UART1 (Serial1) pins for Holybro SiK
#define SIK_RX_PIN 16  // TX-radio -> RX1 (GPIO16)
#define SIK_TX_PIN 17  // RX-radio -> TX1 (GPIO17)
#define SIK_BAUD 57600

// MAVLink IDs
const uint8_t SYSTEM_ID = 1;
const uint8_t COMPONENT_ID = 200;

const unsigned long ATTITUDE_PERIOD_MS = 20; // ~50 Hz
const unsigned long HEARTBEAT_PERIOD_MS = 1000; // 1 Hz
const float ALERT_THRESHOLD_DEG = 40.0f; // fixed per requirement
const unsigned long ALERT_DEBOUNCE_MS = 2000; // prevent spam

// --- Objects ---
MPU6050 mpu;
Kalman1D kalmanRoll(0.001f, 0.03f, 0.0f);
Kalman1D kalmanPitch(0.001f, 0.03f, 0.0f);

// --- State ---
unsigned long lastAttitudeTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastAlertTime = 0;
bool alertActive = false;

void sendMavlinkMessage(mavlink_message_t* msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  Serial2.write(buf, len);
}

void sendHeartbeat() {
  mavlink_message_t msg;
  // pack heartbeat (use generic autopilot)
  mavlink_msg_heartbeat_pack(
    SYSTEM_ID,
    COMPONENT_ID,
    &msg,
    MAV_TYPE_GENERIC,
    MAV_AUTOPILOT_GENERIC,
    MAV_MODE_MANUAL_ARMED,
    0,
    MAV_STATE_ACTIVE
  );
  sendMavlinkMessage(&msg);
}

void sendAttitude(float roll_rad, float pitch_rad, float yaw_rad, float rollspeed, float pitchspeed, float yawspeed) {
  mavlink_message_t msg;
  uint32_t time_boot_ms = millis();
  mavlink_msg_attitude_pack(
    SYSTEM_ID,
    COMPONENT_ID,
    &msg,
    time_boot_ms,
    roll_rad,
    pitch_rad,
    yaw_rad,
    rollspeed,
    pitchspeed,
    yawspeed
  );
  sendMavlinkMessage(&msg);
}

void sendStatustext(uint8_t severity, const char* text) {
  mavlink_message_t msg;
  // statustext text must be null-terminated and <= 50 chars typically
  mavlink_msg_statustext_pack(
    SYSTEM_ID,
    COMPONENT_ID,
    &msg,
    severity,
    text
  );
  sendMavlinkMessage(&msg);
}

void setup() {
  // USB serial for debug (optional)
  Serial.begin(115200);
  delay(100);

  // Init I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(10);

  // Init MPU
  mpu.initMPU();
  delay(10);

  // Init Serial1 to SiK radio
  Serial2.begin(SIK_BAUD);
  delay(10);

  // Warm-up
  lastAttitudeTime = millis();
  lastHeartbeatTime = millis();

  Serial.println("ESP32 MAVLink Attitude (MPU6050 + Kalman) starting...");
  // send initial heartbeat immediately
  sendHeartbeat();
}

float accelRollDeg(float ax, float ay, float az) {
  // roll = atan2(accel_y, accel_z)
  float roll = atan2f(ay, az) * 180.0f / M_PI;
  return roll;
}

float accelPitchDeg(float ax, float ay, float az) {
  // pitch = atan2(-accel_x, sqrt(accel_y^2 + accel_z^2))
  float pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
  return pitch;
}

void loop() {
  unsigned long now = millis();

  // 1) Read sensors as fast as possible, but send MAVLink at fixed rates
  mpu.read_raw_data();
  mpu.convert_data(); // populates accel_x_v, accel_y_v, accel_z_v, gyro_x_v, gyro_y_v, gyro_z_v

  // Heartbeat 1 Hz
  if (now - lastHeartbeatTime >= HEARTBEAT_PERIOD_MS) {
    lastHeartbeatTime = now;
    sendHeartbeat();
  }

  // Attitude update at ATTITUDE_PERIOD_MS (~50 Hz)
  if (now - lastAttitudeTime >= ATTITUDE_PERIOD_MS) {
    lastAttitudeTime = now;

    // compute accel-based angles (degrees)
    float ax = mpu.get_accel_x();
    float ay = mpu.get_accel_y();
    float az = mpu.get_accel_z();

    float roll_acc_deg = accelRollDeg(ax, ay, az);
    float pitch_acc_deg = accelPitchDeg(ax, ay, az);

    // run Kalman filter (your 1D Kalman on accel angle)
    float roll_filtered_deg = kalmanRoll.update(roll_acc_deg);
    float pitch_filtered_deg = kalmanPitch.update(pitch_acc_deg);

    // convert to radians for MAVLink
    float roll_rad = roll_filtered_deg * M_PI / 180.0f;
    float pitch_rad = pitch_filtered_deg * M_PI / 180.0f;
    float yaw_rad = 0.0f; // no magnetometer/yaw available

    // we don't compute angular rates here (set 0). If you want, estimate from gyro.
    float rollspeed = 0.0f;
    float pitchspeed = 0.0f;
    float yawspeed = 0.0f;

    // send ATTITUDE via MAVLink to SiK radio
    sendAttitude(roll_rad, pitch_rad, yaw_rad, rollspeed, pitchspeed, yawspeed);

    // Alert logic: crossing threshold (deg)
    static bool wasAbove = false;
    bool isAbove = (fabsf(roll_filtered_deg) > ALERT_THRESHOLD_DEG);

    if (isAbove && !wasAbove) {
      // just crossed above threshold
      unsigned long sinceAlert = now - lastAlertTime;
      if (sinceAlert >= ALERT_DEBOUNCE_MS) {
        char txt[50];
        // format alert text
        dtostrf(roll_filtered_deg, 4, 1, txt); // convert float to string part
        // build descriptive text
        char msgtxt[80];
        snprintf(msgtxt, sizeof(msgtxt), "ALERT: ROLL LIMIT (%.1f deg)", roll_filtered_deg);
        sendStatustext(MAV_SEVERITY_WARNING, msgtxt);
        lastAlertTime = now;
        Serial.println(msgtxt);
      }
    }

    // update state
    wasAbove = isAbove;
  }

  // small delay to avoid busy loop (keeps reads frequent)
  delay(2);
}
