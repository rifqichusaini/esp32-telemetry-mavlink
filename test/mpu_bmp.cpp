#include <Arduino.h>
#include <Wire.h>
#include <math.h>
// #include <Adafruit_BMP280.h>
// #include <Adafruit_BME280.h>

// Adafruit_BME280 bme;

#include <BMP280.h>

// Headers kamu sendiri
#include "MPU6050.h"
#include "kalman.h"

// MAVLink
#include <common/mavlink.h>

// --- CONFIG ---
#define SDA_PIN 21
#define SCL_PIN 22

const uint8_t SYSTEM_ID = 1;
const uint8_t COMPONENT_ID = 200;

const float ALERT_THRESHOLD_DEG = 40.0f;
const unsigned long ALERT_DEBOUNCE_MS = 2000;

const unsigned long HEARTBEAT_MS = 1000;
const unsigned long ATT_MS       = 200;
const unsigned long ALT_MS       = 200;

// --- OBJECTS ---
MPU6050 mpu;
BMP280 bmp;
Kalman1D kalRoll;
Kalman1D kalPitch;


// --- TIMERS ---
unsigned long lastHb  = 0;
unsigned long lastAtt = 0;
unsigned long lastAlt = 0;
unsigned long lastAlert = 0;

// SEND MAVLINK THROUGH USB
void mavSend(mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    Serial.write(buf, len);
}

// HEARTBEAT
void sendHeartbeat() {
    mavlink_message_t msg;
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
    mavSend(&msg);
}

// STATUSTEXT
void sendAlert(const char *txt) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        MAV_SEVERITY_WARNING,
        txt
    );
    mavSend(&msg);
}

// ATTITUDE
void sendAttitude(float roll_rad, float pitch_rad) {
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        millis(),
        roll_rad,
        pitch_rad,
        0,     // yaw
        0,0,0  // angular velocities
    );
    mavSend(&msg);
}

// ALTITUDE
// void sendAltitude(float alt) {
//     mavlink_message_t msg;
//     mavlink_msg_altitude_pack(
//         SYSTEM_ID,
//         COMPONENT_ID,
//         &msg,
//         micros(),       // time
//         alt,            // altitude_monotonic
//         alt,            // altitude_amsl
//         0,              // local
//         alt,            // relative
//         0,              // terrain
//         0               // bottom clearance
//     );
//     mavSend(&msg);
// }

void sendAltitude(float altitude = 10) {
	mavlink_message_t msg;
	mavlink_msg_vfr_hud_pack(
    SYSTEM_ID,
    COMPONENT_ID,
    &msg,
    0,          // airspeed
    0,          // groundspeed
    0,          // heading
    0,          // throttle
    altitude, // altitude (meter)
    0           // climb rate
	);
	mavSend(&msg);
}

// --- MPU → DEGREE formulas ---
float calcRollDeg(float ax, float ay, float az) {
    return atan2f(ay, az) * 180.0f / M_PI;
}

float calcPitchDeg(float ax, float ay, float az) {
    return atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
}


// ======================================================
//                        SETUP
// ======================================================
void setup() {
    Serial.begin(115200);
    delay(300);

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(200);

    mpu.initMPU();
    delay(200);
		
    bmp.initBMP();
    delay(200);

    Serial.println("MAVLink USB + MPU6050 Ready!");
}


// ======================================================
//                        LOOP
// ======================================================
void loop() {
    unsigned long now = millis();

    // Read MPU6050
    mpu.read_raw_data();
    mpu.convert_data();
    mpu.print_data();

    float ax = mpu.get_accel_x();
    float ay = mpu.get_accel_y();
    float az = mpu.get_accel_z();

    float roll_acc_deg = calcRollDeg(ax, ay, az);
    float pitch_acc_deg = calcPitchDeg(ax, ay, az);

    float roll_deg = kalRoll.update(roll_acc_deg);
    float pitch_deg = kalPitch.update(pitch_acc_deg);

    float roll_rad = roll_deg * M_PI / 180.0f;
    float pitch_rad = pitch_deg * M_PI / 180.0f;

    // Read BMP280
    bmp.read_raw_data();
    bmp.convert_data();
    bmp.print_data();

    float altitude = bmp.get_altitude();

    // HEARTBEAT
    if (now - lastHb >= HEARTBEAT_MS) {
        lastHb = now;
        sendHeartbeat();
    }

    // ATTITUDE
    if (now - lastAtt >= ATT_MS) {
        lastAtt = now;
        sendAttitude(roll_rad, pitch_rad);
    }

    // ALTITUDE
    if (now - lastAlt >= ALT_MS) {
        lastAlt = now;
        sendAltitude(altitude);
    }

    // ALERT logic
    if (fabs(roll_deg) > ALERT_THRESHOLD_DEG) {
        if (now - lastAlert >= ALERT_DEBOUNCE_MS) {
            char msg[50];
            snprintf(msg, sizeof(msg), "ALERT: ROLL LIMIT (%.1f deg)", roll_deg);
            sendAlert(msg);
            Serial.println(msg);
            lastAlert = now;
        }
    }
}
