// header bawaan
#include <Arduino.h>
#include <Wire.h>

#include <common/mavlink.h>
#include <common/common.h>

float pitch_deg = 30.0;
float roll_deg  = 45.0;

const uint8_t SYSTEM_ID = 1;
const uint8_t COMPONENT_ID = 200;

void sendMav(mavlink_message_t* dataFrame){
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, dataFrame);
  Serial.write(buffer, len);
}

void sendHeartbeat(){
  mavlink_message_t dataFrame;
  mavlink_msg_heartbeat_pack(
    SYSTEM_ID,
    COMPONENT_ID,
    &dataFrame,
    MAV_TYPE_GENERIC,
    MAV_AUTOPILOT_GENERIC,
    MAV_MODE_MANUAL_DISARMED,
    0,
    MAV_STATE_ACTIVE
  );
  sendMav(&dataFrame);
}

void sendText(const char* text){
  mavlink_message_t dataFrame;
  mavlink_msg_statustext_pack(
    SYSTEM_ID,
    COMPONENT_ID,
    &dataFrame,
    MAV_SEVERITY_WARNING,
    text
  );
  sendMav(&dataFrame);
}

void setup(){
  Serial.begin(115200);
  delay(300);
}

void loop(){
  sendHeartbeat();
  delay(1000);

  char text[50];
  snprintf(text, sizeof(text), "Roll: %f, Pitch: %f", roll_deg, pitch_deg);
  delay(2000);
}