#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(4, 5);
const uint64_t txAddress = 0xF0F0F0F0D2LL;  // RX kirim ke sini
const uint64_t rxAddress = 0xF0F0F0F0E1LL;  // RX terima dari sini

struct TelemetryPacket {
  float roll;
  float pitch;
  float maxRoll;
};

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23, 5);

  radio.begin();
  radio.setPayloadSize(32);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  
  radio.openWritingPipe(txAddress);    // Kirim ke D2
  radio.openReadingPipe(1, rxAddress); // Terima dari E1
  
  radio.startListening();
  
  Serial.println("RX Ready");
}

void loop() {
  // 1. Terima telemetry
  if (radio.available()) {
    TelemetryPacket packet;
    radio.read(&packet, sizeof(packet));

    Serial.print("RX | roll: ");
    Serial.print(packet.roll);
    Serial.print(" | pitch: ");
    Serial.print(packet.pitch);
    Serial.print(" | maxRoll: ");
    Serial.println(packet.maxRoll);

    if(packet.roll > packet.maxRoll){
      Serial.print("ALERT: ROLL LIMIT ");
      Serial.print(packet.roll);
      Serial.println(" deg");
    }
  }

  // 2. Kirim command ke TX
  if(Serial.available()){
    char c = Serial.read();
    
    if(c == 'w' || c == 's'){
      radio.stopListening();
      delayMicroseconds(130);
      
      bool success = radio.write(&c, sizeof(c));
      
      if(success){
        Serial.print("TX OK: ");
        Serial.println(c);
      } else {
        Serial.println("TX FAILED!");
      }
      
      radio.startListening();
    }
  }
}