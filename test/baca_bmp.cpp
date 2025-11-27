#include <Wire.h>
#include <BMP280.h>

#define SDA_PIN 21
#define SCL_PIN 22

BMP280 bmp(0x76); // coba 0x76, jika gagal ganti ke 0x77

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);   // inisialisasi I2C
  delay(50);

  if (!bmp.initBMP()) {
    Serial.println("BMP init FAILED at 0x76. Trying 0x77...");
    bmp = BMP280(0x77);
    if (!bmp.initBMP()) {
      Serial.println("BMP init FAILED at 0x77 also. Check wiring/address.");
      while (1) delay(1000);
    }
  }
  Serial.println("BMP initialized OK.");
  bmp.setSeaLevel(1013.25f);  // optional
}

void loop() {
  if (bmp.read_raw_data()) {
    if (bmp.convert_data()) {
      bmp.print_data();
    } else {
      Serial.println("Convert failed");
    }
  } else {
    Serial.println("Read raw failed");
  }
  delay(1000);
}
