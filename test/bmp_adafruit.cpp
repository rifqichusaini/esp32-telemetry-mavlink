#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Inisialisasi objek BMP280
Adafruit_BMP280 bmp; // menggunakan I2C
float ketinggianRaw = 0;
float tekanan=0;
float suhu=0;

void setup() {
  Serial.begin(115200);
  Serial.println("BMP280 Test dengan ESP32");

  // Inisialisasi sensor
  if (!bmp.begin(0x76)) {  // Alamat I2C default: 0x76 atau 0x77
    Serial.println("Sensor BMP280 tidak ditemukan!");
    Serial.println("Coba ganti alamat ke 0x77 jika tidak berhasil");
    while (1);  // Berhenti di sini jika sensor tidak terdeteksi
  }

  // Konfigurasi sensor (opsional)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Mode operasi
                  Adafruit_BMP280::SAMPLING_X2,     // Oversampling suhu
                  Adafruit_BMP280::SAMPLING_X16,    // Oversampling tekanan
                  Adafruit_BMP280::FILTER_X16,      // Filter
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time

  Serial.println("Sensor BMP280 berhasil diinisialisasi!");
  delay(2000);

  suhu = bmp.readTemperature();
  tekanan = bmp.readPressure() / 100.0F;  // Konversi Pa ke hPa
  ketinggianRaw = (bmp.readAltitude(1013.25)); // Tekanan permukaan laut standar
  Serial.println(ketinggianRaw);
  Serial.println("Done");
}

void loop() {
  // Baca data dari sensor
  float suhu = bmp.readTemperature();
  float tekanan = bmp.readPressure() / 100.0F;  // Konversi Pa ke hPa
  
  float ketinggian = (bmp.readAltitude(1013.25) - ketinggianRaw); // Tekanan permukaan laut standar

  // Tampilkan hasil di Serial Monitor
  Serial.println("=================================");
  Serial.print("Suhu: ");
  Serial.print(suhu);
  Serial.println(" °C");

  Serial.print("Tekanan: ");
  Serial.print(tekanan);
  Serial.println(" hPa");

  Serial.print("Perkiraan Ketinggian: ");
  Serial.print(ketinggian);
  Serial.println(" meter");
  Serial.println("=================================");

  delay(2000);  // Baca setiap 2 detik
}