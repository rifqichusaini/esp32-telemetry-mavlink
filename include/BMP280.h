#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class BMP280 {
  private:
    uint8_t i2c_addr;
    bool initialized = false;

    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    int32_t t_fine;
    int32_t raw_temp;
    int32_t raw_press;

    float temperature = 0.0f;
    float pressure_hPa = 0.0f;
    float altitude = 0.0f;
    float seaLevel_hPa = 1013.25f;

    bool write8bit(uint8_t reg, uint8_t value) {
      Wire.beginTransmission(i2c_addr);
      Wire.write(reg);
      Wire.write(value);
      return (Wire.endTransmission() == 0);
    }

    // Simple stop-then-read helper (keeps code compact)
    bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
      Wire.beginTransmission(i2c_addr);
      Wire.write(reg);
      if (Wire.endTransmission() != 0) return false;
      delayMicroseconds(50);
      uint8_t got = Wire.requestFrom(i2c_addr, len);
      if (got < len) return false;
      for (uint8_t i=0;i<len;i++) buf[i] = Wire.read();
      return true;
    }

    bool readCalibration() {
      uint8_t buf[26];
      if (!readBytes(0x88, buf, 26)) return false;

      dig_T1 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
      dig_T2 = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
      dig_T3 = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
      dig_P1 = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
      dig_P2 = (int16_t)((uint16_t)buf[8] | ((uint16_t)buf[9] << 8));
      dig_P3 = (int16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8));
      dig_P4 = (int16_t)((uint16_t)buf[12] | ((uint16_t)buf[13] << 8));
      dig_P5 = (int16_t)((uint16_t)buf[14] | ((uint16_t)buf[15] << 8));
      dig_P6 = (int16_t)((uint16_t)buf[16] | ((uint16_t)buf[17] << 8));
      dig_P7 = (int16_t)((uint16_t)buf[18] | ((uint16_t)buf[19] << 8));
      dig_P8 = (int16_t)((uint16_t)buf[20] | ((uint16_t)buf[21] << 8));
      dig_P9 = (int16_t)((uint16_t)buf[22] | ((uint16_t)buf[23] << 8));

      return (dig_T1 != 0 && dig_P1 != 0);
    }

  public:
    BMP280(uint8_t addr = 0x76) : i2c_addr(addr) {}

    bool initBMP(bool doWireBegin = false) {
      if (doWireBegin) Wire.begin();

      write8bit(0xE0, 0xB6); // soft reset
      delay(10);

      uint8_t id;
      if (!readBytes(0xD0, &id, 1)) return false;
      if (id != 0x58 && id != 0x60) return false;

      if (!readCalibration()) return false;

      // Safe defaults (Adafruit-like)
      write8bit(0xF4, 0x57); // osrs_t x2, osrs_p x16, normal
      delay(1);
      write8bit(0xF5, 0x14); // t_sb 125ms, filter x4
      delay(1);

      return (initialized = true);
    }

    bool read_raw_data() {
      if (!initialized) return false;
      uint8_t buf[6];
      if (!readBytes(0xF7, buf, 6)) return false;

      raw_press = (int32_t)( ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((uint32_t)buf[2] >> 4) );
      raw_temp  = (int32_t)( ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | ((uint32_t)buf[5] >> 4) );
      return true;
    }

    bool convert_data(float seaLevel = NAN) {
      if (!initialized) return false;
      if (isnan(seaLevel)) seaLevel = seaLevel_hPa;

      int32_t var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
      int32_t var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
      t_fine = var1 + var2;
      int32_t T = (t_fine * 5 + 128) >> 8;
      temperature = T / 100.0f;

      int64_t v1 = (int64_t)t_fine - 128000;
      int64_t v2 = v1 * v1 * (int64_t)dig_P6;
      v2 += (v1 * (int64_t)dig_P5) << 17;
      v2 += ((int64_t)dig_P4) << 35;
      v1 = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2 << 12));
      v1 = (((((int64_t)1) << 47) + v1) * (int64_t)dig_P1) >> 33;
      if (v1 == 0) { pressure_hPa = 0; altitude = 0; return false; }

      int64_t p = 1048576 - raw_press;
      p = (((p << 31) - v2) * 3125) / v1;
      v1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
      v2 = ((int64_t)dig_P8 * p) >> 19;
      p = ((p + v1 + v2) >> 8) + (((int64_t)dig_P7) << 4);

      pressure_hPa = (float)p / 25600.0f;
      altitude = 44330.0f * (1.0f - powf(pressure_hPa / seaLevel, 0.1903f));
      return true;
    }

    void print_data() {
      Serial.print("Temp: "); Serial.print(temperature);
      Serial.print(" C | Pressure: "); Serial.print(pressure_hPa);
      Serial.print(" hPa | Altitude: "); Serial.print(altitude); Serial.println(" m");
      Serial.println("=========================================================");
    }

    float get_temperature() const { return temperature; }
    float get_pressure() const { return pressure_hPa; }
    float get_altitude() const { return altitude; }
    bool isInitialized() const { return initialized; }
    void setSeaLevel(float sea) { seaLevel_hPa = sea; }
};
