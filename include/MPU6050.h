#pragma once
#include <Arduino.h>
#include <Wire.h>

#define MPU_DATA 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

class MPU6050{
  private:
    int16_t accel_x, accel_y, accel_z;
    int16_t temp;
    int16_t gyro_x, gyro_y, gyro_z;

  public:
    float accel_x_v, accel_y_v, accel_z_v;
    float temp_v;
    float gyro_x_v, gyro_y_v, gyro_z_v;
    
    float get_accel_y(){
      return accel_y_v;
    }
    float get_accel_x(){
      return accel_x_v;
    }
    float get_accel_z(){
      return accel_z_v;
    }
    float get_gyro_x(){
      return gyro_x_v;
    }
    float get_gyro_y(){
      return gyro_y_v;
    }

    void print_data(){
      Serial.print("accel X : ");
      Serial.print(accel_x_v);
      Serial.print(" | accel Y : ");
      Serial.print(accel_y_v);
      Serial.print(" | accel Z : ");
      Serial.print(accel_z_v);
      Serial.print(" | temp : ");
      Serial.print(temp_v);
      Serial.print(" | gyro X : ");
      Serial.print(gyro_x_v);
      Serial.print(" | gyro Y : ");
      Serial.print(gyro_y_v);
      Serial.print(" | gyro Z : ");
      Serial.println(gyro_z_v);
      Serial.println("===================================================================");
    }

    void convert_data(){
      accel_x_v = accel_x / 16384.0;
      accel_y_v = accel_y / 16384.0;
      accel_z_v = accel_z / 16384.0;

      temp_v = temp / 340.0 + 36.53;

      gyro_x_v = gyro_x / 131.0;
      gyro_y_v = gyro_y / 131.0;
      gyro_z_v = gyro_z / 131.0;
    }

    void read_raw_data(){
      Wire.beginTransmission(MPU_DATA);
      Wire.write(ACCEL_XOUT_H);
      Wire.endTransmission(false);
      // Wire.requestFrom(MPU_DATA, 16, false);
      Wire.requestFrom((uint8_t)MPU_DATA, (uint8_t)14, (uint8_t)true);

      accel_x = Wire.read() << 8 | Wire.read();
      accel_y = Wire.read() << 8 | Wire.read();
      accel_z = Wire.read() << 8 | Wire.read();

      temp = Wire.read() << 8 | Wire.read();

      gyro_x = Wire.read() << 8 | Wire.read();
      gyro_y = Wire.read() << 8 | Wire.read();
      gyro_z = Wire.read() << 8 | Wire.read();
    }

    void initMPU(){
      Wire.beginTransmission(MPU_DATA);
      Wire.write(PWR_MGMT_1);
      Wire.write(0);
      Wire.endTransmission(true);
    }
};
