#pragma once

class Kalman1D {
  private:
    float Q; // process noise covariance
    float R; // measurement noise covariance
    float x; // state estimate
    float P; // estimation error covariance

  public:
    Kalman1D(float q = 0.001f, float r = 0.03f, float initial = 0.0f) {
      Q = q; R = r; x = initial;
      P = 1.0f; // initial estimation error
    }

    void setQ(float q){ Q = q; }
    void setR(float r){ R = r; }
    float getState(){ return x; }

    // z = measurement
    float update(float z){
      // Predict step
      P = P + Q;

      // Kalman gain
      float K = P / (P + R);

      // Update estimate with measurement z
      x = x + K * (z - x);

      // Update error covariance
      P = (1 - K) * P;

      return x;
    }
};
