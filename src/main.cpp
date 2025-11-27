#include <UAV.h>

UAV uav;

void setup() {
  uav.initUAV();
}

void loop() {
  uav.systemUAV();
}