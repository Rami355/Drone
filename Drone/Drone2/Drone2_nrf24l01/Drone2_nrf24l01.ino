#include "Drone2_nrf24l01.h"

void setup() {
  Serial.begin(500000);
  antenna_setup();
}

void loop() {
  antenna_read();
  Serial.println(potFromNano);
}
