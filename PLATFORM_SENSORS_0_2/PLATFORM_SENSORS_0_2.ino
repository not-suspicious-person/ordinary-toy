#include "Sensors.h"

Sensors NewSensors;

void setup() {
  Serial.begin(115200);
  delay(100);
  NewSensors.begin();
}

void loop() {
  NewSensors.tick();

}
