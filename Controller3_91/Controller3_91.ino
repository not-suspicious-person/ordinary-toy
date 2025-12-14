#include "Controller.h"

Controller newController;

void setup() {
  Serial.begin(115200);
  newController.begin();
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  newController.tick();
}
