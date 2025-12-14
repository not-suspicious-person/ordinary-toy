#include "Platform.h"

Platform newPlatform;

void setup() {
  Serial.begin(115200);
  delay(2000);
  newPlatform.begin();
}

void loop() {
  newPlatform.tick();
}
