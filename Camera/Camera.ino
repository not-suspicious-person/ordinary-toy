#include "Camera.h"

Camera camera;

void setup() { 
  Serial.begin(115200); 
  delay(200); 
  camera.begin(); 
}

void loop() {
}