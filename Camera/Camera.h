#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_camera.h>

// Піни камери
#define PWDN_GPIO_NUM  32 
#define RESET_GPIO_NUM -1 
#define XCLK_GPIO_NUM  0 
#define SIOD_GPIO_NUM  26 
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35 
#define Y8_GPIO_NUM    34 
#define Y7_GPIO_NUM    39 
#define Y6_GPIO_NUM    36 
#define Y5_GPIO_NUM    21 
#define Y4_GPIO_NUM    19 
#define Y3_GPIO_NUM    18 
#define Y2_GPIO_NUM    5

#define VSYNC_GPIO_NUM 25 
#define HREF_GPIO_NUM  23 
#define PCLK_GPIO_NUM  22

class Camera {
public:
  Camera() = default;
  void begin();
  void tick();   // нічого не робить, стрім у таску

private:
  static Camera* _instance;
  TaskHandle_t   _streamTaskHandle = nullptr;
  uint16_t       _serverPort       = 80;
  bool           _cam_inited       = false;

  bool connectToAP(uint32_t timeout_ms);
  void startStreamTask();
  static void streamTask(void* pv);
};