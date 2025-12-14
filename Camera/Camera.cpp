#include "Camera.h"

Camera* Camera::_instance = nullptr;

static uint32_t g_camLoopCounter = 0;
volatile uint32_t g_streamTick = 0;

/*
bool Camera::initEspNow() {
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Camera: esp_now_init failed");
    return false;
  }
  esp_now_register_recv_cb(Camera::onDataRecvStatic);
  return true;
}
*/

void Camera::begin() {
  _instance = this;

  Serial.begin(115200);
  delay(1000);

  // Wi-Fi STA
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  delay(100);
  Serial.print("Camera MAC: ");
  //Serial.println(WiFi.macAddress());

  // Підключення до AP пульта
  connectToAP(15000);

  // Ініціалізація камери (OV2640) – JPEG, QQVGA
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;          // стандартно 20 МГц
  config.pixel_format = PIXFORMAT_JPEG;    // ВАЖЛИВО: JPEG
  config.frame_size   = FRAMESIZE_QVGA;   // 160x120
  config.jpeg_quality = 20;               // 10–30 (20 – компроміс)
  config.fb_count     = 1;                // один фрейм-буфер

  if (config.pin_pwdn >= 0) {
    pinMode(config.pin_pwdn, OUTPUT);
    digitalWrite(config.pin_pwdn, LOW);
    delay(10);
  }

  if (esp_camera_init(&config) != ESP_OK) {
    //Serial.println("Camera: esp_camera_init failed");
    return;
  }

  _cam_inited = true;
  //Serial.println("Camera: initialized");

  startStreamTask();
}

void Camera::tick() {
}

bool Camera::connectToAP(uint32_t timeout_ms) {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Camera: already connected, IP=");
    //Serial.println(WiFi.localIP());
    Serial.print("GW=");
    //Serial.println(WiFi.gatewayIP());
    return true;
  }

  const char* SSID = "LABETA_CTRL";
  const char* PASS = "";

  Serial.print("Camera: connecting to AP SSID=");
  //Serial.println(SSID);

  WiFi.disconnect(true, true);
  delay(200);
  WiFi.setSleep(false);

  WiFi.begin(SSID, PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) {
    delay(500);
    Serial.print(".");
  }
  //Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    //Serial.println("Camera: AP connect failed");
    return false;
  }

  Serial.print("Camera: connected, IP=");
  //Serial.println(WiFi.localIP());
  Serial.print("GW=");
  //Serial.println(WiFi.gatewayIP());
  return true;
}

void Camera::startStreamTask() {
  if (_streamTaskHandle) return;
  xTaskCreatePinnedToCore(Camera::streamTask, "cam_stream",
                          12288, this, 1, &_streamTaskHandle, 1);
}

void Camera::streamTask(void* pv) {
  Camera* self = static_cast<Camera*>(pv);
  //Serial.println("Camera: streamTask started");

  static uint32_t frameCounter    = 0;
  static uint32_t g_camLoopCounter = 0;

  for (;;) {
    g_camLoopCounter++;
    if ((g_camLoopCounter % 1000) == 0) {
      //Serial.println("Camera: streamTask alive");
    }

    // 1. Wi-Fi підключення
    if (WiFi.status() != WL_CONNECTED) {
      //Serial.println("Camera: WiFi not connected, reconnecting...");
      self->connectToAP(8000);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    IPAddress host = WiFi.gatewayIP();
    if (host == INADDR_NONE) {
      //Serial.println("Camera: invalid gateway IP");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // 2. Встановлення TCP-підключення до пульта
    WiFiClient client;
    uint16_t port = self->_serverPort ? self->_serverPort : 80;
    //Serial.printf("Camera: try long TCP connect to %s:%u\n",
    //              host.toString().c_str(), port);
    if (!client.connect(host, port)) {
      //Serial.println("Camera: long TCP connect failed");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }
    client.setNoDelay(true);
    //Serial.println("Camera: long TCP connection established");

    // 3. Основний цикл відправки JPEG-кадрів
    while (client.connected()) {
      g_streamTick++;
      if (WiFi.status() != WL_CONNECTED) {
        //Serial.println("Camera: WiFi lost");
        break;
      }

      //Serial.printf("Camera: before capture, frame #%u\n", frameCounter);

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        //Serial.println("Camera: capture failed");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        continue;
      }

      // fb->buf – JPEG-дані, fb->len – довжина
      uint32_t frameLen = fb->len;
      //Serial.printf("Camera: got fb, len=%u, frame #%u\n",
      //              (unsigned)frameLen, frameCounter);

      uint8_t lenBuf[4] = {
        (uint8_t)(frameLen & 0xFF),
        (uint8_t)((frameLen >> 8) & 0xFF),
        (uint8_t)((frameLen >> 16) & 0xFF),
        (uint8_t)((frameLen >> 24) & 0xFF)
      };

      if (client.write(lenBuf, 4) != 4) {
        //Serial.println("Camera: failed to send frame length");
        esp_camera_fb_return(fb);
        break;
      }

      uint32_t sendStart = millis();
      size_t sent = 0;
      while (sent < frameLen) {
        if (millis() - sendStart > 500) {
          //Serial.println("Camera: frame send timeout, aborting frame");
          break;
        }

        size_t toSend = frameLen - sent;
        if (toSend > 1024) toSend = 1024;
        size_t chunk = client.write(fb->buf + sent, toSend);
        if (chunk == 0) {
          //Serial.println("Camera: write returned 0");
          break;
        }
        sent += chunk;
        yield();
      }

      Serial.printf("Camera: JPEG frame sent %u/%u bytes, #%u\n",
                    (unsigned)sent, (unsigned)frameLen, frameCounter);
      frameCounter++;
      esp_camera_fb_return(fb);

      if (sent < frameLen) {
        //Serial.println("Camera: partial frame, breaking loop");
        break;
      }

      vTaskDelay(200 / portTICK_PERIOD_MS);  // ~5 FPS
    }

    client.stop();
    //Serial.println("Camera: long TCP connection closed");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  self->_streamTaskHandle = nullptr;
  vTaskDelete(nullptr);
}