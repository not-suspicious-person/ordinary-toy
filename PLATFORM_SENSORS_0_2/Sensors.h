#pragma once
#include <cmath>
#include <cstdint>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_INA3221.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define ARRAY_SIZE     24
#define SENDING_DELAY  100

enum senders {
  CONTROLLER = 0,
  MOTION_PLATFORM = 1,
  SENSOR_PLATFORM = 2,
  WHO_THE_HELL_IS_THIS = -1
};

enum messages {
  DESIRED = 0,
  MOTION_REPORT = 1,
  SENSOR_REPORT = 2,
  NEW_CONTROLLER = 3,
  WHAT_THE_HELL_DO_YOU_WANT = -1
};

class Sensors {
public:
  Sensors();
  void              begin();
  void              tick();
  void              sendData(const uint8_t* mac, const void* data, size_t len);
  void              setOnDataRecvCallback(esp_now_recv_cb_t cb);
  void              setOnDataSentCallback(esp_now_send_cb_t cb);
  void              sendStateTo(const uint8_t* mac);
  void              readSensors();
  void              sendingData();
  void              formatSendingData();
  void              set_new_receiverMac(const esp_now_recv_info_t *info);
private:
  TwoWire           I2C1;
  bool              _is_ESPNOW_Initialized              = false;
  static  Sensors*  _instance;
  int16_t           _current_state[ARRAY_SIZE]          = {0};
  int16_t           _data_received[ARRAY_SIZE + 2]      = {0};
  int16_t           _data_to_send[ARRAY_SIZE + 2]       = {0};
  uint64_t          _last_time_sending                  =  0;
  //uint8_t           _receiverMac[6]                     = {0x2C, 0xBC, 0xBB, 0x06, 0xB6, 0xF0};
  uint8_t           _receiverMac[6]                     = {0x88, 0x57, 0x21, 0x23, 0x58, 0x84};
  static void       defaultOnDataSent(const uint8_t* mac, esp_now_send_status_t status);
  static void       staticOnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  void              handleDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  bool              addPeer(const uint8_t* mac);
  uint8_t           _inaAddress[2]                      = {0x40, 0x41};
  std::unique_ptr<Adafruit_INA3221> _ina3221[2];
  Adafruit_BME280   _bme;
  //Adafruit_AHTX0    _aht;
  //Adafruit_LPS25    _lps;
  float             _current[4]                         = {0};
  float             _voltage[4]                         = {0};
  float             _humidity; 
  float             _temperature;
  float             _pressure;
};