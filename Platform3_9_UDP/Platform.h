#pragma once
#include <cmath>
#include <cstdint>
#include <Arduino.h>
#include <ESP32Encoder.h>
//#include <MyMT6701.hpp>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>
#include <AS5600.h>
#include <Wire.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <WiFiUdp.h>

#define ARRAY_SIZE     16
#define SENDING_DELAY  100

#define PIN_IN1 14
#define PIN_IN2 15
#define PIN_IN3 25
#define PIN_IN4 26

#define CLK_PIN       33
#define DT_PIN        32
#define ENCODER_TYPE  360

#define POT_PIN       34

#define EEPROM_SIZE      512
#define INIT_ADR         127 
#define INIT_KEY         103

#define MT6701_ADDR 0x06

enum directions {
  RIGHT = 54,
  LEFT  = 57,
  UP    = 58,
  DOWN  = 53,
  NONE  = 0,
  NONE_SUB = 63
};

enum adjustment_type {
  FINE,
  COARSE,
  NO_ADJ
};

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
  VERTICAL_CALIBRATION = 3,
  HORIZONTAL_CALIBRATION = 4,
  NEW_CONTROLLER = 5,
  GO_STREAM = 6,
  STOP_STREAM = 7,
  WHAT_THE_HELL_DO_YOU_WANT = -1
};

struct UdpUser {
  IPAddress _UserIP;
  //bool      _IsActive;
  uint64_t  _Last_Packet_Time;
};

class MT6701 {
public:
  MT6701(TwoWire *wire = &Wire, uint16_t updateInterval = 10, uint8_t address = 0x06)
      : _wire(wire), _address(address), _updateIntervalMillis(updateInterval) {}
  bool begin();
  float readAngle();
  int getLastError() const { return _lastError; }
private:
  TwoWire *_wire;
  uint8_t _address;
  uint16_t _updateIntervalMillis;
  int _lastError = 0;
};

class Platform {
public:
  Platform();
  void            begin();
  void            tick();
  bool            initiation_EEPROM();
  void            sendData(const uint8_t* mac, const void* data, size_t len);
  void            setOnDataRecvCallback(esp_now_recv_cb_t cb);
  void            setOnDataSentCallback(esp_now_send_cb_t cb);
  void            sendStateTo(const uint8_t* mac);
  void            sendingData();
  void            movePlatform(directions direction);
  void            movePlatform();
  directions      defineDirection();
  directions      defineSubdirection(directions direction);
  adjustment_type defineAdjustment();
  bool            isDirectionSame();
  void            readSensors();
  void            readAS5600Encoder();
  void            readMT6701Encoder();
  void            defineStopFlags();
  void            newDiectionReport(directions direction);
  int16_t         horizontal_angle_rescale(int16_t angle);
  void            set_vertical_calibration();
  void            set_horizontal_calibration();
  void            set_new_receiverMac(const esp_now_recv_info_t *info);
  void            saveSensorsMac(const uint8_t mac[6]);
  void            saveCameraMac(const uint8_t mac[6]);
  bool            checkCotrollerMAC(const esp_now_recv_info_t *info);
  uint8_t         calculatePID(directions direction);
  float           readAS5600Angle();
  static void     encoderTask(void *pvParameters);
  void            startPIDTask();
  void            PIDTask();
  static void     PIDTaskWrapper(void *param);
  void            CheckingAP();
  void            ReadingUDP();
  void            sendCurrentValues(IPAddress IP);

private:
  bool                    _is_ESPNOW_Initialized              = false;
  static Platform*        _instance;
  int16_t                 _current_state[ARRAY_SIZE]          = {0};
  int16_t                 _desired_state[ARRAY_SIZE]          = {0};
  int16_t                 _data_received[ARRAY_SIZE + 2]      = {0};
  int16_t                 _data_to_send[ARRAY_SIZE + 2]       = {0};
  uint64_t                _last_time_sending                  =  0;
  //uint8_t                 _receiverMac[6]                     = {0x2C, 0xBC, 0xBB, 0x06, 0xB6, 0xF0};
  uint8_t                 _receiverMac[6]                     = {0x2C, 0xBC, 0xBB, 0x06, 0xB6, 0xF0};         //Wrong MAC for TEST only
  uint8_t                 _sensorsMac[6]                      = {0};
  uint8_t                 _cameraMac[6]                       = {0};
  static void             defaultOnDataSent(const uint8_t* mac, esp_now_send_status_t status);
  static void             staticOnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  void                    handleDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  bool                    addPeer(const uint8_t* mac);
  directions              _direction                          = NONE;
  directions              _new_direction                      = NONE;
  directions              _subdirection                       = NONE_SUB;
  adjustment_type         _adjustment                         = NO_ADJ;
  ESP32Encoder            _optic_Encoder;
  SemaphoreHandle_t       _i2cMutex;
  AS5600                  _as5600;
  MT6701                  _mt6701;
  float                   _as5600_data                        = 0;
  float                   _mt6701_data                        = 0;
  uint16_t                _pwm_freq                           = 5000;
  uint8_t                 _pwm_resolution                     = 8;
  bool                    _move_start_flag                    = false;
  uint64_t                _move_start_time                    = 0;
  bool                    _stop_Flags[5]                      = {false};
  bool                    _desired_changed                    = false;
  bool                    _second_chance                      = false;
  bool                    _charge_flag                        = false;
  //[0] - full stop; [1] - stop right; [2] - stop left; [3] - up; [4] - down;
  const int16_t           _min_Y                               = 1644;
  const int16_t           _max_Y                               = 2444;
  int16_t                 _vertical_calibration                = 46;
  int16_t                 _horizontal_calibration              = 0;
  float                   _Kp                                  =  0.5; 
  float                   _Ki                                  =  0.1;
  float                   _Kd                                  =  0.00;
  uint8_t                 _pid_pwm                             = 0;
  TaskHandle_t            _pidTaskHandle                       = nullptr;
  bool                    _isAP                                = false;
  uint64_t                _last_ESPNOW                         = 0;
  const char*             _ssid                                = "Labeta 2.0";
  WiFiUDP                 _udp;
  const int               _udpPort                             = 4210;
  char                    _packetBuffer[255];
  //IPAddress               _remoteIP;
  //uint16_t                _remotePort;
  //bool                    _clientConnected                     = false;
  UdpUser                 _users[10];
  uint8_t                 _udp_user_counter                     = 0;
};
