#pragma once
#include <cstdint>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <ESP32Encoder.h>
#include <Arduino_GFX_Library.h>
#include <XPT2046_Touchscreen.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <esp_wifi.h>
#include <WiFiServer.h>
#include <TJpg_Decoder.h>

#define XPT2046_IRQ               36   // T_IRQ
#define XPT2046_MOSI              23  // T_DIN
#define XPT2046_MISO              22  // T_OUT, T_DO
#define XPT2046_CLK               17   // T_CLK
#define XPT2046_CS                 4    // T_CS

#define TFT_MISO                  12
#define TFT_MOSI                  13
#define TFT_SCLK                  14
#define TFT_CS                    15
#define TFT_DC                    33
#define TFT_RST                   16
#define TFT_BL                    21
#define TFT_BACKLIGHT_ON          HIGH

#define SD_CS                      5
//#define SD_MOSI                   13
//#define SD_MISO                   12
//#define SD_SCK                    14

#define SCREEN_WIDTH             320
#define SCREEN_HEIGHT            240
#define FONT_SIZE_SMALL            1
#define FONT_SIZE_BIG              2
#define FONT_SIZE_LARGE            3

#define TFT_BLACK                 0x0000
#define TFT_WHITE                 0xFFFF
#define TFT_RED                   0xF800
#define TFT_GREEN                 0x07E0
#define TFT_BLUE                  0x001F
#define TFT_YELLOW                0xFFE0
#define TFT_PURPLE                0x780F
#define TFT_DARKGREEN             0x03E0

#define CLK_PIN_1                 18
#define DT_PIN_1                  19
#define CLK_PIN_2                 34
#define DT_PIN_2                  35

#define DAC_PIN                   25

#define BUTTONS_PIN               32

#define DELAY_TO_CONFIRM         100
#define DELAY_TO_STOP_CHANGES   1000
#define DISPLAY_DELAY            500
#define TOUCH_STEP               500    //waiting after touch to set screen to Not_touched
#define NUMBER_OF_BUTTONS         21
#define SENDING_LONG_DELAY      1000
#define SENDING_SHORT_DELAY      200
#define CHECKING_TIME_DELAY     1000
#define DELAY_KEY_RELEASED      1000
#define DELAY_LOG_WRITING       2000

#define ARRAY_SIZE                16

#define EEPROM_SIZE              512
#define INIT_ADR                 127
#define INIT_KEY                 102

#define NUMBER_OF_PRESET           6

const char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

enum directions {
  RIGHT = 54,
  LEFT  = 57,
  UP    = 58,
  DOWN  = 53,
  NONE  =  0,
  NONE_SUB = 63
};

enum screen_states {
  MAIN,
  MENU,
  SENSOR_PANEL,
  PRESETS,
  CAMERA
};

enum menu_buttons {
  HOR_TO_0   =  4,
  VER_TO_0   =  5,
  ALL_TO_0   =  6,
  MUTE       =  7,
  CONNECT    =  8,
  HOR_SET_0  =  9,
  VER_SET_0  = 10,
  _8         = 11,
  CAMERA_BTN = 20
};

/*enum preset_buttons {
  DEFINE_PRESET_1  =  14,
  LOAD_PRESET_1    =  15,
  DEFINE_PRESET_2  =  16,
  LOAD_PRESET_2    =  17,
};*/

enum preset_buttons {
  PRESET_1  =  14,
  PRESET_2  =  15,
  PRESET_3  =  16,
  PRESET_4  =  17,
  PRESET_5  =  18,
  PRESET_6  =  19,
  NO_BUTTON =  -1
};

enum senders {
  CONTROLLER = 0,
  MOTION_PLATFORM = 1,
  SENSOR_PLATFORM = 2,
  CAMERA_PLATFORM = 3,
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


class Controller {
public:
  Controller();
  void begin();
  void tick();
  void sendData(const uint8_t* mac, const void* data, size_t len);
  void setOnDataRecvCallback(esp_now_recv_cb_t cb);
  void setOnDataSentCallback(esp_now_send_cb_t cb);
  void sendStateTo(const uint8_t* mac, messages message = DESIRED);
  static bool tftJpgDrawCallback(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap);

private:
  ESP32Encoder            _encoder_1;
  ESP32Encoder            _encoder_2;
  XPT2046_Touchscreen     _touchscreen;
  SPIClass                _spiVSPI;
  RTC_DS3231              _rtc;
  Arduino_DataBus*        _tft_bus                            = nullptr;
  Arduino_GFX*            _tft                                = nullptr;
  bool                    _is_ESPNOW_Initialized              = false;
  bool                    _is_SD_Initialized                  = false;
  static Controller*      _instance;
  int64_t                 _encoder_1_value                    = 0;
  int64_t                 _encoder_2_value                    = 0;
  uint64_t                _display_lat_time_refreshed         = 0;
  uint64_t                _esp_now_last_time_sending          = 0;
  uint64_t                _esp_now_last_time_pl_receving      = 0;
  uint64_t                _esp_now_last_time_se_receving      = 0;
  uint64_t                _last_time_changed                  = 0;
  uint64_t                _changes_starts_time                = 0;
  bool                    _changes_appears_flag               = false;
  int16_t                 _current_state[ARRAY_SIZE]          = {0};
  int16_t                 _desired_state[ARRAY_SIZE]          = {0};
  int16_t                 _preset__state[ARRAY_SIZE]          = {0};
  int16_t                 _data_received[ARRAY_SIZE + 2]      = {0};
  int16_t                 _data_to_send[ARRAY_SIZE + 2]       = {0};
  uint16_t                _centerX                            = SCREEN_WIDTH / 2;
  uint16_t                _centerY                            = SCREEN_HEIGHT / 2;
  uint8_t                 _receiverMac[6]                     = {0xF4, 0x65, 0x0B, 0x48, 0x6F, 0x90};
  //uint8_t                 _receiverMac[6]                     = {0x3C, 0x8A, 0x1F, 0x5E, 0x6A, 0x44};
  uint8_t                 _broadcastMac[6]                    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t                 _cameraMac[6]                       = {0};
  uint8_t                 _sensorMac[6]                       = {0};
  uint8_t                 _audio_Buffer[1024]                 = {0};
  const char*             _audio_File_Name                    = "/test.wav";
  const char*             _interface_Name                     = "/logo_b.jpg";
  uint8_t                 _buttons_pin                        = BUTTONS_PIN;
  bool                    _key_pressed                        = false;
  int16_t                 _current_key_value                  = 0;
  bool                    _platform_connected_flag            = false;
  bool                    _sensors_connected_flag             = false;
  int16_t                 _x_touch                            = 0;
  int16_t                 _y_touch                            = 0;
  int16_t                 _z_touch                            = 0;
  screen_states           _current_screen_state               = MAIN;
  screen_states           _previous_screen_state              = MENU;
  screen_states           _changed_screen_state               = MAIN;
//  menu_buttons            _current_menu_button                = _8;
  bool                    _preset_slider_on_set               = false;
  bool                    _buttons_active[NUMBER_OF_BUTTONS]  = {false};
  uint64_t                _menu_buttons_pressed_time[NUMBER_OF_BUTTONS] = {0};
  uint64_t                _preset_buttons_pressed_time[NUMBER_OF_BUTTONS] = {0};
  bool                    _screen_state_changed               = false;
  uint64_t                _screen_state_changed_time          = 0;
  bool                    _rtc_inited                         = false;
  DateTime                _date_time;
  File                    _log_file;
  String                  _log_Dir;
  String                  _log_file_name;                   
  bool                    _file_created                       = false;
  String                  _log_Buffer;
  uint64_t                _last_time_log_written              = 0;
  bool                    _waiting_for_new_MAC                = false;
  int16_t                 _presets[NUMBER_OF_PRESET][2]       = {0};
  uint8_t                 _softap_bssid[6]                    = {0};
  uint8_t                 _softap_channel                     = 1;
  WiFiServer*             _camServer                          = nullptr;
  uint16_t                _cam_server_port                    = 80;
  bool                    _camera_frame_received              = false;
  WiFiClient              _camClient;          // поточний TCP‑клієнт камери
  bool                    _camClientConnected                 = false;
  uint32_t                _nextFrameLen                       = 0;
  uint32_t                _frameBytesRead                     = 0;
  uint8_t*                _frameBuf                           = nullptr;
  uint32_t                _frameStartTime                     = 0;
  int16_t                 _x_touch_prev                       = 0;
  int16_t                 _y_touch_prev                       = 0;
  int16_t                 _z_touch_prev                       = 0;
  bool                    _prev_touch_on_button               = false;
  bool                    _prev_touch_drawn                   = false;
  int8_t                  _selected_preset                    = -1;
  const char*             apSsid                              = "LABETA_CTRL";
  const char*             apPass                              = "";
  
  void                    tftDrawString(const String& s, int16_t x, int16_t y, uint8_t textSize);
  void                    tftDrawCentreString(const String& s, int16_t cx, int16_t y, uint8_t textSize);
  uint16_t                blinkButtonColor(bool &blink, bool buttonActive, uint16_t baseColor, uint16_t blinkColor = TFT_PURPLE);
  bool                    initialization_EEPROM();
  void                    drawInterfaceJpgFromSD(const char* path, int16_t x, int16_t y);
  void                    display_Data();
  void                    read_Encoders();
  static void             staticOnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  void                    handleDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  static void             defaultOnDataSent(const uint8_t* mac, esp_now_send_status_t status);
  bool                    addPeer(const uint8_t* mac);
  bool                    checkPlatformMAC(const esp_now_recv_info_t *info);
  void                    confirmChanges();
  void                    smartSending();
  void                    playAudio();
  int16_t                 get_key_value();
  int16_t                 get_button_number_by_value(int16_t value);
  void                    define_key_value();
  void                    check_platform_connection();
  void                    check_sensors_connection();
  void                    touchScreenReading();        
  void                    defineScreenState();
  void                    defineMenuButton();
  void                    definePresetsButton();
  void                    checkingDateTime();
  String                  findNextLogFile();
  void                    writeToLog(String message);
  void                    makeLogReport();
  void                    set_new_receiverMAC(const esp_now_recv_info_t *info);
  void                    newPlatformConnection();  
  void                    savePreset(uint8_t preset_number);
  void                    startCameraServer(uint16_t port);
  void                    stopCameraServer();
  void                    startCameraStream(uint16_t port = 80); 
  void                    stopCameraStream();
  void                    handleCameraTCP();
  bool                    isTouchOnButtonArea(int16_t x, int16_t y);
};