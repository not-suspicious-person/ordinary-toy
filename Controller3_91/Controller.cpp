#include "Controller.h"
#include <Arduino.h>
#include <esp_wifi.h>

#define MEASURE_ERROR 50
//Standart function that show, if two int16_t aprovimatly equal
bool    aprox_eq(int16_t val1, int16_t val2, int16_t delta = MEASURE_ERROR) {
  return abs(val1 - val2) < delta;
}

senders intToSenders(int16_t value) {
  switch (value) {
    case  0 : return CONTROLLER;
    case  1 : return MOTION_PLATFORM;
    case  2 : return SENSOR_PLATFORM;
    case  3 : return CAMERA_PLATFORM;
    default : return WHO_THE_HELL_IS_THIS;
  }
}

messages intToMessages (int16_t value) {
  switch (value) {
    case  0 : return DESIRED;
    case  1 : return MOTION_REPORT;
    case  2 : return SENSOR_REPORT;
    case  3 : return VERTICAL_CALIBRATION;
    case  4 : return HORIZONTAL_CALIBRATION;
    case  5 : return NEW_CONTROLLER;
    case  6 : return GO_STREAM;
    case  7 : return STOP_STREAM;
    default : return WHAT_THE_HELL_DO_YOU_WANT;
  }
}

void Controller::tftDrawString(const String& s, int16_t x, int16_t y, uint8_t textSize) { 
  _tft->setTextSize(textSize); 
  _tft->setCursor(x, y); 
  _tft->print(s); 
}

void Controller::tftDrawCentreString(const String& s, int16_t cx, int16_t y, uint8_t textSize) { 
  int16_t x1, y1; 
  uint16_t w, h; 
  _tft->setTextSize(textSize); 
  _tft->getTextBounds(s, 0, 0, &x1, &y1, &w, &h); 
  int16_t x = cx - (int16_t)(w / 2); 
  _tft->setCursor(x, y); 
  _tft->print(s); 
}

uint16_t Controller::blinkButtonColor(bool &blink, bool buttonActive, uint16_t baseColor, uint16_t blinkColor) {
  if (buttonActive) {
    blink = !blink;
  }
  return blink ? blinkColor : baseColor;
}

String FormatedString(int16_t number, uint8_t width) {
  String myString = String(number);
  uint8_t len = myString.length();
  if (len >= width) {
    String result = "";
    for (uint8_t i = 0; i < width; i ++) result += "*";
    return result;
  }
  uint8_t total_spaces = width - len;
  uint8_t left_spaces = total_spaces / 2;
  uint8_t right_spaces = total_spaces - left_spaces;
  String result = "";
  for (uint8_t i = 0; i < left_spaces;  i++) result += " ";
  result += myString;
  for (uint8_t i = 0; i < right_spaces; i++) result += " ";
  return result;
}

String FormatedString(String myString, uint8_t width) {
  uint8_t len = myString.length();
  if (len >= width) {
    String result = "";
    for (uint8_t i = 0; i < width; i ++) result +="*";
    return result;
  }
  uint8_t total_spaces = width - len;
  uint8_t left_spaces = total_spaces / 2;
  uint8_t right_spaces = total_spaces - left_spaces;
  String result = "";
  for (uint8_t i = 0; i < left_spaces;  i++) result += " ";
  result += myString;
  for (uint8_t i = 0; i < right_spaces; i++) result += " ";
  return result;
}
//Pointer on current object yo work with ESPNOW callback
Controller* Controller::_instance = nullptr;

bool Controller::tftJpgDrawCallback(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (!_instance || !_instance->_tft) return false;

  _instance->_tft->draw16bitRGBBitmap(x, y, bitmap, w, h);
  return true;
}

Controller::Controller() :
//initialisation display on HSPI and touchscreen on VSPI 
  _touchscreen(XPT2046_CS, XPT2046_IRQ),
  _spiVSPI(VSPI)
{
  _instance = this;
}

bool Controller::initialization_EEPROM() {
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Error EEPROM");
    return false;
  }
  if (EEPROM.read(INIT_ADR) != INIT_KEY) {
    uint16_t address = 4;
    EEPROM.put(INIT_ADR, INIT_KEY);
    //Platform MAC 4-9
    address = 4;
    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.put(address, _receiverMac[i]);
      address += sizeof(uint8_t);
    }
    //Sensors MAC 10-15
    address = 10;
    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.put(address, _sensorMac[i]);
      address += sizeof(uint8_t);
    }
    //Presets 16-...
    address = 16;
    for (uint8_t i = 0; i < NUMBER_OF_PRESET; i++) {
      for (uint8_t j = 0; j < 2; j++) {
        EEPROM.put(address, _presets[i][j]);
        address += sizeof(int16_t);
      }
    }
    if (EEPROM.commit()) 
    { 
      Serial.print("EEPROM data has been written"); return true; 
    }
    else                 
    { 
      Serial.print("Some EEPROM errors"); 
      return false; 
    }
  }
  else {
    //Platform MAC 4-9
    uint16_t address = 4;
    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.get(address, _receiverMac[i]);
      address += sizeof(uint8_t);
    }
    //Sensors MAC 10-15
    address = 10;
    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.get(address, _sensorMac[i]);
      address += sizeof(uint8_t);
    }
    //Presets 16-...
    address = 16;
    for (uint8_t i = 0; i < NUMBER_OF_PRESET; i++) {
      for (uint8_t j = 0; j < 2; j++) {
        EEPROM.get(address, _presets[i][j]);
        address += sizeof(int16_t);
      }
    }
    Serial.printf("Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  _receiverMac[0], _receiverMac[1], _receiverMac[2],
                  _receiverMac[3], _receiverMac[4], _receiverMac[5]);
    Serial.printf("Sensors  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  _sensorMac[0], _sensorMac[1], _sensorMac[2],
                  _sensorMac[3], _sensorMac[4], _sensorMac[5]);
  }
  return true;
}

void Controller::begin() {
  analogReadResolution(12);
  delay(100);

  /*-----------------ENCODERS---------------*/
  ESP32Encoder::useInternalWeakPullResistors = puType::none;
  pinMode(CLK_PIN_1, INPUT_PULLUP);
  pinMode(DT_PIN_1,  INPUT_PULLUP);
  pinMode(CLK_PIN_2, INPUT_PULLUP);
  pinMode(DT_PIN_2,  INPUT_PULLUP);
  _encoder_1.attachHalfQuad(CLK_PIN_1, DT_PIN_1);
  _encoder_1.setFilter(1023);
  _encoder_2.attachHalfQuad(CLK_PIN_2, DT_PIN_2);
  _encoder_2.setFilter(1023);
  delay(500);

  /*-----------------DISPLAY---------------*/
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);

  _tft_bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO, HSPI);
  _tft     = new Arduino_ILI9341(_tft_bus, TFT_RST, 0, false);

  if (_tft && _tft->begin(27000000)) {
    Serial.println("TFT begin OK");
    _tft->setRotation(1);
    _tft->fillScreen(TFT_BLACK);
    _tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tftDrawCentreString("Start. SD initialization", _centerX, 30, FONT_SIZE_SMALL);

    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(false);
    TJpgDec.setCallback(Controller::tftJpgDrawCallback);
  } else {
    Serial.println("TFT init failed!");
  }

  delay(500);

  /*-----------------TOUCHSCREEN---------------*/
  _spiVSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  _touchscreen.begin(_spiVSPI);
  _touchscreen.setRotation(1);
  delay(500);

  /*-----------------SD---------------*/

  digitalWrite(XPT2046_CS, HIGH); 
  if (!SD.begin(SD_CS, _spiVSPI, 10000000U)) {
    Serial.println("SD Card initialization failed!"); 
    _is_SD_Initialized = false; 
    _tft->setTextColor(TFT_RED, TFT_BLACK); 
    tftDrawCentreString("SD Card initialization failed!", _centerX, 50, FONT_SIZE_SMALL); 
  } else { 
    Serial.println("SD Card initialized.");
    _is_SD_Initialized = true; 
    _tft->setTextColor(TFT_GREEN, TFT_BLACK); 
    tftDrawCentreString("SD Card initialized.", _centerX, 50, FONT_SIZE_SMALL); 
  } 
  digitalWrite(SD_CS, HIGH);

  delay(500);

  drawInterfaceJpgFromSD(_interface_Name, 0, 62);

  /*---------WI-FI-AND-ESP-NOW--------*/
  WiFi.mode(WIFI_AP);

  bool ap_ok = WiFi.softAP(apSsid, apPass, 1, false);
  if (!ap_ok) {
    Serial.println("Failed to start SoftAP");
  } else {
    Serial.print("SoftAP started. SSID: ");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initialization ESP-NOW");
    _is_ESPNOW_Initialized = false;
  } else {
    //esp_now_register_send_cb(defaultOnDataSent);
    esp_now_register_recv_cb(staticOnDataRecv);
    _is_ESPNOW_Initialized = true;
  }

  /*-----------------RTC---------------*/
  WiFi.setSleep(false);
  Wire.begin(27, 26); 
  delay(200); 
  _rtc_inited = _rtc.begin(&Wire); 
  Serial.println(_rtc_inited ? "Clock inited" : "Clock is not inited"); 
  Serial.print("Compiled at: "); 
  Serial.print(__DATE__); 
  Serial.print(" "); 
  Serial.println(__TIME__); 
  if (_rtc_inited && _rtc.lostPower()) { 
    Serial.println("RTC lost power, setting time..."); 
    _rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
  delay(500);

  /*-----------------EVERYTHING ELSE---------------*/ 

  this->checkingDateTime(); 
  _log_file_name = this->findNextLogFile(); 
  Serial.print("Creating file: "); 
  Serial.println(_log_file_name); 
  _log_file = SD.open(_log_file_name, FILE_WRITE); 
  _file_created = (bool)_log_file; 
  if (!_file_created) Serial.println("Failed to create file!"); 
  if (_file_created) { 
    _log_Buffer.reserve(2048); 
    this->writeToLog("Log created."); 
  }
    this->playAudio();

  this->initialization_EEPROM(); 
  delay(500); 
  dacWrite(DAC_PIN, 0); 
  Serial.println("Brand new Start!");
}

void Controller::tick() {
  this->touchScreenReading();
  this->defineScreenState();
  this->display_Data();
  this->handleCameraTCP();

  if (_current_screen_state != CAMERA) {
    this->define_key_value();
    this->read_Encoders();
    this->checkingDateTime();
    this->check_platform_connection();
    this->check_sensors_connection();
    this->defineMenuButton();
    this->definePresetsButton();
    this->makeLogReport();
    this->smartSending();
  }

  static uint32_t t_ap = 0;
  if (_current_screen_state == CAMERA && millis() - t_ap > 1000) {
    Serial.printf("AP stations: %d\n", WiFi.softAPgetStationNum());
    t_ap = millis();
  }
}

void Controller::drawInterfaceJpgFromSD(const char* path, int16_t x, int16_t y) {
  if (!_is_SD_Initialized) {
    Serial.println("SD not initialized, cannot draw JPG");
    return;
  }

  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.print("JPG file not found via SD.open: ");
    Serial.println(path);
    return;
  }

  size_t size = f.size();
  Serial.print("JPG size: ");
  Serial.println(size);

  uint8_t* jpgBuf = (uint8_t*)malloc(size);
  if (!jpgBuf) {
    Serial.println("Failed to allocate buffer for JPG");
    f.close();
    return;
  }

  size_t bytesRead = f.read(jpgBuf, size);
  f.close();

  if (bytesRead != size) {
    Serial.println("Failed to read full JPG file");
    free(jpgBuf);
    return;
  }

  TJpgDec.drawJpg(x, y, jpgBuf, size);

  free(jpgBuf);
}

void Controller::display_Data() {
  uint64_t now = millis();
  if (now - _display_lat_time_refreshed < DISPLAY_DELAY) return;
  this->defineScreenState();

  _tft->setTextColor(TFT_WHITE, TFT_BLACK);
  if (_current_screen_state != _previous_screen_state) {
    _tft->fillScreen(TFT_BLACK);
    if (_current_screen_state != CAMERA) {
      if (_is_SD_Initialized) {
        _tft->setTextColor(TFT_GREEN, TFT_BLACK);
        tftDrawString("SD", 10, 225, FONT_SIZE_SMALL);
        _tft->fillRect(50, 225, 12, 10, TFT_BLACK);
        _tft->drawLine( 51, 225 + 4, 56, 225 + 8, TFT_GREEN);
        _tft->drawLine( 56, 225 + 8, 61, 225 + 1, TFT_GREEN);
      } else {
        _tft->setTextColor(TFT_RED, TFT_BLACK);
        tftDrawString("SD", 10, 225, FONT_SIZE_SMALL);
        _tft->fillRect(50, 225, 12, 10, TFT_BLACK);
        _tft->drawLine(51, 225 + 1, 60, 225 + 8, TFT_RED);
        _tft->drawLine(60, 225 + 1, 51, 225 + 8, TFT_RED);
      }

      if (_platform_connected_flag) {
        _tft->setTextColor(TFT_GREEN, TFT_BLACK);
        tftDrawString("PLATFORM", 90, 225, FONT_SIZE_SMALL);
        _tft->fillRect(170, 225, 12, 10, TFT_BLACK);
        _tft->drawLine(171, 225 + 4, 176, 225 + 8, TFT_GREEN);
        _tft->drawLine(176, 225 + 8, 181, 225 + 1, TFT_GREEN);
      } else {
        _tft->setTextColor(TFT_RED, TFT_BLACK);
        tftDrawString("PLATFORM", 90, 225, FONT_SIZE_SMALL);
        _tft->fillRect(170, 225, 12, 10, TFT_BLACK);
        _tft->drawLine(171, 225 + 1, 180, 225 + 8, TFT_RED);
        _tft->drawLine(180, 225 + 1, 171, 225 + 8, TFT_RED);
      }

      if (_sensors_connected_flag) {
        _tft->setTextColor(TFT_GREEN, TFT_BLACK);
        tftDrawString("SENSORS", 200, 225, FONT_SIZE_SMALL);
        _tft->fillRect(280, 225, 12, 10, TFT_BLACK);
        _tft->drawLine(281, 225 + 4, 286, 225 + 8, TFT_GREEN);
        _tft->drawLine(286, 225 + 8, 291, 225 + 1, TFT_GREEN);
      } else {
        _tft->setTextColor(TFT_RED, TFT_BLACK);
        tftDrawString("SENSORS", 200, 225, FONT_SIZE_SMALL);
        _tft->fillRect(280, 225, 12, 10, TFT_BLACK);
        _tft->drawLine(281, 225 + 1, 290, 225 + 8, TFT_RED);
        _tft->drawLine(290, 225 + 1, 281, 225 + 8, TFT_RED);
      }
    }
  }
  switch (_current_screen_state) {
    case MAIN: {      
      const int tableTop    = 90;
      const int tableBottom = 220;
      const int tableHeight = tableBottom - tableTop;
      const int rowHeight   = tableHeight / 2;
      const int colLeft     = 64;
      const int colRight    = 280;
      const int tableWidth  = colRight - colLeft;
      const int colWidth    = tableWidth / 2;

      for (int r = 0; r <= 2; r++) {
        int y = tableTop + r * rowHeight;
        _tft->drawLine(colLeft, y, colRight, y, TFT_WHITE);
      }

      for (int c = 0; c <= 2; c++) {
        int x = colLeft + c * colWidth;
        _tft->drawLine(x, tableTop, x, tableBottom, TFT_WHITE);
      }

      int colCenter[2];
      for (int c = 0; c < 2; c++) {
        colCenter[c] = colLeft + c * colWidth + colWidth / 2;
      }

      int rowCenter[2];
      for (int r = 0; r < 2; r++) {
        rowCenter[r] = tableTop + r * rowHeight + rowHeight / 2;
      }

      String tempText;
      const int headerY = tableTop - 20;
      tempText = "SET:";
      tftDrawCentreString(tempText, colCenter[0], headerY, FONT_SIZE_BIG);
      tempText = "CURRENT:";
      tftDrawCentreString(tempText, colCenter[1], headerY, FONT_SIZE_BIG);
      const int rowLabelX = colLeft - 30;
      tempText = "HOR:";
      tftDrawCentreString(tempText, rowLabelX, rowCenter[0], FONT_SIZE_BIG);
      tempText = "VER:";
      tftDrawCentreString(tempText, rowLabelX, rowCenter[1], FONT_SIZE_BIG);
      tempText = FormatedString(_preset__state[0], 4);
      tftDrawCentreString(tempText, colCenter[0], rowCenter[0], FONT_SIZE_LARGE);
      tempText = FormatedString(_preset__state[1], 4);
      tftDrawCentreString(tempText, colCenter[0], rowCenter[1], FONT_SIZE_LARGE);
      tempText = FormatedString(_current_state[0], 4);
      tftDrawCentreString(tempText, colCenter[1], rowCenter[0], FONT_SIZE_LARGE);
      tempText = FormatedString(_current_state[1], 4);
      tftDrawCentreString(tempText, colCenter[1], rowCenter[1], FONT_SIZE_LARGE);

      static bool blink = false;
      static uint32_t temp_color = TFT_DARKGREEN;
      _tft->setTextColor(TFT_WHITE);

      if (!_buttons_active[0] && !_buttons_active[1] && !_buttons_active[12]) {
        blink = false;
      }

      if ((_current_screen_state != _previous_screen_state) || _buttons_active[0]) {
        temp_color = blinkButtonColor(blink, _buttons_active[0], TFT_DARKGREEN);
        _tft->fillRect(0, 0, 102, 50, temp_color);
        tftDrawCentreString("MENU", 53, 15, FONT_SIZE_BIG);
      }
      if ((_current_screen_state != _previous_screen_state) || _buttons_active[1]) {
        temp_color = blinkButtonColor(blink, _buttons_active[1], TFT_DARKGREEN);
        _tft->fillRect(104, 0, 102, 50, temp_color);
        tftDrawCentreString("SENSOR", 156, 15, FONT_SIZE_BIG);
      }
      if ((_current_screen_state != _previous_screen_state) || _buttons_active[12]) {
        temp_color = blinkButtonColor(blink, _buttons_active[12], TFT_DARKGREEN);
        _tft->fillRect(208, 0, 102, 50, temp_color);
        tftDrawCentreString("PRESET", 263, 15, FONT_SIZE_BIG);
      }
    }
    case MENU: {
      const int menuTopRowY    = 70;
      const int menuRowHeight  = 75;
      const int menuSecondRowY = menuTopRowY + menuRowHeight + 1;
      static bool blink = false;
      static uint32_t temp_color = TFT_DARKGREEN;
      static bool previous_buttons_state[NUMBER_OF_BUTTONS] = {false};

      if (_current_screen_state != _previous_screen_state)
        blink = false;

      _tft->setTextColor(TFT_WHITE);

      if ((_current_screen_state != _previous_screen_state) || _buttons_active[2]) {
        temp_color = blinkButtonColor(blink, _buttons_active[2], TFT_DARKGREEN);
        _tft->fillRect(0, 0, 160, 50, temp_color);
        tftDrawCentreString("BACK", 75, 15, FONT_SIZE_BIG);
      }

      if ((_current_screen_state != _previous_screen_state) || _buttons_active[CAMERA_BTN]) {
        temp_color = blinkButtonColor(blink, _buttons_active[CAMERA_BTN], TFT_DARKGREEN);
        _tft->fillRect(161, 0, 160, 50, temp_color);
        tftDrawCentreString("CAMERA", 240, 15, FONT_SIZE_BIG);
      }

      if ((_current_screen_state != _previous_screen_state) ||
          _buttons_active[HOR_TO_0] || previous_buttons_state[HOR_TO_0]) {

        if (previous_buttons_state[HOR_TO_0] && !_buttons_active[HOR_TO_0])
          blink = false;

        temp_color = blinkButtonColor(blink, _buttons_active[HOR_TO_0], TFT_DARKGREEN);
        _tft->fillRect(8, menuTopRowY, 75, menuRowHeight, temp_color);
        tftDrawCentreString("HOR TO 0", 46, menuTopRowY + menuRowHeight / 2 - 5, FONT_SIZE_SMALL);
      }

      if ((_current_screen_state != _previous_screen_state) ||
          _buttons_active[VER_TO_0] || previous_buttons_state[VER_TO_0]) {

        if (previous_buttons_state[VER_TO_0] && !_buttons_active[VER_TO_0])
          blink = false;
    
        temp_color = blinkButtonColor(blink, _buttons_active[VER_TO_0], TFT_DARKGREEN);
        _tft->fillRect(84, menuTopRowY, 75, menuRowHeight, temp_color);
        tftDrawCentreString("VER TO 0", 122, menuTopRowY + menuRowHeight / 2 - 5, FONT_SIZE_SMALL);
      }

      if ((_current_screen_state != _previous_screen_state) ||
          _buttons_active[ALL_TO_0] || previous_buttons_state[ALL_TO_0]) {

        if (previous_buttons_state[ALL_TO_0] && !_buttons_active[ALL_TO_0])
          blink = false;

        temp_color = blinkButtonColor(blink, _buttons_active[ALL_TO_0], TFT_DARKGREEN);
        _tft->fillRect(160, menuTopRowY, 75, menuRowHeight, temp_color);
        tftDrawCentreString("ALL TO 0", 198, menuTopRowY + menuRowHeight / 2 - 5, FONT_SIZE_SMALL);
      }

      if ((_current_screen_state != _previous_screen_state) ||
      _buttons_active[MUTE] || previous_buttons_state[MUTE]) {

        if (previous_buttons_state[MUTE] && !_buttons_active[MUTE])
          blink = false;

        temp_color = blinkButtonColor(blink, _buttons_active[MUTE], TFT_DARKGREEN);
        _tft->fillRect(236, menuTopRowY, 75, menuRowHeight, temp_color);
        tftDrawCentreString("MUTE", 274, menuTopRowY + menuRowHeight / 2 - 5, FONT_SIZE_SMALL);
      }

      if ((_current_screen_state != _previous_screen_state) ||
          _buttons_active[CONNECT] || previous_buttons_state[CONNECT]) {

        if (previous_buttons_state[CONNECT] && !_buttons_active[CONNECT])
          blink = false;
    
        temp_color = blinkButtonColor(blink, _buttons_active[CONNECT], TFT_DARKGREEN);
        _tft->fillRect(8, menuSecondRowY, 75, menuRowHeight, temp_color);
        tftDrawCentreString("CONNECT", 46, menuSecondRowY + menuRowHeight / 2 - 5, FONT_SIZE_SMALL);
      }

      if ((_current_screen_state != _previous_screen_state) ||
          _buttons_active[HOR_SET_0] || previous_buttons_state[HOR_SET_0]) {

        if (previous_buttons_state[HOR_SET_0] && !_buttons_active[HOR_SET_0])
          blink = false;

        temp_color = blinkButtonColor(blink, _buttons_active[HOR_SET_0], TFT_DARKGREEN);
        _tft->fillRect(84, menuSecondRowY, 75, menuRowHeight, temp_color);
        tftDrawCentreString("HOR SET 0", 122, menuSecondRowY + menuRowHeight / 2 - 5, FONT_SIZE_SMALL);
      }

      if ((_current_screen_state != _previous_screen_state) ||
          _buttons_active[VER_SET_0] || previous_buttons_state[VER_SET_0]) {

        if (previous_buttons_state[VER_SET_0] && !_buttons_active[VER_SET_0])
          blink = false;

        temp_color = blinkButtonColor(blink, _buttons_active[VER_SET_0], TFT_DARKGREEN);
        _tft->fillRect(160, menuSecondRowY, 75, menuRowHeight, temp_color);
        tftDrawCentreString("VER_SET_0", 198, menuSecondRowY + menuRowHeight / 2 - 5, FONT_SIZE_SMALL);
      }

      if ((_current_screen_state != _previous_screen_state) ||
          _buttons_active[_8] || previous_buttons_state[_8]) {

        if (previous_buttons_state[_8] && !_buttons_active[_8])
          blink = false;

        temp_color = blinkButtonColor(blink, _buttons_active[_8], TFT_DARKGREEN);
        _tft->fillRect(236, menuSecondRowY, 75, menuRowHeight, temp_color);
        tftDrawCentreString("8", 274, menuSecondRowY + menuRowHeight / 2 - 5, FONT_SIZE_SMALL);
      }

      for (uint8_t i = 0; i < NUMBER_OF_BUTTONS; i++) {
        previous_buttons_state[i] = _buttons_active[i];
      }

      break;
    }
    case SENSOR_PANEL: {
      static bool blink = false;
      static uint32_t temp_color = TFT_DARKGREEN;
      if (_current_screen_state != _previous_screen_state) blink = false;
      _tft->setTextColor(TFT_WHITE);

      if ((_current_screen_state != _previous_screen_state) || _buttons_active[3]) {
        temp_color = blinkButtonColor(blink, _buttons_active[3], TFT_DARKGREEN);
        _tft->fillRect(0, 0, 160, 50, temp_color);
        tftDrawCentreString("BACK", 75, 15, FONT_SIZE_BIG);
      }

      _tft->drawLine(  65,  67, 300,  67, TFT_WHITE);
      _tft->drawLine(  65,  87, 300,  87, TFT_WHITE);
      _tft->drawLine(  65, 107, 300, 107, TFT_WHITE);
      _tft->drawLine(  65,  67,  65, 107, TFT_WHITE);
      _tft->drawLine( 124,  67, 124, 107, TFT_WHITE);
      _tft->drawLine( 183,  67, 183, 107, TFT_WHITE);
      _tft->drawLine( 242,  67, 242, 107, TFT_WHITE);
      _tft->drawLine( 300,  67, 300, 107, TFT_WHITE);

      String tempText = "C  (mA) : ";
      tftDrawCentreString(tempText, _centerX - 125, _centerY - 50, FONT_SIZE_SMALL);
      tempText = "V  (V)  : ";
      tftDrawCentreString(tempText, _centerX - 125, _centerY - 30, FONT_SIZE_SMALL);
      tempText = "H  (%)  : ";
      tftDrawCentreString(tempText, _centerX - 125, _centerY - 10, FONT_SIZE_SMALL);
      tempText = "T  (C)  : ";
      tftDrawCentreString(tempText, _centerX - 125, _centerY + 10, FONT_SIZE_SMALL);
      tempText = "P  (hPa): ";
      tftDrawCentreString(tempText, _centerX - 125, _centerY + 30, FONT_SIZE_SMALL);
      _tft->setTextColor(TFT_WHITE, TFT_BLACK);
      tempText = String(_current_state[3] / 100.0, 2);                                          //current1
      tftDrawCentreString(tempText, _centerX - 65, _centerY - 50, FONT_SIZE_SMALL);
      tempText = String(_current_state[4] / 100.0, 2);                                          //voltage1
      tftDrawCentreString(tempText, _centerX - 65, _centerY - 30, FONT_SIZE_SMALL);
      tempText = String(_current_state[5] / 100.0, 2);                                          //current2
      tftDrawCentreString(tempText, _centerX - 3, _centerY - 50, FONT_SIZE_SMALL);
      tempText = String(_current_state[6] / 100.0, 2);                                          //voltage2
      tftDrawCentreString(tempText, _centerX - 3, _centerY - 30, FONT_SIZE_SMALL);
      tempText = String(_current_state[7] / 100.0, 2);                                          //current3
      tftDrawCentreString(tempText, _centerX + 50, _centerY - 50, FONT_SIZE_SMALL);
      tempText = String(_current_state[8] / 100.0, 2);                                          //voltage3
      tftDrawCentreString(tempText, _centerX + 50, _centerY - 30, FONT_SIZE_SMALL);
      tempText = String(_current_state[9] / 100.0, 2);                                          //current4
      tftDrawCentreString(tempText, _centerX + 112, _centerY - 50, FONT_SIZE_SMALL);
      tempText = String(_current_state[10] / 100.0, 2);                                         //voltage4
      tftDrawCentreString(tempText, _centerX + 112, _centerY - 30, FONT_SIZE_SMALL);
      tempText = String(_current_state[11] / 100.0 - 2.0, 2) + " C";                                 //temperature
      tftDrawCentreString(tempText, _centerX - 67, _centerY + 10, FONT_SIZE_SMALL);
      tempText = String(_current_state[12] / 100.0, 2) + "  %";                                  //humidity
      tftDrawCentreString(tempText, _centerX - 67, _centerY - 10, FONT_SIZE_SMALL);
      tempText = String(_current_state[13]) + " hPa";                                 //pressure
      tftDrawCentreString(tempText, _centerX - 67, _centerY + 30, FONT_SIZE_SMALL);
      break;
    }
    case PRESETS: {
      static bool blink = false;
      static uint32_t temp_color = TFT_DARKGREEN;
      static bool previous_buttons_state[NUMBER_OF_BUTTONS] = {false};
      static bool previous_slider_state = _preset_slider_on_set;

      if (_current_screen_state != _previous_screen_state) blink = false;
      _tft->setTextColor(TFT_WHITE);

      if ((_current_screen_state != _previous_screen_state) || _buttons_active[13]) {
        temp_color = blinkButtonColor(blink, _buttons_active[13], TFT_DARKGREEN);
        _tft->fillRect(0, 0, 160, 50, temp_color);
        tftDrawCentreString("BACK", 75, 15, FONT_SIZE_BIG);
      }

      if ((_current_screen_state != _previous_screen_state) ||
          (previous_slider_state != _preset_slider_on_set))
      {
        if (previous_slider_state != _preset_slider_on_set) {
          blink = !blink;
        }

        if (_preset_slider_on_set) {
          temp_color = blink ? TFT_PURPLE : TFT_RED;
          _tft->fillRect(240, 0, 70, 50, temp_color);
          tftDrawCentreString("SET", 275, 25, FONT_SIZE_SMALL);
          _tft->fillRect(170, 0, 70, 50, TFT_BLACK);
        } else {
          temp_color = blink ? TFT_PURPLE : TFT_DARKGREEN;
          _tft->fillRect(170, 0, 70, 50, temp_color);
          tftDrawCentreString("MOVE", 205, 25, FONT_SIZE_SMALL);
          _tft->fillRect(240, 0, 70, 50, TFT_BLACK);
        }
      }

      const int presetsLeft     = 5;
      const int presetsRight    = SCREEN_WIDTH - 5;
      const int presetsTopY     = 55;
      const int presetsBottomY  = 215;
      const int presetsCols     = 3;
      const int presetsRows     = 2;
      const int colGap          = 5;
      const int rowGap          = 5;

      const int totalWidth      = presetsRight - presetsLeft; // 310
      const int buttonWidth     = (totalWidth - colGap * (presetsCols - 1)) / presetsCols; // 100

      const int totalHeight     = presetsBottomY - presetsTopY; // 115
      const int buttonHeight    = (totalHeight - rowGap) / presetsRows; // 55

      int colX[3];
      colX[0] = presetsLeft;
      colX[1] = presetsLeft + buttonWidth + colGap;
      colX[2] = presetsLeft + 2 * (buttonWidth + colGap);

      int rowY[2];
      rowY[0] = presetsTopY;
      rowY[1] = presetsTopY + buttonHeight + rowGap;

      // Базовий колір для невибраних пресетів: зелений/червоний залежно від MOVE/SET
      uint16_t basePresetColor = _preset_slider_on_set ? TFT_RED : TFT_DARKGREEN;

      // --- PRESET_1 (індекс 0, кол.0, ряд0) ---
      {
        bool selected = (_selected_preset == 0);
        uint16_t color = selected ? TFT_YELLOW : basePresetColor;

        _tft->fillRect(colX[0], rowY[0], buttonWidth, buttonHeight, color);

        uint16_t textColor = selected ? TFT_BLACK : TFT_WHITE;
        _tft->setTextColor(textColor, color);

        String tempString = String(_presets[0][0]) + "    " + String(_presets[0][1]);
        tftDrawCentreString(tempString,
                            colX[0] + buttonWidth / 2,
                            rowY[0] + buttonHeight / 2 - 5,
                            FONT_SIZE_SMALL);
      }

      // --- PRESET_2 (індекс 1, кол.1, ряд0) ---
      {
        bool selected = (_selected_preset == 1);
        uint16_t color = selected ? TFT_YELLOW : basePresetColor;

        _tft->fillRect(colX[1], rowY[0], buttonWidth, buttonHeight, color);

        uint16_t textColor = selected ? TFT_BLACK : TFT_WHITE;
        _tft->setTextColor(textColor, color);

        String tempString = String(_presets[1][0]) + "    " + String(_presets[1][1]);
        tftDrawCentreString(tempString,
                            colX[1] + buttonWidth / 2,
                            rowY[0] + buttonHeight / 2 - 5,
                            FONT_SIZE_SMALL);
      }

      // --- PRESET_3 (індекс 2, кол.2, ряд0) ---
      {
        bool selected = (_selected_preset == 2);
        uint16_t color = selected ? TFT_YELLOW : basePresetColor;

        _tft->fillRect(colX[2], rowY[0], buttonWidth, buttonHeight, color);

        uint16_t textColor = selected ? TFT_BLACK : TFT_WHITE;
        _tft->setTextColor(textColor, color);

        String tempString = String(_presets[2][0]) + "    " + String(_presets[2][1]);
        tftDrawCentreString(tempString,
                            colX[2] + buttonWidth / 2,
                            rowY[0] + buttonHeight / 2 - 5,
                            FONT_SIZE_SMALL);
      }

      // --- PRESET_4 (індекс 3, кол.0, ряд1) ---
      {
        bool selected = (_selected_preset == 3);
        uint16_t color = selected ? TFT_YELLOW : basePresetColor;

        _tft->fillRect(colX[0], rowY[1], buttonWidth, buttonHeight, color);

        uint16_t textColor = selected ? TFT_BLACK : TFT_WHITE;
        _tft->setTextColor(textColor, color);

        String tempString = String(_presets[3][0]) + "    " + String(_presets[3][1]);
        tftDrawCentreString(tempString,
                            colX[0] + buttonWidth / 2,
                            rowY[1] + buttonHeight / 2 - 5,
                            FONT_SIZE_SMALL);
      }

      // --- PRESET_5 (індекс 4, кол.1, ряд1) ---
      {
        bool selected = (_selected_preset == 4);
        uint16_t color = selected ? TFT_YELLOW : basePresetColor;

        _tft->fillRect(colX[1], rowY[1], buttonWidth, buttonHeight, color);

        uint16_t textColor = selected ? TFT_BLACK : TFT_WHITE;
        _tft->setTextColor(textColor, color);

        String tempString = String(_presets[4][0]) + "    " + String(_presets[4][1]);
        tftDrawCentreString(tempString,
                            colX[1] + buttonWidth / 2,
                            rowY[1] + buttonHeight / 2 - 5,
                            FONT_SIZE_SMALL);
      }

      // --- PRESET_6 (індекс 5, кол.2, ряд1) ---
      {
        bool selected = (_selected_preset == 5);
        uint16_t color = selected ? TFT_YELLOW : basePresetColor;

        _tft->fillRect(colX[2], rowY[1], buttonWidth, buttonHeight, color);

        uint16_t textColor = selected ? TFT_BLACK : TFT_WHITE;
        _tft->setTextColor(textColor, color);

        String tempString = String(_presets[5][0]) + "    " + String(_presets[5][1]);
        tftDrawCentreString(tempString,
                            colX[2] + buttonWidth / 2,
                            rowY[1] + buttonHeight / 2 - 5,
                            FONT_SIZE_SMALL);
      }

      // --- Зберігаємо попередні стани кнопок і слайдера ---
      for (uint8_t i = 0; i < NUMBER_OF_BUTTONS; i++) {
        previous_buttons_state[i] = _buttons_active[i];
      }
      previous_slider_state = _preset_slider_on_set;

      break;
    }
        case CAMERA: { 
          if (_current_screen_state != _previous_screen_state) {
            this->sendStateTo(_receiverMac, GO_STREAM);
            _tft->fillScreen(TFT_BLACK);
            _camera_frame_received = false;

            startCameraStream(80);
          }

          if (!_camera_frame_received) {
            _tft->setTextColor(TFT_WHITE, TFT_BLACK);
            tftDrawCentreString("Waiting for the camera...", _centerX, _centerY, FONT_SIZE_SMALL);
          }

          _tft->setTextColor(TFT_WHITE); 
          tftDrawCentreString("BACK", 75, 15, FONT_SIZE_BIG);

          break;
        }
  }

  if (_previous_screen_state == CAMERA && _current_screen_state != CAMERA) { 
    this->sendStateTo(_receiverMac, STOP_STREAM);
    stopCameraStream();
    //stopSoftAP(); 
  }
  //_tft->fillRect(_centerX - 50, _centerY + 50, 100, 40, TFT_BLACK);
  //tftDrawCentreString(String(analogRead(_buttons_pin)), _centerX + 0, _centerY + 70, FONT_SIZE_BIG);
  if (_current_screen_state != CAMERA) {
    bool touchOnButton = isTouchOnButtonArea(_x_touch, _y_touch);

    if (_prev_touch_drawn && (!_prev_touch_on_button) && (!_z_touch || _x_touch != _x_touch_prev || _y_touch != _y_touch_prev)) {
      _tft->fillCircle(_x_touch_prev, _y_touch_prev, 5, TFT_BLACK);
      _prev_touch_drawn = false;
    }

    if ((_z_touch != 0) && !touchOnButton) {
      _tft->fillCircle(_x_touch, _y_touch, 5, TFT_YELLOW);
      _prev_touch_drawn = true;
    }

    _x_touch_prev = _x_touch;
    _y_touch_prev = _y_touch;
    _z_touch_prev = _z_touch;
    _prev_touch_on_button = touchOnButton;
  }

  if (_rtc_inited) {
    String yearStr = String(_date_time.year(), DEC);
    String monthStr = (_date_time.month() < 10 ? "0" : "") + String(_date_time.month(), DEC);
    String dayStr = (_date_time.day() < 10 ? "0" : "") + String(_date_time.day(), DEC);
    String hourStr = (_date_time.hour() < 10 ? "0" : "") + String(_date_time.hour(), DEC); 
    String minuteStr = (_date_time.minute() < 10 ? "0" : "") + String(_date_time.minute(), DEC);
    //String secondStr = (_date_time.second() < 10 ? "0" : "") + String(_date_time.second(), DEC);
    String dayOfWeek = daysOfTheWeek[_date_time.dayOfTheWeek()];
    String formated_time = dayOfWeek + ", " + yearStr + "-" + monthStr + "-" + dayStr + " " + hourStr + ":" + minuteStr;
    //tftDrawCentreString(formated_time, 100, 5, FONT_SIZE_SMALL);
  }
  //_display_lat_time_refreshed = now;
  _previous_screen_state = _current_screen_state;
}

void Controller::read_Encoders() {
  if (_key_pressed) return; //Encoders are not counting while keys are pressed
  uint64_t now = millis();
  int64_t new_encoder_1_value =  _encoder_1.getCount();
  int64_t new_encoder_2_value =  _encoder_2.getCount();
  int16_t prev_value1  = _preset__state[0];
  int16_t prev_value2  = _preset__state[1];
  if (abs(new_encoder_1_value - _encoder_1_value) > 1) {
    _preset__state[0] += (new_encoder_1_value - _encoder_1_value) / 2;
    if (_preset__state[0] >  170) {_preset__state[0] =  170;}
    if (_preset__state[0] < -170) {_preset__state[0] = -170;}
    _encoder_1_value = new_encoder_1_value;
  }
  if (abs(new_encoder_2_value - _encoder_2_value) > 1) {
    _preset__state[1] += (new_encoder_2_value - _encoder_2_value) / 2;
    if (_preset__state[1] >  30) {_preset__state[1] =  30;}
    if (_preset__state[1] < -30) {_preset__state[1] = -30;}
    _encoder_2_value = new_encoder_2_value;
  }
  //Making pause before start reacting on encoders position changing
  if (_preset__state[0] != prev_value1 || _preset__state[1] != prev_value2) { 
    //_changes_ap = false;
    if (!_changes_appears_flag) _changes_starts_time = now; //time when first change appeares after pause
    _changes_appears_flag = true;
    _last_time_changed = now;
  }
}

void Controller::staticOnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (_instance) {_instance->handleDataRecv(info, data, len);}
}

void Controller::handleDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len < sizeof(_data_received)) {
    Serial.println("Invalid data received");
    return;
  }
  memcpy(_data_received, data, sizeof(_data_received));
  Serial.printf("Data received from %02X:%02X:%02X:%02X:%02X:%02X: %d, %d, %d\n",
               info->src_addr[0], info->src_addr[1], info->src_addr[2],
               info->src_addr[3], info->src_addr[4], info->src_addr[5],
               _data_received[0], _data_received[1], _data_received[2]);
  senders  sender  =  intToSenders(_data_received[ARRAY_SIZE]);
  messages message = intToMessages(_data_received[ARRAY_SIZE + 1]);
  if ((sender == MOTION_PLATFORM) && (message == MOTION_REPORT) && checkPlatformMAC(info)) {
    Serial.println("New data from my platform");
    for (int i = 0; i < 3; i++) {
      _current_state[i] = _data_received[i];
    }
    _esp_now_last_time_pl_receving = millis();
  }

  if ((sender == SENSOR_PLATFORM) && (message == SENSOR_REPORT)) {
    for (int i = 3; i < ARRAY_SIZE; i++) {
      _current_state[i] = _data_received[i];
    }
    _esp_now_last_time_se_receving = millis();
  }

  if ((sender == MOTION_PLATFORM) && _waiting_for_new_MAC) {
    this->set_new_receiverMAC(info);
    _waiting_for_new_MAC = false;
  }
}

void Controller::defaultOnDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.printf("Data has been sent to %02X:%02X:%02X:%02X:%02X:%02X | Status: %s\n", 
  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
  status == ESP_NOW_SEND_SUCCESS ? "Success" : "Error");
}

bool Controller::addPeer(const uint8_t* mac) {
  if (!_is_ESPNOW_Initialized) return false;

  esp_now_peer_info_t peerInfo;
  if (esp_now_get_peer(mac, &peerInfo) == ESP_OK) {return true;} 

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return false;
  }
  return true;
}

void Controller::sendStateTo(const uint8_t* mac, messages message) {
  if (!_is_ESPNOW_Initialized) {
    Serial.println("ESP-NOW not initialized");
    return;
  }

  if (!addPeer(mac)) {return;}

  for (uint8_t i = 0; i < ARRAY_SIZE; i ++) {_data_to_send[i] = _desired_state[i];}
  _data_to_send[ARRAY_SIZE]     = static_cast<int16_t>(CONTROLLER);
  _data_to_send[ARRAY_SIZE + 1] = static_cast<int16_t>(message);

  esp_err_t result = esp_now_send(mac, (uint8_t*)&_data_to_send, sizeof(_data_to_send));
  
  //if (result == ESP_OK) {Serial.printf("State sent successfully with message: %d\n", _data_to_send[ARRAY_SIZE + 1]);}
  //else {Serial.println("Error sending state");}
}

bool Controller::checkPlatformMAC(const esp_now_recv_info_t *info) {
  for (uint8_t i = 0; i < 6; i++) {
    if (info->src_addr[i] != _receiverMac[i]) return false;
  }
  return true;
}

void Controller::confirmChanges() {
  if (_key_pressed) {
    if (_current_key_value > 2){              //Horizontal button pressed, so we're using current horizontal position as desired
      _preset__state[0] = _current_state[0];
      _desired_state[0] = _preset__state[0];
      return;
    }
    _preset__state[1] = _current_state[1];    //Vertical button pressed, so we're using current vertical position as desired
    _desired_state[1] = _preset__state[1];
    return;
  }
  if (!_changes_appears_flag) return;        //no button pressed and no changes on encoders
  uint64_t now = millis();
  if (now - _changes_starts_time > DELAY_TO_CONFIRM) {//HERE IS A DELAY BETWEEN ENCODER CHANGES AND CONTROLLER REACTION
    _desired_state[0] = _preset__state[0];
    _desired_state[1] = _preset__state[1];
  }
  if (now - _last_time_changed > DELAY_TO_STOP_CHANGES) _changes_appears_flag = false; //No changes for 1 sec, so paause
}

void Controller::smartSending() {
  //SHORT DELAY IF SOMETHING SHOULD TO BE CHANGED IN POSITION. IF NOT - LONGER DELAY JUST TO KEEP IN CONTACT
  this->confirmChanges();
  uint16_t delay_Sending = ((_current_state[0] != _desired_state[0]) || (_current_state[1] != _desired_state[1]) || (_current_state[2] != 0)) ? SENDING_SHORT_DELAY : SENDING_LONG_DELAY;
  uint64_t now = millis();
  if (now - _esp_now_last_time_sending < delay_Sending) {return;}
  this->sendStateTo(_receiverMac);
  _esp_now_last_time_sending = now;
}

void  Controller::playAudio() {
  File audioFile = SD.open(_audio_File_Name);
  if (!audioFile) {
    Serial.println("Failed to open audio file");
    return;
  }

  audioFile.seek(44);
  while (audioFile.available()) {
    size_t bytesRead = audioFile.read(_audio_Buffer, sizeof(_audio_Buffer));
    for (size_t i = 0; i < bytesRead; i++) {
      dacWrite(DAC_PIN, _audio_Buffer[i]);
      delayMicroseconds(22);
    }
  }

  audioFile.close();
  Serial.println("Playback finised");
}

int16_t   Controller::get_key_value() {
  static uint16_t   count;
  static int16_t    oldKeyValue;
  static int16_t    innerKeyValue;  

  uint8_t actualKeyValue = this->get_button_number_by_value(analogRead(_buttons_pin));
  delayMicroseconds(100);
  
  if (innerKeyValue != actualKeyValue) {
    count = 0;
    innerKeyValue = actualKeyValue;
  }
  else {count++;}

  if ((count >= 10) && (actualKeyValue != oldKeyValue)) {
    oldKeyValue = actualKeyValue;
    _key_pressed = (oldKeyValue > 0);
  }
  return    oldKeyValue;
}

int16_t   Controller::get_button_number_by_value(int16_t value) {
  //uint16_t values[5] = {0, 600, 1420, 1930, 3060};
  uint16_t values[5] = {0, 440, 1170, 1950, 2935};
  for (uint8_t i = 0; i < 5; i++) {
    if (aprox_eq(value, values[i])) return i;
  }
  return 0;
}

void   Controller::define_key_value() {
  int16_t new_key_value = this->get_key_value();
  if (_current_key_value != new_key_value) {
    _current_key_value = new_key_value;
    _desired_state[2] = _current_key_value;
    Serial.print("NEW DESIRED STATE: ");
    Serial.println(_desired_state[2]);
    if (_key_pressed) {
      //_desired_state[0] = _current_state[0];
      //_desired_state[1] = _current_state[1];
      _encoder_1.pauseCount();
      _encoder_2.pauseCount();
    }
    else {
      _encoder_1.resumeCount();
      _encoder_2.resumeCount();
    }
  }
}

void  Controller::check_platform_connection() {
  //If conection with platform apears - current position set as start. 
  //_platform_connected_flag = (millis() - _esp_now_last_time_receving > 1000)
  if (millis() - _esp_now_last_time_pl_receving < 1000) {
    if (!_platform_connected_flag) {
      Serial.println("Platform connected...");
      for (int i = 0; i < ARRAY_SIZE; i++) {
        _desired_state[i] = _current_state[i];
        _preset__state[i] = _current_state[i];
      }
      _platform_connected_flag = true;
    }
    return;
  }
  _platform_connected_flag = false;
}

void  Controller::check_sensors_connection() {
  //If conection with platform apears - current position set as start. 
  //_platform_connected_flag = (millis() - _esp_now_last_time_receving > 1000)
  if (millis() - _esp_now_last_time_se_receving < 1000) {
    if (!_sensors_connected_flag) {
      Serial.println("Sensors connected...");
      _sensors_connected_flag = true;
    }
    return;
  }
  _sensors_connected_flag = false;
}

void  Controller::touchScreenReading() {
  uint64_t now = millis();
  static uint64_t last_touch_time;
  if (_touchscreen.tirqTouched() && _touchscreen.touched()) {
    TS_Point p = _touchscreen.getPoint();
    _x_touch = map(p.x, 200, 3700, SCREEN_WIDTH,  1);
    _y_touch = map(p.y, 240, 3800, SCREEN_HEIGHT, 1);
    _z_touch = p.z;
    last_touch_time = now;
  }
  if (now - last_touch_time > TOUCH_STEP) {
    _x_touch = 0;
    _y_touch = 0;
    _z_touch = 0;
  }
}

void  Controller::defineScreenState() {
  uint64_t now = millis();
  if (_screen_state_changed && (now - _screen_state_changed_time > TOUCH_STEP)) {
    _current_screen_state = _changed_screen_state;
    for (uint8_t i = 0; i < NUMBER_OF_BUTTONS; i++) {
      _buttons_active[i] = false;
    }
    _screen_state_changed = false;
    _z_touch = 0;
    return;
  }

  if (!_screen_state_changed && _z_touch) {
    switch (_current_screen_state) {
      case MAIN:
        if ((_x_touch < 105) && (_y_touch < 50)) {
          _changed_screen_state = MENU;
          _screen_state_changed_time = now;
          _screen_state_changed = true;
          _buttons_active[0] = true;
        }
        if ((_x_touch > 106) && (_x_touch < 210) && (_y_touch < 50)) {
          _changed_screen_state = SENSOR_PANEL;
          _screen_state_changed_time = now;
          _screen_state_changed = true;
          _buttons_active[1] = true;
        }
        if ((_x_touch > 211) && (_y_touch < 50)) {
          _changed_screen_state = PRESETS;
          _screen_state_changed_time = now;
          _screen_state_changed = true;
          _buttons_active[12] = true;
        }
        break;
      case MENU:
        if ((_x_touch < 155) && (_y_touch < 50)) {
          _changed_screen_state = MAIN;
          _screen_state_changed_time = now;
          _screen_state_changed = true;
          _buttons_active[2] = true;
        }
        if ((_x_touch >= 160) && (_y_touch < 50)) {
          _changed_screen_state = CAMERA;
          _screen_state_changed_time = now;
          _screen_state_changed = true;
          _buttons_active[20] = true;
        }
        break;
      case SENSOR_PANEL:
        if ((_x_touch < 155) && (_y_touch < 50)) {
          _changed_screen_state = MAIN;
          _screen_state_changed_time = now;
          _screen_state_changed = true;
          _buttons_active[3] = true;
        }
        break;
      case PRESETS:
        if ((_x_touch < 155) && (_y_touch < 50)) {
          _changed_screen_state = MAIN;
          _screen_state_changed_time = now;
          _screen_state_changed = true;
          _buttons_active[13] = true;
        }
        break;
      case CAMERA: 
        if ((_x_touch < 155) && (_y_touch < 50)) { 
          _changed_screen_state = MENU; 
          _screen_state_changed_time = now; 
          _screen_state_changed = true; 
        } 
        break;
    }
  }
}

void  Controller::defineMenuButton() {
  if (_current_screen_state != MENU) return;
  uint64_t now = millis();
  for (uint8_t i = 4; i < 12; i++) {
    if (_buttons_active[i]) {
      if (now - _menu_buttons_pressed_time[i] < TOUCH_STEP * 2) return;
      _buttons_active[i] = false;
    }
  }

  if (!_z_touch) return;
  
    //Serial.printf("Touch appeared. Current button is: %d", _current_menu_button);
  if ((_y_touch > 70) && (_y_touch < 121)) {
    if ((_x_touch > 8) && (_x_touch < 84)) {
      Serial.println("Button HOR TO 0 is active!");
      _desired_state[0] = 0;
      _preset__state[0] = 0;
      _buttons_active[HOR_TO_0] = true;
      _menu_buttons_pressed_time[HOR_TO_0] = now;
      return;
    }
    if ((_x_touch > 84) && (_x_touch < 160)) {
      Serial.println("Button VER TO 0 is active!");
      _desired_state[1] = 0;
      _preset__state[1] = 0;
      _buttons_active[VER_TO_0] = true;
      _menu_buttons_pressed_time[VER_TO_0] = now;
      return;
    }
    if ((_x_touch > 160) && (_x_touch < 236)) {
      _desired_state[0] = 0;
      _desired_state[1] = 0;
      _preset__state[0] = 0;
      _preset__state[1] = 0;
      Serial.println("Button ALL TO 0 is active!");
      _buttons_active[ALL_TO_0] = true;
      _menu_buttons_pressed_time[ALL_TO_0] = now;
      return;
    }
    if ((_x_touch > 236) && (_x_touch < 311)) {
      Serial.println("Button MUTE is active!");
      _buttons_active[MUTE] = true;
      _menu_buttons_pressed_time[MUTE] = now;
      return;
    }
  }
 
  if ((_y_touch > 121) && (_y_touch < 171)) {
    if ((_x_touch > 8) && (_x_touch < 84)) {
      _waiting_for_new_MAC = true;
      this->sendStateTo(_broadcastMac, NEW_CONTROLLER);
      //Serial.println("Button CONNECT is active!");
      _buttons_active[CONNECT] = true;
      _menu_buttons_pressed_time[CONNECT] = now;
      return;
    }
    if ((_x_touch > 84) && (_x_touch < 160)) {
      _desired_state[0] = 0;
      _preset__state[0] = 0;      
      this->sendStateTo(_receiverMac, HORIZONTAL_CALIBRATION);
      
      Serial.println("Button HOR SET 0 is active!");
      _buttons_active[HOR_SET_0] = true;
      _menu_buttons_pressed_time[HOR_SET_0] = now;
      return;
    }
    if ((_x_touch > 160) && (_x_touch < 236)) {
      _desired_state[1] = 0;
      _preset__state[1] = 0;
      this->sendStateTo(_receiverMac, VERTICAL_CALIBRATION);
      Serial.println("Button VER SET 0 is active!");
      _buttons_active[VER_SET_0] = true;
      _menu_buttons_pressed_time[VER_SET_0] = now;
      return;
    }
    if ((_x_touch > 236) && (_x_touch < 311)) {
      Serial.println("Button 8 is active!");
      _buttons_active[_8] = true;
      _menu_buttons_pressed_time[_8] = now;
      return;
    }
  }
  return;
}

void  Controller::definePresetsButton() {
  static preset_buttons button_pressed = NO_BUTTON;
  if (_current_screen_state != PRESETS) return;
  uint64_t now = millis();
  for (uint8_t i = 14; i < NUMBER_OF_BUTTONS; i++) {
    if (_buttons_active[i]) {
      if (now - _preset_buttons_pressed_time[i] < TOUCH_STEP * 2) return;
      _buttons_active[i] = false;
    }
  }

  if (!_z_touch) return;

  //CREATE NEW BUTTONS!!!
  
    //Serial.printf("Touch appeared. Current button is: %d", _current_menu_button);
  if ((_y_touch > 70) && (_y_touch < 121)) {
    if ((_x_touch > 10) && (_x_touch < 90)) {
      //Serial.println("Button DEFINE_PRESET_1 is active!");
      //this->savePreset(0);
      _buttons_active[PRESET_1] = true;
      _menu_buttons_pressed_time[PRESET_1] = now;
      button_pressed = PRESET_1;
      //return;
    }
    if ((_x_touch > 110) && (_x_touch < 190)) {
      //Serial.println("Button LOAD_PRESET_1 is active!");
      //_desired_state[0] = _presets[0][0];
      //_preset__state[0] = _presets[0][0];
      //_desired_state[1] = _presets[0][1];
      //_preset__state[1] = _presets[0][1];
      _buttons_active[PRESET_2] = true;
      _menu_buttons_pressed_time[PRESET_2] = now;
      button_pressed = PRESET_2;
      //return;
    }
    if ((_x_touch > 210) && (_x_touch < 290)) {

      _buttons_active[PRESET_3] = true;
      _menu_buttons_pressed_time[PRESET_3] = now;
      button_pressed = PRESET_3;
      //return;
    }
  }
 
  if ((_y_touch > 121) && (_y_touch < 171)) {
    if ((_x_touch > 10) && (_x_touch < 90)) {
      //Serial.println("Button DEFINE_PRESET_2 is active!");
      //this->savePreset(1);
      _buttons_active[PRESET_4] = true;
      _menu_buttons_pressed_time[PRESET_4] = now;
      button_pressed = PRESET_4;
      //return;
    }
    if ((_x_touch > 110) && (_x_touch < 190)) {
      //Serial.println("Button LOAD_PRESET_2 is active!");
      //_desired_state[0] = _presets[1][0];
      //_preset__state[0] = _presets[1][0];
      //_desired_state[1] = _presets[1][1];
      //_preset__state[1] = _presets[1][1];
      _buttons_active[PRESET_5] = true;
      _menu_buttons_pressed_time[PRESET_5] = now;
      button_pressed = PRESET_5;
      //return;
    }
    if ((_x_touch > 210) && (_x_touch < 290)) {
      _buttons_active[PRESET_6] = true;
      _menu_buttons_pressed_time[PRESET_6] = now; 
      button_pressed = PRESET_6;
      //return;
    }
  }

  if ( _preset_slider_on_set && (_x_touch > 170) && (_x_touch < 240) && (_y_touch < 50)) {_preset_slider_on_set = false;}
  if (!_preset_slider_on_set && (_x_touch > 240) && (_x_touch < 310) && (_y_touch < 50)) {_preset_slider_on_set = true; }
  if (button_pressed != NO_BUTTON) {
    int8_t idx = button_pressed - 14;

    if (_preset_slider_on_set) {
      this->savePreset(idx);
    } else {
      _desired_state[0] = _presets[idx][0];
      _preset__state[0] = _presets[idx][0];
      _desired_state[1] = _presets[idx][1];
      _preset__state[1] = _presets[idx][1];
    }

    _selected_preset = idx;

    button_pressed = NO_BUTTON;
  }
  return;
}

void Controller::checkingDateTime() {
  uint64_t now = millis();
  static uint64_t last_time_checked = 0;
  if (_rtc_inited && (now - last_time_checked > CHECKING_TIME_DELAY)) {
    _date_time = _rtc.now();
    last_time_checked = now;
    Serial.printf("Year is : %d\tMounth is : %d\n", _date_time.year(), _date_time.month());
    Serial.printf("Hour is : %d\tMin is : %d\n", _date_time.hour(), _date_time.minute());
  }
}

String Controller::findNextLogFile() {
  if (!_is_SD_Initialized) return "";
  if (!_rtc_inited) {
    if (!SD.exists("/no_date")) {
      if (SD.mkdir("/no_date")) {
        Serial.println(" /no_date has been created");
      } else {
        Serial.println("Failed to create directory!");
      }
    }
    for (uint16_t i = 0; i < 65535; i++) {
      String name = "/no_date/" + String(i) + ".txt";
      if (!SD.exists(name)) return name;
    }
  }
  String yearStr = String(_date_time.year(), DEC);
  String monthStr = (_date_time.month() < 10 ? "0" : "") + String(_date_time.month(), DEC);
  String dayStr = (_date_time.day() < 10 ? "0" : "") + String(_date_time.day(), DEC);
  _log_Dir = "/" +  yearStr + "-"  + monthStr + "-" + dayStr;
  if (!SD.exists(_log_Dir)) {
    if (SD.mkdir(_log_Dir)) {
      Serial.printf("%s has been created\n", _log_Dir);
    } else {
      Serial.println("Failed to create directory!");
    }
  }
  for (uint16_t i = 0; i < 65535; i++) {
    String name = _log_Dir + "/" + String(i) + ".txt"; 
    if (!SD.exists(name)) return name;
  }
  return "/data.txt";
}

void  Controller::writeToLog(String message) {
  if (!_file_created) return;
  String hourStr = (_date_time.hour() < 10 ? "0" : "") + String(_date_time.hour(), DEC); 
  String minuteStr = (_date_time.minute() < 10 ? "0" : "") + String(_date_time.minute(), DEC);
  String secondStr = (_date_time.second() < 10 ? "0" : "") + String(_date_time.second(), DEC);
  _log_Buffer += hourStr + ":" + minuteStr + ":" + secondStr + "\t" + message + "\n";
  if (_log_Buffer.length() > 1024) {
    _log_file.print(_log_Buffer);
    _log_file.flush();
    _log_Buffer = "";
  }
}

void  Controller::makeLogReport() {
  uint64_t now = millis();
  if (now - _last_time_log_written < DELAY_LOG_WRITING) return;
  if (!_platform_connected_flag) return;
  String Message = "Regular report: Position hor:\t";
  Message += String(_current_state[0]);
  Message += "\tPosition ver: \t";
  Message += String(_current_state[1]);
  if (!_sensors_connected_flag) {
    Message += ".\tSensors are not connected.";
    this->writeToLog(Message);
    return;
  }
  for (uint8_t i = 0; i < 4; i++) {
    Message += ("\tCurrent_" + String(i) + ":\t");
    Message += String(_current_state[3 + 2 * i]);
    Message += ("\tVoltage_" + String(i) + ":\t");
    Message += String(_current_state[4 + 2 * i]);
  }
  Message += "\tTemperature:\t";
  Message += String(_current_state[11]);
  Message += "\tHumidity:\t";
  Message += String(_current_state[12]);
  Message += "\tPreasure:\t";
  Message += String(_current_state[12]);
  Message += ".";
  this->writeToLog(Message);
  _last_time_log_written = now;
}

void  Controller::set_new_receiverMAC(const esp_now_recv_info_t *info) {
  for (uint8_t i = 0; i < 6; i++) {
    _receiverMac[i] = info->src_addr[i];
  }
  uint16_t address = 4;
  for (uint8_t i = 0; i < 6; i++) {
    EEPROM.put(address, _receiverMac[i]);
    address += sizeof(int8_t);
  }
  if (EEPROM.commit()) {Serial.println("EEPROM data has been written");}
}

void  Controller::newPlatformConnection() {
}

void  Controller::savePreset(uint8_t preset_number) {
  _presets[preset_number][0] = _current_state[0];
  _presets[preset_number][1] = _current_state[1];
  EEPROM.put(10 + preset_number * 4, _presets[preset_number][0]);
  EEPROM.put(12 + preset_number * 4, _presets[preset_number][1]);
  if (EEPROM.commit()) {Serial.printf("New presset %d has been successfully saved\n", preset_number);}
}

void Controller::startCameraServer(uint16_t port) {
  if (_camServer) {
    _camServer->stop();
    delete _camServer;
    _camServer = nullptr;
  }
  _cam_server_port = port;
  _camServer = new WiFiServer(_cam_server_port);
  _camServer->begin();
}

void Controller::stopCameraServer() {
  if (_camServer) {
    _camServer->stop();
    delete _camServer;
    _camServer = nullptr;
  }
  Serial.println("Camera TCP server stopped");
}

void Controller::startCameraStream(uint16_t port) {
  this->sendStateTo(_receiverMac, GO_STREAM);
  startCameraServer(port);
}

void Controller::stopCameraStream() {
  this->sendStateTo(_receiverMac, STOP_STREAM);
  stopCameraServer();
}

void Controller::handleCameraTCP() {
  if (!_camServer) return;

  if (_current_screen_state != CAMERA) {
    if (_camClientConnected) {
      _camClient.stop();
      _camClientConnected = false;
    }
    if (_frameBuf) {
      free(_frameBuf);
      _frameBuf = nullptr;
    }
    _nextFrameLen   = 0;
    _frameBytesRead = 0;
    return;
  }

  if (!_camClientConnected) {
    WiFiClient newClient = _camServer->available();
    if (!newClient) return;

    _camClient = std::move(newClient);
    _camClient.setNoDelay(true);
    _camClientConnected = true;
    Serial.println("Controller: camera client connected (JPEG stream)");

    _nextFrameLen   = 0;
    _frameBytesRead = 0;
    if (_frameBuf) {
      free(_frameBuf);
      _frameBuf = nullptr;
    }
  }

  if (!_camClient.connected()) {
    Serial.println("Controller: camera client disconnected");
    _camClient.stop();
    _camClientConnected = false;
    if (_frameBuf) {
      free(_frameBuf);
      _frameBuf = nullptr;
    }
    _nextFrameLen   = 0;
    _frameBytesRead = 0;
    return;
  }

  if (_nextFrameLen == 0) {
    if (_camClient.available() < 4) return;
    uint8_t lenBuf[4];
    int n = _camClient.read(lenBuf, 4);
    if (n != 4) {
      Serial.println("Controller: failed to read frame length");
      _camClient.stop();
      _camClientConnected = false;
      return;
    }
    _nextFrameLen = (uint32_t)lenBuf[0] |
                    ((uint32_t)lenBuf[1] << 8) |
                    ((uint32_t)lenBuf[2] << 16) |
                    ((uint32_t)lenBuf[3] << 24);

    if (_nextFrameLen == 0 || _nextFrameLen > 65535) {
      Serial.printf("Controller: invalid frame length: %u\n", _nextFrameLen);
      _nextFrameLen = 0;
      return;
    }

    if (_frameBuf) {
      free(_frameBuf);
      _frameBuf = nullptr;
    }
    _frameBuf = (uint8_t*)malloc(_nextFrameLen);
    if (!_frameBuf) {
      Serial.println("Controller: failed to allocate frame buffer");
      _nextFrameLen = 0;
      return;
    }
    _frameBytesRead = 0;
    _frameStartTime = millis();
    Serial.printf("Controller: expecting JPEG frame of %u bytes\n", _nextFrameLen);
  }

  if (_frameBytesRead < _nextFrameLen) {
    if (millis() - _frameStartTime > 1000) {
      Serial.println("Controller: frame read timeout, closing camera client");
      if (_frameBuf) {
        free(_frameBuf);
        _frameBuf = nullptr;
      }
      _nextFrameLen   = 0;
      _frameBytesRead = 0;

      _camClient.stop();
      _camClientConnected = false;
      return;
    }

    int avail = _camClient.available();
    if (avail <= 0) return;

    int toRead = _nextFrameLen - _frameBytesRead;
    if (toRead > avail) toRead = avail;
    int n = _camClient.read(_frameBuf + _frameBytesRead, toRead);
    if (n > 0) {
      _frameBytesRead += n;
    }
  }

  if (_frameBytesRead == _nextFrameLen) {
    Serial.printf("Controller: full JPEG frame received: %u bytes\n", _nextFrameLen);
    _camera_frame_received = true;

    if (_tft) {
      // _tft->fillScreen(TFT_BLACK);
      TJpgDec.drawJpg(0, 0, _frameBuf, _nextFrameLen);
    }

    free(_frameBuf);
    _frameBuf       = nullptr;
    _nextFrameLen   = 0;
    _frameBytesRead = 0;
  }
}

bool Controller::isTouchOnButtonArea(int16_t x, int16_t y) {
  switch (_current_screen_state) {
    case MAIN:
      if (y < 50) return true;
      return false;

    case MENU:
      if (y < 50) return true;
      if (y >= 70 && y <= 70 + 74 && x >= 8 && x <= 311) return true;
      if (y >= 70 + 74 + 1 && y <= 70 + 74 + 1 + 74 && x >= 8 && x <= 311) return true;
      return false;

    case SENSOR_PANEL:
      if (y < 50) return true;
      return false;

    case PRESETS:
      if (y < 50) return true;
      if (y >= 55 && y <= 215 && x >= 5 && x <= 315) return true;
      return false;

  return false;
  }
}