#include "Platform.h"
#include <Arduino.h>
//To compare two integers. We asume that difference less than delta is negligible
bool A_much_bigger_B(int16_t a, int16_t b, int16_t delta = 3) {
  return a - b > delta;
}

bool aprox_eq(int16_t a, int16_t b, int16_t delta = 1) {
  return abs(a - b) <= delta;
}

int16_t custom_round(float x, int16_t target) {
    float diff = x - target;
    return static_cast<int16_t>(std::round(diff)) + target;
}
// For filtering data from potentiometer with 3-size window
class FastMedianFilter {
private:
  int16_t _buffer[3];
  byte _index = 0;
  bool _buffer_not_full = true;
public:
  int16_t addValue(int16_t new_Val) {
    _buffer[_index] = new_Val;
    _index = (_index + 1) % 3;
    if (_buffer_not_full && (_index < 2)) return new_Val;
    _buffer_not_full = false;
    int16_t a = _buffer[0];
    int16_t b = _buffer[1];
    int16_t c = _buffer[2];
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
  }
};

class ExpSmoother {
private:
  float _smoothedValue = 0;
  float _alpha;
public:
  ExpSmoother(float a = 0.2) : _alpha(a) {}
  int16_t addValue(int16_t new_Val) {
    _smoothedValue = _alpha * new_Val + (1 - _alpha) * _smoothedValue;
    return (int16_t)_smoothedValue;
  }
};

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

bool MT6701::begin() {
  if (!_wire) {
    Serial.println("MT6701: Wire object is null!");
    return false;
  }
  _wire->beginTransmission(_address);
  _lastError = _wire->endTransmission();
  if (_lastError != 0) {
    Serial.printf("MT6701: device not found at 0x%02X (error %d)\n", _address, _lastError);
    return false;
  }
  Serial.println("MT6701: initialized successfully");
  return true;
}

float MT6701::readAngle() {
  uint8_t buffer[2];
  _wire->beginTransmission(_address);
  _wire->write(0x03);
  _lastError = _wire->endTransmission(false);
  if (_lastError != 0) {
    Serial.printf("MT6701: write error (%d) \n", _lastError);
    return NAN;
  }
  uint8_t bytesRead = _wire->requestFrom((int)_address, 2);
  if (bytesRead != 2) {
    Serial.println("MT6701: read error (less than 2 bytes)");
    _lastError = -1;
    return NAN;
  }
  buffer[0] = _wire->read();
  buffer[1] = _wire->read();

  uint raw = ((buffer[0] << 6) | buffer[1]);
  float angle = (raw * 360.0f) / 16384.0f;
  _lastError = 0;
  return angle;
}

/*Pointer on current object. We need this, cause esp_now_register_recv_cb (function that is called, when packet from ESP-NOW received)
does not take simple method as agument. It could be simple function or static method.*/
Platform* Platform::_instance = nullptr;

Platform::Platform()
{  
  _instance = this; //static pointer now points on current object
}

bool Platform::initiation_EEPROM() { 

  // | 0 - 1 : ver cal | 2 - 3 : hor cal | 4 - 9 : MAC CONTROLLER | 10 - 15 : MAC SENSORS

  if (!EEPROM.begin(EEPROM_SIZE)) { 
    Serial.println("Error EEPROM"); 
    return false; 
  }
  if (EEPROM.read(INIT_ADR) != INIT_KEY) { 
    uint16_t address = 0; EEPROM.put(INIT_ADR, INIT_KEY);

    EEPROM.put(address, _vertical_calibration);
    address += sizeof(_vertical_calibration);
    EEPROM.put(address, _horizontal_calibration);
    address += sizeof(_horizontal_calibration);

    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.put(address, _receiverMac[i]);
      address += sizeof(uint8_t);
    }

    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.put(address, _sensorsMac[i]);
      address += sizeof(uint8_t);
    }

    for(uint8_t i=0;i<6;i++) { 
      EEPROM.put(address, _cameraMac[i]); 
      address += sizeof(uint8_t); 
    }

    if (EEPROM.commit()) {
      Serial.println("EEPROM data has been written");
      return true;
    } else {
      Serial.println("Some EEPROM errors");
      return false;
    }
  } else { 
    uint16_t address = 0;

    EEPROM.get(address, _vertical_calibration);
    Serial.printf("Vercical calibration is: %d\n", _vertical_calibration);
    address += sizeof(_vertical_calibration);

    EEPROM.get(address, _horizontal_calibration);
    Serial.printf("Horizontal calibration is: %d\n", _horizontal_calibration);
    address += sizeof(_horizontal_calibration);

    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.get(address, _receiverMac[i]);
      address += sizeof(uint8_t);
    }
    Serial.printf("Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  _receiverMac[0], _receiverMac[1], _receiverMac[2],
                  _receiverMac[3], _receiverMac[4], _receiverMac[5]);

    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.get(address, _sensorsMac[i]);
      address += sizeof(uint8_t);
    }
    Serial.printf("Sensors MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  _sensorsMac[0], _sensorsMac[1], _sensorsMac[2],
                  _sensorsMac[3], _sensorsMac[4], _sensorsMac[5]);
    
    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.get(address, _cameraMac[i]);
      address += sizeof(uint8_t);
    }
    Serial.printf("Camera MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  _cameraMac[0], _cameraMac[1], _cameraMac[2],
                  _cameraMac[3], _cameraMac[4], _cameraMac[5]);
  }
  return true; 
}

void Platform::begin() {
  const esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 3000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
   /*               ESP-NOW initialization              */  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
  //  Serial.println("Error initialization ESP-NOW");
    _is_ESPNOW_Initialized = false;
    return;
  }
  //Registration callback functions
  esp_now_register_send_cb(defaultOnDataSent); // Here is non-static method just because it never use this-poiner (doing basicaly nothing), so C compiler optimize it to simple function
  esp_now_register_recv_cb(staticOnDataRecv); 
  _is_ESPNOW_Initialized  = true;
  /*               Encoder initialization            */
  delay(100); // just during initialization. To be on a safe side.

  /*THIS SHOULD BE CHANGED ON MAGNETIC ENCODER*/
  //pinMode(CLK_PIN, INPUT_PULLUP);
  //pinMode(DT_PIN,  INPUT_PULLUP);
  //_optic_Encoder.attachHalfQuad(CLK_PIN, DT_PIN);

  /*              Magnetic Encoder MT6701 (horizontal) setup          */
  Wire.begin(21, 22);
  _mt6701.begin(); 

  /*               Potentiometer setup               */
  /*THIS SHOULD BE CHANGED ON MAGNETIC ENCODER*/
  //analogReadResolution(12);

  /*                Magnetic Encoder AS5600 (vertical) setup          */
  
  _as5600.begin(4);
  _as5600.setDirection(AS5600_COUNTERCLOCK_WISE);
  Serial.print("Connecting as5600... ");
  Serial.println(String(_as5600.isConnected() ? "DONE" : "ERROR"));
  delay(100);
  _i2cMutex = xSemaphoreCreateMutex();
  BaseType_t result = xTaskCreatePinnedToCore(encoderTask, "Encoder Task", 4096, this, 2, NULL, 1);
  if (result != pdPASS) {
    Serial.println("Failed to create Encoder Task!");
  }
  delay(100);
  this->startPIDTask();
  /*Pin initialization. VERY important!*/
  delay(100);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  ledcAttach(PIN_IN1, _pwm_freq, _pwm_resolution);
  ledcAttach(PIN_IN2, _pwm_freq, _pwm_resolution);
  ledcAttach(PIN_IN3, _pwm_freq, _pwm_resolution);
  ledcAttach(PIN_IN4, _pwm_freq, _pwm_resolution);
  ledcWrite(PIN_IN1, 0); ledcWrite(PIN_IN2, 0); ledcWrite(PIN_IN3, 0); ledcWrite(PIN_IN4, 0);
  /*The way how ESP works with PWM. This is for 3. core version. For 2. it's quite different*/
  //-----------------EEPROM INITIALIZATION--------------//
  this->initiation_EEPROM();
}

void Platform::staticOnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (_instance) {_instance->handleDataRecv(info, data, len);}  //just calling non-static method throught pointer
}

void Platform::handleDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len < sizeof(_data_received)) {
    Serial.println("Invalid data received");
    return;
  }
  _last_ESPNOW = millis();
  memcpy(_data_received, data, sizeof(_data_received));
  /*Serial.printf("Data received from %02X:%02X:%02X:%02X:%02X:%02X: %d, %d\n",
               info->src_addr[0], info->src_addr[1], info->src_addr[2],
               info->src_addr[3], info->src_addr[4], info->src_addr[5],
               _data_received[0], _data_received[1]);*/
  senders  sender  =  intToSenders(_data_received[ARRAY_SIZE]);
  messages message = intToMessages(_data_received[ARRAY_SIZE + 1]);
  if (message != DESIRED) {Serial.printf("Not desired message %d from %d\n", message, sender);}
  if ((sender == CONTROLLER) && (message == DESIRED) && checkCotrollerMAC(info)) {
    for (int i = 0; i < 3; i++) {
      if (_desired_state[i] != _data_received[i]) {_desired_changed = true; _second_chance = true;}
      _desired_state[i] = _data_received[i];
    }
  }
  if ((sender == CONTROLLER) && (message == HORIZONTAL_CALIBRATION) && checkCotrollerMAC(info)) {
    Serial.println("Horizontal calibration message");
    this->set_horizontal_calibration();
  }
  if ((sender == CONTROLLER) && (message == VERTICAL_CALIBRATION) && checkCotrollerMAC(info)) {
    Serial.println("Verical calibration message");
    this->set_vertical_calibration();
  }
  if ((sender == CONTROLLER) && (message == NEW_CONTROLLER)) {
    Serial.println("New controller connection...");
    this->set_new_receiverMac(info);
    this->addPeer(_receiverMac);
  }
  if ((sender == CONTROLLER) && (message == GO_STREAM) && checkCotrollerMAC(info)) {
    Serial.println("GO_STREAM");
  }

  if ((sender == CONTROLLER) && (message == STOP_STREAM) && checkCotrollerMAC(info)) {
    Serial.println("STOP_STREAM");
  }
  if (sender == SENSOR_PLATFORM && message == SENSOR_REPORT) { 
    Serial.println("New sensor connection...");
    this->saveSensorsMac(info->src_addr);
  }
}

void Platform::defaultOnDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  //DOING NOTHING. May add some Serial prints. This works when ESP sends packets
  //Serial.printf("Data has been sent to %02X:%02X:%02X:%02X:%02X:%02X | Status: %s\n", 
  //mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
  //status == ESP_NOW_SEND_SUCCESS ? "Success" : "Error");
}

bool Platform::addPeer(const uint8_t* mac) {
  if (!_is_ESPNOW_Initialized) return false;

  esp_now_peer_info_t peerInfo;
  if (esp_now_get_peer(mac, &peerInfo) == ESP_OK) {return true;} 
  //Setting peer parameters
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

//Sending _current_state function//
void Platform::sendStateTo(const uint8_t* mac) {
  if (!_is_ESPNOW_Initialized) {
    Serial.println("ESP-NOW not initialized");
    return;
  }

  if (!addPeer(mac)) {return;}

  for (uint8_t i = 0; i < ARRAY_SIZE; i ++) {_data_to_send[i] = _current_state[i];}
  _data_to_send[ARRAY_SIZE]     = static_cast<int16_t>(MOTION_PLATFORM);
  _data_to_send[ARRAY_SIZE + 1] = static_cast<int16_t>(MOTION_REPORT);
  esp_err_t result = esp_now_send(mac, (uint8_t*)&_data_to_send, sizeof(_data_to_send));
  
  if (result == ESP_OK) {/*Serial.println("State sent successfully");*/} 
  else {/*Serial.println("Error sending state");*/}
}

void Platform::sendingData() {
  uint64_t now = millis();
  if (now - _last_time_sending < SENDING_DELAY) {return;} //Time step between sending
  sendStateTo(_receiverMac);
  _last_time_sending = now;
}

void       Platform::newDiectionReport(directions direction) {
  if (_direction) {
    Serial.print("Direction now is: ");
    Serial.println(_direction);
    Serial.print("Adjustment now is: ");
    Serial.println(_adjustment);
  }
  if (direction == UP || direction == DOWN) {
    Serial.println("#########################################");
    Serial.print("Current  vertical angle: ");
    Serial.println(_current_state[1]);
    Serial.print("Desired vertical angle: ");
    Serial.println(_desired_state[1]);
    //return;
  }
  if (direction == LEFT || direction == RIGHT) {
    Serial.println("#########################################");
    Serial.print("Current horizontal angle: ");
    Serial.println(_current_state[0]);
    Serial.print("Desired horizontal angle: ");
    Serial.println(_desired_state[0]);
    return;
  }
}

/*directions Platform::defineDirection() {
  //Defining direction Should be changed when encoders and motors changed//
  //if (_desired_state[0] > _current_state[0]) return RIGHT;
  //if (_desired_state[0] < _current_state[0]) return LEFT;
  //if (_desired_state[1] > _current_state[1]) return UP;
  //if (_desired_state[1] < _current_state[1]) return DOWN;
  if ((_desired_state[0] > _current_state[0]) && (_desired_state[2] != 3)) return RIGHT;
  if ((_desired_state[0] < _current_state[0]) && (_desired_state[2] != 4)) return  LEFT;
  if ((_desired_state[1] > _current_state[1]) && (_desired_state[2] != 1)) return    UP;
  if ((_desired_state[1] < _current_state[1]) && (_desired_state[2] != 2)) return  DOWN;
  if (_desired_state[2] == 4) return RIGHT;
  if (_desired_state[2] == 3) return  LEFT;
  if (_desired_state[2] == 2) return    UP;
  if (_desired_state[2] == 1) return  DOWN;

  return NONE;
}*/

bool      Platform::isDirectionSame() {     //or NONE
  if ((_direction == RIGHT) && A_much_bigger_B(_desired_state[0], _current_state[0], 0)) return true;
  if ((_direction == LEFT ) && A_much_bigger_B(_current_state[0], _desired_state[0], 0)) return true;
  if ((_direction == UP   ) && A_much_bigger_B(_desired_state[1], _current_state[1], 0)) return true;
  if ((_direction == DOWN ) && A_much_bigger_B(_current_state[1], _desired_state[1], 0)) return true;
  return false;
}

adjustment_type      Platform::defineAdjustment() {
  //if (_desired_state[2])    return COARSE;
  if (_direction == RIGHT) {return (_desired_state[0] - _current_state[0] > 3) ? COARSE : FINE;}
  if (_direction ==  LEFT) {return (_current_state[0] - _desired_state[0] > 3) ? COARSE : FINE;}
  if (_direction ==    UP) {return (_desired_state[1] - _current_state[1] > 3) ? COARSE : FINE;}
  if (_direction ==  DOWN) {return (_current_state[1] - _desired_state[1] > 3) ? COARSE : FINE;}
  if (_direction ==  NONE) return NO_ADJ;
  return NO_ADJ;
}


directions Platform::defineDirection() {
  static uint16_t no_changes_counter = 0;
  uint64_t now = millis();
  uint16_t second_chance_timer = 0;
  static bool second_chance_timer_activated = false;
  static uint64_t almost_on_position_time = now;
  static bool almost_on_position_flag = false;
  static uint64_t _chage_start_time = now;
  if (!_desired_state[2]) {
    if ((_direction != NONE) && !this->isDirectionSame()) {
      return NONE;
    }
    if (_direction != NONE) {
      return _direction;
    }

    if (!almost_on_position_flag && _desired_changed && aprox_eq(_desired_state[0], _current_state[0]) && aprox_eq(_desired_state[1], _current_state[1])) {
      almost_on_position_flag = true;
      almost_on_position_time = now;
    }

    if ((almost_on_position_flag) && (!aprox_eq(_desired_state[0], _current_state[0], 3) || !aprox_eq(_desired_state[1], _current_state[1], 3))) {
      //almost_on_position_flag = false;
    }
    if (almost_on_position_flag) {Serial.printf("Almost on position for: %d\n", now - almost_on_position_time);}

    if (almost_on_position_flag && (now - almost_on_position_time > 2000)) {
      almost_on_position_flag = false;
      _desired_changed = false;
      no_changes_counter = 0;
      return NONE;
    }
    if (aprox_eq(_desired_state[0],_current_state[0], 1) && !aprox_eq(_desired_state[1],_current_state[1], 1)) {
      if (A_much_bigger_B(_desired_state[1], _current_state[1], _desired_changed ? 0 : 3)) {/*no_changes_counter = 0*/ return    UP;}
      if (A_much_bigger_B(_current_state[1], _desired_state[1], _desired_changed ? 0 : 3)) {/*no_changes_counter = 0*/ return  DOWN;}
    }
    if (aprox_eq(_desired_state[1],_current_state[1], 1) && !aprox_eq(_desired_state[0],_current_state[0], 1)) {
      if (A_much_bigger_B(_desired_state[0], _current_state[0], _desired_changed ? 0 : 3)) {/*no_changes_counter = 0*/; return RIGHT;}
      if (A_much_bigger_B(_current_state[0], _desired_state[0], _desired_changed ? 0 : 3)) {/*no_changes_counter = 0*/ return  LEFT;}
    }
    if (A_much_bigger_B(_desired_state[0], _current_state[0], _desired_changed ? 0 : 3)) {/*no_changes_counter = 0*/; return RIGHT;}
    if (A_much_bigger_B(_current_state[0], _desired_state[0], _desired_changed ? 0 : 3)) {/*no_changes_counter = 0*/ return  LEFT;}
    if (A_much_bigger_B(_desired_state[1], _current_state[1], _desired_changed ? 0 : 3)) {/*no_changes_counter = 0*/ return    UP;}
    if (A_much_bigger_B(_current_state[1], _desired_state[1], _desired_changed ? 0 : 3)) {/*no_changes_counter = 0*/ return  DOWN;}
    
    if (_desired_changed)  {
      no_changes_counter++;
      //_desired_changed = false;
    }
    if (no_changes_counter > 50) {_desired_changed = false; no_changes_counter = 0; almost_on_position_flag = false;}
    if (!_desired_changed && _second_chance) {second_chance_timer = now; second_chance_timer_activated = true; _second_chance = false;}
    if (second_chance_timer_activated && (now - second_chance_timer > 500)) {second_chance_timer_activated = false; _desired_changed = true;}
    return NONE;    
  }
  if (_desired_state[2] == 4) return RIGHT;
  if (_desired_state[2] == 3) return  LEFT;
  if (_desired_state[2] == 2) return    UP;
  if (_desired_state[2] == 1) return  DOWN;
}

directions Platform::defineSubdirection(directions direction) {
  if ((direction == RIGHT) || (direction == LEFT)) {
    if (aprox_eq(_desired_state[0],  _current_state[0], 3)) return NONE_SUB;
    if (_desired_state[1] > _current_state[1]) return   UP;
    if (_desired_state[1] < _current_state[1]) return DOWN;
  }
  //_desired_changed = false;
  return NONE_SUB;
}

uint8_t Platform::calculatePID(directions direction) {
  uint64_t now = millis();
  static uint64_t last_time = now;
  //last_time = max(_move_start_time, last_time);

  static int16_t error_sum = 0;
  static int16_t last_error = 0;
  static directions last_direction = direction;
  if (last_time < _move_start_time) {
    last_time = _move_start_time;
    error_sum = 0;
    last_error = 0;
  }
  if (last_direction != direction) {
    error_sum = 0;
    last_error = 0;
    last_direction = direction;
  }
  if ((direction == NONE) || (direction == NONE_SUB)) return 0;
  uint8_t ax = ((direction == LEFT) || (direction == RIGHT)) ? 0 : 1;
  int16_t error = abs(_desired_state[ax] - _current_state[ax]);
  if (!error || ((error <= 1) && (last_error > 1))) {
    error_sum = 0;
    last_error = 0;
    last_time = now;
    return 0;
  }
  error_sum += error;
  error_sum = constrain(error_sum, -3000, 3000);
  float deltaT = 5 / 1000.0f;
  float dError = (error - last_error) / deltaT;
  dError = constrain(dError, -300, 300);
  float PID_out   = _Kp * error + _Ki * error_sum + _Kd * dError;
  //if (error < 2) PID_out *= 0.5;
  uint8_t min_pwm = (error <= 2) ? 135 : 140;
  uint8_t max_pwm = (error <= 2) ? 160 : 200;
  uint8_t pwm_value = constrain(min_pwm + uint8_t(PID_out), min_pwm, max_pwm);
  Serial.printf("PID results: error =  %d, error_sum = %d, dError = %.2f, PID_out = %.2f, pwm_value = %d\n", error, error_sum, dError, PID_out, pwm_value);
  last_error = error;
  return pwm_value;
}

void Platform::PIDTaskWrapper(void *param) {
    Platform *instance = static_cast<Platform*>(param);
    instance->PIDTask();
}

void Platform::PIDTask() {
  const TickType_t delay = pdMS_TO_TICKS(5);

  while (true) {
    _new_direction = this->defineDirection();
    _pid_pwm = this->calculatePID(_new_direction);
    vTaskDelay(delay);
  }
}

void Platform::startPIDTask() {
  if (_pidTaskHandle == nullptr) {
    BaseType_t result = xTaskCreatePinnedToCore(Platform::PIDTaskWrapper, "PID Task", 4096, this, 2, &_pidTaskHandle, 1);
    if (result != pdPASS) {
      Serial.println("Failed to create PID Task!");
    }
  }
  Serial.println("PID task already exist");
}

void Platform::movePlatform() {
  //directions new_direction = this->defineDirection();
  if (_stop_Flags[0])                             _new_direction = NONE;
  if (_stop_Flags[1] && (_new_direction == RIGHT)) _new_direction = NONE;
  if (_stop_Flags[2] && (_new_direction ==  LEFT)) _new_direction = NONE;
  if (_stop_Flags[3] && (_new_direction ==    UP)) _new_direction = NONE;
  if (_stop_Flags[4] && (_new_direction ==  DOWN)) _new_direction = NONE;

  uint64_t now = millis();
  if (_new_direction == NONE) _move_start_flag = false;
  if (!_move_start_flag && _new_direction != NONE) {
    _move_start_flag = true;
    _move_start_time =  now;
  }
  if ((_new_direction != _direction) &&  !(_desired_state[2])) {_move_start_time = now;}
  //Seting "speed" by PWM and making start smooth
  _direction = _new_direction;
  _subdirection = this->defineSubdirection(_direction);
  if (_subdirection != NONE_SUB)   {Serial.print("Now subdirection is "); Serial.println(_subdirection);}
  //this->newDiectionReport(_new_direction);
  uint8_t velocity = 0;
  if (_desired_state[2]) velocity = 180;
  else {if (_direction != NONE) velocity = _pid_pwm;}
  ledcWrite(PIN_IN1, ((_direction & _subdirection) >> 3 & 1) * velocity);
  ledcWrite(PIN_IN2, ((_direction & _subdirection) >> 2 & 1) * velocity);
  ledcWrite(PIN_IN3, ((_direction & _subdirection) >> 1 & 1) * velocity);
  ledcWrite(PIN_IN4, ((_direction & _subdirection) >> 0 & 1) * velocity);

}

/*void Platform::movePlatform() {
  //Same, but direction is preset

  directions new_direction = this->defineDirection();
  if (_stop_Flags[0])                             new_direction = NONE;
  if (_stop_Flags[1] && (new_direction == RIGHT)) new_direction = NONE;
  if (_stop_Flags[2] && (new_direction ==  LEFT)) new_direction = NONE;
  if (_stop_Flags[3] && (new_direction ==    UP)) new_direction = NONE;
  if (_stop_Flags[4] && (new_direction ==  DOWN)) new_direction = NONE;
  uint64_t now = millis();
  if (new_direction == NONE) _move_start_flag = false;
  if (!_move_start_flag && new_direction != NONE) {
    _move_start_flag = true;
    _move_start_time =  now;
  }
  if ((new_direction != _direction) &&  !(_desired_state[2])) {_move_start_time = now;}
  //Seting "speed" by PWM and making start smooth
  _direction = new_direction;
  _subdirection = this->defineSubdirection(_direction);
  if (_subdirection != NONE_SUB)   {Serial.print("Now subdirection is "); Serial.println(_subdirection);}

  _adjustment = this->defineAdjustment();
  this->newDiectionReport(new_direction);
  uint8_t min_speed = 130;
  uint8_t max_speed = 180;
  uint8_t time_step = _adjustment == COARSE ? 4 : 10;
  if (_desired_state[2]) time_step = 1;
  uint16_t velocity = constrain(int((now - _move_start_time) / time_step), min_speed, max_speed);
  velocity = _charge_flag ? 255 : velocity;
  ledcWrite(PIN_IN1, ((_direction & _subdirection) >> 3 & 1) * velocity);
  ledcWrite(PIN_IN2, ((_direction & _subdirection) >> 2 & 1) * velocity);
  ledcWrite(PIN_IN3, ((_direction & _subdirection) >> 1 & 1) * velocity);
  ledcWrite(PIN_IN4, ((_direction & _subdirection) >> 0 & 1) * velocity);

  //ledcWrite(PIN_IN1, (_direction >> 3 & 1) * velocity - (40 * (_subdirection >> 3 & 1)));
  //ledcWrite(PIN_IN2, (_direction >> 2 & 1) * velocity - (40 * (_subdirection >> 2 & 1)));
  //ledcWrite(PIN_IN3, (_direction >> 1 & 1) * velocity - (40 * (_subdirection >> 1 & 1)));
  //ledcWrite(PIN_IN4, (_direction >> 0 & 1) * velocity - (40 * (_subdirection >> 0 & 1)));
  //if (_direction) Serial.println("Moing in direction: " + String(_direction));
}*/

void Platform::readSensors() {
  //Serial.println("Reading sensors");
  static bool first_measure = true;
  this->readMT6701Encoder();
  this->readAS5600Encoder();
  _current_state[2] = _desired_state[2]; //Not necessary and not sensor as well. But important if controller is checking starting position for beginning of his work
  if (first_measure) {
    for (uint8_t i = 0; i < 3; i++) {
      _desired_state[i] = _current_state[i];
    }
    first_measure = false;
  }
}

int16_t Platform::horizontal_angle_rescale(int16_t angle) {
  angle = angle - _horizontal_calibration + 180;
  if (angle < 0)   angle += 360;
  if (angle > 360) angle -= 360;
  return map(angle, 0, 360, 180, -180);
}

void Platform::readMT6701Encoder() {
  static FastMedianFilter medianFilter_H;

  //_mt6701.updateCount();

  int16_t mt6701_Encodervalue = custom_round(_mt6701_data, _desired_state[0]);
  int16_t filtered_val = medianFilter_H.addValue(mt6701_Encodervalue);

  _current_state[0] = this->horizontal_angle_rescale(filtered_val);

}

float Platform::readAS5600Angle() {
  uint32_t now = millis();
  while (millis() - now < 20) {
    uint16_t raw = _as5600.readAngle();
    if (raw != 0xFFFF) {
      return raw * 360.0f / 4096.0f;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  Serial.println("AS5600 read timeout!");
  return NAN;
}

void Platform::encoderTask (void *pvParameters) {
  Platform *self = static_cast<Platform *>(pvParameters);
  esp_task_wdt_add(NULL);

  while(true) {
    esp_task_wdt_reset();
    if (xSemaphoreTake(self->_i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      self->_as5600_data = self->readAS5600Angle();
      self->_mt6701_data = self->_mt6701.readAngle();
      if (self->_mt6701.getLastError() != 0) {
        Serial.printf("MT6701 I2C error: %d\n", self->_mt6701.getLastError());
      }
      xSemaphoreGive(self->_i2cMutex);
    }
    /*if (!isnan(self->_as5600_data) && !isnan(self->_mt6701_data)) {
      Serial.printf("AS5600: %.2f°, MT6701: %.2f°\n", self->_as5600_data, self->_mt6701_data)
    }*/
    vTaskDelay(pdMS_TO_TICKS(50));
  } 
}

void Platform::readAS5600Encoder() {
  static FastMedianFilter medianFilter_V;
  //static ExpSmoother smooter_V(0.3);
  //int16_t v_angle = static_cast<int16_t>(_as5600.readAngle() * AS5600_RAW_TO_DEGREES - 46.0);
  int16_t v_angle = custom_round(_as5600_data - (float)_vertical_calibration, _desired_state[1]);
  int16_t filtered_val = medianFilter_V.addValue(v_angle);
  //filtered_val = smooter_V.addValue(filtered_val);
  _current_state[1] = filtered_val;
}

void Platform::set_vertical_calibration() {
  _vertical_calibration += _current_state[1];
  EEPROM.put(0, _vertical_calibration);
  EEPROM.commit();
  Serial.println("Vertical calibration has been set");
}

void Platform::set_horizontal_calibration() {
  //_horizontal_calibration = _current_state[0];
  _horizontal_calibration = _mt6701.readAngle();
  _current_state[0] = 0;
  _desired_state[0] = 0;
  EEPROM.put(2, _horizontal_calibration);
  if (EEPROM.commit()) {Serial.println("EEPROM data has been written");}
  Serial.printf("Horizontal calibration has been set to %d. Current state is: %d\n", _horizontal_calibration, _current_state[0]);
}

void Platform::set_new_receiverMac(const esp_now_recv_info_t *info) {
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

void Platform::saveSensorsMac(const uint8_t mac[6]) { 
  memcpy(_sensorsMac, mac, 6); 
  uint16_t address = 10; 
  for (uint8_t i = 0; i < 6; i++) { 
    EEPROM.put(address + i, _sensorsMac[i]); 
  }  
  if (EEPROM.commit()) {Serial.println("Sensors MAC has been written");}
}

void Platform::saveCameraMac(const uint8_t mac[6]) { 
  memcpy(_cameraMac, mac, 6); 
  uint16_t address = 10; 
  for (uint8_t i = 0; i < 6; i++) { 
    EEPROM.put(address + i, _cameraMac[i]); 
  }  
  if (EEPROM.commit()) {Serial.println("Camera MAC has been written");}
}

//Defining Stop flag. Absolute stop [0], Right [1] and Left [2] will be added soon
void Platform::defineStopFlags() {
  _stop_Flags[1] = (_current_state[0] >  170);
  _stop_Flags[1] = (_current_state[0] < -170);
  _stop_Flags[3] = (_current_state[1] >   30);
  _stop_Flags[4] = (_current_state[1] <  -30);
}

//Tick ;)
void Platform::tick() {
  this->readSensors();
  this->defineStopFlags();
  this->CheckingAP();
  this->ReadingUDP();
  this->sendingData();
  this->movePlatform();
}

bool    Platform::checkCotrollerMAC(const esp_now_recv_info_t *info) {
  for (uint8_t i = 0; i < 6; i ++) {
    if (info->src_addr[i] != _receiverMac[i]) return false;
  }
  return true;
}

void    Platform::CheckingAP() {
  uint64_t now = millis();
  if (now - _last_ESPNOW > 5000) {
    if (!_isAP) {
      WiFi.disconnect();
      WiFi.softAP(_ssid);
      _udp.begin(_udpPort);
      _isAP = true;
      Serial.println("System ready!");
      Serial.println(WiFi.softAPIP());
    }
  } else {
    if (_isAP) {
      WiFi.softAPdisconnect(true);
      _isAP = false;
    }
  }
}

void    Platform::ReadingUDP() {
  uint64_t now = millis();
  if (!_isAP) return;
  int16_t packetSize = _udp.parsePacket();
  if (packetSize) {
    IPAddress _remoteIP        = _udp.remoteIP();
    uint16_t  _remotePort      = _udp.remotePort();
    bool found = false;
    for (uint8_t i = 0; i < _udp_user_counter; i++) {
      if (_users[i]._UserIP == _remoteIP) {
        found = true;
        _users[i]._Last_Packet_Time = now;
        Serial.printf("Packet from user number %d\n", i);
      }
    }
    if (!found && _udp_user_counter < 10) {
      Serial.println("New user appeas");
      _users[_udp_user_counter]._UserIP           = _remoteIP;
      _users[_udp_user_counter]._Last_Packet_Time = now;
      _udp_user_counter++;
    }
    Serial.print("Remote IP addres: ");
    Serial.println(_remoteIP);
    int16_t len = _udp.read(_packetBuffer, sizeof(_packetBuffer) - 1);
    if (len > 0) {
      _packetBuffer[len] = 0;
      String command = String(_packetBuffer);
      if (command == "HOR+")      {_desired_state[0] = min(_desired_state[0] + 1,  170); _desired_changed = true;}
      else if (command == "HOR-") {_desired_state[0] = max(_desired_state[0] - 1, -170); _desired_changed = true;}
      else if (command == "VER+") {_desired_state[1] = min(_desired_state[1] + 1,   30); _desired_changed = true;}
      else if (command == "VER-") {_desired_state[1] = max(_desired_state[1] - 1,  -30); _desired_changed = true;}
    }
    this->sendCurrentValues(_remoteIP);
  }

  static uint64_t lastSendTime = 0;
  if (now - lastSendTime > 100) {
    for (uint8_t i = 0; i < _udp_user_counter; i++) {
      this->sendCurrentValues(_users[i]._UserIP);
    }
    lastSendTime = millis();
  }
}

void    Platform::sendCurrentValues(IPAddress IP) {
    String response = String(_current_state[0]) + "," + String(_desired_state[0]) + "," + String(_current_state[1]) + "," + String(_desired_state[1]);
    _udp.beginPacket(IP, _udpPort);
    _udp.print(response);
    _udp.endPacket();
    Serial.println("Sent: " + response);
}