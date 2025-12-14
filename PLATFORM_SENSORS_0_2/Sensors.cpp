#include <memory>
#include "Sensors.h"
#include <Arduino.h>

senders intToSenders(int16_t value) {
  switch (value) {
    case  0 : return CONTROLLER;
    case  1 : return MOTION_PLATFORM;
    case  2 : return SENSOR_PLATFORM;
    default : return WHO_THE_HELL_IS_THIS;
  }
}

messages intToMessages (int16_t value) {
  switch (value) {
    case  0 : return DESIRED;
    case  1 : return MOTION_REPORT;
    case  2 : return SENSOR_REPORT;
    case  3 : return NEW_CONTROLLER;
    default : return WHAT_THE_HELL_DO_YOU_WANT;
  }
}


Sensors* Sensors::_instance = nullptr;

Sensors::Sensors() : I2C1(0)
{
  _instance = this; //static pointer now points on current object
}

bool Sensors::initiation_EEPROM() {
  /*
  | 0 - 1 : ver cal | 2 - 3 : hor cal | 4 - 9 : MAC CONTROLLER | | 10 - 15 : MAC SENSORS | | 16 - 21 : MAC CAMERA |
  */
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Error EEPROM");
    return false;
  }
  if (EEPROM.read(INIT_ADR) != INIT_KEY) {
    uint16_t address_Controller = 4;
    EEPROM.put(INIT_ADR, INIT_KEY);
    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.put(address_Controller, _receiverMac[i]);
      address_Controller += sizeof(int8_t);
    }
    if (EEPROM.commit()) {Serial.println("EEPROM data has been written"); return true;}
    else {Serial.println("Some EEPROM errors"); return false;}
  }
  else {
    uint16_t address_Controller = 4;
    for (uint8_t i = 0; i < 6; i++) {
      EEPROM.get(address_Controller, _receiverMac[i]);
      address_Controller += sizeof(int8_t);
    }
    Serial.printf("Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  _receiverMac[0], _receiverMac[1], _receiverMac[2],
                  _receiverMac[3], _receiverMac[4], _receiverMac[5]);
  }
  return true;
}

void    Sensors::begin() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
  //  Serial.println("Error initialization ESP-NOW");
    _is_ESPNOW_Initialized = false;
    return;
  }

  esp_now_register_send_cb(defaultOnDataSent); // Here is non-static method just because it never use this-poiner (doing basicaly nothing), so C compiler optimize it to simple function
  esp_now_register_recv_cb(staticOnDataRecv); 
  _is_ESPNOW_Initialized  = true;
  delay(2000);

  I2C1.begin(21, 22);
  delay(100);
  for (uint8_t i = 0; i < 2; i++) {
    _ina3221[i] = std::make_unique<Adafruit_INA3221>();
    if (_ina3221[i]->begin(_inaAddress[i], &I2C1)) {
      _ina3221[i]->setAveragingMode(INA3221_AVG_16_SAMPLES);
      for (uint8_t j = 0; j < 3; j++) _ina3221[i]->setShuntResistance(j, 0.1);
      Serial.print("INA sensor ");
      Serial.print(_inaAddress[i]);
      Serial.println(" started");
    }
    delay(100);
  }
  //if (_aht.begin(&I2C1))                  Serial.println("ATH started");
  if (_bme.begin(0x76, &I2C1))            Serial.println("BME started");
  delay(100);
  //if (_lps.begin_I2C(0x5D, &I2C1, 12345)) {Serial.println("LPS started");}
  //else {Serial.println("Something Wrong with LPS");}
    //-----------------EEPROM INITIALIZATION--------------//
  this->initiation_EEPROM();
}

void Sensors::staticOnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (_instance) {_instance->handleDataRecv(info, data, len);}  //just calling non-static method throught pointer
}

void Sensors::handleDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len < sizeof(_data_received)) {
    Serial.println("Invalid data received");
    return;
  }
  memcpy(_data_received, data, sizeof(_data_received));
  /*Serial.printf("Data received from %02X:%02X:%02X:%02X:%02X:%02X: %d, %d\n",
               info->src_addr[0], info->src_addr[1], info->src_addr[2],
               info->src_addr[3], info->src_addr[4], info->src_addr[5],
               _data_received[0], _data_received[1]);*/
  senders  sender  =  intToSenders(_data_received[ARRAY_SIZE]);
  messages message = intToMessages(_data_received[ARRAY_SIZE + 1]);

  if (sender == CONTROLLER && message == NEW_CONTROLLER) { 
    uint16_t address = 4;
    for (uint8_t i = 0; i < 6; i++) { _receiverMac[i] = info->src_addr[i];
    EEPROM.put(address, _receiverMac[i]); 
    address += sizeof(uint8_t); 
    } 
    if (EEPROM.commit()) { 
    Serial.printf("Controller MAC saved: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  _receiverMac[0], _receiverMac[1], _receiverMac[2], 
                  _receiverMac[3], _receiverMac[4], _receiverMac[5]); 
    } else {Serial.println("EEPROM new controller error");} 
    return; 
  }
}

void Sensors::defaultOnDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  //DOING NOTHING. May add some Serial prints. This works when ESP sends packets
  //Serial.printf("Data has been sent to %02X:%02X:%02X:%02X:%02X:%02X | Status: %s\n", 
  //mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
  //status == ESP_NOW_SEND_SUCCESS ? "Success" : "Error");
}

bool Sensors::addPeer(const uint8_t* mac) {
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

void Sensors::sendStateTo(const uint8_t* mac) {
  if (!_is_ESPNOW_Initialized) {
    Serial.println("ESP-NOW not initialized");
    return;
  }

  if (!addPeer(mac)) {return;}

  for (uint8_t i = 0; i < ARRAY_SIZE; i ++) {_data_to_send[i] = _current_state[i];}
  _data_to_send[ARRAY_SIZE]     = static_cast<int16_t>(SENSOR_PLATFORM);
  _data_to_send[ARRAY_SIZE + 1] = static_cast<int16_t>(SENSOR_REPORT);
  esp_err_t result = esp_now_send(mac, (uint8_t*)&_data_to_send, sizeof(_data_to_send));
  
  if (result == ESP_OK) {/*Serial.println("State sent successfully");*/} 
  else {/*Serial.println("Error sending state");*/}
}

void Sensors::sendingData() {
  uint64_t now = millis();
  if (now - _last_time_sending < SENDING_DELAY) {return;} //Time step between sending
  sendStateTo(_receiverMac);
  _last_time_sending = now;
}

void Sensors::readSensors() {
  for (uint8_t i = 0; i < 4; i++) {
    _current[i] = _ina3221[i / 2]->getCurrentAmps(i % 2) * 1000;
    _voltage[i] = _ina3221[i / 2]->getBusVoltage(i % 2);
  }
  //sensors_event_t humidity, temp, pressure;
  //if (_aht.getEvent(&humidity, &temp)) {
  _humidity    = _bme.readHumidity();
  _temperature = _bme.readTemperature();
  //}
  //if (_lps.getEvent(&pressure, &temp)) {
  //  Serial.println("Data has been read");
  _pressure = _bme.readPressure() / 100.0F;
  //  }
  Serial.printf("Temperature is %.2f\thumidity is %.2f\tpreasure is %.2f\n", _temperature, _humidity, _pressure);
}

void Sensors::formatSendingData() {
  for (uint8_t i = 0; i < 4; i ++) {
    _current_state[3 + 2 * i] = static_cast<int16_t>(_current[i - 3] * 100);
    _current_state[4 + 2 * i] = static_cast<int16_t>(_voltage[i - 3] * 100);
  }
  _current_state[11] = static_cast<int16_t>(_temperature * 100);
  _current_state[12] = static_cast<int16_t>(_humidity    * 100);
  _current_state[13] = static_cast<int16_t>(_pressure         );
}

void Sensors::tick() {
  this->readSensors();
  this->formatSendingData();
  this->sendingData();
}

void Sensors::set_new_receiverMac(const esp_now_recv_info_t *info) {
  for (uint8_t i = 0; i < 6; i++) {
    _receiverMac[i] = info->src_addr[i];
  }
  uint16_t address_Sensors = 10;
  for (uint8_t i = 0; i < 6; i++) {
    EEPROM.put(address_Sensors, _receiverMac[i]);
    address_Sensors += sizeof(int8_t);
  }
  if (EEPROM.commit()) {Serial.println("EEPROM data has been written");}
}
