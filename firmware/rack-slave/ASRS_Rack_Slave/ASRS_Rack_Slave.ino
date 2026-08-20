#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <ASRS_RackProtocol.h>

#if !defined(ESP32)
#error "ASRS_Rack_Slave requires an ESP32."
#endif

constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint8_t SENSOR_PINS[4] = {25, 26, 27, 32};
constexpr uint8_t OCCUPIED_LEVEL = LOW;
constexpr uint32_t SENSOR_INTERVAL_MS = 100;
constexpr uint32_t REPORT_INTERVAL_MS = 1000;
constexpr uint8_t FILTER_SAMPLES = 5;
const uint8_t BROADCAST_MAC[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

bool occupied[4] = {false,false,false,false};
volatile bool statusRequested = false;
bool espNowReady = false;
uint16_t sequence = 0;
uint32_t lastSensorMs = 0;
uint32_t lastReportMs = 0;

bool filteredRead(uint8_t pin) {
  uint8_t detected=0;
  for(uint8_t i=0;i<FILTER_SAMPLES;i++){if(digitalRead(pin)==OCCUPIED_LEVEL)detected++;delay(2);}
  return detected >= (FILTER_SAMPLES/2+1);
}

void updateSensors(){for(uint8_t i=0;i<4;i++)occupied[i]=filteredRead(SENSOR_PINS[i]);}

void sendStatusNow(){
  if(!espNowReady)return; ASRS_RackPacket p={};p.magic=ASRS_RACK_MAGIC;p.version=ASRS_RACK_VERSION;
  p.type=ASRS_RACK_STATUS;p.sequence=++sequence;for(uint8_t i=0;i<4;i++)p.slots[i]=occupied[i]?1:0;p.uptimeMs=millis();
  esp_now_send(BROADCAST_MAC,reinterpret_cast<uint8_t*>(&p),sizeof(p));
}

void processPacket(const uint8_t *data,int length){
  if(!asrsIsRackPacket(data,length))return;ASRS_RackPacket p;memcpy(&p,data,sizeof(p));
  if(p.type==ASRS_RACK_STATUS_REQUEST)statusRequested=true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void receiveCallback(const esp_now_recv_info_t*,const uint8_t*data,int length){processPacket(data,length);}
#else
void receiveCallback(const uint8_t*,const uint8_t*data,int length){processPacket(data,length);}
#endif

bool beginEspNow(){
  if(esp_now_init()!=ESP_OK)return false;esp_now_register_recv_cb(receiveCallback);
  esp_now_peer_info_t peer={};memcpy(peer.peer_addr,BROADCAST_MAC,6);peer.channel=ESPNOW_CHANNEL;peer.encrypt=false;
  return esp_now_is_peer_exist(BROADCAST_MAC)||esp_now_add_peer(&peer)==ESP_OK;
}

void setup(){
  Serial.begin(115200);for(uint8_t i=0;i<4;i++)pinMode(SENSOR_PINS[i],INPUT);updateSensors();
  WiFi.mode(WIFI_STA);WiFi.disconnect(false,true);WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL,WIFI_SECOND_CHAN_NONE);espNowReady=beginEspNow();
  Serial.println("Rack sensor node started without Wi-Fi AP or web server.");
  Serial.println(espNowReady?"ESP-NOW ready":"ESP-NOW failed");
}

void loop(){
  uint32_t now=millis();
  if(now-lastSensorMs>=SENSOR_INTERVAL_MS){lastSensorMs=now;updateSensors();}
  if(now-lastReportMs>=REPORT_INTERVAL_MS){lastReportMs=now;sendStatusNow();}
  if(statusRequested){statusRequested=false;sendStatusNow();}
}
