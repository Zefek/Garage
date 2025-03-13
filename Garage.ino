#include <EspDrv.h>
#include <MQTTClient.h>
#include <SoftwareSerial.h>
#include "config.h"
#include <avr/wdt.h>

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length);

unsigned long doorChangeTime = 0;
int doorState = LOW;
bool doorSignal = false;
SoftwareSerial serial(4, 5);
EspDrv espDrv(&serial);
MQTTClient mqttClient(&espDrv, MQTTMessageReceive);
MQTTConnectData mqttConnectData = { MQTTHost, 1883, "Garage", MQTTUsername, MQTTPassword, "", 0, false, "", false, 0x0 }; 

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length)
{
  if(strcmp(topic, GARAGE_CMD) == 0)
  {
    doorSignal = true;
  }
}

void Connect()
{
  uint8_t wifiStatus = espDrv.GetConnectionStatus();
  bool wifiConnected = wifiStatus == WL_CONNECTED;
  if(wifiStatus == WL_DISCONNECTED || wifiStatus == WL_IDLE_STATUS)
  {
    wifiConnected = espDrv.Connect(WifiSSID, WifiPassword);
  }
  if(wifiConnected)
  {
    uint8_t clientStatus = espDrv.GetClientStatus();
    if(clientStatus == CL_DISCONNECTED)
    {
      if(mqttClient.Connect(mqttConnectData))
      {
        mqttClient.Publish(GARAGE_STATE, doorState == HIGH? "Closed" : "Open", true);
        mqttClient.Subscribe(GARAGE_CMD);
      }
    }
  }
}

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  Serial.begin(115200);
  serial.begin(115200);
  espDrv.Init(32);
  espDrv.Connect(WifiSSID, WifiPassword);
  wdt_enable(WDTO_8S);
  Serial.println("Setup OK");
}

void loop() {
  wdt_reset();
  unsigned long time = millis();
  if(!mqttClient.Loop())
  {
    Connect();
  }
  if(time - doorChangeTime > 1000)
  {
    int doorValue = digitalRead(2);
    if(doorValue != doorState)
    {
      doorState = doorValue;
      mqttClient.Publish(GARAGE_STATE, doorState == HIGH? "Closed" : "Open", true);
    }
    doorChangeTime = time;
  }
  if(doorSignal)
  {
    digitalWrite(3, LOW);
    delay(500);
    digitalWrite(3, HIGH);
    doorSignal = false;
  }
}
