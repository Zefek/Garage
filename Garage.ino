#include <EspDrv.h>
#include <MQTTClient.h>
#include <SoftwareSerial.h>
#include "config.h"
#include <avr/wdt.h>

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length);

unsigned long doorChangeTime = 0;
int doorState = LOW;
int moveState = HIGH;
bool doorSignal = false;
SoftwareSerial serial(4, 5);
EspDrv espDrv(&serial);
MQTTClient mqttClient(&espDrv, MQTTMessageReceive);
MQTTConnectData mqttConnectData = { MQTTHost, 1883, "Garage", MQTTUsername, MQTTPassword, "", 0, false, "", false, 0x0 }; 
unsigned long mqttConnectionTimeout = 0;
unsigned long mqttLastConnectionTry = 0;
unsigned long currentMillis = 0;

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length)
{
  if(strcmp(topic, GARAGE_CMD) == 0)
  {
    doorSignal = true;
  }
}

void PublishDoorState(int state, int movement)
{
  //state - HIGH (door closed), LOW (door open) 
  //movement - HIGH (door stopped), LOW (door moving)
  if (state == HIGH && movement == HIGH)
  {
    mqttClient.Publish(GARAGE_STATE, "Closed;Stop", true);
  }
  if (state == LOW && movement == HIGH)
  {
    mqttClient.Publish(GARAGE_STATE, "Open;Stop", true);
  }
  if (state == HIGH && movement == LOW)
  {
    mqttClient.Publish(GARAGE_STATE, "Closed;Move", true);
  }
  if (state == LOW && movement == LOW)
  {
    mqttClient.Publish(GARAGE_STATE, "Open;Move", true);
  }
}

void Connect()
{
  if(currentMillis - mqttLastConnectionTry < mqttConnectionTimeout)
  {
    return;
  }
  int wifiStatus = espDrv.GetConnectionStatus();
  bool wifiConnected = wifiStatus == WL_CONNECTED;
  if(wifiStatus == WL_DISCONNECTED || wifiStatus == WL_IDLE_STATUS)
  {
    wifiConnected = espDrv.Connect(WifiSSID, WifiPassword);
  }
  if(wifiConnected)
  {
    bool isConnected = mqttClient.IsConnected();
    if(!isConnected)
    {
      if(mqttClient.Connect(mqttConnectData))
      {
        PublishDoorState(doorState, moveState);
        mqttClient.Subscribe(GARAGE_CMD);

        mqttLastConnectionTry = currentMillis;
        mqttConnectionTimeout = 0;

      }
      else
      {
        mqttLastConnectionTry = currentMillis;
        mqttConnectionTimeout = min(mqttConnectionTimeout * 2 + random(0, 5000), 300000);
      }
    }
  }
  else
  {
    mqttLastConnectionTry = currentMillis;
    mqttConnectionTimeout = min(mqttConnectionTimeout * 2 + random(0, 5000), 300000);
  }
}

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  pinMode(4, INPUT_PULLUP);
  digitalWrite(3, HIGH);
  Serial.begin(57600);
  serial.begin(57600);
  espDrv.Init(32);
  espDrv.Connect(WifiSSID, WifiPassword);
  doorState = digitalRead(2);
  moveState = digitalRead(4);
  wdt_enable(WDTO_8S);
  Serial.println("Setup OK");
}

void loop() {
  wdt_reset();
  currentMillis = millis();
  if(!mqttClient.Loop())
  {
    Connect();
  }
  if(currentMillis - doorChangeTime > 1000)
  {
    int doorValue = digitalRead(2);
    int moveValue = digitalRead(4);
    if(doorValue != doorState || moveValue != moveState)
    {
      doorState = doorValue;
      moveState = moveValue;
      PublishDoorState(doorState, moveState);
    }
    doorChangeTime = currentMillis;
  }
  if(doorSignal)
  {
    digitalWrite(3, LOW);
    delay(500);
    digitalWrite(3, HIGH);
    doorSignal = false;
  }
}
