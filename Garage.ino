#include <EspDrv.h>
#include <MQTTClient.h>
#include <SoftwareSerial.h>
#include "config.h"
#include "secret.h"
#include <avr/wdt.h>
#include <AM2302-Sensor.h>

#define DOORSWITCH_PIN 2
#define DOORBUTTON_PIN 3
#define RX_PIN 4
#define TX_PIN 5
#define DOORFLASH_PIN 6
#define TEMPERATURE_SENSOR_PIN 7

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length);
void OnBusy(uint8_t count);
void DataTimeout();

#pragma pack(push, 1)
struct DiagData {
  uint32_t uptime;
  uint16_t freeRam;
  uint16_t wifiReconn;
  uint16_t mqttReconn;
  uint8_t  sensorErr;
  uint8_t  resetReason;
  uint16_t loopMaxMs;
  uint16_t doorCycles;
  int8_t   rssi;
};
#pragma pack(pop)

unsigned long doorChangeTime = 0;
int doorState = LOW;
int moveState = HIGH;
bool doorSignal = false;
SoftwareSerial serial(RX_PIN, TX_PIN);
EspDrv espDrv(&serial);
MQTTClient mqttClient(&espDrv, MQTTMessageReceive);
MQTTConnectData mqttConnectData = { MQTTHost, 1883, "Garage", MQTTUsername, MQTTPassword, "", 0, false, "", false, 0x0 };
unsigned long mqttConnectionTimeout = 0;
unsigned long mqttLastConnectionTry = 0;
unsigned long currentMillis = 0;
unsigned long temperatureHumidityReadMillis = 0;
AM2302::AM2302_Sensor am2302{ TEMPERATURE_SENSOR_PIN };
char temperatureData[10];
bool doorMoveDetected = false;
bool closeRequired = false;
DiagData currentDiagData = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long lastDiagSendMillis = 0;

extern int __heap_start, *__brkval;
int freeRam() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length)
{
  if(strcmp(topic, GARAGE_CMD) == 0)
  {
    doorSignal = true;
  }
}

void DataTimeout()
{
  closeRequired = true;
}

void OnBusy(uint8_t count)
{
  if(count > 10)
  {
    closeRequired = true;
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
    if(currentDiagData.wifiReconn < 0xFFFF)
    {
      currentDiagData.wifiReconn++;
    }
    wifiConnected = espDrv.Connect(WifiSSID, WifiPassword);
  }
  if(wifiConnected)
  {
    bool isConnected = mqttClient.IsConnected();
    if(!isConnected)
    {
      if(currentDiagData.mqttReconn < 0xFFFF)
      {
        currentDiagData.mqttReconn++;
      }
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

void sendDiag()
{
  currentDiagData.uptime = currentMillis / 60000;
  currentDiagData.freeRam = freeRam();
  currentDiagData.rssi = espDrv.GetRssi();
  uint8_t buffer[sizeof(DiagData)];
  memcpy(buffer, &currentDiagData, sizeof(DiagData));
  mqttClient.Publish(GARAGE_DIAG, buffer, sizeof(DiagData), false);
  currentDiagData.loopMaxMs = 0;
  currentDiagData.sensorErr = 0;
}

void setup() {
  currentDiagData.resetReason = MCUSR;
  MCUSR = 0;
  pinMode(DOORSWITCH_PIN, INPUT_PULLUP);
  pinMode(DOORBUTTON_PIN, OUTPUT);
  pinMode(DOORFLASH_PIN, INPUT_PULLUP);
  pinMode(TEMPERATURE_SENSOR_PIN, INPUT);
  digitalWrite(DOORBUTTON_PIN, HIGH);
  Serial.begin(57600);
  serial.begin(57600);
  espDrv.Init(32);
  espDrv.OnBusy = OnBusy;
  espDrv.DataTimeout = DataTimeout;
  espDrv.Connect(WifiSSID, WifiPassword);
  doorState = digitalRead(DOORSWITCH_PIN);
  if (am2302.begin()) 
  {
    delay(3000);
  }
  wdt_enable(WDTO_8S);
  Serial.println("Setup OK");
}

void loop() {
  wdt_reset();
  currentMillis = millis();
  if(closeRequired)
  {
    espDrv.Close();
    closeRequired = false;
  }
  if(!mqttClient.Loop())
  {
    Connect();
  }
  if(digitalRead(DOORFLASH_PIN) == LOW)
  {
    doorMoveDetected = true;
  }
  if(currentMillis - doorChangeTime > 1000)
  {
    int doorValue = digitalRead(DOORSWITCH_PIN);
    if(doorValue != doorState || (doorMoveDetected && moveState == HIGH) || (!doorMoveDetected && moveState == LOW))
    {
      if(doorState == HIGH && doorValue == LOW && currentDiagData.doorCycles < 0xFFFF)
      {
        currentDiagData.doorCycles++;
      }
      doorState = doorValue;
      moveState = doorMoveDetected? LOW : HIGH;
      PublishDoorState(doorState, moveState);
    }
    doorChangeTime = currentMillis;
    doorMoveDetected = false;
  }
  if(doorSignal)
  {
    digitalWrite(DOORBUTTON_PIN, LOW);
    delay(500);
    digitalWrite(DOORBUTTON_PIN, HIGH);
    doorSignal = false;
  }
  if(currentMillis - temperatureHumidityReadMillis > 60000)
  {
    uint8_t status = am2302.read();
    if(status != AM2302::AM2302_READ_OK)
    {
      currentDiagData.sensorErr |= 0x01;
    }
    int temperature = (int)(am2302.get_Temperature() * 10);
    int humidity = (int)am2302.get_Humidity();
    sprintf(temperatureData, "%d;%d", temperature, humidity);
    mqttClient.Publish(GARAGE_TEMPERATURE, temperatureData);
    temperatureHumidityReadMillis = currentMillis;
  }
  if(currentMillis - lastDiagSendMillis > 900000UL)
  {
    sendDiag();
    lastDiagSendMillis = currentMillis;
  }
  uint16_t loopDuration = (uint16_t)(millis() - currentMillis);
  if(loopDuration > currentDiagData.loopMaxMs)
  {
    currentDiagData.loopMaxMs = loopDuration;
  }
}
