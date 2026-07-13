#include <EspDrv.h>
#include <MQTTClient.h>
#include <SoftwareSerial.h>
#include "config.h"
#include "secret.h"
#include "sha256.h"
#include <avr/wdt.h>
#include <AM2302-Sensor.h>
#include <EEPROM.h>

#ifndef FW_VERSION
#define FW_VERSION 0
#endif
#define SENSOR_ID_ADDR 0
#define SENSOR_CHANNEL 4

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
char temperatureData[20];
uint16_t sensorId = 0;
bool doorMoveDetected = false;
bool closeRequired = false;
DiagData currentDiagData = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long lastDiagSendMillis = 0;

extern int __heap_start, *__brkval;
int freeRam() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

#define GARAGE_STATUS_OPENED 1
#define GARAGE_STATUS_EXPIRED 2
#define GARAGE_STATUS_BADSIG 3

uint8_t signingKey[GARAGE_KEY_LEN];
struct OpenSlot {
  uint8_t nonce[GARAGE_NONCE_LEN];
  uint32_t issuedAt;
  uint32_t correlationId;
  bool valid;
};
OpenSlot openSlot = { { 0 }, 0, 0, false };
bool challengePending = false;
bool responsePending = false;
uint8_t reqBuf[4];
uint8_t respBuf[4 + GARAGE_NONCE_LEN + GARAGE_SIG_LEN];

static uint32_t readLE32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void writeLE32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint8_t hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return 0;
}

static void parseSigningKey() {
  const char* h = SigningKeyHex;
  for (uint8_t i = 0; i < GARAGE_KEY_LEN; i++) {
    signingKey[i] = (hexNibble(h[i * 2]) << 4) | hexNibble(h[i * 2 + 1]);
  }
}

static void generateNonce(uint8_t* out) {
  for (uint8_t i = 0; i < GARAGE_NONCE_LEN; i++) {
    out[i] = (uint8_t)(random(256) ^ (analogRead(A0) & 0xFF) ^ (micros() >> (i & 7)));
  }
}

static bool cryptoSelfTest() {
  uint8_t mac[32];
  hmac_sha256((const uint8_t*)"Jefe", 4, (const uint8_t*)"what do ya want for nothing?", 28, mac);
  const uint8_t expected[8] = { 0x5b, 0xdc, 0xc1, 0x46, 0xbf, 0x60, 0x75, 0x4e };
  for (uint8_t i = 0; i < 8; i++) {
    if (mac[i] != expected[i]) return false;
  }
  return true;
}

static void publishChallenge(uint32_t r) {
  uint8_t buf[4 + GARAGE_NONCE_LEN];
  writeLE32(buf, r);
  memcpy(buf + 4, openSlot.nonce, GARAGE_NONCE_LEN);
  mqttClient.Publish(GARAGE_OPEN_CHALLENGE, buf, sizeof(buf), false);
}

static void publishResult(uint32_t r, uint8_t status) {
  uint8_t buf[5];
  writeLE32(buf, r);
  buf[4] = status;
  mqttClient.Publish(GARAGE_OPEN_RESULT, buf, sizeof(buf), false);
}

static void processHandshake() {
  if (challengePending) {
    challengePending = false;
    uint32_t r = readLE32(reqBuf);
    generateNonce(openSlot.nonce);
    openSlot.issuedAt = currentMillis;
    openSlot.correlationId = r;
    openSlot.valid = true;
    publishChallenge(r);
  }
  if (responsePending) {
    responsePending = false;
    uint32_t r = readLE32(respBuf);
    uint8_t status = GARAGE_STATUS_EXPIRED;
    if (openSlot.valid
        && (currentMillis - openSlot.issuedAt) <= GARAGE_OPEN_TTL
        && r == openSlot.correlationId
        && memcmp(respBuf + 4, openSlot.nonce, GARAGE_NONCE_LEN) == 0) {
      uint8_t msg[4 + GARAGE_NONCE_LEN + 4];
      writeLE32(msg, r);
      memcpy(msg + 4, openSlot.nonce, GARAGE_NONCE_LEN);
      memcpy(msg + 4 + GARAGE_NONCE_LEN, "open", 4);
      uint8_t mac[32];
      hmac_sha256(signingKey, GARAGE_KEY_LEN, msg, sizeof(msg), mac);
      uint8_t diff = 0;
      for (uint8_t i = 0; i < GARAGE_SIG_LEN; i++) {
        diff |= mac[i] ^ respBuf[4 + GARAGE_NONCE_LEN + i];
      }
      if (diff == 0) {
        doorSignal = true;
        openSlot.valid = false;
        status = GARAGE_STATUS_OPENED;
      } else {
        status = GARAGE_STATUS_BADSIG;
      }
    }
    publishResult(r, status);
  }
}

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length)
{
  if(strcmp(topic, GARAGE_CMD) == 0)
  {
    doorSignal = true;
  }
  else if(strcmp(topic, GARAGE_OPEN_REQUEST) == 0 && length >= 4)
  {
    memcpy(reqBuf, payload, 4);
    challengePending = true;
  }
  else if(strcmp(topic, GARAGE_OPEN_RESPONSE) == 0 && length == sizeof(respBuf))
  {
    memcpy(respBuf, payload, sizeof(respBuf));
    responsePending = true;
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
        mqttClient.Subscribe(GARAGE_OPEN_REQUEST, 1);
        mqttClient.Subscribe(GARAGE_OPEN_RESPONSE, 1);

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
  EEPROM.get(SENSOR_ID_ADDR, sensorId);
  if (sensorId == 0xFFFF || sensorId == 0)
  {
    randomSeed(analogRead(A0));
    sensorId = random(256, 65535);
    EEPROM.put(SENSOR_ID_ADDR, sensorId);
  }
  randomSeed(analogRead(A0) ^ micros());
  parseSigningKey();
  Serial.println(cryptoSelfTest() ? F("HMAC selftest OK") : F("HMAC selftest FAIL"));
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
  processHandshake();
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
    sprintf(temperatureData, "%u;%d;%d;%d", sensorId, temperature, humidity, SENSOR_CHANNEL);
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
