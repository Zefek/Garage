#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <AM2302-Sensor.h>
#include "time.h"
#include "esp_task_wdt.h"
#include "esp_random.h"
#include "config.h"
#include "secret.h"
#include "crypto.h"
#include "ota.h"

#ifndef FW_VERSION
#define FW_VERSION 0
#endif

#define SENSOR_CHANNEL 4
#define DEFAULT_SENSOR_ID 39033

#define DOORSWITCH_PIN 22
#define DOORBUTTON_PIN 23
#define DOORFLASH_PIN 21
#define TEMPERATURE_SENSOR_PIN 19

#define MQTT_CLIENT_ID "GarageESP32"
#define MQTT_TLS_PORT 8883
#define WDT_TIMEOUT_S 90
#define WIFI_CONNECT_TIMEOUT_MS 15000UL
#define TIME_SYNC_TIMEOUT_MS 15000UL
#define TIME_VALID_THRESHOLD 1700000000UL
#define MQTT_BACKOFF_MAX_MS 60000UL
#define MQTT_KEEPALIVE_S 60

#define T_FULL_MS 16000UL
#define T_FULL_MARGIN_MS 1000UL
#define T_LEAD_DEFAULT_MS 1500UL
#define FLASH_TIMEOUT_MS 1200UL
#define REVERSE_MARGIN_MS 2000UL
#define REED_DEBOUNCE_MS 50UL
#define MOVE_PUBLISH_INTERVAL 1000UL

#define DOOR_PULSE_MS 500UL
#define TEMPERATURE_INTERVAL 60000UL
#define DIAG_INTERVAL 900000UL

#define GARAGE_STATUS_OPENED 1
#define GARAGE_STATUS_EXPIRED 2
#define GARAGE_STATUS_BADSIG 3

void MQTTMessageReceive(char* topic, uint8_t* payload, unsigned int length);
void PublishDoorState(bool force = false);

enum DoorState { DoorUnknown, DoorClosed, DoorOpening, DoorOpen, DoorClosing, DoorStopped };

#pragma pack(push, 1)
struct DiagData {
  uint32_t uptime;
  uint16_t freeRamKb;
  uint16_t wifiReconn;
  uint16_t mqttReconn;
  uint8_t  sensorErr;
  uint8_t  resetReason;
  uint16_t loopMaxMs;
  uint16_t doorCycles;
  int8_t   rssi;
  uint16_t fwVersion;
  uint16_t otaFailCount;
  uint16_t lastTravelMs;
  uint16_t lastLeadMs;
};
#pragma pack(pop)
static_assert(sizeof(DiagData) == 25, "DiagData wire layout must stay 25 bytes");

WiFiClientSecure net;
PubSubClient mqtt(net);
Preferences preferences;
AM2302::AM2302_Sensor am2302{ TEMPERATURE_SENSOR_PIN };

unsigned long currentMillis = 0;
unsigned long doorPulseStart = 0;
unsigned long mqttConnectionTimeout = 0;
unsigned long mqttLastConnectionTry = 0;
unsigned long temperatureHumidityReadMillis = 0;
unsigned long lastDiagSendMillis = 0;
unsigned long lastStatePublish = 0;

bool doorSignal = false;
bool doorPulseActive = false;
bool timeSynced = false;

DoorState doorState = DoorUnknown;
int8_t lastDirection = 0;
unsigned long travelMs = 0;
unsigned long travelBase = 0;
unsigned long movementOrigin = 0;
bool movementOriginValid = false;
bool moving = false;
bool awaitingReed = false;
bool travelFromClosed = false;
unsigned long leadMs = T_LEAD_DEFAULT_MS;
uint16_t measuredTravelMs = 0;
uint16_t measuredLeadMs = 0;

volatile unsigned long lastFlashMillis = 0;
bool flashActive = false;
unsigned long burstStartMillis = 0;

int reedStable = HIGH;
int reedRaw = HIGH;
unsigned long reedRawChangeAt = 0;

char temperatureData[20];
char lastPayload[24] = "";
uint16_t sensorId = 0;

DiagData currentDiagData = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t otaFailures = 0;

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
uint8_t respBuf[4 + GARAGE_SIG_LEN];

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

static bool isHexChar(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

static bool parseSigningKey() {
  const char* h = SigningKeyHex;
  if (strnlen(h, GARAGE_KEY_LEN * 2 + 2) != (size_t)(GARAGE_KEY_LEN * 2)) return false;
  for (uint8_t i = 0; i < GARAGE_KEY_LEN * 2; i++) {
    if (!isHexChar(h[i])) return false;
  }
  for (uint8_t i = 0; i < GARAGE_KEY_LEN; i++) {
    signingKey[i] = (hexNibble(h[i * 2]) << 4) | hexNibble(h[i * 2 + 1]);
  }
  return true;
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

void IRAM_ATTR OnDoorFlash() {
  lastFlashMillis = millis();
}

static unsigned long currentTravel()
{
  if(!moving || !movementOriginValid)
  {
    return travelMs;
  }
  long delta = (long)(lastFlashMillis - movementOrigin);
  if(delta <= 0)
  {
    return travelBase;
  }
  unsigned long elapsed = (unsigned long)delta;
  if(lastDirection > 0)
  {
    unsigned long t = travelBase + elapsed;
    return t > T_FULL_MS ? T_FULL_MS : t;
  }
  if(lastDirection < 0)
  {
    return elapsed >= travelBase ? 0 : travelBase - elapsed;
  }
  return travelBase;
}

static int positionPercent()
{
  if(doorState == DoorUnknown)
  {
    return -1;
  }
  return (int)((currentTravel() * 100UL) / T_FULL_MS);
}

static const char* motionText()
{
  if(!moving)
  {
    return "Stop";
  }
  if(lastDirection > 0)
  {
    return "Opening";
  }
  if(lastDirection < 0)
  {
    return "Closing";
  }
  return "Move";
}

void PublishDoorState(bool force)
{
  char payload[24];
  sprintf(payload, "%s;%s;%d", reedStable == HIGH ? "Closed" : "Open", motionText(), positionPercent());
  lastStatePublish = currentMillis;
  if(!force && strcmp(payload, lastPayload) == 0)
  {
    return;
  }
  strcpy(lastPayload, payload);
  mqtt.publish(GARAGE_STATE, payload, true);
}

void OnMovementStart(unsigned long firstFlashAt)
{
  burstStartMillis = firstFlashAt;
  travelBase = travelMs;
  travelFromClosed = false;
  awaitingReed = false;
  movementOriginValid = false;
  moving = true;

  if(doorState == DoorClosed)
  {
    lastDirection = 1;
    doorState = DoorOpening;
    travelBase = 0;
    awaitingReed = true;
  }
  else if(doorState == DoorOpen)
  {
    lastDirection = -1;
    doorState = DoorClosing;
    movementOrigin = firstFlashAt + leadMs;
    movementOriginValid = true;
  }
  else if(doorState == DoorStopped && lastDirection != 0)
  {
    lastDirection = lastDirection > 0 ? -1 : 1;
    doorState = lastDirection > 0 ? DoorOpening : DoorClosing;
    movementOrigin = firstFlashAt + leadMs;
    movementOriginValid = true;
  }
  else
  {
    lastDirection = 0;
    doorState = DoorUnknown;
  }
  PublishDoorState();
}

void OnMovementEnd(unsigned long lastFlashAt)
{
  moving = false;
  if(movementOriginValid)
  {
    long delta = (long)(lastFlashAt - movementOrigin);
    unsigned long elapsed = delta > 0 ? (unsigned long)delta : 0;
    if(lastDirection > 0)
    {
      travelMs = travelBase + elapsed;
      if(travelMs > T_FULL_MS)
      {
        travelMs = T_FULL_MS;
      }
      if(travelFromClosed)
      {
        measuredTravelMs = elapsed > 65535UL ? 65535 : (uint16_t)elapsed;
      }
    }
    else if(lastDirection < 0)
    {
      if(elapsed > travelBase + REVERSE_MARGIN_MS)
      {
        travelMs = T_FULL_MS;
        lastDirection = 1;
        currentDiagData.sensorErr |= 0x02;
      }
      else
      {
        travelMs = elapsed >= travelBase ? 0 : travelBase - elapsed;
      }
    }
  }
  movementOriginValid = false;
  awaitingReed = false;
  travelFromClosed = false;

  if(reedStable == HIGH)
  {
    travelMs = 0;
    doorState = DoorClosed;
  }
  else if(lastDirection == 0)
  {
    doorState = DoorUnknown;
  }
  else if(travelMs + T_FULL_MARGIN_MS >= T_FULL_MS)
  {
    travelMs = T_FULL_MS;
    doorState = DoorOpen;
  }
  else
  {
    doorState = DoorStopped;
  }
  PublishDoorState();
}

void OnReedChanged(unsigned long edgeAt)
{
  if(reedStable == HIGH)
  {
    travelMs = 0;
    travelBase = 0;
    movementOriginValid = false;
    awaitingReed = false;
    travelFromClosed = false;
    doorState = DoorClosed;
  }
  else
  {
    if(currentDiagData.doorCycles < 0xFFFF)
    {
      currentDiagData.doorCycles++;
    }
    if(awaitingReed)
    {
      awaitingReed = false;
      movementOrigin = edgeAt;
      movementOriginValid = true;
      travelBase = 0;
      travelFromClosed = true;
      long lead = (long)(edgeAt - burstStartMillis);
      if(lead > 0)
      {
        leadMs = (unsigned long)lead;
        measuredLeadMs = lead > 65535L ? 65535 : (uint16_t)lead;
      }
    }
  }
  PublishDoorState();
}

static void publishChallenge(uint32_t r) {
  uint8_t buf[4 + GARAGE_NONCE_LEN];
  writeLE32(buf, r);
  memcpy(buf + 4, openSlot.nonce, GARAGE_NONCE_LEN);
  mqtt.publish(GARAGE_OPEN_CHALLENGE, buf, sizeof(buf), false);
}

static void publishResult(uint32_t r, uint8_t status) {
  uint8_t buf[5];
  writeLE32(buf, r);
  buf[4] = status;
  mqtt.publish(GARAGE_OPEN_RESULT, buf, sizeof(buf), false);
}

static void processHandshake() {
  if (challengePending) {
    challengePending = false;
    uint32_t r = readLE32(reqBuf);
    esp_fill_random(openSlot.nonce, GARAGE_NONCE_LEN);
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
        && r == openSlot.correlationId) {
      uint8_t msg[4 + GARAGE_NONCE_LEN + 4];
      writeLE32(msg, r);
      memcpy(msg + 4, openSlot.nonce, GARAGE_NONCE_LEN);
      memcpy(msg + 4 + GARAGE_NONCE_LEN, "open", 4);
      uint8_t mac[32];
      hmac_sha256(signingKey, GARAGE_KEY_LEN, msg, sizeof(msg), mac);
      uint8_t diff = 0;
      for (uint8_t i = 0; i < GARAGE_SIG_LEN; i++) {
        diff |= mac[i] ^ respBuf[4 + i];
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

void MQTTMessageReceive(char* topic, uint8_t* payload, unsigned int length)
{
  if(strcmp(topic, GARAGE_OPEN_REQUEST) == 0 && length >= 4)
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

bool SyncTime()
{
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  unsigned long start = millis();
  time_t now = time(nullptr);
  while(now < TIME_VALID_THRESHOLD && millis() - start < TIME_SYNC_TIMEOUT_MS)
  {
    esp_task_wdt_reset();
    delay(200);
    now = time(nullptr);
  }
  return now >= TIME_VALID_THRESHOLD;
}

bool Connect()
{
  if(mqtt.connected())
  {
    return true;
  }
  if(currentMillis - mqttLastConnectionTry < mqttConnectionTimeout)
  {
    return false;
  }
  mqttLastConnectionTry = currentMillis;
  if(WiFi.status() != WL_CONNECTED)
  {
    if(currentDiagData.wifiReconn < 0xFFFF)
    {
      currentDiagData.wifiReconn++;
    }
    WiFi.begin(WifiSSID, WifiPassword);
    unsigned long start = millis();
    while(WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS)
    {
      esp_task_wdt_reset();
      delay(100);
    }
  }
  if(WiFi.status() != WL_CONNECTED)
  {
    mqttConnectionTimeout = min(mqttConnectionTimeout * 2 + random(0, 5000), MQTT_BACKOFF_MAX_MS);
    return false;
  }
  if(!timeSynced)
  {
    timeSynced = SyncTime();
    if(!timeSynced)
    {
      mqttConnectionTimeout = min(mqttConnectionTimeout * 2 + random(0, 5000), MQTT_BACKOFF_MAX_MS);
      return false;
    }
  }
  if(currentDiagData.mqttReconn < 0xFFFF)
  {
    currentDiagData.mqttReconn++;
  }
  if(!mqtt.connect(MQTT_CLIENT_ID, MQTTUsername, MQTTPassword))
  {
    mqttConnectionTimeout = min(mqttConnectionTimeout * 2 + random(0, 5000), MQTT_BACKOFF_MAX_MS);
    return false;
  }
  mqtt.subscribe(GARAGE_OPEN_REQUEST, 1);
  mqtt.subscribe(GARAGE_OPEN_RESPONSE, 1);
  PublishDoorState(true);
  mqttConnectionTimeout = 0;
  return true;
}

void sendDiag()
{
  currentDiagData.uptime = currentMillis / 60000UL;
  currentDiagData.freeRamKb = (uint16_t)(ESP.getFreeHeap() / 1024);
  currentDiagData.rssi = (int8_t)WiFi.RSSI();
  currentDiagData.fwVersion = (uint16_t)FW_VERSION;
  currentDiagData.otaFailCount = otaFailures;
  currentDiagData.lastTravelMs = measuredTravelMs;
  currentDiagData.lastLeadMs = measuredLeadMs;
  mqtt.publish(GARAGE_DIAG, (const uint8_t*)&currentDiagData, sizeof(DiagData), false);
  currentDiagData.loopMaxMs = 0;
  currentDiagData.sensorErr = 0;
}

bool OtaAllowed()
{
  return doorState == DoorClosed
    && !moving
    && !doorPulseActive
    && !doorSignal
    && !openSlot.valid
    && !challengePending
    && !responsePending;
}

void setup() {
  currentDiagData.resetReason = (uint8_t)esp_reset_reason();

  digitalWrite(DOORBUTTON_PIN, HIGH);
  pinMode(DOORBUTTON_PIN, OUTPUT);
  digitalWrite(DOORBUTTON_PIN, HIGH);

  pinMode(DOORSWITCH_PIN, INPUT_PULLUP);
  pinMode(DOORFLASH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DOORFLASH_PIN), OnDoorFlash, FALLING);

  Serial.begin(115200);

  preferences.begin("garage", false);
  sensorId = preferences.getUShort("sensorId", 0);
  if (sensorId == 0xFFFF || sensorId == 0)
  {
    sensorId = DEFAULT_SENSOR_ID;
    preferences.putUShort("sensorId", sensorId);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WifiSSID, WifiPassword);
  net.setCACert(MQTTCACert);
  mqtt.setServer(MQTTHost, MQTT_TLS_PORT);
  mqtt.setCallback(MQTTMessageReceive);
  mqtt.setBufferSize(256);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);

  if (am2302.begin())
  {
    delay(3000);
  }

  reedRaw = digitalRead(DOORSWITCH_PIN);
  reedStable = reedRaw;
  doorState = reedStable == HIGH ? DoorClosed : DoorUnknown;
  randomSeed(esp_random());

  if (!parseSigningKey()) {
    Serial.println("SigningKey INVALID");
  }
  Serial.println(cryptoSelfTest() ? "HMAC selftest OK" : "HMAC selftest FAIL");

  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&wdtConfig);
  esp_task_wdt_add(NULL);
  Serial.println("Setup OK");
}

void loop() {
  currentMillis = millis();
  esp_task_wdt_reset();

  if(!mqtt.connected())
  {
    Connect();
  }
  mqtt.loop();

  if(digitalRead(DOORFLASH_PIN) == LOW)
  {
    lastFlashMillis = currentMillis;
  }
  unsigned long flashAt = lastFlashMillis;
  bool nowFlashing = flashAt != 0 && currentMillis - flashAt < FLASH_TIMEOUT_MS;

  int raw = digitalRead(DOORSWITCH_PIN);
  if(raw != reedRaw)
  {
    reedRaw = raw;
    reedRawChangeAt = currentMillis;
  }
  else if(reedStable != reedRaw && currentMillis - reedRawChangeAt >= REED_DEBOUNCE_MS)
  {
    reedStable = reedRaw;
    OnReedChanged(reedRawChangeAt);
  }

  if(nowFlashing && !flashActive)
  {
    flashActive = true;
    OnMovementStart(flashAt);
  }
  else if(!nowFlashing && flashActive)
  {
    flashActive = false;
    OnMovementEnd(flashAt);
  }

  if(moving && currentMillis - lastStatePublish >= MOVE_PUBLISH_INTERVAL)
  {
    PublishDoorState();
  }

  processHandshake();

  if(doorSignal && !doorPulseActive)
  {
    digitalWrite(DOORBUTTON_PIN, LOW);
    doorPulseStart = currentMillis;
    doorPulseActive = true;
    doorSignal = false;
  }
  if(doorPulseActive && currentMillis - doorPulseStart >= DOOR_PULSE_MS)
  {
    digitalWrite(DOORBUTTON_PIN, HIGH);
    doorPulseActive = false;
  }

  if(currentMillis - temperatureHumidityReadMillis > TEMPERATURE_INTERVAL)
  {
    uint8_t status = am2302.read();
    if(status != AM2302::AM2302_READ_OK)
    {
      currentDiagData.sensorErr |= 0x01;
    }
    int temperature = (int)lroundf(am2302.get_Temperature() * 10);
    int humidity = (int)lroundf(am2302.get_Humidity());
    sprintf(temperatureData, "%u;%d;%d;%d", sensorId, temperature, humidity, SENSOR_CHANNEL);
    mqtt.publish(GARAGE_TEMPERATURE, temperatureData);
    temperatureHumidityReadMillis = currentMillis;
  }

  if(currentMillis - lastDiagSendMillis > DIAG_INTERVAL)
  {
    sendDiag();
    lastDiagSendMillis = currentMillis;
  }

  if(OtaAllowed())
  {
    otaLoop();
  }

  unsigned long loopDuration = millis() - currentMillis;
  if(loopDuration > currentDiagData.loopMaxMs)
  {
    currentDiagData.loopMaxMs = (loopDuration > 65535UL) ? 65535 : (uint16_t)loopDuration;
  }
}
