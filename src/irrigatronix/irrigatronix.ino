/**
* Based on Losan ESP8266 example.
 *
 * Copyright (c) 2023 Losant. All rights reserved.
 * http://losant.com
 */

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <Losant.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

/* This include contains SSID e password of WIFI and
* Losant device_id, access_key and access_secret. I put it in .gitignore.
// WiFi credentials.
const char* WIFI_SSID = "SSID";
const char* WIFI_PASS = "passord";

// Losant credentials.
const char* LOSANT_DEVICE_ID = "000000000000000000000000";
const char* LOSANT_ACCESS_KEY = "12345678-9abc-ffff-abba-1234567890ab";
const char* LOSANT_ACCESS_SECRET = "1111111111111111111111111111111111111111111111111111111111111111";
*/
#include "credentials.h"

const int BUTTON_PIN = 14;
const int LED_PIN = 2;

#define DHTPIN      10
#define DHTTYPE     DHT22

/*Relay attached to solenoid valve*/
#define RELAY_PIN 16

bool ledState = false;

bool isIrrigating = false;

// Cert taken from 
// https://github.com/Losant/losant-mqtt-ruby/blob/master/lib/losant_mqtt/RootCA.crt
static const char digicert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

BearSSL::WiFiClientSecure wifiClient;

LosantDevice device(LOSANT_DEVICE_ID);

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
unsigned long lastMeasure = 0;
unsigned long lastMsg = 0;
unsigned long lastIrrig = 0;
uint8_t max_wait = 120;
bool sentBootInfo = 0;

// Set time via NTP, as required for x.509 validation
void setClock() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    now = time(nullptr);
  }
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
}

void irrigar() {
  unsigned long now = millis();
  Serial.println("Irrigando por 30 segundos");

  /* to SET relay must put pin in LOW State,
   to unset it put pin in Z state */
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  lastIrrig = now;
  isIrrigating = true;

  StaticJsonDocument<200> jsonBuffer;
  JsonObject state = jsonBuffer.to<JsonObject>();
  state["isIrrigating"] = isIrrigating;
  device.sendState(state); 
}

// Called whenever the device receives a command from the Losant platform.
void handleCommand(LosantCommand *command) {
  Serial.print("Command received: ");
  Serial.println(command->name);
  // Optional command payload. May not be present on all commands.
  //JsonObject payload = *command->payload;
  
  // Perform action specific to the command received.
  if(strcmp(command->name, "irrigar") == 0) {
    irrigar();
  }
}

void connect() {

  // Connect to Wifi.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint8_t count = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    ESP.wdtFeed();
    count++;
    if (count > max_wait) {
      ESP.restart();
    }
  }

  // Validating  the SSL for for a secure connection you must
  // set trust anchor, as well as set the time.
  BearSSL::X509List cert(digicert);
  wifiClient.setTrustAnchors(&cert);
  setClock();
  

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to Losant.
  Serial.println();
  Serial.print("Connecting to Losant...");

  device.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);

  count = 0;
  while(!device.connected()) {
    delay(500);
    Serial.print(".");
    ESP.wdtFeed();
    count++;
    if (count > max_wait) {
      ESP.restart();
    }
  }

  Serial.println("Connected!");
}

void setup() {
  digitalWrite(RELAY_PIN, HIGH);
  pinMode(RELAY_PIN, INPUT);

  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;

  Serial.begin(115200);
  while(!Serial) { }
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Register the command handler to be called when a command is received
  // from the Losant platform.
  device.onCommand(&handleCommand);

  connect();

  ESP.wdtDisable();
}

void buttonPressed() {
  Serial.println("Button Pressed!");

  // Losant uses a JSON protocol. Construct the simple state object.
  // { "button" : true }
  StaticJsonDocument<200> jsonBuffer;
  JsonObject root = jsonBuffer.to<JsonObject>();
  root["button"] = true;

  // Send the state to Losant.
  device.sendState(root);
}

int buttonState = 0;

void loop() {
  static float temperature = 0;
  static float humidity = 0;
  float mult = 1000.;
  static bool isTempVal = 0;
  static bool isHumiVal = 0;

  bool toReconnect = false;
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("Disconnected from WiFi");
    toReconnect = true;
  }

  if(!device.connected()) {
    Serial.println("Disconnected from Losant");
    toReconnect = true;
  }

  if(toReconnect) {
    delay(5000);
    connect();
  }

  device.loop();

  if(device.connected()){
    unsigned long now = millis();

    if (!sentBootInfo) {
      sentBootInfo = 1;
      StaticJsonDocument<200> jsonBuffer;
      JsonObject state = jsonBuffer.to<JsonObject>();
      state["rstcause"] = String(ESP.getResetReason());
      state["freeheap"] = int(ESP.getFreeHeap());
      device.sendState(state);
    }

    if (now - lastMeasure > delayMS) {
      lastMeasure = now;
      
      sensors_event_t event;
      dht.temperature().getEvent(&event);
      if (!(isnan(event.temperature))){
        temperature = event.temperature;
        temperature *= mult;
        isTempVal = 1;
      }
      else {
        isTempVal = 0;
      }
      dht.humidity().getEvent(&event);
      if (!(isnan(event.relative_humidity))){
        humidity = event.relative_humidity;
        humidity *= mult;
        isHumiVal = 1;
      }
      else {
        isHumiVal = 0;
      }
      if (isTempVal & isHumiVal) {
        Serial.println(temperature);
        Serial.println(humidity);
      }
      else {
        Serial.println(F("DHT22 Sensor Fail"));
      }
      
    }

    if (now - lastMsg > 60000) {
      lastMsg = now;
      uint8_t rssi = WiFi.RSSI();
      if (isTempVal & isHumiVal) {
        StaticJsonDocument<200> jsonBuffer;
        JsonObject state = jsonBuffer.to<JsonObject>();
        state["temp"] = int(temperature);
        state["humi"] = int(humidity);
        state["rssi"] = int8_t(rssi);
        // Send the state to Losant.
        device.sendState(state); 
      }
    }

    if (isIrrigating) {
      if (now - lastIrrig > 30000) {
        lastIrrig = now;
        isIrrigating = false;
        digitalWrite(RELAY_PIN, HIGH);
        pinMode(RELAY_PIN, INPUT);

      }
    }
  }

  ESP.wdtFeed();

  delay(200);
}
