#include <Wire.h>
#include <EasyMqtt.h>
#include "config.h"

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define DEBUG="true";

EasyMqtt mqtt;

// https://openenergymonitor.org/forum-archive/node/5786.html

uint8_t ADDRESS = 0x4A;
double ICAL = 30; // SCT-013-030

struct Gain {
  uint16_t config;
  float multiplier;
} GAIN;

Gain gain[] = {
  {0x0000, 0.1875F},    // TWOTHIRDS
  {0x0200, 0.125F},     // ONE
  {0x0400, 0.0625F},    // TWO
  {0x0600, 0.03125F},   // FOUR
  {0x0800, 0.015625F},  // EIGHT
  {0x0A00, 0.0078125F}  // SIXTEEN
};

static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)(value>>8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)2);
  return ((Wire.read() << 8) | Wire.read());
}

static uint16_t readADC(uint8_t i2cAddress, Gain gain, uint8_t channel) {
  uint16_t config = 0x81E3 | gain.config | (0x4000 + (channel*0x1000));
  writeRegister(i2cAddress, 0x01, config);
  delay(1);
  return readRegister(i2cAddress, 0x00);
}

double squareRoot(double fg) {
  double n = fg / 2.0;
  double lstX = 0.0;
  while (n != lstX) {
    lstX = n;
    n = (n + fg / n) / 2.0;
  }
  return n;
}

double sum[4] = {0,0,0,0};
long samples[4] = {0,0,0,0};
int lastSample[4] = {0,0,0,0};
double filtered[4] = {0,0,0,0};
int running[4] = {0,0,0,0};

void takeSample(uint8_t i2cAddress, Gain gain, uint8_t channel) {
  uint16_t sample = readADC(i2cAddress, gain, channel);
  if(lastSample[channel] == 0) lastSample[channel] = sample;
  filtered[channel] = 0.996*(filtered[channel]+sample-lastSample[channel]);
  sum[channel] += (filtered[channel] * filtered[channel]);
  lastSample[channel] = sample;
  samples[channel]++;
}

String getCurrent(Gain gain, uint8_t channel) {
  if(samples[channel] == 0) return "";
  double irms = ICAL * squareRoot(sum[channel] / samples[channel]) * gain.multiplier / 1000;
  mqtt["power"][channel]["samples"].publish(String(samples[channel]));
  sum[channel] = 0;
  samples[channel] = 0;
  int power = irms * 230;
  
  if(running[channel] == 0 && power <= 11) {
    mqtt["power"][channel]["running"].publish("OFF");
  } else if(running[channel] == 3 && power > 11) {
    mqtt["power"][channel]["running"].publish("ON");
  } else {
    running[channel] += (power > 11) ? 1 : -1;
  }

  return String(irms * 230);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mqtt.wifi(wifi_ssid, wifi_pass);
  mqtt.mqtt(mqtt_server, mqtt_port, mqtt_user, mqtt_password);

  mqtt["power"].setInterval(15);
  mqtt["power"][0]["current"] << [](){ return getCurrent(gain[1], 0); };
  //mqtt["power"][1]["current"] << [](){ return getCurrent(gain[1], 1); };
  mqtt["power"][2]["current"] << [](){ return getCurrent(gain[1], 2); };

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  ArduinoOTA.setPassword((const char *)wifi_pass);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void loop() {
  mqtt.loop();
  takeSample(ADDRESS, gain[1], 0);
  //takeSample(ADDRESS, gain[1], 1);
  takeSample(ADDRESS, gain[1], 2);
  ArduinoOTA.handle();
}
