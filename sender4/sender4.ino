#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <DallasTemperature.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DFRobot_BMI160.h>
#include <SPI.h>

#define DS18_PIN 4

// I2C BMI160
#define SDA_PIN 5
#define SCL_PIN 6

// spi BME280
#define BME_CS 2
#define BME_SCK 7
#define BME_MOSI 9
#define BME_MISO 8

#define PROTOCOL_VERSION 1
#define WIFI_CHANNEL 6

uint8_t receiverMAC[] = {0xD8, 0x3B, 0xDA, 0x45, 0xCD, 0x24};

typedef struct {
  uint8_t version;
  uint32_t packetNumber;
  uint32_t senderTimestamp;
  float p;
  float temperature;
  float v;
  float a;
  uint16_t payloadSize;
} test_packet;

test_packet myData;
uint32_t counter = 0;

volatile bool blinkOnFail = false;
unsigned long lastBlink = 0;
bool ledState = false;

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {

  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");

  if (status == ESP_NOW_SEND_SUCCESS) {
    blinkOnFail = false;
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    blinkOnFail = true;
  }
}

void setup() {

  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  WiFi.mode(WIFI_STA);

  esp_err_t err;

  err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  if (err != ESP_OK) Serial.printf("Protocol set failed: %d\n", err);

  err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  if (err != ESP_OK) Serial.printf("Bandwidth set failed: %d\n", err);

  err = esp_wifi_set_max_tx_power(78);
  if (err != ESP_OK) Serial.printf("TX power set failed: %d\n", err);

  err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) Serial.printf("Channel set failed: %d\n", err);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Peer add failed");
    return;
  }

  Serial.println("Sender ready.");
}

void loop() {

  // Blink LED when failure state active
  if (blinkOnFail) {
    if (millis() - lastBlink > 100) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
    return;   // stop sending while failing
  }

  myData.version = PROTOCOL_VERSION;
  myData.packetNumber = counter++;
  myData.senderTimestamp = millis();

  myData.p = random(100, 200) / 10.0f;
  myData.temperature = temperatureRead();
  myData.v = random(0, 300) / 10.0f;
  myData.a = random(0, 100) / 10.0f;

  myData.payloadSize = sizeof(test_packet);

  esp_err_t result = esp_now_send(
      receiverMAC,
      (uint8_t*)&myData,
      sizeof(test_packet)
  );

  if (result != ESP_OK) {
    Serial.printf("Send error: %d\n", result);
    blinkOnFail = true;
  }

  delay(1000);
}