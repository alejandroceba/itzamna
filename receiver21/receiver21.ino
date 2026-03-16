#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
//#include <SPI.h>
//#include <SD.h>
#include <math.h>

// ---------------- CONFIGURATION ----------------
//#define SD_CS 21
#define PROTOCOL_VERSION 1
#define WIFI_CHANNEL 6
#define LINK_TIMEOUT 10000

// ---------------- ERROR COUNTERS ----------------
uint32_t invalidSizePackets = 0;
uint32_t invalidVersionPackets = 0;
//uint32_t sdWriteErrors = 0;

// ---------------- PACKET STRUCT ----------------
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

// ---------------- GLOBAL DATA ----------------
volatile bool newPacketFlag = false;
test_packet receivedData;
int lastRSSI = 0;

uint32_t received = 0, lost = 0, lastPacket = 0;
uint32_t bytesReceived = 0;
unsigned long lastThroughputCalc = 0;

float smoothedRSSI = -60;
float alpha = 0.2;

// ---------------- DISTANCE MODEL ----------------
float calculateDistance(float rssi) {
  float rssiAt1Meter = -56;
  float n = 2.0;
  return pow(10.0, ((rssiAt1Meter - rssi) / (10.0 * n)));
}

// ---------------- ESP-NOW CALLBACK ----------------
void OnReceive(const esp_now_recv_info_t *info,
               const uint8_t *incomingData,
               int len) {

  if (len != sizeof(test_packet)) {
    invalidSizePackets++;
    return;
  }

  memcpy(&receivedData, incomingData, sizeof(receivedData));

  if (receivedData.version != PROTOCOL_VERSION) {
    invalidVersionPackets++;
    return;
  }

  lastRSSI = info->rx_ctrl->rssi;
  newPacketFlag = true;
}

// ---------------- SETUP ----------------
void setup() {

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_max_tx_power(78);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnReceive);

  // ==============================
  // SD INITIALIZATION (ONLY HERE)
  // ==============================
  //if (!SD.begin(SD_CS)) {
    //Serial.println("SD init failed");
    //while (1);
  //}

  // ==============================
  // HEADER WRITTEN ONLY ONCE
  // ==============================
  //File file = SD.open("/range_test.csv", FILE_APPEND);

  //if (file) {
    //if (file.size() == 0) {
      //file.println("packetNumber,distance,rssi,smoothedRSSI,lossRate,throughput,p,temperature,v,a");
    //}
    //file.close();
  //} else {
    //Serial.println("Header file open failed");
  //}

  Serial.println("Receiver ready");
}

// ---------------- LOOP ----------------
void loop() {

  if (newPacketFlag) {

    newPacketFlag = false;

    received++;
    bytesReceived += sizeof(test_packet);

    // -------- PACKET LOSS --------
    if (lastPacket != 0 && receivedData.packetNumber > lastPacket + 1) {
      lost += (receivedData.packetNumber - lastPacket - 1);
    }
    lastPacket = receivedData.packetNumber;

    // -------- RSSI SMOOTHING --------
    smoothedRSSI = alpha * lastRSSI + (1 - alpha) * smoothedRSSI;

    float lossRate = 0;
    if ((received + lost) > 0) {
      lossRate = 100.0 * lost / (received + lost);
    }

    float distance = calculateDistance(smoothedRSSI);

    float throughput = 0;
    if (millis() - lastThroughputCalc > 1000) {
      throughput = (bytesReceived * 8) / 1000.0;
      bytesReceived = 0;
      lastThroughputCalc = millis();
    }

    // -------- CLEAN CSV OUTPUT FOR PYTHON --------
    Serial.print(receivedData.packetNumber); Serial.print(",");
    Serial.print(distance); Serial.print(",");
    Serial.print(lastRSSI); Serial.print(",");
    Serial.print(smoothedRSSI); Serial.print(",");
    Serial.print(lossRate); Serial.print(",");
    Serial.print(throughput); Serial.print(",");
    Serial.print(receivedData.p); Serial.print(",");
    Serial.print(receivedData.temperature); Serial.print(",");
    Serial.print(receivedData.v); Serial.print(",");
    Serial.println(receivedData.a);

    // ==========================================
    // OPEN → WRITE → CLOSE (CRITICAL FIX HERE)
    // ==========================================
    //File logFile = SD.open("/range_test.csv", FILE_APPEND);

    //if (logFile) {

      //logFile.print(receivedData.packetNumber); logFile.print(",");
      //logFile.print(distance); logFile.print(",");
      //logFile.print(lastRSSI); logFile.print(",");
      //logFile.print(smoothedRSSI); logFile.print(",");
      //logFile.print(lossRate); logFile.print(",");
      //logFile.print(throughput); logFile.print(",");
      //logFile.print(receivedData.p); logFile.print(",");
      //logFile.print(receivedData.temperature); logFile.print(",");
      //logFile.print(receivedData.v); logFile.print(",");
      //logFile.println(receivedData.a);

      //logFile.close();   // ← THIS guarantees FAT update
    //}
    //else {
      //sdWriteErrors++;
    //}
  }
}