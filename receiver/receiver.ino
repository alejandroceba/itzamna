#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
//#include <SPI.h>
//#include <SD.h>
#include <math.h>

// ---------------- CONFIGURATION ----------------
//#define SD_CS 21
#define WIFI_CHANNEL 6
#define LINK_TIMEOUT 10000

// DEBUG: Set to 1 for detailed diagnostic output
#define DEBUG 0

// ---------------- ERROR COUNTERS ----------------
uint32_t invalidSizePackets = 0;
//uint32_t sdWriteErrors = 0;

// ============================================================================
// PACKET STRUCT - Must match sender.ino
// ============================================================================
typedef struct {
  // --- Transmission Metadata ---
  uint32_t packetNumber;            // Incremental packet counter
  uint32_t senderTimestamp;         // milliseconds since startup
  
  // --- Temperature (Two Independent Sensors) ---
  float temperature_bme280;         // BME280 temperature in °C
  float temperature_ds18b20;        // DS18B20 temperature in °C
  
  // --- Barometric Data ---
  float pressure_hpa;               // BME280 pressure in hPa
  float altitude_m;                 // BME280 altitude in meters
  
  // --- Velocity (Integrated from accelerometer) ---
  float velocity_x;                 // X-axis velocity in m/s
  float velocity_y;                 // Y-axis velocity in m/s
  float velocity_z;                 // Z-axis velocity in m/s
  
  // --- Acceleration (Corrected: bias-removed, gravity-compensated) ---
  float accel_x;                    // X-axis acceleration in m/s²
  float accel_y;                    // Y-axis acceleration in m/s²
  float accel_z;                    // Z-axis acceleration in m/s²
} sensor_packet;

// ---------------- GLOBAL DATA ----------------
volatile bool newPacketFlag = false;
sensor_packet receivedData;           // Received sensor packet data
int lastRSSI = 0;                     // Signal strength of last packet

uint32_t received = 0, lost = 0, lastPacket = 0;
uint32_t bytesReceived = 0;
unsigned long lastThroughputCalc = 0;

float smoothedRSSI = -60;             // Exponentially smoothed RSSI
float alpha = 0.2;                    // Smoothing factor (0.2 = 20% weight to new sample)

// ---------------- DISTANCE MODEL ----------------
float calculateDistance(float rssi) {
  float rssiAt1Meter = -56;
  float n = 2.0;
  return pow(10.0, ((rssiAt1Meter - rssi) / (10.0 * n)));
}

// ============================================================================
// ESP-NOW RECEPTION CALLBACK
// ============================================================================
void OnReceive(const esp_now_recv_info_t *info,
               const uint8_t *incomingData,
               int len) {

  // Validate packet size matches our expected structure
  if (len != sizeof(sensor_packet)) {
    invalidSizePackets++;
    if (DEBUG) {
      Serial.printf("ERROR: Invalid packet size %d (expected %d)\n", len, sizeof(sensor_packet));
    }
    return;
  }

  // Copy received data into our packet structure
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  // Record signal strength and flag new packet for processing
  lastRSSI = info->rx_ctrl->rssi;
  newPacketFlag = true;
}

// ============================================================================
// SETUP - Initialize ESP-NOW Reception
// ============================================================================
void setup() {

  Serial.begin(115200);
  delay(500);

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

  // Print CSV header for received sensor data
  Serial.println(
    "packetNum,"
    "distance_m,"
    "rssi_dbm,"
    "smoothedRSSI_dbm,"
    "lossRate_pct,"
    "throughput_kbps,"
    "temp_bme280_C,"
    "temp_ds18b20_C,"
    "pressure_hpa,"
    "altitude_m,"
    "velocity_x_ms,"
    "velocity_y_ms,"
    "velocity_z_ms,"
    "accel_x_ms2,"
    "accel_y_ms2,"
    "accel_z_ms2"
  );

  Serial.println("Receiver ready - listening for packets...");
}

// ============================================================================
// MAIN LOOP - Process Received Packets
// ============================================================================
void loop() {

  // Process any newly received packets
  if (newPacketFlag) {

    newPacketFlag = false;

    // Track packet reception
    received++;
    bytesReceived += sizeof(sensor_packet);

    // -------- DETECT PACKET LOSS --------
    // If we skipped packet numbers, some were lost in transmission
    if (lastPacket != 0 && receivedData.packetNumber > lastPacket + 1) {
      lost += (receivedData.packetNumber - lastPacket - 1);
    }
    lastPacket = receivedData.packetNumber;

    // -------- RSSI SMOOTHING (Exponential Moving Average) --------
    // Smoothed RSSI gives more stable distance estimation
    smoothedRSSI = alpha * lastRSSI + (1 - alpha) * smoothedRSSI;

    // -------- CALCULATE PACKET LOSS RATE --------
    float lossRate = 0;
    if ((received + lost) > 0) {
      lossRate = 100.0 * lost / (received + lost);
    }

    // -------- ESTIMATE DISTANCE FROM RSSI --------
    // Uses free-space path loss model: RSSI = RSSI_1m - 20*log10(distance)
    float distance = calculateDistance(smoothedRSSI);

    // -------- CALCULATE THROUGHPUT --------
    // Bits per second transferred (sample every 1000ms)
    float throughput = 0;
    if (millis() - lastThroughputCalc > 1000) {
      throughput = (bytesReceived * 8) / 1000.0;  // Convert bytes to bits, divide by 1 second
      bytesReceived = 0;
      lastThroughputCalc = millis();
    }

    // -------- OUTPUT CSV DATA LINE --------
    // Data goes to Serial (can be captured by Python script or Serial Monitor)
    Serial.print(receivedData.packetNumber); Serial.print(",");
    Serial.print(distance, 4); Serial.print(",");
    Serial.print(lastRSSI); Serial.print(",");
    Serial.print(smoothedRSSI, 4); Serial.print(",");
    Serial.print(lossRate, 4); Serial.print(",");
    Serial.print(throughput, 4); Serial.print(",");
    Serial.print(receivedData.temperature_bme280, 4); Serial.print(",");
    Serial.print(receivedData.temperature_ds18b20, 4); Serial.print(",");
    Serial.print(receivedData.pressure_hpa, 4); Serial.print(",");
    Serial.print(receivedData.altitude_m, 4); Serial.print(",");
    Serial.print(receivedData.velocity_x, 4); Serial.print(",");
    Serial.print(receivedData.velocity_y, 4); Serial.print(",");
    Serial.print(receivedData.velocity_z, 4); Serial.print(",");
    Serial.print(receivedData.accel_x, 4); Serial.print(",");
    Serial.print(receivedData.accel_y, 4); Serial.print(",");
    Serial.println(receivedData.accel_z, 4);

    // ==========================================
    // SD CARD LOGGING (Optional/Future)
    // ==========================================
    // Uncomment and implement if SD card logging is needed
    //File logFile = SD.open("/range_test.csv", FILE_APPEND);
    //if (logFile) {
      // ... write (as above) ...
      //logFile.close();
    //}
  }
}