// ============================================================================
// INTEGRATED SENSOR + ESP-NOW SENDER
// Combines: sender4.ino (ESP-NOW) + Sensado_Cansat_1.ino (Sensor Suite)
// ============================================================================

// ============================================================================
// INCLUDES
// ============================================================================
#include <Wire.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DallasTemperature.h>
#include <DFRobot_BMI160.h>

#include <string.h>

// ============================================================================
// DEBUGGING & CONFIGURATION
// ============================================================================
#define DEBUG 1                        // Set to 1 for detailed CSV logs, 0 for silent operation

// ============================================================================
// ESP-NOW & WIFI CONFIGURATION
// ============================================================================
#define WIFI_CHANNEL 6
//uint8_t receiverMAC[] = {0xd8, 0x3b, 0xda, 0x46, 0x57, 0x84};  // Receiver MAC address
uint8_t receiverMAC[] = {0xd8, 0x3b, 0xda, 0x45, 0xcd, 0x24};    // Sender MAC address
// ============================================================================
// SENSOR PIN CONFIGURATION
// ============================================================================
// DS18B20 (OneWire temperature sensor)
#define DS18_PIN 4

// BMI160 (I2C accelerometer/gyroscope)
#define SDA_PIN 5
#define SCL_PIN 6
#define BMI160_I2C_ADDR 0x68
// BME280 (SPI barometric/temperature sensor)
#define BME_CS 2
#define BME_SCK 7
#define BME_MOSI 9
#define BME_MISO 8

// ============================================================================
// SENSOR TIMING & PARAMETERS
// ============================================================================
#define ACCEL_SENSITIVITY_THRESHOLD 0.3  // m/s² - threshold for velocity reset
#define ACCEL_SCALE 16384.0              // ±2g sensitivity scale factor
#define MUESTRAS_BIAS 2000                // Number of samples for bias calibration


// Sampling rates
const unsigned long SAMPLE_INTERVAL_MS = 10.0;    // 100 Hz sensor reading rate
const unsigned long SEND_INTERVAL_MS = 1000.0;    // 1 Hz ESP-NOW transmission rate

// Image forwarding modes:
// 0 = conservative (higher telemetry protection, higher image latency)
// 1 = faster (lower image latency, still bounded by per-loop budget)
#define IMG_FORWARD_MODE 1

const uint8_t IMG_FORWARD_PACKETS_PER_LOOP_SAFE = 1;
const uint8_t IMG_FORWARD_PACKETS_PER_LOOP_FAST = 4;
const unsigned long IMG_FORWARD_MIN_GAP_MS_SAFE = 12;
const unsigned long IMG_FORWARD_MIN_GAP_MS_FAST = 3;

// ============================================================================
// SENSOR OBJECT INSTANTIATION
// ============================================================================
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);  // SPI mode
OneWire oneWire(DS18_PIN);
DallasTemperature ds18b20(&oneWire);
DFRobot_BMI160 bmi;

//const int8_t BMI160_I2C_ADDR = 0x68;

// ============================================================================
// DATA PACKET STRUCTURE
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

// ============================================================================
// IMAGE RELAY PACKET STRUCTS (must match receiver image protocol)
// ============================================================================
static const uint16_t PACKET_MAGIC = 0xA66A;
static const uint8_t PKT_BEGIN = 1;
static const uint8_t PKT_CHUNK = 2;
static const uint8_t PKT_END = 3;
static const uint16_t ESPNOW_MAX_PAYLOAD = 250;

struct __attribute__((packed)) BeginPacket {
  uint8_t type;
  uint16_t magic;
  uint16_t imageId;
  uint16_t width;
  uint16_t height;
  uint32_t dataLen;
  uint16_t chunkPayload;
};

struct __attribute__((packed)) ImageForwardItem {
  uint16_t len;
  uint8_t data[ESPNOW_MAX_PAYLOAD];
};

static const uint8_t IMG_FORWARD_QUEUE_SIZE = 24;

// ============================================================================
// GLOBAL VARIABLES - TRANSMISSION
// ============================================================================
sensor_packet sensorData;
uint32_t packetCounter = 0;
volatile bool transmitFailed = false;
unsigned long lastTransmitTime = 0;

// ============================================================================
// GLOBAL VARIABLES - IMAGE RELAY
// ============================================================================
ImageForwardItem imageQueue[IMG_FORWARD_QUEUE_SIZE];
volatile uint8_t imageQueueHead = 0;
volatile uint8_t imageQueueTail = 0;
volatile uint32_t imageQueueDrops = 0;

bool haveImageSourceMac = false;
uint8_t imageSourceMac[6] = {0};

unsigned long lastImageForwardTime = 0;
uint32_t forwardedImagePackets = 0;
uint32_t droppedImagePackets = 0;
uint32_t invalidImagePackets = 0;
uint32_t rejectedSourcePackets = 0;
unsigned long lastRelayStatusPrint = 0;

portMUX_TYPE imageMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// GLOBAL VARIABLES - SENSOR STATE
// ============================================================================
int16_t rawSensorData[6];           // [0-2] gyro, [3-5] accel from BMI160

// Accelerometer calibration (bias removal)
float bias_accel_x = 0.0;
float bias_accel_y = 0.0;
float bias_accel_z = 0.0;

// Current acceleration values (after bias removal)
float current_accel_x = 0.0;
float current_accel_y = 0.0;
float current_accel_z = 0.0;

// Integrated velocity
float velocity_integral_x = 0.0;
float velocity_integral_y = 0.0;
float velocity_integral_z = 0.0;

// Timing for velocity integration
unsigned long lastIntegrationTime = 0;

// Sensor reading tracking
unsigned long lastSampleTime = 0;

// CSV header tracking
bool headerPrinted = false;

// LED indicators
unsigned long lastBlinkTime = 0;
bool ledState = false;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initializeSensors();
void calibrateAccelerometer();
void readAllSensors();
void integrateVelocity();
void transmitSensorData();
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
void blinkLEDOnFailure();
void printDebugHeader();
void printDebugData();
void processImageForwarding(unsigned long now);
bool enqueueImagePacket(const uint8_t *data, uint16_t len);
bool dequeueImagePacket(ImageForwardItem &out);
bool isValidImagePacket(const uint8_t *data, int len);
bool isSameMac(const uint8_t *a, const uint8_t *b);
void copyMac(uint8_t *dst, const uint8_t *src);

// ============================================================================
// SETUP - Initialize All Subsystems
// ============================================================================
void setup() {
  // --- Serial Communication ---
  Serial.begin(115200);
  delay(500);  // Allow serial stabilization
  
  if (DEBUG) {
    Serial.println("\n========================================");
    Serial.println("INTEGRATED SENDER INITIALIZATION");
    Serial.println("========================================\n");
  }
  
  // --- LED Setup ---
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // --- WiFi & ESP-NOW Setup ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  // Configure WiFi parameters for reliable communication
  esp_err_t err;
  
  err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  if (err != ESP_OK && DEBUG) Serial.printf("WiFi protocol config failed: %d\n", err);
  
  err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  if (err != ESP_OK && DEBUG) Serial.printf("WiFi bandwidth config failed: %d\n", err);
  
  err = esp_wifi_set_max_tx_power(78);
  if (err != ESP_OK && DEBUG) Serial.printf("WiFi TX power config failed: %d\n", err);
  
  err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK && DEBUG) Serial.printf("WiFi channel config failed: %d\n", err);
  
  // Initialize ESP-NOW protocol
  if (esp_now_init() != ESP_OK) {
    if (DEBUG) Serial.println("ERROR: ESP-NOW initialization failed");
    return;
  }
  
  // Register transmission callback
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  
  // Add receiver as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    if (DEBUG) Serial.println("ERROR: Failed to add receiver as peer");
    return;
  }
  
  if (DEBUG) Serial.println("✓ ESP-NOW configured\n");
  
  // --- Sensor Initialization ---
  initializeSensors();
  
  if (DEBUG) {
    Serial.println("\n========================================");
    Serial.println("SENSOR INITIALIZATION COMPLETE");
    Serial.println("========================================\n");
  }
  
  // --- Accelerometer Calibration ---
  calibrateAccelerometer();
  
  if (DEBUG) {
    Serial.println("\n========================================");
    Serial.println("STARTING DATA STREAM");
    Serial.println("========================================\n");
  }
  
  // Initialize timing
  lastSampleTime = millis();
  lastIntegrationTime = millis();
  lastTransmitTime = millis();
  
  // Print CSV header for debug output
  if (DEBUG) {
    printDebugHeader();
  }
}

// ============================================================================
// MAIN LOOP - Read Sensors and Transmit
// ============================================================================
void loop() {
  unsigned long now = millis();
  
  // --- Handle LED blinking on transmission failure ---
  blinkLEDOnFailure();
  
  // --- Read sensors at 100 Hz (10 ms interval) ---
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    readAllSensors();
    integrateVelocity();
  }
  
  // --- Transmit sensor data at 1 Hz (1000 ms interval) ---
  if (now - lastTransmitTime >= SEND_INTERVAL_MS) {
    lastTransmitTime = now;
    transmitSensorData();
    
    // Print debug CSV line
    if (DEBUG) {
      printDebugData();
    }
  }

  // Relay image packets with a bounded per-loop budget.
  processImageForwarding(now);

  if (DEBUG && (now - lastRelayStatusPrint) >= 2000) {
    uint8_t queued;
    portENTER_CRITICAL(&imageMux);
    queued = (uint8_t)((imageQueueHead + IMG_FORWARD_QUEUE_SIZE - imageQueueTail) % IMG_FORWARD_QUEUE_SIZE);
    portEXIT_CRITICAL(&imageMux);

    Serial.printf(
      "RELAY_STATUS mode=%u queued=%u qdrops=%lu forwarded=%lu dropped=%lu invalid=%lu src_reject=%lu\n",
      (unsigned)IMG_FORWARD_MODE,
      (unsigned)queued,
      (unsigned long)imageQueueDrops,
      (unsigned long)forwardedImagePackets,
      (unsigned long)droppedImagePackets,
      (unsigned long)invalidImagePackets,
      (unsigned long)rejectedSourcePackets
    );
    lastRelayStatusPrint = now;
  }
}

// ============================================================================
// IMAGE RELAY HELPERS
// ============================================================================
bool isSameMac(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

void copyMac(uint8_t *dst, const uint8_t *src) {
  for (int i = 0; i < 6; i++) {
    dst[i] = src[i];
  }
}

bool enqueueImagePacket(const uint8_t *data, uint16_t len) {
  if (!data || len == 0 || len > ESPNOW_MAX_PAYLOAD) return false;

  uint8_t nextHead = (uint8_t)((imageQueueHead + 1) % IMG_FORWARD_QUEUE_SIZE);
  if (nextHead == imageQueueTail) {
    imageQueueDrops++;
    return false;
  }

  imageQueue[imageQueueHead].len = len;
  memcpy(imageQueue[imageQueueHead].data, data, len);
  imageQueueHead = nextHead;
  return true;
}

bool dequeueImagePacket(ImageForwardItem &out) {
  if (imageQueueTail == imageQueueHead) return false;

  out = imageQueue[imageQueueTail];
  imageQueueTail = (uint8_t)((imageQueueTail + 1) % IMG_FORWARD_QUEUE_SIZE);
  return true;
}

bool isValidImagePacket(const uint8_t *data, int len) {
  if (!data || len <= 0 || len > ESPNOW_MAX_PAYLOAD) return false;

  uint8_t type = data[0];
  if (type == PKT_BEGIN) {
    if (len != (int)sizeof(BeginPacket)) return false;
    BeginPacket begin{};
    memcpy(&begin, data, sizeof(BeginPacket));
    return begin.magic == PACKET_MAGIC;
  }

  if (type == PKT_CHUNK) {
    return len >= 7;
  }

  if (type == PKT_END) {
    return len == 9;
  }

  return false;
}

void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!info || !incomingData || !isValidImagePacket(incomingData, len)) {
    invalidImagePackets++;
    return;
  }

  const uint8_t *src = info->src_addr;
  uint8_t type = incomingData[0];

  portENTER_CRITICAL_ISR(&imageMux);

  // Learn/refresh source MAC only from valid begin packets.
  if (type == PKT_BEGIN) {
    if (!haveImageSourceMac || !isSameMac(imageSourceMac, src)) {
      copyMac(imageSourceMac, src);
      haveImageSourceMac = true;
    }
  }

  // Accept chunk/end only from learned source.
  if (!haveImageSourceMac || !isSameMac(imageSourceMac, src)) {
    rejectedSourcePackets++;
    portEXIT_CRITICAL_ISR(&imageMux);
    return;
  }

  bool queued = enqueueImagePacket(incomingData, (uint16_t)len);
  if (!queued) {
    droppedImagePackets++;
  }

  portEXIT_CRITICAL_ISR(&imageMux);
}

void processImageForwarding(unsigned long now) {
  const uint8_t perLoopBudget = (IMG_FORWARD_MODE == 0)
    ? IMG_FORWARD_PACKETS_PER_LOOP_SAFE
    : IMG_FORWARD_PACKETS_PER_LOOP_FAST;
  const unsigned long minGapMs = (IMG_FORWARD_MODE == 0)
    ? IMG_FORWARD_MIN_GAP_MS_SAFE
    : IMG_FORWARD_MIN_GAP_MS_FAST;

  if (now - lastImageForwardTime < minGapMs) {
    return;
  }

  uint8_t sentThisLoop = 0;
  while (sentThisLoop < perLoopBudget) {
    ImageForwardItem item{};

    portENTER_CRITICAL(&imageMux);
    bool hasItem = dequeueImagePacket(item);
    portEXIT_CRITICAL(&imageMux);

    if (!hasItem) break;

    esp_err_t err = esp_now_send(receiverMAC, item.data, item.len);
    if (err == ESP_OK) {
      forwardedImagePackets++;
      sentThisLoop++;
      lastImageForwardTime = now;
    } else {
      droppedImagePackets++;
      break;
    }
  }
}

// ============================================================================
// SENSOR INITIALIZATION
// ============================================================================
void initializeSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);

  if (bmi.softReset() != BMI160_OK) {
    if (DEBUG) Serial.println("ERROR: BMI160 soft reset failed");
  }

  if (bmi.I2cInit(BMI160_I2C_ADDR) != BMI160_OK) {
    if (DEBUG) Serial.println("ERROR: BMI160 I2C init failed");
  } else if (DEBUG) {
    Serial.println("BMI160 initialized");
  }

  SPI.begin(BME_SCK, BME_MISO, BME_MOSI, BME_CS);
  pinMode(BME_CS, OUTPUT);
  digitalWrite(BME_CS, HIGH);

  if (!bme.begin(BME_CS)) {
    if (DEBUG) Serial.println("ERROR: BME280 initialization failed");
  } else if (DEBUG) {
    Serial.println("BME280 initialized");
  }

  ds18b20.begin();
  if (DEBUG) Serial.println("DS18B20 initialized");
}

// ============================================================================
// ACCELEROMETER BIAS CALIBRATION
// ============================================================================
void calibrateAccelerometer() {
  if (DEBUG) Serial.println("Calibrating accelerometer bias... keep sensor still");

  bias_accel_x = 0.0f;
  bias_accel_y = 0.0f;
  bias_accel_z = 0.0f;

  for (int i = 0; i < MUESTRAS_BIAS; i++) {
    if (bmi.getAccelGyroData(rawSensorData) == BMI160_OK) {
      bias_accel_x += (float)rawSensorData[3] / ACCEL_SCALE;
      bias_accel_y += (float)rawSensorData[4] / ACCEL_SCALE;
      bias_accel_z += (float)rawSensorData[5] / ACCEL_SCALE;
    }
    delay(5);
  }

  bias_accel_x /= (float)MUESTRAS_BIAS;
  bias_accel_y /= (float)MUESTRAS_BIAS;
  bias_accel_z /= (float)MUESTRAS_BIAS;

  if (DEBUG) {
    Serial.printf("Accel bias: %.6f, %.6f, %.6f g\n", bias_accel_x, bias_accel_y, bias_accel_z);
  }
}

// ============================================================================
// READ ALL SENSORS AT 100 Hz
// ============================================================================
void readAllSensors() {
  sensorData.temperature_bme280 = bme.readTemperature();
  sensorData.pressure_hpa = (bme.readPressure() / 100.0f) - 74.0f;
  sensorData.altitude_m = bme.readAltitude(1013.25f);

  ds18b20.requestTemperatures();
  sensorData.temperature_ds18b20 = ds18b20.getTempCByIndex(0);

  if (bmi.getAccelGyroData(rawSensorData) == BMI160_OK) {
    current_accel_x = ((float)rawSensorData[3] / ACCEL_SCALE) * 9.81f;
    current_accel_y = ((float)rawSensorData[4] / ACCEL_SCALE) * 9.81f;
    current_accel_z = ((float)rawSensorData[5] / ACCEL_SCALE) * 9.81f;

    current_accel_x -= bias_accel_x * 9.81f;
    current_accel_y -= bias_accel_y * 9.81f;
    current_accel_z -= bias_accel_z * 9.81f;

    // Remove gravity assuming sensor Z axis is vertical.
    current_accel_z -= 9.81f;
  }

  sensorData.accel_x = current_accel_x;
  sensorData.accel_y = current_accel_y;
  sensorData.accel_z = current_accel_z;
}

// ============================================================================
// INTEGRATE ACCELERATION TO VELOCITY
// ============================================================================
void integrateVelocity() {
  unsigned long now = millis();
  float dt = (now - lastIntegrationTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.01f;
  lastIntegrationTime = now;

  velocity_integral_x += current_accel_x * dt;
  velocity_integral_y += current_accel_y * dt;
  velocity_integral_z += current_accel_z * dt;

  if (fabsf(current_accel_x) < ACCEL_SENSITIVITY_THRESHOLD &&
      fabsf(current_accel_y) < ACCEL_SENSITIVITY_THRESHOLD &&
      fabsf(current_accel_z) < ACCEL_SENSITIVITY_THRESHOLD) {
    velocity_integral_x = 0.0f;
    velocity_integral_y = 0.0f;
    velocity_integral_z = 0.0f;
  }

  sensorData.velocity_x = velocity_integral_x;
  sensorData.velocity_y = velocity_integral_y;
  sensorData.velocity_z = velocity_integral_z;
}

// ============================================================================
// TRANSMIT DATA PACKET VIA ESP-NOW
// ============================================================================
void transmitSensorData() {
  // --- Update transmission metadata ---
  sensorData.packetNumber = packetCounter++;
  sensorData.senderTimestamp = millis();
  
  // --- Send packet via ESP-NOW ---
  esp_err_t result = esp_now_send(
    receiverMAC,
    (uint8_t *)&sensorData,
    sizeof(sensor_packet)
  );
  
  velocity_integral_x = 0.0;
  velocity_integral_y = 0.0;
  velocity_integral_z = 0.0;

  // --- Handle transmission result ---
  if (result != ESP_OK) {
    if (DEBUG) {
      Serial.printf("ESP-NOW send error: %d\n", result);
    }
    transmitFailed = true;
  } else {
    transmitFailed = false;
    digitalWrite(LED_BUILTIN, LOW);  // LED off on success
  }
}

// ============================================================================
// ESP-NOW TRANSMISSION CALLBACK
// ============================================================================
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Called automatically after transmission attempt
  // (Success or failure handled in transmitSensorData function)
  
  if (status == ESP_NOW_SEND_SUCCESS) {
    transmitFailed = false;
  } else {
    transmitFailed = true;
  }
}

// ============================================================================
// LED BLINK INDICATOR FOR TRANSMISSION FAILURES
// ============================================================================
void blinkLEDOnFailure() {
  // If transmission failed, blink LED as visual feedback
  if (transmitFailed) {
    unsigned long now = millis();
    
    if (now - lastBlinkTime > 100) {
      lastBlinkTime = now;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    }
  }
}

// ============================================================================
// DEBUG OUTPUT - CSV HEADER LINE
// ============================================================================
void printDebugHeader() {
  // CSV column header for easy import to spreadsheet/analysis tools
  Serial.println(
    "packetNum,"
    "timestamp_ms,"
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
}

// ============================================================================
// DEBUG OUTPUT - CSV DATA LINE
// ============================================================================
void printDebugData() {
  // Print sensor data as comma-separated values (CSV format)
  Serial.print(sensorData.packetNumber); Serial.print(",");
  Serial.print(sensorData.senderTimestamp); Serial.print(",");
  Serial.print(sensorData.temperature_bme280, 4); Serial.print(",");
  Serial.print(sensorData.temperature_ds18b20, 4); Serial.print(",");
  Serial.print(sensorData.pressure_hpa, 4); Serial.print(",");
  Serial.print(sensorData.altitude_m, 4); Serial.print(",");
  Serial.print(sensorData.velocity_x, 4); Serial.print(",");
  Serial.print(sensorData.velocity_y, 4); Serial.print(",");
  Serial.print(sensorData.velocity_z, 4); Serial.print(",");
  Serial.print(sensorData.accel_x, 4); Serial.print(",");
  Serial.print(sensorData.accel_y, 4); Serial.print(",");
  Serial.println(sensorData.accel_z, 4);
}

