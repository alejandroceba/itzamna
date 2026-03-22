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

// ============================================================================
// DEBUGGING & CONFIGURATION
// ============================================================================
#define DEBUG 1                        // Set to 1 for detailed CSV logs, 0 for silent operation

// ESP-NOW & WIFI CONFIGURATION
// ============================================================================
#define WIFI_CHANNEL 6
uint8_t receiverMAC[] = {0xD8, 0x3B, 0xDA, 0x45, 0xCD, 0x24};  // Receiver MAC address

// ============================================================================
// SENSOR PIN CONFIGURATION
// ============================================================================
// COMMENTED OUT - SENSOR TESTING MODE (Using random values)
/*
// DS18B20 (OneWire temperature sensor)
#define DS18_PIN 4

// BMI160 (I2C accelerometer/gyroscope)
#define SDA_PIN 5
#define SCL_PIN 6
// const int8_t BMI160_I2C_ADDR = 0x68;

// BME280 (SPI barometric/temperature sensor)
#define BME_CS 2
#define BME_SCK 7
#define BME_MOSI 9
#define BME_MISO 8
*/

// ============================================================================
// SENSOR TIMING & PARAMETERS
// ============================================================================
#define ACCEL_SENSITIVITY_THRESHOLD 0.3  // m/s² - threshold for velocity reset
#define ACCEL_SCALE 16384.0              // ±2g sensitivity scale factor
#define MUESTRAS_BIAS 200                // Number of samples for bias calibration

// Sampling rates
const unsigned long SAMPLE_INTERVAL_MS = 10;    // 100 Hz sensor reading rate
const unsigned long SEND_INTERVAL_MS = 1000;    // 1 Hz ESP-NOW transmission rate

// ============================================================================
// SENSOR OBJECT INSTANTIATION
// ============================================================================
// COMMENTED OUT - SENSOR TESTING MODE (Using random values)
/*
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);  // SPI mode
OneWire oneWire(DS18_PIN);
DallasTemperature ds18b20(&oneWire);
DFRobot_BMI160 bmi;
*/

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
// GLOBAL VARIABLES - TRANSMISSION
// ============================================================================
sensor_packet sensorData;
uint32_t packetCounter = 0;
volatile bool transmitFailed = false;
unsigned long lastTransmitTime = 0;

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
void blinkLEDOnFailure();
void printDebugHeader();
void printDebugData();

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
}

// ============================================================================
// SENSOR INITIALIZATION
// ============================================================================
void initializeSensors() {
  // SENSOR TESTING MODE - Sensors commented out, using random values
  Serial.println("✓ Running in SENSOR TESTING MODE (random values)");
  
  /*
  // --- Initialize I2C Bus for BMI160 ---
  Serial.println("Initializing I2C bus...");
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Reset BMI160
  if (bmi.softReset() != BMI160_OK) {
    Serial.println("ERROR: BMI160 soft reset failed");
    while (1);  // Halt execution
  }
  
  // Initialize BMI160 via I2C
  if (bmi.I2cInit(BMI160_I2C_ADDR) != BMI160_OK) {
    Serial.println("ERROR: BMI160 I2C initialization failed");
    while (1);  // Halt execution
  }
  
  Serial.println("✓ BMI160 initialized\n");
  
  // --- Initialize SPI Bus and BME280 ---
  Serial.println("Initializing SPI and BME280...");
  SPI.begin(BME_SCK, BME_MISO, BME_MOSI, BME_CS);
  pinMode(BME_CS, OUTPUT);
  digitalWrite(BME_CS, HIGH);
  
  if (!bme.begin(BME_CS)) {
    Serial.println("ERROR: BME280 initialization failed");
    while (1);  // Halt execution
  }
  
  Serial.println("✓ BME280 initialized\n");
  
  // --- Initialize OneWire and DS18B20 ---
  Serial.println("Initializing DS18B20...");
  ds18b20.begin();
  
  Serial.println("✓ DS18B20 initialized\n");
  */
}

// ============================================================================
// ACCELEROMETER BIAS CALIBRATION
// ============================================================================
void calibrateAccelerometer() {
  // SENSOR TESTING MODE - Calibration skipped
  Serial.println("================================");
  Serial.println("ACCELEROMETER CALIBRATION");
  Serial.println("SKIPPED (Sensor Testing Mode)");
  Serial.println("================================\n");
  
  /*
  // Reset bias accumulators
  bias_accel_x = 0.0;
  bias_accel_y = 0.0;
  bias_accel_z = 0.0;
  
  // Collect MUESTRAS_BIAS samples
  for (int i = 0; i < MUESTRAS_BIAS; i++) {
    // Read raw sensor data
    bmi.getAccelGyroData(rawSensorData);
    
    // Convert raw values to g (gravity units)
    float ax_g = rawSensorData[3] / ACCEL_SCALE;
    float ay_g = rawSensorData[4] / ACCEL_SCALE;
    float az_g = rawSensorData[5] / ACCEL_SCALE;
    
    // Convert to m/s²
    float ax_ms2 = ax_g * 9.81;
    float ay_ms2 = ay_g * 9.81;
    float az_ms2 = az_g * 9.81;
    
    // Accumulate bias
    bias_accel_x += ax_ms2;
    bias_accel_y += ay_ms2;
    bias_accel_z += az_ms2;
    
    delay(5);  // Small delay between samples
  }
  
  // Average the bias
  bias_accel_x /= MUESTRAS_BIAS;
  bias_accel_y /= MUESTRAS_BIAS;
  bias_accel_z /= MUESTRAS_BIAS;
  
  if (DEBUG) {
    Serial.printf("Bias X: %.4f m/s²\n", bias_accel_x);
    Serial.printf("Bias Y: %.4f m/s²\n", bias_accel_y);
    Serial.printf("Bias Z: %.4f m/s²\n\n", bias_accel_z);
  }
  */
}

// ============================================================================
// READ ALL SENSORS AT 100 Hz
// ============================================================================
void readAllSensors() {
  // SENSOR TESTING MODE - Generating random values to maintain packet structure
  
  // --- Generate Random Temperature Values ---
  sensorData.temperature_bme280 = random(200, 400) / 10.0;  // 20-40°C
  sensorData.temperature_ds18b20 = random(200, 400) / 10.0;  // 20-40°C
  
  // --- Generate Random Pressure and Altitude ---
  sensorData.pressure_hpa = random(95000, 105000) / 1000.0;  // 950-1050 hPa
  sensorData.altitude_m = random(0, 10000) / 100.0;  // 0-100m
  
  // --- Generate Random Acceleration Values ---
  current_accel_x = (random(-200, 200) / 100.0);  // -2 to +2 m/s²
  current_accel_y = (random(-200, 200) / 100.0);  // -2 to +2 m/s²
  current_accel_z = (random(-200, 200) / 100.0);  // -2 to +2 m/s²
  
  // Store acceleration in packet (same as sensor version)
  sensorData.accel_x = current_accel_x;
  sensorData.accel_y = current_accel_y;
  sensorData.accel_z = current_accel_z;
  
  /*
  // --- Original Sensor Code (Commented for Reference) ---
  // --- Read BME280 (pressure, altitude, temperature) ---
  sensorData.temperature_bme280 = bme.readTemperature();
  sensorData.pressure_hpa = (bme.readPressure() / 100.0) - 74.0;  // Pa → hPa, offset correction
  sensorData.altitude_m = bme.readAltitude(1013.25);
  
  // --- Read DS18B20 (external temperature) ---
  // Note: DS18B20 requires async request/read cycle
  ds18b20.requestTemperatures();
  sensorData.temperature_ds18b20 = ds18b20.getTempCByIndex(0);
  
  // --- Read BMI160 (accelerometer) ---
  bmi.getAccelGyroData(rawSensorData);
  
  // Convert raw accelerometer values to m/s²
  // [3], [4], [5] are accelerometer X, Y, Z
  float ax_g = rawSensorData[3] / ACCEL_SCALE;  // Convert to g
  float ay_g = rawSensorData[4] / ACCEL_SCALE;
  float az_g = rawSensorData[5] / ACCEL_SCALE;
  
  // Convert g to m/s²
  float ax_raw_ms2 = ax_g * 9.81;
  float ay_raw_ms2 = ay_g * 9.81;
  float az_raw_ms2 = az_g * 9.81;
  
  // Apply bias correction (remove calibration offset)
  current_accel_x = ax_raw_ms2 - bias_accel_x;
  current_accel_y = ay_raw_ms2 - bias_accel_y;
  current_accel_z = az_raw_ms2 - bias_accel_z;
  
  // Apply gravity compensation (Z-axis points upward, gravity = -9.81 m/s²)
  // Remove gravitational component so Z-axis true acceleration remains
  current_accel_z -= 9.81;
  
  // Store corrected acceleration in packet
  sensorData.accel_x = current_accel_x;
  sensorData.accel_y = current_accel_y;
  sensorData.accel_z = current_accel_z;
  */
}

// ============================================================================
// INTEGRATE ACCELERATION TO VELOCITY
// ============================================================================
void integrateVelocity() {
  // Calculate time delta since last integration
  unsigned long now = millis();
  float dt = (now - lastIntegrationTime) / 1000.0;  // Convert ms to seconds
  lastIntegrationTime = now;
  
  // Guard against invalid dt
  if (dt <= 0 || dt > 1.0) {
    return;  // Skip if dt is invalid (shouldn't happen with 10ms interval)
  }
  
  // Integrate: velocity += acceleration * dt
  velocity_integral_x += current_accel_x * dt;
  velocity_integral_y += current_accel_y * dt;
  velocity_integral_z += current_accel_z * dt;
  
  // --- Reset velocity to zero if motion below sensitivity threshold ---
  // This prevents drift when the sensor is stationary
  if (abs(current_accel_x) < ACCEL_SENSITIVITY_THRESHOLD &&
      abs(current_accel_y) < ACCEL_SENSITIVITY_THRESHOLD &&
      abs(current_accel_z) < ACCEL_SENSITIVITY_THRESHOLD) {
    
    velocity_integral_x = 0.0;
    velocity_integral_y = 0.0;
    velocity_integral_z = 0.0;
  }
  
  // Store integrated velocity in packet
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
