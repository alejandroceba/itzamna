#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DallasTemperature.h>
#include <DFRobot_BMI160.h>
#include <Math.h>
#define DS18_PIN 4

// -------- I2C BMI160 --------
#define SDA_PIN 5
#define SCL_PIN 6

// -------- SPI BME280 --------
#define BME_CS   2
#define BME_SCK  7
#define BME_MOSI 9
#define BME_MISO 8

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // Constructor SPI
OneWire oneWire(DS18_PIN);
DallasTemperature ds18b20(&oneWire);
DFRobot_BMI160 bmi;

const int8_t i2c_addr = 0x68;
// -------- Muestreo --------
unsigned long lastSample = 0;
const unsigned long sampleInterval = 10; // 100 Hz
float dt = sampleInterval / 1000.0;

unsigned long lastPrint = 0;
const unsigned long printInterval = 500;
bool headerPrinted = false;

int16_t sensorData[6];  // [0-2] gyro, [3-5] accel

unsigned long t_prev = 0;
float  velocidad_x=0;
float velocidad_y=0;
float  velocidad_z=0;
float magn=0;

#define MUESTRAS_BIAS 500
#define ACC_SCALE 16384.0   // ±2g
#define GYRO_SCALE 131.2   // para ±250°/s
float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  autoCalibrateAccelerometer();
  Serial.println("Calibrando... NO MOVER");

 if (bmi.softReset() != BMI160_OK) {
    Serial.println("Error reset");
    while(1);
  }

  if (bmi.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("Error I2C");
    while(1);
  }
  for (int i = 0; i < MUESTRAS_BIAS; i++) {
    bmi.getAccelGyroData(sensorData);
  
    gx_bias += sensorData[0];
    gy_bias += sensorData[1];
    gz_bias += sensorData[2];
    ax_bias += sensorData[3];
    ay_bias += sensorData[4];
    az_bias += sensorData[5];
    delay(5);
  }
  ax_bias /= MUESTRAS_BIAS;
  ay_bias /= MUESTRAS_BIAS;
  az_bias /= MUESTRAS_BIAS;
  gx_bias /= MUESTRAS_BIAS;
  gy_bias /= MUESTRAS_BIAS;
  gz_bias /= MUESTRAS_BIAS;
  Serial.println("Calibración completa");
  //BME280 
  SPI.begin(BME_SCK, BME_MISO, BME_MOSI, BME_CS);
  pinMode(BME_CS, OUTPUT);
  digitalWrite(BME_CS, HIGH);

  if (!bme.begin(BME_CS)) {
    Serial.println("Error BME280");
  } else {
    Serial.println("BME280 OK");
  }
  //DS18B20
  ds18b20.begin();

  Serial.println("Setup terminado");
  lastSample = millis();
}

void loop() {
  unsigned long t_now = millis();
  if (t_now - lastSample < sampleInterval) return;
  lastSample = t_now;
  float t_bme, p_bme, alt_bme;
  float t_ds18 = NAN;
   // -------- BME280 (SPI) --------
  t_bme = bme.readTemperature();
  p_bme = bme.readPressure() /100.00 - 74.00; // Pa → hPa
  alt_bme = bme.readAltitude(1013.25);
    // ---------- DS18B20 ----------
  ds18b20.requestTemperatures();
  t_ds18 = ds18b20.getTempCByIndex(0);
  
// -------- BMI160 --------
unsigned long now = millis();
  bmi.getAccelGyroData(sensorData);

  float ax = (sensorData[3] - ax_bias) * (9.81 / ACC_SCALE);
  float ay = (sensorData[4] - ay_bias) * (9.81 / ACC_SCALE);
  float az = (sensorData[5]) * (9.81 / ACC_SCALE);

  // Convertir giroscopio a °/s
  float gx = (sensorData[0] - gx_bias) / GYRO_SCALE;
  float gy = (sensorData[1] - gy_bias) / GYRO_SCALE;
  float gz = (sensorData[2] - gz_bias) / GYRO_SCALE;

  velocidad_x += ax * dt;
  velocidad_y += ay * dt;
  velocidad_z += az * dt;
  magn = sqrt(pow(velocidad_x,2) + pow(velocidad_y,2)+ pow(velocidad_z,2));

   static unsigned long lastPrint = 0;
  if (now - lastPrint > 1000) {
    lastPrint = now;
    if(!headerPrinted){
      Serial.println("Vx\tVy\tVz\tV\tax\tay\taz");
      headerPrinted = true;
    }
  Serial.print(t_bme,4); Serial.print('\t');
  Serial.print(t_ds18,4);Serial.print('\t');
  Serial.print(p_bme,4); Serial.print('\t');
  Serial.print(alt_bme,4); Serial.print('\t');
  Serial.print(velocidad_x, 4); Serial.print('\t');
  Serial.print(velocidad_y, 4); Serial.print('\t');
  Serial.print(velocidad_z, 4); Serial.print('\t');
  Serial.print(magn,4); Serial.print('\t');
  Serial.print(ax,4); Serial.print('\t');
  Serial.print(ay,4); Serial.print('\t');
  Serial.println(az,4); 
    // Opcional: reset de velocidades si quieres evitar drift acumulado
  velocidad_x =0; velocidad_y =0; velocidad_z =0;
  }
}

void autoCalibrateAccelerometer() { // Configure accelerometer for auto-calibration 
Wire.beginTransmission(i2c_addr); 
Wire.write(0x7E); // Command register 
Wire.write(0x37); // Start accelerometer offset calibration 
Wire.endTransmission(); delay(100); // Wait for calibration to complete
delay(1000); 
Serial.println("Accelerometer Auto-Calibration Complete"); 
}

