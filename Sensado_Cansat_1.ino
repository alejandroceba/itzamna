#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DallasTemperature.h>
#include <DFRobot_BMI160.h>

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

float umbral = 0.3;  // sensibilidad
// -------- Muestreo --------
unsigned long lastSample = 0;
const unsigned long sampleInterval = 10; // 100 Hz

unsigned long lastPrint = 0;
const unsigned long printInterval = 500;
bool headerPrinted = false;

int16_t sensorData[6];  // [0-2] gyro, [3-5] accel

float ax, ay, az;
float magn;
float velocidad_x = 0;
float velocidad_y = 0;
float velocidad_z = 0;

float bias_ax = 0;
float bias_ay = 0;
float bias_az = 0;

unsigned long t_prev = 0;

#define MUESTRAS_BIAS 200
#define ACC_SCALE 16384.0   // ±2g

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (bmi.softReset() != BMI160_OK) {
    Serial.println("Error reset");
    while(1);
  }

  if (bmi.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("Error I2C");
    while(1);
  }

  Serial.println("Calibrando... NO MOVER");

  for (int i = 0; i < MUESTRAS_BIAS; i++) {

    bmi.getAccelGyroData(sensorData);

    ax = sensorData[3] / ACC_SCALE;
    ay = sensorData[4] / ACC_SCALE;
    az = sensorData[5] / ACC_SCALE;

    bias_ax += ax;
    bias_ay += ay;
    bias_az += az;

    delay(5);
  }

  bias_ax /= MUESTRAS_BIAS;
  bias_ay /= MUESTRAS_BIAS;
  bias_az /= MUESTRAS_BIAS;

  Serial.println("Bias listo");

  t_prev = millis();
  SPI.begin(BME_SCK, BME_MISO, BME_MOSI, BME_CS);
  pinMode(BME_CS, OUTPUT);
  digitalWrite(BME_CS, HIGH);

  if (!bme.begin(BME_CS)) {
    Serial.println("Error BME280");
  } else {
    Serial.println("BME280 OK");
  }

  ds18b20.begin();

  Serial.println("Setup terminado");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample < sampleInterval) return;
  lastSample = now;
  float t_bme, p_bme, alt_bme;
  float t_ds18 = NAN;
   // -------- BME280 (SPI) --------
  t_bme = bme.readTemperature();
  p_bme = bme.readPressure() /100.00 - 74.00; // Pa → hPa
  alt_bme = bme.readAltitude(1013.25);
    // ---------- DS18B20 ----------
  ds18b20.requestTemperatures();
  t_ds18 = ds18b20.getTempCByIndex(0);

   unsigned long t_now = millis();
  float dt = (t_now - t_prev) / 1000.0;
  t_prev = t_now;

  bmi.getAccelGyroData(sensorData);

  ax = sensorData[3] / ACC_SCALE;
  ay = sensorData[4] / ACC_SCALE;
  az = sensorData[5] / ACC_SCALE;

  // convertir a m/s²
  ax *= 9.81;
  ay *= 9.81;
  az *= 9.81;

  // quitar bias
  ax -= bias_ax * 9.81;
  ay -= bias_ay * 9.81;
  az -= bias_az * 9.81;

 // eliminar gravedad (asumiendo Z vertical)
  az -= 9.81;
  velocidad_x += ax * dt;
  velocidad_y += ay * dt;
  velocidad_z += az * dt;

if (abs(ax) < umbral && 
    abs(ay) < umbral && 
    abs(az) < umbral) {

    velocidad_x = 0;
    velocidad_y = 0;
    velocidad_z = 0;
}

  magn = sqrt(pow(velocidad_x,2) + pow(velocidad_y,2)+pow(velocidad_z,2));
  
  //delay(500);

  //Serial.println(F("T_BME\tT_DS18\tP_BME\tALT_BME\tV_x\tV_y\tV_z\tV\ta_x\ta_y\ta_z\n"));
  if(!headerPrinted){
      Serial.println("T_BME\tT_DS18\tP_BME\tALT_BME\tVx\tVy\tVz\tV\tax\tay\taz");
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
  
}
