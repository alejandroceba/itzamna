#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

TaskHandle_t Task1 = NULL;

const int servoPin = D1;
const int readPin = D2;
Servo servo;

#define N_PALAS  4          /* Numero de palas del rotor. */
#define M        0.420      /* Masa total del sistema [kg]. */
#define R        0.22869    /* Radio exterior del rotor [m]. */
#define R0       0.04776    /* Radio donde inicia la pala [m]. */
#define C_CUERDA 0.0204     /* Cuerda de la pala [m]. */
#define RHO      1.225      /* Densidad del aire [kg/m^3]. */
#define G        9.81       /* Gravedad [m/s^2]. */

// BME280
Adafruit_BME280 bme;
bool bmeOk = false;
#define BME280_ADDR_1 0x76
#define BME280_ADDR_2 0x77
#define TIEMPO_MUESTREO 500 // ms
unsigned long t_anterior = 0;
// RPM sensor
#define SENSOR1 D0
#define PULSOS_POR_REV 2
#define DEBOUNCE_US 2000UL
#define N_PROMEDIO 5
volatile unsigned long pulsos1 = 0;
volatile unsigned long t1 = 0;
volatile unsigned long alpha = 0;
float buffer1[N_PROMEDIO];
int indice = 0;

// ISR for RPM pulses (debounced)
void IRAM_ATTR isr1() {
  unsigned long ahora = micros();
  if (ahora - t1 > DEBOUNCE_US) {
    pulsos1++;
    t1 = ahora;
  }
}

float promedio(float *buffer) {
  float suma = 0;
  for (int i = 0; i < N_PROMEDIO; i++) suma += buffer[i];
  return suma / N_PROMEDIO;
}

void Task1code(void *pvParameters) {
  Serial.print("Task1 (servo PWM reader) running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    long duration = pulseIn(readPin, HIGH);

    if (duration > 500 && duration < 2500) {
      int angle = map(duration, 500, 2500, 0, 180);

      alpha = servo.read();

      if (angle < 6) {
        alpha = angle + 1;
        servo.write(alpha);
      }
      else {
        alpha = angle - 1;
        servo.write(alpha);
      }

      Serial.print("Input Pulse: ");
      Serial.print(duration);
      Serial.print(" | Set Angle: ");
      Serial.println(angle);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Dual-task servo + sensor example starting");

  pinMode(readPin, INPUT);

  // Attach servo here (setup runs on core 1 and attach proved reliable)
  Serial.println("Setup: allocating PWM timer and attaching servo");
  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(servoPin, 500, 2500);
  delay(200);
  Serial.println("Setup: servo attached");

  // Initialize I2C and probe BME280
  Wire.begin();
  Serial.println("Scanning I2C for BME280...");
  bool found = false;
  for (uint8_t addr : { (uint8_t)BME280_ADDR_1, (uint8_t)BME280_ADDR_2 }) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C device found at 0x"); Serial.println(addr, HEX);
      found = true;
      if (bme.begin(addr)) {
        Serial.print("BME280 initialized at 0x"); Serial.println(addr, HEX);
        bmeOk = true;
      } else {
        Serial.print("BME280 probe failed at 0x"); Serial.println(addr, HEX);
        bmeOk = false;
      }
      break;
    }
  }
  if (!found) {
    Serial.println("No BME280 found on I2C bus. Continuing without altitude.");
    bmeOk = false;
  }

  // Configure RPM sensor pin and attach ISR
  pinMode(SENSOR1, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR1), isr1, FALLING);
  Serial.println("Attached RPM interrupt on SENSOR1");

  // initialize RPM buffer
  for (int i = 0; i < N_PROMEDIO; i++) buffer1[i] = 0.0;

  xTaskCreatePinnedToCore(
    Task1code,
    "Task1",
    10000,
    NULL,
    1,
    &Task1,
    0
  );
  delay(500);

}

void loop() {
  unsigned long t_actual = millis();
  if (t_actual - t_anterior >= TIEMPO_MUESTREO) {
    t_anterior = t_actual;
    float altitude = NAN;
    if (bmeOk) {
      altitude = bme.readAltitude(1013.25f);
    }

    noInterrupts();
    unsigned long c1 = pulsos1;
    pulsos1 = 0;
    interrupts();

    float rpm1 = ((c1 * 60000.0) / (PULSOS_POR_REV * TIEMPO_MUESTREO)) / 2.0;
    buffer1[indice] = rpm1;
    indice++;
    if (indice >= N_PROMEDIO) indice = 0;
    float rpm1_suave = promedio(buffer1);

    Serial.print("Tiempo(ms):"); Serial.print(t_actual);
    Serial.print("\tAltitud(m):");
    if (bmeOk) Serial.print(altitude, 2); else Serial.print("N/A");
    Serial.print("\tRPM:"); Serial.print(rpm1_suave, 2);
    Serial.println();
  }

  delay(5);
}
