#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;

const int servoPin = D1;
Servo servo;

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
  Serial.print("Task1 (servo sweep) running on core ");
  Serial.println(xPortGetCoreID());

  // Servo is attached in setup() to avoid attach-time stalls on this board.

  // Continuous sweep: 0 -> 180 -> 0
  const int stepDelayMs = 15; // delay between degree steps for smooth motion
  for (;;) {
    for (int angle = 0; angle <= 180; ++angle) {
      servo.write(angle);
      vTaskDelay(pdMS_TO_TICKS(stepDelayMs));
    }
    for (int angle = 180; angle >= 0; --angle) {
      servo.write(angle);
      vTaskDelay(pdMS_TO_TICKS(stepDelayMs));
    }
  }
}

// Task2 removed — LED blink moved to loop()

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Dual-task servo sweep example starting");

  // Prepare built-in LED for blinking in loop()
  pinMode(LED_BUILTIN, OUTPUT);

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
  // Read altitude every TIEMPO_MUESTREO ms
  unsigned long t_actual = millis();
  if (t_actual - t_anterior >= TIEMPO_MUESTREO) {
    t_anterior = t_actual;
    float altitude = NAN;
    if (bmeOk) {
      altitude = bme.readAltitude(1013.25f);
    }
    // Read and compute RPM
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
}
