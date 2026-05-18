#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

TaskHandle_t Task1 = NULL;

const int servo1Pin = D1;
const int servo2Pin = D2;
const int pwmInput1Pin = D3;
const int pwmInput2Pin = D8;
const int pwmMinUs = 1000;
const int pwmMaxUs = 2000;
const float pitchStepDeg = 1.0f;
const double VOBJ = 0.1;
const double VOBJ_EPS = 0.001;
const float pwmDegPerUs = 180.0f / (pwmMaxUs - pwmMinUs);
const long pitchStepUs = (long)(pitchStepDeg / pwmDegPerUs + 0.5f);
const float ALTITUDE_THRESHOLD_M = 1.0; // configurable altitude threshold in meters

Servo servo1;
Servo servo2;

#define N_PALAS  4          /* Numero de palas del rotor. */
#define M        0.420      /* Masa total del sistema [kg]. */
#define R        0.22869    /* Radio exterior del rotor [m]. */
#define R0       0.04776    /* Radio donde inicia la pala [m]. */
#define C_CUERDA 0.0204     /* Cuerda de la pala [m]. */
#define RHO      1.225      /* Densidad del aire [kg/m^3]. */
#define G        9.81       /* Gravedad [m/s^2]. */

const double R3r0 = R * R * R - R0 * R0 * R0;
const double R4r0 = R * R * R * R - R0 * R0 * R0 * R0;
const double W_SYSTEM = M * G;
const double K_eq = (16.0 * N_PALAS / 54.0) * RHO * C_CUERDA
                    * (R3r0 * R3r0 * R3r0) / (R4r0 * R4r0);

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

volatile unsigned long pwm1RiseUs = 0;
volatile unsigned long pwm2RiseUs = 0;
volatile unsigned long pwm1WidthUs = 1500;
volatile unsigned long pwm2WidthUs = 1500;
volatile bool task1Active = false;
volatile unsigned long servo1AdjustedPwmUs = 1500;
volatile unsigned long servo2AdjustedPwmUs = 1500;
volatile float servo1AngleDeg = NAN;
volatile float servo2AngleDeg = NAN;
volatile float alpha = NAN;
volatile float liftCoeffCL = NAN;
volatile float dragCoeffCD = NAN;
volatile double equilibriumVdEq = NAN;
volatile long pitchOffsetUs = 0;

float initialAltitude = NAN;
float lastAltitude = NAN;

unsigned long clampPwmUs(long value) {
  return constrain(value, pwmMinUs, pwmMaxUs);
}

long clampPitchOffsetUs(long requestedOffsetUs, unsigned long width1Us, unsigned long width2Us) {
  long maxPositiveOffset = min((long)width1Us - pwmMinUs, pwmMaxUs - (long)width2Us);
  long maxNegativeOffset = min(pwmMaxUs - (long)width1Us, (long)width2Us - pwmMinUs);

  if (requestedOffsetUs > 0) {
    return min(requestedOffsetUs, maxPositiveOffset);
  }

  if (requestedOffsetUs < 0) {
    return max(requestedOffsetUs, -maxNegativeOffset);
  }

  return 0;
}

float pwmToDegrees(unsigned long pulseWidthUs) {
  pulseWidthUs = constrain(pulseWidthUs, (unsigned long)pwmMinUs, (unsigned long)pwmMaxUs);
  return (pulseWidthUs - pwmMinUs) * pwmDegPerUs;
}

void IRAM_ATTR isrPwm1() {
  unsigned long nowUs = micros();
  if (digitalRead(pwmInput1Pin) == HIGH) {
    pwm1RiseUs = nowUs;
  } else {
    unsigned long widthUs = nowUs - pwm1RiseUs;
    if (widthUs >= pwmMinUs && widthUs <= pwmMaxUs) {
      pwm1WidthUs = widthUs;
      // Only notify Task1 when it's allowed to run to avoid unnecessary wakeups
      if (Task1 != NULL && task1Active) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(Task1, &higherPriorityTaskWoken);
        if (higherPriorityTaskWoken) {
          portYIELD_FROM_ISR();
        }
      }
    }
  }
}

void IRAM_ATTR isrPwm2() {
  unsigned long nowUs = micros();
  if (digitalRead(pwmInput2Pin) == HIGH) {
    pwm2RiseUs = nowUs;
  } else {
    unsigned long widthUs = nowUs - pwm2RiseUs;
    if (widthUs >= pwmMinUs && widthUs <= pwmMaxUs) {
      pwm2WidthUs = widthUs;
      // Only notify Task1 when it's allowed to run to avoid unnecessary wakeups
      if (Task1 != NULL && task1Active) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(Task1, &higherPriorityTaskWoken);
        if (higherPriorityTaskWoken) {
          portYIELD_FROM_ISR();
        }
      }
    }
  }
}

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
  Serial.print("Task1 (PWM-to-servo) running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    unsigned long width1Us = pwm1WidthUs;
    unsigned long width2Us = pwm2WidthUs;

    float alphaBeforeOffset = alpha;
    if (!isnan(alphaBeforeOffset)) {
      liftCoeffCL = 2.0f * M_PI * (alphaBeforeOffset * M_PI / 180.0f);
      dragCoeffCD = 0.008f + 0.01f * liftCoeffCL * liftCoeffCL;
      equilibriumVdEq = sqrt(W_SYSTEM / (K_eq * ((double)liftCoeffCL * liftCoeffCL * liftCoeffCL)
                              / ((double)dragCoeffCD * dragCoeffCD)));
    } else {
      liftCoeffCL = NAN;
      dragCoeffCD = NAN;
      equilibriumVdEq = NAN;
    }

    const char *accion = "sin ajuste";
    if (!isnan(equilibriumVdEq)) {
      if (equilibriumVdEq - VOBJ > VOBJ_EPS) {
        accion = "aumentar pitch";
        pitchOffsetUs = pitchStepUs;
      } else {
        accion = "disminuir pitch";
        pitchOffsetUs = -pitchStepUs;
      }
    } else {
      pitchOffsetUs = 0;
    }

    pitchOffsetUs = clampPitchOffsetUs(pitchOffsetUs, width1Us, width2Us);

    servo1AdjustedPwmUs = clampPwmUs((long)width1Us - pitchOffsetUs);
    servo2AdjustedPwmUs = clampPwmUs((long)width2Us + pitchOffsetUs);
    float servo1OutDeg = pwmToDegrees(servo1AdjustedPwmUs);
    float servo2OutDeg = pwmToDegrees(servo2AdjustedPwmUs);

    servo1AngleDeg = servo1OutDeg;
    servo2AngleDeg = servo2OutDeg;
    alpha = (servo1OutDeg + servo2OutDeg) / 2.0f;
    alpha = constrain(alpha, 0.0f, 180.0f);

    servo1.writeMicroseconds(servo1AdjustedPwmUs);
    servo2.writeMicroseconds(servo2AdjustedPwmUs);

    Serial.print("Task1 pulse1=");
    Serial.print(width1Us);
    Serial.print("us -> servo1=");
    Serial.print(servo1AdjustedPwmUs);
    Serial.print(" us (");
    Serial.print(servo1OutDeg, 1);
    Serial.print(" deg) | pulse2=");
    Serial.print(width2Us);
    Serial.print("us -> servo2=");
    Serial.print(servo2AdjustedPwmUs);
    Serial.print(" us (");
    Serial.print(servo2OutDeg, 1);
    Serial.print(" deg) | alpha=");
    Serial.print(alpha, 1);
    Serial.print(" deg | CL=");
    Serial.print(liftCoeffCL, 4);
    Serial.print(" | CD=");
    Serial.print(dragCoeffCD, 5);
    Serial.print(" | Vd_eq=");
    Serial.print(equilibriumVdEq, 3);
    Serial.print(" m/s | accion=");
    Serial.print(accion);
    Serial.print(" | offset=");
    Serial.print(pitchOffsetUs);
    Serial.println(" us");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Dual-task servo + sensor example starting");

  pinMode(pwmInput1Pin, INPUT);
  pinMode(pwmInput2Pin, INPUT);

  Serial.println("Setup: allocating PWM timer and attaching servos");
  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(servo1Pin, pwmMinUs, pwmMaxUs);
  servo2.attach(servo2Pin, pwmMinUs, pwmMaxUs);
  delay(200);
  Serial.println("Setup: servos attached");

  attachInterrupt(digitalPinToInterrupt(pwmInput1Pin), isrPwm1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInput2Pin), isrPwm2, CHANGE);
  Serial.println("Setup: PWM input interrupts attached");

  // Initialize I2C and probe BME280.
  // Wire.begin() uses the board's default SDA/SCL pins unless overridden.
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
    4096,
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

    // Altitude gating logic
    if (bmeOk && !isnan(altitude)) {
      if (isnan(initialAltitude)) {
        initialAltitude = altitude; // store initial altitude at power-on
      }

      float altDiffFromInit = altitude - initialAltitude; // measured - initial
      float altDelta = NAN;
      if (!isnan(lastAltitude)) altDelta = altitude - lastAltitude;

      // Condition: below configured threshold from initial AND descending
      if (altDiffFromInit < ALTITUDE_THRESHOLD_M && !isnan(altDelta) && altDelta < 0.0) {
        // Force servos to fixed positions, then enable Task1 to start offset processing
        servo1.writeMicroseconds(1200);
        servo2.writeMicroseconds(1800);
        task1Active = true;
        if (Task1 != NULL) xTaskNotifyGive(Task1);
      } else {
        // While altitude <200 but ascending, or altitude >=200, disable Task1
        task1Active = false;
      }

      lastAltitude = altitude;
    }

    Serial.print("Tiempo(ms):"); Serial.print(t_actual);
    Serial.print("\tAltitud(m):");
    if (bmeOk) Serial.print(altitude, 2); else Serial.print("N/A");
    Serial.print("\tRPM:"); Serial.print(rpm1_suave, 2);
    Serial.println();
  }

  delay(5);
}
