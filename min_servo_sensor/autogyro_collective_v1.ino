#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

TaskHandle_t Task1 = NULL;

const int servo1Pin = D1;
const int servo2Pin = D2;
const int pwmInput1Pin = D3;
const int pwmInput2Pin = D9; // changed: FC -> Xiao input moved to D9
const int pwmMinUs = 1000;
const int pwmMaxUs = 2000;
const float pitchStepDeg = 1.0f;
const float pwmDegPerUs = 180.0f / (pwmMaxUs - pwmMinUs);
const long pitchStepUs = (long)(pitchStepDeg / pwmDegPerUs + 0.5f);

const float DEPLOY_ALTITUDE_M = 200.0f;
const float DEPLOY_MAX_PITCH_DURATION_MS = 2000UL;
const float SPINUP_ALTITUDE_M = 60.0f;
const float LANDING_RAMP_START_M = 60.0f;
const float LANDING_RAMP_END_M = 20.0f;
// Suggested self-sustaining RPM threshold computed from the rotor model (see notebook)
const float RPM_SELF_SUSTAINING_THRESHOLD = 1300.0f;
const float RPM_TREND_EPS = 5.0f;
const float RPM_BIAS_STEP_EPS = 2.0f;

Servo servo1;
Servo servo2;
Servo servo3;

#define N_PALAS  4          /* Numero de palas del rotor. */
#define M        0.420      /* Masa total del sistema [kg]. */
#define R        0.22869    /* Radio exterior del rotor [m]. */
#define R0       0.04776    /* Radio donde inicia la pala [m]. */
#define C_CUERDA 0.0204     /* Cuerda de la pala [m]. */
#define RHO      1.225      /* Densidad del aire [kg/m^3]. */
#define G        9.81       /* Gravedad [m/s^2]. */

Adafruit_BME280 bme;
bool bmeOk = false;
#define BME280_ADDR_1 0x76
#define BME280_ADDR_2 0x77
#define TIEMPO_MUESTREO 500 // ms
unsigned long t_anterior = 0;

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

volatile unsigned long servo1AdjustedPwmUs = 1500;
volatile unsigned long servo2AdjustedPwmUs = 1500;
volatile unsigned long servo3AdjustedPwmUs = 1500;
volatile float servo1AngleDeg = NAN;
volatile float servo2AngleDeg = NAN;
volatile float alpha = NAN;
volatile long collectiveBiasUs = 0;

volatile float initialAltitude = NAN;
volatile float lastAltitude = NAN;
volatile float currentAltitude = NAN;
volatile float lastRpm1Smooth = NAN;
volatile float currentRpm1Smooth = NAN;
unsigned long deployStartMs = 0UL;
bool deployReleaseTriggered = false;
// Servo pin for the third servo (same polarity as servo1).
// Wiring: Xiao -> servo on D8, FC -> Xiao on D9 (input)
const int servo3Pin = D8;

enum MissionState : uint8_t {
  WAIT_DEPLOY = 0,
  DEPLOY = 1,
  SPINUP = 2,
  LANDING = 3
};

volatile MissionState missionState = WAIT_DEPLOY;

unsigned long clampPwmUs(long value) {
  return constrain(value, pwmMinUs, pwmMaxUs);
}

long clampCollectiveBiasUs(long requestedBiasUs, unsigned long width1Us, unsigned long width2Us) {
  long maxPositiveBias = min(pwmMaxUs - (long)width1Us, pwmMaxUs - (long)width2Us);
  long maxNegativeBias = min((long)width1Us - pwmMinUs, (long)width2Us - pwmMinUs);

  if (requestedBiasUs > 0) {
    return min(requestedBiasUs, maxPositiveBias);
  }

  if (requestedBiasUs < 0) {
    return max(requestedBiasUs, -maxNegativeBias);
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
      if (Task1 != NULL) {
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
      if (Task1 != NULL) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(Task1, &higherPriorityTaskWoken);
        if (higherPriorityTaskWoken) {
          portYIELD_FROM_ISR();
        }
      }
    }
  }
}

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

MissionState selectMissionState(float dropHeightM, unsigned long nowMs, float rpmCurrent, bool isDescending) {
  if (dropHeightM < DEPLOY_ALTITUDE_M) {
    return WAIT_DEPLOY;
  }

  // Only trigger deploy if already descending (released from carrier and falling)
  if (!deployReleaseTriggered && isDescending) {
    return DEPLOY;
  }

  if (dropHeightM > SPINUP_ALTITUDE_M) {
    return SPINUP;
  }

  return LANDING;
}

long computeCollectiveBiasUs(MissionState state, float dropHeightM, float rpmNow, float rpmPrev, unsigned long nowMs) {
  const long maxCollectiveBiasUs = 12L * pitchStepUs;
  const long minCollectiveBiasUs = -12L * pitchStepUs;

  if (state == WAIT_DEPLOY) {
    return 0;
  }

  if (state == DEPLOY) {
    if (!deployReleaseTriggered && (nowMs - deployStartMs) >= DEPLOY_MAX_PITCH_DURATION_MS) {
      deployReleaseTriggered = true;
    }
    return maxCollectiveBiasUs;
  }

  if (state == SPINUP) {
    long biasUs = minCollectiveBiasUs;

    if (!isnan(rpmNow) && !isnan(rpmPrev) && rpmNow + RPM_BIAS_STEP_EPS < rpmPrev) {
      biasUs = minCollectiveBiasUs;
    }

    return biasUs;
  }

  float landingSpanM = LANDING_RAMP_START_M - LANDING_RAMP_END_M;
  if (landingSpanM < 1.0f) landingSpanM = 1.0f;

  float progress = (LANDING_RAMP_START_M - dropHeightM) / landingSpanM;
  progress = constrain(progress, 0.0f, 1.0f);

  // Two-stage linear ramp: from minCollective -> 0, then 0 -> maxCollective
  float mid = 0.5f;
  long targetBiasUs = 0;
  if (progress <= mid) {
    float s = (mid > 0.0f) ? (progress / mid) : 1.0f;
    // interpolate from minCollectiveBiasUs to 0
    float val = (float)minCollectiveBiasUs + s * (0.0f - (float)minCollectiveBiasUs);
    targetBiasUs = (long)lroundf(val);
  } else {
    float s = (1.0f - mid > 0.0f) ? ((progress - mid) / (1.0f - mid)) : 1.0f;
    // interpolate from 0 to maxCollectiveBiasUs
    float val = 0.0f + s * (float)maxCollectiveBiasUs;
    targetBiasUs = (long)lroundf(val);
  }

  // Safety: if RPM is dropping rapidly, avoid raising positive bias
  if (!isnan(rpmNow) && !isnan(rpmPrev) && rpmNow + RPM_TREND_EPS < rpmPrev) {
    targetBiasUs = min(targetBiasUs, 0L);
  }

  return constrain(targetBiasUs, minCollectiveBiasUs, maxCollectiveBiasUs);
}

void Task1code(void *pvParameters) {
  Serial.print("Task1 (collective bias) running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    unsigned long width1Us = pwm1WidthUs;
    unsigned long width2Us = pwm2WidthUs;
    float altitude = currentAltitude;
    float rpmNow = currentRpm1Smooth;
    float rpmPrev = lastRpm1Smooth;
    MissionState state = missionState;
    unsigned long nowMs = millis();

    float dropHeightM = NAN;
    if (!isnan(initialAltitude) && !isnan(altitude)) {
      dropHeightM = initialAltitude - altitude;
    }

    long requestedBiasUs = 0;
    if (!isnan(dropHeightM)) {
      requestedBiasUs = computeCollectiveBiasUs(state, dropHeightM, rpmNow, rpmPrev, nowMs);
    }

    collectiveBiasUs = clampCollectiveBiasUs(requestedBiasUs, width1Us, width2Us);

    servo1AdjustedPwmUs = clampPwmUs((long)width1Us + collectiveBiasUs);
    servo2AdjustedPwmUs = clampPwmUs((long)width2Us + collectiveBiasUs);

    float servo1OutDeg = pwmToDegrees(servo1AdjustedPwmUs);
    float servo2OutDeg = pwmToDegrees(servo2AdjustedPwmUs);

    servo1AngleDeg = servo1OutDeg;
    servo2AngleDeg = servo2OutDeg;
    alpha = (servo1OutDeg + servo2OutDeg) / 2.0f;
    alpha = constrain(alpha, 0.0f, 180.0f);

    servo1.writeMicroseconds(servo1AdjustedPwmUs);
    servo2.writeMicroseconds(servo2AdjustedPwmUs);

    Serial.print("Task1 state=");
    Serial.print((int)state);
    Serial.print(" drop=");
    if (!isnan(dropHeightM)) Serial.print(dropHeightM, 2); else Serial.print("N/A");
    Serial.print("m | rpm=");
    if (!isnan(rpmNow)) Serial.print(rpmNow, 2); else Serial.print("N/A");
    Serial.print(" | bias=");
    Serial.print(collectiveBiasUs);
    Serial.print(" us | alpha=");
    Serial.print(alpha, 1);
    Serial.print(" deg | servo1=");
    Serial.print(servo1AdjustedPwmUs);
    Serial.print(" us | servo2=");
    Serial.print(servo2AdjustedPwmUs);
    Serial.println(" us");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Autogyro collective control v1 starting");

  pinMode(pwmInput1Pin, INPUT);
  pinMode(pwmInput2Pin, INPUT);

  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo1.attach(servo1Pin, pwmMinUs, pwmMaxUs);
  servo2.attach(servo2Pin, pwmMinUs, pwmMaxUs);
  servo3.attach(servo3Pin, pwmMinUs, pwmMaxUs);
  delay(200);

  // Start with centered servos to ensure defined outputs before deploy
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);

  attachInterrupt(digitalPinToInterrupt(pwmInput1Pin), isrPwm1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInput2Pin), isrPwm2, CHANGE);

  Wire.begin();
  bool found = false;
  for (uint8_t addr : { (uint8_t)BME280_ADDR_1, (uint8_t)BME280_ADDR_2 }) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      found = true;
      if (bme.begin(addr)) {
        bmeOk = true;
      } else {
        bmeOk = false;
      }
      break;
    }
  }
  if (!found) {
    bmeOk = false;
  }

  pinMode(SENSOR1, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR1), isr1, FALLING);

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
      currentAltitude = altitude;
    }

    noInterrupts();
    unsigned long c1 = pulsos1;
    pulsos1 = 0;
    interrupts();

    float rpm1 = ((c1 * 60000.0f) / (PULSOS_POR_REV * TIEMPO_MUESTREO)) / 2.0f;
    buffer1[indice] = rpm1;
    indice++;
    if (indice >= N_PROMEDIO) indice = 0;

    lastRpm1Smooth = currentRpm1Smooth;
    currentRpm1Smooth = promedio(buffer1);

    if (bmeOk && !isnan(altitude)) {
      if (isnan(initialAltitude)) {
        initialAltitude = altitude;
        deployStartMs = t_actual;
      }

      float dropHeightM = initialAltitude - altitude;
      float altDelta = NAN;
      bool isDescending = false;
      if (!isnan(lastAltitude)) {
        altDelta = altitude - lastAltitude;
        isDescending = altDelta < 0.0f;
      }
      missionState = selectMissionState(dropHeightM, t_actual, currentRpm1Smooth, isDescending);

      Serial.print("Tiempo(ms):"); Serial.print(t_actual);
      Serial.print("\tAltitud(m):"); Serial.print(altitude, 2);
      Serial.print("\tDrop(m):"); Serial.print(dropHeightM, 2);
      Serial.print("\tRPM:"); Serial.print(currentRpm1Smooth, 2);
      Serial.print("\tState:"); Serial.print((int)missionState);
      Serial.print("\tBiasUs:"); Serial.println(collectiveBiasUs);
      lastAltitude = altitude;
    } else {
      Serial.print("Tiempo(ms):"); Serial.print(t_actual);
      Serial.print("\tAltitud(m):N/A\tRPM:"); Serial.print(currentRpm1Smooth, 2);
      Serial.println();
    }

    if (Task1 != NULL) {
      xTaskNotifyGive(Task1);
    }
  }

  delay(5);
}