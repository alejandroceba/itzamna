#include <Arduino.h>
#include <ESP32Servo.h>

TaskHandle_t Task1 = NULL;

const int servo1Pin = D1;
const int servo2Pin = D2;
const int servo3Pin = D3;
const int pwmInput1Pin = D8;
const int pwmInput2Pin = D9;
const int pwmInput3Pin = D10;
const int pwmMinUs = 1000;
const int pwmMaxUs = 2000;
const float pitchStepDeg = 1.0f;
const float pwmDegPerUs = 180.0f / (pwmMaxUs - pwmMinUs);
const long pitchStepUs = (long)(pitchStepDeg / pwmDegPerUs + 0.5f);
const float DEPLOY_AGL_THRESHOLD_M = 200.0f;
const float DEPLOY_MAX_PITCH_DURATION_MS = 2000UL;
const float SPINUP_AGL_M = 60.0f;
const float LANDING_RAMP_START_AGL_M = 60.0f;
const float LANDING_RAMP_END_AGL_M = 20.0f;
const float RPM_TREND_EPS = 5.0f;
const float RPM_BIAS_STEP_EPS = 2.0f;
const float OPTIMAL_RPM_ALPHA_CONSTANT = 4810.0544f;
const float CONTROL_STAGE_MIN_HEIGHT_M = 20.0f;
const float CONTROL_STAGE_MAX_HEIGHT_M = 200.0f;
const float FINAL_DESCENT_RATE_MPS = 0.1f;
const float MID_DESCENT_RATE_MPS = 12.0f;
#define TIEMPO_MUESTREO 500 // ms
unsigned long t_anterior = 0;
#define SENSOR1 D0
#define N_PROMEDIO 5
float buffer1[N_PROMEDIO];

Servo servo1;
Servo servo2;
Servo servo3;

volatile unsigned long pwm1RiseUs = 0;
volatile unsigned long pwm2RiseUs = 0;
volatile unsigned long pwm3RiseUs = 0;
volatile unsigned long pwm1WidthUs = 1500;
volatile unsigned long pwm2WidthUs = 1500;
volatile unsigned long pwm3WidthUs = 1500;
volatile unsigned long servo1AdjustedPwmUs = 1500;
volatile unsigned long servo2AdjustedPwmUs = 1500;
volatile unsigned long servo3AdjustedPwmUs = 1500;
volatile float servo1AngleDeg = NAN;
volatile float servo2AngleDeg = NAN;
volatile float servo3AngleDeg = NAN;

volatile float alpha = 12.0f;
volatile long collectiveBiasUs = 0;

volatile float groundAltitude = 0.0f;
volatile float lastAltitude = NAN;
volatile float currentAltitude = NAN;
volatile float lastRpm1Smooth = NAN;
volatile float currentRpm1Smooth = NAN;
volatile bool task1Active = false;
volatile unsigned long lastTaskRunMs = 0UL;
const unsigned long TASK_MIN_INTERVAL_MS = 50UL;
unsigned long deployStartMs = 0UL;
volatile bool deployAltitudeSeenAbove = false;
volatile bool deployWindowActive = false;
bool deployReleaseTriggered = false;
volatile bool postDeployMinPitchPending = false;
volatile bool deployControlEntered = false;
volatile bool deployControlStalled = false;
volatile bool deployControlWaitingAltitude = false;
volatile unsigned long deployControlArmedMs = 0UL;
const unsigned long collectiveCenterUs = (pwmMinUs + pwmMaxUs) / 2;
const unsigned long DEPLOY_CONTROL_STALL_MS = 1500UL;

enum MissionState : uint8_t {
  WAIT_FOR_RELEASE = 0,
  DEPLOY_PULSE = 1,
  UPPER_SWEEP = 2,
  LOWER_SWEEP = 3
};
volatile MissionState missionState = WAIT_FOR_RELEASE;

unsigned long clampPwmUs(long value) {
  return constrain(value, pwmMinUs, pwmMaxUs);
}

long clampCollectiveBiasUs(long requestedBiasUs, unsigned long width1Us, unsigned long width2Us) {
  long maxPositiveBias = min(pwmMaxUs - (long)width1Us, pwmMaxUs - (long)width2Us);
  long maxNegativeBias = min((long)width1Us - pwmMinUs, (long)width2Us - pwmMinUs);
  if (requestedBiasUs > 0) return min(requestedBiasUs, maxPositiveBias);
  if (requestedBiasUs < 0) return max(requestedBiasUs, -maxNegativeBias);
  return 0;
}

float pwmToDegrees(unsigned long pulseWidthUs) {
  pulseWidthUs = constrain(pulseWidthUs, (unsigned long)pwmMinUs, (unsigned long)pwmMaxUs);
  return (pulseWidthUs - pwmMinUs) * pwmDegPerUs;
}

float computeLegacySweepAlphaDeg(float heightAboveGroundM) {
  float sweepProgress = (CONTROL_STAGE_MAX_HEIGHT_M - heightAboveGroundM) / (CONTROL_STAGE_MAX_HEIGHT_M - CONTROL_STAGE_MIN_HEIGHT_M);
  sweepProgress = constrain(sweepProgress, 0.0f, 1.0f);
  if (heightAboveGroundM > SPINUP_AGL_M) {
    float highBandProgress = (CONTROL_STAGE_MAX_HEIGHT_M - heightAboveGroundM) / (CONTROL_STAGE_MAX_HEIGHT_M - SPINUP_AGL_M);
    highBandProgress = constrain(highBandProgress, 0.0f, 1.0f);
    return -12.0f * (1.0f - highBandProgress);
  }
  float lowBandProgress = (SPINUP_AGL_M - heightAboveGroundM) / (SPINUP_AGL_M - CONTROL_STAGE_MIN_HEIGHT_M);
  lowBandProgress = constrain(lowBandProgress, 0.0f, 1.0f);
  return 12.0f * lowBandProgress;
}

float computeOptimalRpmFromAlphaDeg(float alphaDeg) {
  if (isnan(alphaDeg)) return NAN;
  if (alphaDeg <= 0.0f) return 1300.0f;
  return OPTIMAL_RPM_ALPHA_CONSTANT / sqrtf(alphaDeg);
}

void IRAM_ATTR isrPwm1() {
  unsigned long nowUs = micros();
  if (digitalRead(pwmInput1Pin) == HIGH) {
    pwm1RiseUs = nowUs;
  } else {
    unsigned long widthUs = nowUs - pwm1RiseUs;
    if (widthUs >= pwmMinUs && widthUs <= pwmMaxUs) {
      pwm1WidthUs = widthUs;
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

void IRAM_ATTR isrPwm3() {
  unsigned long nowUs = micros();
  if (digitalRead(pwmInput3Pin) == HIGH) {
    pwm3RiseUs = nowUs;
  } else {
    unsigned long widthUs = nowUs - pwm3RiseUs;
    if (widthUs >= pwmMinUs && widthUs <= pwmMaxUs) {
      pwm3WidthUs = widthUs;
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

MissionState selectMissionState(float heightAboveGroundM, unsigned long nowMs, float rpmCurrent, bool isDescending) {
  if (!deployWindowActive) return WAIT_FOR_RELEASE;
  if (deployWindowActive && deployStartMs != 0UL) {
    if ((nowMs - deployStartMs) < (unsigned long)DEPLOY_MAX_PITCH_DURATION_MS) {
      return DEPLOY_PULSE;
    }
    if (!deployReleaseTriggered) {
      deployReleaseTriggered = true;
      postDeployMinPitchPending = true;
      deployControlArmedMs = nowMs;
      deployControlWaitingAltitude = true;
    }
  }
  if (heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M) return LOWER_SWEEP;
  if (heightAboveGroundM <= SPINUP_AGL_M) return LOWER_SWEEP;
  if (heightAboveGroundM <= CONTROL_STAGE_MAX_HEIGHT_M) return UPPER_SWEEP;
  return WAIT_FOR_RELEASE;
}

long computeLegacyCollectiveBiasUs(MissionState state, float heightAboveGroundM, float rpmNow, float rpmPrev, unsigned long nowMs) {
  const long maxCollectiveBiasUs = 12L * pitchStepUs;
  const long minCollectiveBiasUs = -12L * pitchStepUs;
  if (state == WAIT_FOR_RELEASE) return 0;
  if (state == DEPLOY_PULSE) return maxCollectiveBiasUs;
  if (state == UPPER_SWEEP) {
    long biasUs = minCollectiveBiasUs;
    if (!isnan(rpmNow) && !isnan(rpmPrev) && rpmNow + RPM_BIAS_STEP_EPS < rpmPrev) biasUs = minCollectiveBiasUs;
    return biasUs;
  }

  float landingSpanM = LANDING_RAMP_START_AGL_M - LANDING_RAMP_END_AGL_M;
  if (landingSpanM < 1.0f) landingSpanM = 1.0f;
  float progress = (LANDING_RAMP_START_AGL_M - heightAboveGroundM) / landingSpanM;
  progress = constrain(progress, 0.0f, 1.0f);
  float mid = 0.5f;
  long targetBiasUs = 0;
  if (progress <= mid) {
    float s = (mid > 0.0f) ? (progress / mid) : 1.0f;
    float val = (float)minCollectiveBiasUs + s * (0.0f - (float)minCollectiveBiasUs);
    targetBiasUs = (long)lroundf(val);
  } else {
    float s = (1.0f - mid > 0.0f) ? ((progress - mid) / (1.0f - mid)) : 1.0f;
    float val = 0.0f + s * (float)maxCollectiveBiasUs;
    targetBiasUs = (long)lroundf(val);
  }
  if (!isnan(rpmNow) && !isnan(rpmPrev) && rpmNow + RPM_TREND_EPS < rpmPrev) {
    targetBiasUs = min(targetBiasUs, 0L);
  }
  return constrain(targetBiasUs, minCollectiveBiasUs, maxCollectiveBiasUs);
}

float computeAdaptiveAlphaDeg(float heightAboveGroundM, float descentRateMps, float rpmNow, float alphaBaseCurveDeg) {
  if (isnan(heightAboveGroundM)) return NAN;
  if (heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M) {
    if (isnan(rpmNow) || rpmNow < 1300.0f) return NAN;
    return alphaBaseCurveDeg;
  }
  if (heightAboveGroundM > CONTROL_STAGE_MAX_HEIGHT_M) return NAN;
  float rpmOptimal = computeOptimalRpmFromAlphaDeg(alphaBaseCurveDeg);
  if (isnan(rpmNow) || isnan(rpmOptimal) || isnan(descentRateMps)) return NAN;
  
  if (rpmNow < rpmOptimal) {
    if (descentRateMps > MID_DESCENT_RATE_MPS) return constrain(alphaBaseCurveDeg - 1.5f, -12.0f, 12.0f);
    return constrain(alphaBaseCurveDeg - 1.0f, -12.0f, 12.0f);
  }
  if (rpmNow > rpmOptimal && descentRateMps > MID_DESCENT_RATE_MPS) {
    return constrain(alphaBaseCurveDeg + 1.5f, -12.0f, 12.0f);
  }
  return alphaBaseCurveDeg;
}

void updateSyntheticProfile(unsigned long nowMs) {
  const float climbRateMps = 5.0f;
  const unsigned long climbEndMs = 44000UL;
  const unsigned long holdEndMs = 46000UL;

  if (nowMs < climbEndMs) {
    currentAltitude = climbRateMps * (nowMs / 1000.0f);
  } else if (nowMs < holdEndMs) {
    currentAltitude = DEPLOY_AGL_THRESHOLD_M + 20.0f;
  } else {
    float baseDescentRate = 10.0f; 
    if (!isnan(alpha)) {
      baseDescentRate -= alpha * 0.45f; 
    }
    baseDescentRate += sinf(nowMs / 1200.0f) * 2.5f;
    if (baseDescentRate < 1.5f) baseDescentRate = 1.5f;

    if (!isnan(lastAltitude)) {
      currentAltitude = lastAltitude - (baseDescentRate * 0.5f);
    } else {
      currentAltitude = (DEPLOY_AGL_THRESHOLD_M + 20.0f) - (baseDescentRate * 0.5f);
    }
    if (currentAltitude < 0.0f) currentAltitude = 0.0f;
  }

  float heightAboveGroundM = currentAltitude - groundAltitude;
  float descentRate = NAN;
  if (!isnan(lastAltitude)) {
    descentRate = (lastAltitude - currentAltitude) / 0.5f;
  }

  if (!deployWindowActive) {
    currentRpm1Smooth = 0.0f;
    return;
  }

  float deployElapsedS = 0.0f;
  if (deployStartMs != 0UL && nowMs >= deployStartMs) {
    deployElapsedS = (nowMs - deployStartMs) / 1000.0f;
  }

  float rpmTarget = 0.0f;
  if (deployElapsedS < 2.0f) {
    rpmTarget = 650.0f * (deployElapsedS / 2.0f);
  } else {
    rpmTarget = 1300.0f;
    if (!isnan(alpha)) {
      rpmTarget = computeOptimalRpmFromAlphaDeg(alpha);
      if (isnan(rpmTarget)) rpmTarget = 1300.0f;
    }
    if (!isnan(heightAboveGroundM) && heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M && nowMs > 46000UL) {
      rpmTarget = 1250.0f;
    }
  }

  float drift = rpmTarget - currentRpm1Smooth;
  float nextRpm = currentRpm1Smooth + 0.08f * drift;
  if (!isnan(descentRate) && descentRate > 12.0f) {
    nextRpm -= 35.0f;
  }
  if (!isnan(descentRate) && descentRate < 1.0f) {
    nextRpm += 10.0f;
  }

  static float rpmNoise = 0.0f;
  rpmNoise = 0.85f * rpmNoise + 0.15f * (sinf(nowMs / 800.0f) * 65.0f);
  currentRpm1Smooth = constrain(nextRpm + rpmNoise, 0.0f, 2600.0f);
}

void Task1code(void *pvParameters) {
  Serial.print("Task1 (collective bias) running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    unsigned long nowMs = millis();
    if (nowMs - lastTaskRunMs < TASK_MIN_INTERVAL_MS) {
      continue;
    }
    lastTaskRunMs = nowMs;
    unsigned long width1Us = pwm1WidthUs;
    unsigned long width2Us = pwm2WidthUs;
    unsigned long width3Us = pwm3WidthUs;
    float altitude = currentAltitude;
    float rpmNow = currentRpm1Smooth;
    float rpmPrev = lastRpm1Smooth;
    MissionState state = missionState;

    float heightAboveGroundM = altitude - groundAltitude;
    float descentRateMps = NAN;
    if (!isnan(lastAltitude)) {
      descentRateMps = (lastAltitude - altitude) / 0.5f;
    }

    float alphaStoredDeg = alpha;
    float alphaNextDeg = alphaStoredDeg;
    long requestedBiasUs = 0;
    bool usedAdaptiveLogic = false;
    bool forceDeployMaxPitch = false;
    bool forcePostDeployMinPitch = false;
    bool controlWindowReady = deployReleaseTriggered && !isnan(heightAboveGroundM) && heightAboveGroundM > CONTROL_STAGE_MIN_HEIGHT_M && heightAboveGroundM <= CONTROL_STAGE_MAX_HEIGHT_M;
    float alphaTargetDeg = alphaStoredDeg;

    if (postDeployMinPitchPending) {
      alphaNextDeg = -12.0f;
      forcePostDeployMinPitch = true;
      usedAdaptiveLogic = true;
      postDeployMinPitchPending = false;
    } else if (state == DEPLOY_PULSE) {
      forceDeployMaxPitch = true;
      alphaNextDeg = alphaStoredDeg;
      usedAdaptiveLogic = true;
    } else if (deployReleaseTriggered) {
      if (controlWindowReady) {
        deployControlWaitingAltitude = false;
        alphaTargetDeg = computeLegacySweepAlphaDeg(heightAboveGroundM);
        
        alphaNextDeg = computeAdaptiveAlphaDeg(heightAboveGroundM, descentRateMps, rpmNow, alphaTargetDeg);
        if (isnan(alphaNextDeg)) {
          alphaNextDeg = alphaTargetDeg;
        }
        
        requestedBiasUs = (long)(alphaNextDeg * pitchStepUs);
        usedAdaptiveLogic = true;
        deployControlEntered = true;
      }
    }

    if (!usedAdaptiveLogic) {
      if (controlWindowReady) {
        alphaTargetDeg = computeLegacySweepAlphaDeg(heightAboveGroundM);
        alphaNextDeg = alphaTargetDeg;
        requestedBiasUs = (long)(alphaNextDeg * pitchStepUs);
        deployControlEntered = true;
        deployControlWaitingAltitude = false;
      } else {
        requestedBiasUs = computeLegacyCollectiveBiasUs(state, heightAboveGroundM, rpmNow, rpmPrev, nowMs);
        alphaNextDeg = alphaStoredDeg;
      }
    }

    deployControlStalled = deployReleaseTriggered && !deployControlEntered && deployControlArmedMs != 0UL && (nowMs - deployControlArmedMs) > DEPLOY_CONTROL_STALL_MS && !deployControlWaitingAltitude;
    const char *deployPhase = "WAIT_RELEASE";
    if (state == DEPLOY_PULSE) {
      deployPhase = "DEPLOY_MAX";
    } else if (forcePostDeployMinPitch) {
      deployPhase = "POST_DEPLOY_MIN";
    } else if (controlWindowReady && deployControlEntered) {
      deployPhase = "CONTROL_ACTIVE";
    } else if (controlWindowReady) {
      deployPhase = "CONTROL_READY";
    } else if (deployReleaseTriggered) {
      deployPhase = "LOWER_SWEEP";
    }

    if (forceDeployMaxPitch) {
      collectiveBiasUs = (long)pwmMaxUs - (long)collectiveCenterUs;
      servo1AdjustedPwmUs = pwmMaxUs;
      servo2AdjustedPwmUs = pwmMinUs;
      servo3AdjustedPwmUs = pwmMaxUs;
    } else if (forcePostDeployMinPitch) {
      collectiveBiasUs = -12L * pitchStepUs;
      servo1AdjustedPwmUs = clampPwmUs((long)collectiveCenterUs + collectiveBiasUs);
      servo2AdjustedPwmUs = clampPwmUs((long)collectiveCenterUs - collectiveBiasUs);
      servo3AdjustedPwmUs = clampPwmUs((long)collectiveCenterUs + collectiveBiasUs);
    } else {
      collectiveBiasUs = clampCollectiveBiasUs(requestedBiasUs, collectiveCenterUs, collectiveCenterUs);
      servo1AdjustedPwmUs = clampPwmUs((long)collectiveCenterUs + collectiveBiasUs);
      servo2AdjustedPwmUs = clampPwmUs((long)collectiveCenterUs - collectiveBiasUs);
      servo3AdjustedPwmUs = clampPwmUs((long)collectiveCenterUs + collectiveBiasUs);
    }

    float servo1OutDeg = pwmToDegrees(servo1AdjustedPwmUs);
    float servo2OutDeg = pwmToDegrees(servo2AdjustedPwmUs);
    float servo3OutDeg = pwmToDegrees(servo3AdjustedPwmUs);

    servo1AngleDeg = servo1OutDeg;
    servo2AngleDeg = servo2OutDeg;
    servo3AngleDeg = servo3OutDeg;
    alpha = constrain(alphaNextDeg, -12.0f, 12.0f);

    servo1.writeMicroseconds(servo1AdjustedPwmUs);
    servo2.writeMicroseconds(servo2AdjustedPwmUs);
    servo3.writeMicroseconds(servo3AdjustedPwmUs);

    Serial.print("Task1 state=");
    Serial.print((int)state);
    Serial.print(" drop=");
    Serial.print(heightAboveGroundM, 2);
    Serial.print("m | rpm=");
    Serial.print(rpmNow, 1);
    Serial.print(" | dHdt=");
    if (!isnan(descentRateMps)) Serial.print(descentRateMps, 2);
    else Serial.print("N/A");
    Serial.print(" | bias=");
    Serial.print(collectiveBiasUs);
    Serial.print(" us | alpha=");
    Serial.print(alpha, 1);
    Serial.print(" | alphaTgt=");
    Serial.print(alphaTargetDeg, 1);
    Serial.print(" deg | servo1=");
    Serial.print(servo1AdjustedPwmUs);
    Serial.print(" us | servo2=");
    Serial.print(servo2AdjustedPwmUs);
    Serial.print(" us | servo3=");
    Serial.print(servo3AdjustedPwmUs);
    Serial.print(" | deployArmed=");
    Serial.print(deployReleaseTriggered ? 1 : 0);
    Serial.print(" | phase=");
    Serial.print(deployPhase);
    Serial.print(" | ctrlWindow=");
    Serial.print(controlWindowReady ? 1 : 0);
    Serial.print(" | ctrlEntered=");
    Serial.print(deployControlEntered ? 1 : 0);
    Serial.print(" | ctrlWaitAlt=");
    Serial.print(deployControlWaitingAltitude ? 1 : 0);
    Serial.print(" | ctrlStalled=");
    Serial.print(deployControlStalled ? 1 : 0);
    Serial.print(" (deg=");
    Serial.print(servo3OutDeg, 1);
    Serial.println(") us");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Autogyro collective control v1 synthetic test starting");

  pinMode(pwmInput1Pin, INPUT);
  pinMode(pwmInput2Pin, INPUT);
  pinMode(pwmInput3Pin, INPUT);

  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo1.attach(servo1Pin, pwmMinUs, pwmMaxUs);
  servo2.attach(servo2Pin, pwmMinUs, pwmMaxUs);
  servo3.attach(servo3Pin, pwmMinUs, pwmMaxUs);
  delay(200);

  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);

  attachInterrupt(digitalPinToInterrupt(pwmInput1Pin), isrPwm1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInput2Pin), isrPwm2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInput3Pin), isrPwm3, CHANGE);

  groundAltitude = 0.0f;
  currentAltitude = 0.0f;
  lastAltitude = NAN;
  currentRpm1Smooth = 0.0f;
  lastRpm1Smooth = 0.0f;

  pinMode(SENSOR1, INPUT);
  for (int i = 0; i < N_PROMEDIO; i++) {
    buffer1[i] = 0.0f;
  }

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
  unsigned long nowMs = millis();
  if (nowMs - t_anterior >= TIEMPO_MUESTREO) {
    t_anterior = nowMs;

    lastAltitude = currentAltitude;
    lastRpm1Smooth = currentRpm1Smooth;

    updateSyntheticProfile(nowMs);
    bool isDescending = false;
    if (!isnan(lastAltitude)) {
      isDescending = (currentAltitude - lastAltitude) < 0.0f;
    }

    float heightAboveGroundM = currentAltitude - groundAltitude;
    if (!deployAltitudeSeenAbove && !isnan(heightAboveGroundM) && heightAboveGroundM >= DEPLOY_AGL_THRESHOLD_M) {
      deployAltitudeSeenAbove = true;
    }
    if (!deployWindowActive && deployAltitudeSeenAbove && !isnan(heightAboveGroundM) && heightAboveGroundM <= DEPLOY_AGL_THRESHOLD_M && isDescending) {
      deployWindowActive = true;
      deployStartMs = nowMs;
      deployControlEntered = false;
      deployControlStalled = false;
      deployControlWaitingAltitude = false;
      deployControlArmedMs = 0UL;
    }

    missionState = selectMissionState(heightAboveGroundM, nowMs, currentRpm1Smooth, isDescending);
    task1Active = deployWindowActive ||
      (!isnan(heightAboveGroundM) && heightAboveGroundM > 0.0f && isDescending);

    Serial.print("Tiempo(ms):"); Serial.print(nowMs);
    Serial.print("\tAltitud(m):"); Serial.print(currentAltitude, 2);
    Serial.print("\tAGL(m):"); Serial.print(heightAboveGroundM, 2);
    Serial.print("\tRPM:"); Serial.print(currentRpm1Smooth, 2);
    Serial.print("\tState:"); Serial.print((int)missionState);
    Serial.print("\tBiasUs:"); Serial.println(collectiveBiasUs);

    if (Task1 != NULL && task1Active) {
      xTaskNotifyGive(Task1);
    }
  }

  delay(5);
}