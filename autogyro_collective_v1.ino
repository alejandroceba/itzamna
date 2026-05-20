
// Core Arduino API used for timing, GPIO, math helpers, and serial output.
#include <Arduino.h>
// Servo driver for the three collective-pitch actuators.
#include <ESP32Servo.h>
// I2C bus support for the BME280 sensor.
#include <Wire.h>
// Temperature, pressure, and altitude sensor driver.
#include <Adafruit_BME280.h>

// FreeRTOS task handle used to wake the control task from the PWM ISR.
TaskHandle_t Task1 = NULL;

// PWM pin that commands servo 1.
const int servo1Pin = D1;
// PWM pin that commands servo 2.
const int servo2Pin = D2;
// PWM pin that commands servo 3, which moves with the same polarity as servo 1
// Polarity: servo1 and servo3 apply a negative offset (subtract bias),
//          servo2 applies the inverse (adds bias).
const int servo3Pin = D3; // moved to D3 to free D8 for the third FC input and because bme uses scl D4 (xiao esp32s3)
// Input pin that receives the first channel from the flight controller.
const int pwmInput1Pin = D8; // changed: FC -> Xiao input moved to D8
// Input pin that receives the second channel from the flight controller.
const int pwmInput2Pin = D9; // changed: FC -> Xiao input moved to D9
// Input pin that receives the third channel from the flight controller (separate input for servo3)
const int pwmInput3Pin = D10;
// Minimum servo pulse width used by the whole pitch system.
const int pwmMinUs = 1000;
// Maximum servo pulse width used by the whole pitch system.
const int pwmMaxUs = 2000;
// One control step equals one degree of pitch change.
const float pitchStepDeg = 1.0f;
// Conversion factor between microseconds and degrees.
const float pwmDegPerUs = 180.0f / (pwmMaxUs - pwmMinUs);
// PWM step in microseconds that corresponds to one degree of pitch.
const long pitchStepUs = (long)(pitchStepDeg / pwmDegPerUs + 0.5f);

// Deployment trigger threshold measured above ground level.
const float DEPLOY_AGL_THRESHOLD_M = 200.0f;
// Maximum time allowed to hold the deploy pitch command.
const float DEPLOY_MAX_PITCH_DURATION_MS = 2000UL;
// Altitude above ground level above which the upper-sweep logic is used.
const float SPINUP_AGL_M = 60.0f;
// Start of the lower-sweep ramp used by the legacy fallback.
const float LANDING_RAMP_START_AGL_M = 60.0f;
// End of the lower-sweep ramp used by the legacy fallback.
const float LANDING_RAMP_END_AGL_M = 20.0f;
// Small deadband used when comparing current and previous RPM values.
const float RPM_TREND_EPS = 5.0f;
// Larger deadband used when deciding if the RPM is clearly falling.
const float RPM_BIAS_STEP_EPS = 2.0f;
// Constant from the notebook model that converts alpha into optimal RPM.
const float OPTIMAL_RPM_ALPHA_CONSTANT = 4810.0544f; // Constant used to compute the optimal RPM from alpha.
// Lower bound of the new adaptive control window.
const float CONTROL_STAGE_MIN_HEIGHT_M = 20.0f; // Lower bound of the adaptive control stage.
// Upper bound of the new adaptive control window.
const float CONTROL_STAGE_MAX_HEIGHT_M = 200.0f; // Upper bound of the adaptive control stage.
// Requested descent rate below 20 m.
const float FINAL_DESCENT_RATE_MPS = 0.1f; // Target descent rate below 20 m.
// Requested descent rate between 20 m and 200 m.
const float MID_DESCENT_RATE_MPS = 12.0f; // Target descent rate between 20 m and 200 m.

// Servo object for actuator 1.
Servo servo1;
// Servo object for actuator 2.
Servo servo2;
// Servo object for actuator 3.
Servo servo3;

// Rotor geometry and vehicle constants used by the aerodynamic model.
#define N_PALAS  4          /* Numero de palas del rotor. */
#define M        0.420      /* Masa total del sistema [kg]. */
#define R        0.22869    /* Radio exterior del rotor [m]. */
#define R0       0.04776    /* Radio donde inicia la pala [m]. */
#define C_CUERDA 0.0204     /* Cuerda de la pala [m]. */
#define RHO      1.225      /* Densidad del aire [kg/m^3]. */
#define G        9.81       /* Gravedad [m/s^2]. */

// BME280 instance used to read the current altitude.
Adafruit_BME280 bme;
// Flag that tells the code whether the sensor was found and initialized.
bool bmeOk = false;
// First possible I2C address for the BME280.
#define BME280_ADDR_1 0x76
// Second possible I2C address for the BME280.
#define BME280_ADDR_2 0x77
// Main loop sampling period.
#define TIEMPO_MUESTREO 500 // ms
// Timestamp of the last sample update.
unsigned long t_anterior = 0;

// Encoder or hall sensor input for rotor RPM measurement.
#define SENSOR1 D0
// Pulses per revolution for the rotor sensor.
#define PULSOS_POR_REV 2
// Simple debouncing window for the RPM sensor ISR.
#define DEBOUNCE_US 2000UL
// Number of samples used in the moving average.
#define N_PROMEDIO 5
// Pulse count accumulated during the sampling window.
volatile unsigned long pulsos1 = 0;
// Timestamp of the last accepted pulse edge.
volatile unsigned long t1 = 0;
// Buffer that stores the last RPM samples.
float buffer1[N_PROMEDIO];
// Circular index for the averaging buffer.
int indice = 0;

// Input edge timestamp for the first flight-controller PWM channel.
volatile unsigned long pwm1RiseUs = 0;
// Input edge timestamp for the second flight-controller PWM channel.
volatile unsigned long pwm2RiseUs = 0;
// Input edge timestamp for the third flight-controller PWM channel.
volatile unsigned long pwm3RiseUs = 0;
// Latest measured width for PWM channel 1.
volatile unsigned long pwm1WidthUs = 1500;
// Latest measured width for PWM channel 2.
volatile unsigned long pwm2WidthUs = 1500;
// Latest measured width for PWM channel 3.
volatile unsigned long pwm3WidthUs = 1500;

// PWM command actually sent to servo 1 after the control law.
volatile unsigned long servo1AdjustedPwmUs = 1500;
// PWM command actually sent to servo 2 after the control law.
volatile unsigned long servo2AdjustedPwmUs = 1500;
// PWM command actually sent to servo 3 after the control law.
volatile unsigned long servo3AdjustedPwmUs = 1500;
// Servo 1 angle reconstructed from its PWM command.
volatile float servo1AngleDeg = NAN;
// Servo 2 angle reconstructed from its PWM command.
volatile float servo2AngleDeg = NAN;
// Servo 3 angle reconstructed from its PWM command.
volatile float servo3AngleDeg = NAN;

// Stored pitch command used as the reference for the next model evaluation.
volatile float alpha = 12.0f; // Aerodynamic pitch command stored from the previous control step.
// Collective bias in microseconds used by the legacy fallback controller.
volatile long collectiveBiasUs = 0;

// Ground-referenced altitude baseline measured before the drone lift.
volatile float groundAltitude = NAN;
// Previous altitude sample.
volatile float lastAltitude = NAN;
// Current altitude sample.
volatile float currentAltitude = NAN;
// Previous smoothed RPM sample.
volatile float lastRpm1Smooth = NAN;
// Current smoothed RPM sample.
volatile float currentRpm1Smooth = NAN;
// Flag that enables Task1 notifications from the PWM ISRs (reduces wakeups).
volatile bool task1Active = false;
// Rate-limit Task1 processing to avoid running faster than this interval.
volatile unsigned long lastTaskRunMs = 0UL;
const unsigned long TASK_MIN_INTERVAL_MS = 50UL; // minimum time between Task1 runs
// Timestamp when the deploy sequence started.
unsigned long deployStartMs = 0UL;
// Latch that records the first time the vehicle has been above the 200 m AGL threshold.
volatile bool deployAltitudeSeenAbove = false;
// Latch that keeps the deploy/control window active after the second 200 m AGL crossing.
volatile bool deployWindowActive = false;
// Latch that tells the code the deploy pulse has been completed.
bool deployReleaseTriggered = false;
// (servo3Pin and pwmInput3Pin defined above)

// High-level mission phases used by the controller.
enum MissionState : uint8_t {
  // Waiting before deployment.
  WAIT_FOR_RELEASE = 0,
  // Short deploy pulse after release.
  DEPLOY_PULSE = 1,
  // Upper-altitude sweep or adaptive window.
  UPPER_SWEEP = 2,
  // Lower-altitude sweep or maximum-pitch phase.
  LOWER_SWEEP = 3
};

// Global state variable that holds the current mission phase.
volatile MissionState missionState = WAIT_FOR_RELEASE;

// Clamp a PWM value to the legal servo pulse range.
unsigned long clampPwmUs(long value) {
  return constrain(value, pwmMinUs, pwmMaxUs);
}

// Limit the requested collective bias so it does not push either servo outside its PWM range.
long clampCollectiveBiasUs(long requestedBiasUs, unsigned long width1Us, unsigned long width2Us) {
  // Maximum positive offset allowed before one of the servos hits its upper limit.
  long maxPositiveBias = min(pwmMaxUs - (long)width1Us, pwmMaxUs - (long)width2Us);
  // Maximum negative offset allowed before one of the servos hits its lower limit.
  long maxNegativeBias = min((long)width1Us - pwmMinUs, (long)width2Us - pwmMinUs);

  // Positive requested bias is clipped to the available upward margin.
  if (requestedBiasUs > 0) {
    return min(requestedBiasUs, maxPositiveBias);
  }

  // Negative requested bias is clipped to the available downward margin.
  if (requestedBiasUs < 0) {
    return max(requestedBiasUs, -maxNegativeBias);
  }

  // Zero bias stays zero.
  return 0;
}

// Convert a PWM pulse width into an equivalent pitch angle in degrees.
float pwmToDegrees(unsigned long pulseWidthUs) {
  // Force the pulse width into the legal range before converting.
  pulseWidthUs = constrain(pulseWidthUs, (unsigned long)pwmMinUs, (unsigned long)pwmMaxUs);
  // Return the equivalent angle using the linear servo mapping.
  return (pulseWidthUs - pwmMinUs) * pwmDegPerUs;
}

  // Legacy sweep used across the full 200 m AGL to 20 m AGL fallback band.
float computeLegacySweepAlphaDeg(float heightAboveGroundM) {
  // Use one normalized progress variable for the full 200 m AGL to 20 m AGL legacy sweep.
  float sweepProgress = (CONTROL_STAGE_MAX_HEIGHT_M - heightAboveGroundM) / (CONTROL_STAGE_MAX_HEIGHT_M - CONTROL_STAGE_MIN_HEIGHT_M);
  // Keep the sweep progress inside the valid interval.
  sweepProgress = constrain(sweepProgress, 0.0f, 1.0f);
  // From 200 m AGL to 60 m AGL, sweep from -12 deg up to 0 deg.
  if (heightAboveGroundM > SPINUP_AGL_M) {
    float highBandProgress = (CONTROL_STAGE_MAX_HEIGHT_M - heightAboveGroundM) / (CONTROL_STAGE_MAX_HEIGHT_M - SPINUP_AGL_M);
    highBandProgress = constrain(highBandProgress, 0.0f, 1.0f);
    return -12.0f * (1.0f - highBandProgress);
  }

  // From 60 m AGL to 20 m AGL, sweep from 0 deg up to 12 deg.
  float lowBandProgress = (SPINUP_AGL_M - heightAboveGroundM) / (SPINUP_AGL_M - CONTROL_STAGE_MIN_HEIGHT_M);
  lowBandProgress = constrain(lowBandProgress, 0.0f, 1.0f);
  return 12.0f * lowBandProgress;
}

// Evaluate the notebook-based optimal RPM law for a given alpha in degrees.
float computeOptimalRpmFromAlphaDeg(float alphaDeg) {
  // Handle NaN input.
  if (isnan(alphaDeg)) { // Invalid input cannot be processed.
    // Signal that the optimal RPM cannot be computed.
    return NAN; // Signal that the optimal RPM cannot be computed.
  }

  // For zero or negative alpha, use a fixed fallback RPM (no lift, but rotor still spinning).
  if (alphaDeg <= 0.0f) { // The model is not valid for zero or negative alpha.
    // Return a minimum target RPM for the no-lift regime (e.g., during spinup or descent).
    return 1300.0f; // Fallback RPM target when alpha <= 0 (no lift, rotor drag-driven).
  }

  // One-line model extracted from the notebook.
  return OPTIMAL_RPM_ALPHA_CONSTANT / sqrtf(alphaDeg); // One-line model extracted from the notebook.
}

void IRAM_ATTR isrPwm1() {
  unsigned long nowUs = micros();
  if (digitalRead(pwmInput1Pin) == HIGH) {
    pwm1RiseUs = nowUs;
  } else {
    unsigned long widthUs = nowUs - pwm1RiseUs;
    if (widthUs >= pwmMinUs && widthUs <= pwmMaxUs) {
      pwm1WidthUs = widthUs;
        // Only wake the control task if it has been enabled to run.
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
        // Only wake the control task if it has been enabled to run.
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
      // Only wake the control task if it has been enabled to run.
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

MissionState selectMissionState(float heightAboveGroundM, unsigned long nowMs, float rpmCurrent, bool isDescending) {
  // Before the second 200 m AGL crossing, keep the system waiting.
  if (!deployWindowActive) { // Before the second deploy-window crossing, remain conservative.
    // No deployment yet, so remain in the waiting state.
    return WAIT_FOR_RELEASE; // Stay idle while the vehicle is still attached.
  }

  // While the deploy hold timer is still running, keep the deploy-pulse behavior active.
  if (!deployReleaseTriggered) { // The deploy pulse is active until the hold timer expires.
    // Keep the deploy-pulse behavior for the short release window.
    return DEPLOY_PULSE; // Keep the deploy-pulse behavior for the short release window.
  }

  // Below 20 m AGL the controller should switch to the lower sweep phase.
  if (heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M) { // Below 20 m the final stage starts.
    // Lower sweep uses the maximum pitch to bias the rotor into the final configuration.
    return LOWER_SWEEP; // Use the lower-sweep state.
  }

  // Between 60 m AGL and 20 m AGL the lower half of the legacy sweep is active.
  if (heightAboveGroundM <= SPINUP_AGL_M) { // Between 60 m and 20 m the lower-sweep controller is active.
    // Reuse the lower-sweep state for the low-altitude sweep.
    return LOWER_SWEEP; // Reuse LOWER_SWEEP as the low-altitude sweep state.
  }

  // Between 200 m AGL and 60 m AGL the upper half of the legacy sweep is active.
  if (heightAboveGroundM <= CONTROL_STAGE_MAX_HEIGHT_M) { // Between 200 m and 60 m the upper-sweep controller is active.
    // Reuse the upper-sweep state for the upper half of the sweep.
    return UPPER_SWEEP; // Reuse UPPER_SWEEP as the adaptive control state.
  }

  // Outside the modeled window, keep the state conservative.
  return WAIT_FOR_RELEASE; // Outside the modeled window, keep the state conservative.
}

// Legacy controller kept as a fallback when the adaptive rules cannot run.
long computeLegacyCollectiveBiasUs(MissionState state, float heightAboveGroundM, float rpmNow, float rpmPrev, unsigned long nowMs) {
  // Maximum and minimum collective bias correspond to the physical pitch endpoints.
  const long maxCollectiveBiasUs = 12L * pitchStepUs;
  const long minCollectiveBiasUs = -12L * pitchStepUs;


  ///esto tb ocurre antes de adaptive?
  // While waiting, keep the servos centered.
  if (state == WAIT_FOR_RELEASE) {
    return 0;
  }

  // During deploy, keep the blades in deploy pitch while descending at the 200 m deployment point.
  if (state == DEPLOY_PULSE) {
    // Start the deploy timer the first time the deploy phase is entered.
    if (deployStartMs == 0UL) {
      deployStartMs = nowMs;
    }

    // Once the deploy hold time expires, latch the release flag so the adaptive controller can take over.
    if (!deployReleaseTriggered && (nowMs - deployStartMs) >= DEPLOY_MAX_PITCH_DURATION_MS) {
      deployReleaseTriggered = true;
    }
    // Return the maximum positive bias while the blades are still deploying.
    return maxCollectiveBiasUs;
  }
  ///

  // During the upper sweep, hold the minimum pitch and avoid aggressive changes.
  if (state == UPPER_SWEEP) {
    // Start from the minimum bias.
    long biasUs = minCollectiveBiasUs;

    // If RPM is dropping, stay conservative and keep the minimum bias.
    if (!isnan(rpmNow) && !isnan(rpmPrev) && rpmNow + RPM_BIAS_STEP_EPS < rpmPrev) {
      biasUs = minCollectiveBiasUs;
    }

    // Return the conservative upper-sweep bias.
    return biasUs;
  }

  // In the lower sweep, bias the collective according to a two-stage linear ramp.
  float landingSpanM = LANDING_RAMP_START_AGL_M - LANDING_RAMP_END_AGL_M;
  // Keep the span nonzero to avoid division by zero.
  if (landingSpanM < 1.0f) landingSpanM = 1.0f;

  // Normalize height into a 0..1 progress variable across the landing band.
  float progress = (LANDING_RAMP_START_AGL_M - heightAboveGroundM) / landingSpanM;
  // Clamp the landing progress to the valid interval.
  progress = constrain(progress, 0.0f, 1.0f);

  // Two-stage linear ramp: from minCollective -> 0, then 0 -> maxCollective.
  // Split the ramp in the middle so the bias changes smoothly in two halves.
  float mid = 0.5f;
  // This will hold the requested bias before servo range clamping.
  long targetBiasUs = 0;
  // First half of the ramp moves from minimum bias toward neutral.
  if (progress <= mid) {
    // Convert the first half of the progress range into a 0..1 interpolation factor.
    float s = (mid > 0.0f) ? (progress / mid) : 1.0f;
    // Interpolate from minCollectiveBiasUs to 0.
    // Map the interpolation factor to the actual bias range.
    float val = (float)minCollectiveBiasUs + s * (0.0f - (float)minCollectiveBiasUs);
    // Round the result to the nearest microsecond.
    targetBiasUs = (long)lroundf(val);
  } else {
    // Convert the second half of the progress range into a 0..1 interpolation factor.
    float s = (1.0f - mid > 0.0f) ? ((progress - mid) / (1.0f - mid)) : 1.0f;
    // Interpolate from 0 to maxCollectiveBiasUs.
    // Map the interpolation factor to the upper half of the bias range.
    float val = 0.0f + s * (float)maxCollectiveBiasUs;
    // Round the result to the nearest microsecond.
    targetBiasUs = (long)lroundf(val);
  }

  // Safety: if RPM is dropping rapidly, avoid raising positive bias.
  // A falling rotor speed means we should not increase collective aggressively.
  if (!isnan(rpmNow) && !isnan(rpmPrev) && rpmNow + RPM_TREND_EPS < rpmPrev) {
    targetBiasUs = min(targetBiasUs, 0L);
  }

  // Clip the requested bias to the physically available range.
  return constrain(targetBiasUs, minCollectiveBiasUs, maxCollectiveBiasUs);
}

// New adaptive pitch selector based on altitude, descent rate, and measured RPM.
float computeAdaptiveAlphaDeg(float heightAboveGroundM, float descentRateMps, float rpmNow, float alphaStoredDeg) {
  // Missing altitude means there is no adaptive decision to make.
  if (isnan(heightAboveGroundM)) { // Missing altitude means there is no adaptive decision to make.
    // Tell the caller to use the legacy controller.
    return NAN; // Tell the caller to use the legacy controller.
  }

  ///CORREGIR

  // Below 20 m the final phase takes over. no, incorrect: below 20 m, optimal delta H is decreased such that ptich can be increased, given rpms are high. however, it is not a bad idea to force max pitch
  if (heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M) { // Below 20 m, the final phase takes over.
    // Force the pitch to the maximum value.
    return 12.0f; // Force the pitch to the maximum value.
  }
  ///

  // Outside the adaptive window the controller does nothing.
  if (heightAboveGroundM > CONTROL_STAGE_MAX_HEIGHT_M) { // Outside the adaptive window.
    // Keep the previous strategy instead of extrapolating.
    return NAN; // Keep the previous strategy instead of extrapolating.
  }

  // Compute the model RPM from the last stored alpha.
  float rpmOptimal = computeOptimalRpmFromAlphaDeg(alphaStoredDeg); // Compute the model RPM from the last stored alpha.
  // Any missing signal invalidates the adaptive step.
  if (isnan(rpmNow) || isnan(rpmOptimal) || isnan(descentRateMps)) { // Any missing signal invalidates the adaptive step.
    // Fall back to the legacy control law.
    return NAN; // Fall back to the legacy control law.
  }

  // If the measured RPM is below the model optimum, reduce alpha so the blades go more negative.
  if (rpmNow < rpmOptimal) { // The rotor is slower than the model optimum.
    // If the descent is still too fast, make only a small negative pitch correction.
    if (descentRateMps > MID_DESCENT_RATE_MPS) { // The vehicle is still descending too fast.
      // Apply the small 0.5 deg decrease requested.
      return constrain(alphaStoredDeg - 0.5f, -12.0f, 12.0f); // Apply the small 0.5 deg decrease requested.
    }

    // If the descent is already slow enough, move the pitch further negative by 1 deg.
    return constrain(alphaStoredDeg - 1.0f, -12.0f, 12.0f); // Otherwise reduce alpha by 1 deg.
  }

  // If RPM is above optimum and the descent is still too fast, increase alpha by 1 deg.
  if (rpmNow > rpmOptimal && descentRateMps > MID_DESCENT_RATE_MPS) { // The rotor is above optimum and descent is still too fast.
    // Increase alpha by 1 deg.
    return constrain(alphaStoredDeg + 1.0f, -12.0f, 12.0f); // Increase alpha by 1 deg.
  }

  // Otherwise keep the previous alpha command.
  return alphaStoredDeg; // Leave alpha unchanged when no rule matches.
}

// RTOS task that translates the high-level control decision into servo PWM outputs.
void Task1code(void *pvParameters) {
  // Announce which core is running the control task.
  Serial.print("Task1 (collective bias) running on core ");
  Serial.println(xPortGetCoreID());

  // Keep the task alive for the lifetime of the firmware.
  for (;;) {
    // Sleep until one of the PWM ISR callbacks wakes the task.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Rate-limit Task1 processing to avoid reacting faster than `TASK_MIN_INTERVAL_MS`.
    unsigned long nowMs = millis();
    if (nowMs - lastTaskRunMs < TASK_MIN_INTERVAL_MS) {
      // Skip this wake and wait for the next notification.
      continue;
    }
    lastTaskRunMs = nowMs;

    unsigned long width1Us = pwm1WidthUs; // Snapshot the current PWM width of servo 1.
    unsigned long width2Us = pwm2WidthUs; // Snapshot the current PWM width of servo 2.
    unsigned long width3Us = pwm3WidthUs; // Snapshot the current PWM width of servo 3.
    float altitude = currentAltitude; // Read the current altitude estimate from the BME280.
    float rpmNow = currentRpm1Smooth; // Read the current smoothed rotor RPM.
    float rpmPrev = lastRpm1Smooth; // Read the previous smoothed rotor RPM.
    MissionState state = missionState; // Keep the current mission state for deploy/fallback logic.
    unsigned long nowMs = millis(); // Capture the current time for deploy timing.

    float heightAboveGroundM = NAN; // Store the altitude above ground level.
    if (!isnan(groundAltitude) && !isnan(altitude)) { // Only compute height above ground when both altitudes are valid.
      heightAboveGroundM = altitude - groundAltitude; // Height above ground grows as the vehicle climbs and shrinks as it descends.
    }

    float descentRateMps = NAN; // Store the measured descent rate in m/s.
    if (!isnan(lastAltitude) && !isnan(altitude)) { // Compute a rate only when two consecutive altitudes are valid.
      descentRateMps = (lastAltitude - altitude) / (TIEMPO_MUESTREO / 1000.0f); // Convert altitude change per sample into m/s.
    }

    float alphaStoredDeg = alpha; // Keep the pitch command from the previous control step.
    float alphaNextDeg = alphaStoredDeg; // Prepare the pitch command for this step.
    long requestedBiasUs = 0; // Store the PWM increment or decrement derived from the new pitch step.
    bool usedAdaptiveLogic = false; // Track whether the new control law was applied.

    if (deployWindowActive && !isnan(heightAboveGroundM) && heightAboveGroundM > CONTROL_STAGE_MIN_HEIGHT_M && heightAboveGroundM <= CONTROL_STAGE_MAX_HEIGHT_M) { // Only use the new law after the second 200 m AGL crossing and between 20 m and 200 m AGL.
      // Compute the next alpha using the model and the last stored alpha.
      float adaptiveAlphaDeg = computeAdaptiveAlphaDeg(heightAboveGroundM, descentRateMps, rpmNow, alphaStoredDeg); // Compute the next alpha from the model.
      // Accept the new command only if the model returned a valid result.
      if (!isnan(adaptiveAlphaDeg)) { // Continue only if the adaptive model returned a valid result.
        // Save the new pitch command.
        alphaNextDeg = adaptiveAlphaDeg; // Store the new alpha command.
        // Convert the alpha change into a signed PWM offset.
        requestedBiasUs = lroundf((alphaNextDeg - alphaStoredDeg) * pitchStepUs); // Convert the alpha change into PWM microseconds.
        // Record that the new control logic was used.
        usedAdaptiveLogic = true; // Mark that the adaptive controller was used.
      }
    } else if (deployWindowActive && !isnan(heightAboveGroundM) && heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M) { // Final phase below 20 m AGL after deploy.
      // Force the pitch to the maximum value once the vehicle reaches the final stage.
      alphaNextDeg = 12.0f; // Jump directly to the maximum pitch command.
      // Convert the final-stage step into PWM microseconds.
      requestedBiasUs = lroundf((alphaNextDeg - alphaStoredDeg) * pitchStepUs); // Convert the final-pitch step into PWM microseconds.
      // Mark the final-phase logic as active.
      usedAdaptiveLogic = true; // Mark the final-phase logic as active.
    }

    // If the new logic could not be used, preserve the current controller as fallback.
    if (!usedAdaptiveLogic) { // If the new logic could not be used, preserve the current controller as fallback.
      // Use the legacy sweep across the entire 200 m to 20 m band.
      if (!isnan(heightAboveGroundM) && heightAboveGroundM > CONTROL_STAGE_MIN_HEIGHT_M && heightAboveGroundM <= CONTROL_STAGE_MAX_HEIGHT_M) { // Only sweep between 200 m and 20 m AGL.
        // Compute the legacy sweep target directly in degrees.
        alphaNextDeg = computeLegacySweepAlphaDeg(heightAboveGroundM); // Sweep from -12 deg at 200 m AGL to 0 deg at 60 m AGL, then from 0 deg to 12 deg at 20 m AGL.
        // Convert the alpha change into a signed PWM offset.
        requestedBiasUs = lroundf((alphaNextDeg - alphaStoredDeg) * pitchStepUs); // Convert the alpha change into PWM microseconds.
      } else {
        // Reuse the previous control law for the deploy pulse and the low-altitude sweep.
        requestedBiasUs = computeLegacyCollectiveBiasUs(state, heightAboveGroundM, rpmNow, rpmPrev, nowMs); // Reuse the previous control law.
        // Keep the stored pitch unchanged when the fallback is active outside the sweep band.
        alphaNextDeg = alphaStoredDeg; // Keep the stored pitch unchanged when the fallback is active.
      }
    }

    // Limit the requested PWM change to the available servo range.
    collectiveBiasUs = clampCollectiveBiasUs(requestedBiasUs, width1Us, width2Us); // Limit the requested PWM change to the available servo range.

    // Apply bias with polarity matching the tested `min_servo_sensor` logic:
    // - servo1 and servo3: subtract the bias (width - bias)
    // - servo2: add the bias (width + bias)
    servo1AdjustedPwmUs = clampPwmUs((long)width1Us - collectiveBiasUs); // Servo 1 uses negative offset.
    servo2AdjustedPwmUs = clampPwmUs((long)width2Us + collectiveBiasUs); // Servo 2 uses positive offset (inverse polarity).
    servo3AdjustedPwmUs = clampPwmUs((long)width3Us - collectiveBiasUs); // Servo 3 mirrors servo1 polarity.

    // Convert servo 1 PWM back into degrees for logging.
    float servo1OutDeg = pwmToDegrees(servo1AdjustedPwmUs); // Convert servo 1 PWM back into degrees for logging.
    // Convert servo 2 PWM back into degrees for logging.
    float servo2OutDeg = pwmToDegrees(servo2AdjustedPwmUs); // Convert servo 2 PWM back into degrees for logging.
    // Convert servo 3 PWM back into degrees for logging.
    float servo3OutDeg = pwmToDegrees(servo3AdjustedPwmUs); // Convert servo 3 PWM back into degrees for logging.

    // Save the physical angle of servo 1.
    servo1AngleDeg = servo1OutDeg; // Save the physical angle of servo 1.
    // Save the physical angle of servo 2.
    servo2AngleDeg = servo2OutDeg; // Save the physical angle of servo 2.
    // Save the physical angle of servo 3.
    servo3AngleDeg = servo3OutDeg; // Save the physical angle of servo 3.
    // Store the commanded pitch angle for the next control step and for the model.
    alpha = constrain(alphaNextDeg, -12.0f, 12.0f); // Store the commanded pitch angle for the next control step and for the model.

    // Drive servo 1 with the updated PWM.
    servo1.writeMicroseconds(servo1AdjustedPwmUs); // Drive servo 1 with the updated PWM.
    // Drive servo 2 with the inverse PWM.
    servo2.writeMicroseconds(servo2AdjustedPwmUs); // Drive servo 2 with the inverse PWM.
    // Drive servo 3 with the mirrored PWM.
    servo3.writeMicroseconds(servo3AdjustedPwmUs); // Drive servo 3 with the mirrored PWM.

    // Print a compact snapshot of the control decision for debugging.
    Serial.print("Task1 state=");
    Serial.print((int)state);
    Serial.print(" drop=");
    if (!isnan(heightAboveGroundM)) Serial.print(heightAboveGroundM, 2); else Serial.print("N/A");
    Serial.print("m | rpm=");
    if (!isnan(rpmNow)) Serial.print(rpmNow, 2); else Serial.print("N/A");
    Serial.print(" | rpm_opt=");
    if (!isnan(alphaStoredDeg) && alphaStoredDeg > 0.0f) Serial.print(computeOptimalRpmFromAlphaDeg(alphaStoredDeg), 2); else Serial.print("N/A");
    Serial.print(" | dHdt=");
    if (!isnan(descentRateMps)) Serial.print(descentRateMps, 2); else Serial.print("N/A");
    Serial.print(" | target_dH=");
    Serial.print((heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M) ? FINAL_DESCENT_RATE_MPS : MID_DESCENT_RATE_MPS, 1); // Report the requested descent target for the active stage.
    Serial.print(" | bias=");
    Serial.print(collectiveBiasUs);
    Serial.print(" us | alpha=");
    Serial.print(alpha, 1);
    Serial.print(" deg | servo1=");
    Serial.print(servo1AdjustedPwmUs);
    Serial.print(" us | servo2=");
    Serial.print(servo2AdjustedPwmUs);
    Serial.print(" us | servo3=");
    Serial.print(servo3AdjustedPwmUs);
    Serial.print(" (deg=");
    Serial.print(servo3OutDeg, 1);
    Serial.print(")");
    Serial.println(" us");
  }
}

// Initialize hardware, sensors, interrupts, and the control task.
void setup() {
  // Start serial logging.
  Serial.begin(115200);
  // Give the serial port time to come up.
  delay(500);
  // Announce firmware startup.
  Serial.println("Autogyro collective control v1 starting");

  // Configure PWM input pins as plain inputs.
  pinMode(pwmInput1Pin, INPUT);
  pinMode(pwmInput2Pin, INPUT);
  pinMode(pwmInput3Pin, INPUT);

  // Reserve an ESP32 timer for servo PWM generation.
  ESP32PWM::allocateTimer(0);
  // Configure the servo update frequency to the standard 50 Hz.
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  // Attach servo 1 to its pin and legal PWM range.
  servo1.attach(servo1Pin, pwmMinUs, pwmMaxUs);
  // Attach servo 2 to its pin and legal PWM range.
  servo2.attach(servo2Pin, pwmMinUs, pwmMaxUs);
  // Attach servo 3 to its pin and legal PWM range.
  servo3.attach(servo3Pin, pwmMinUs, pwmMaxUs);
  // Let the PWM hardware settle before commanding the outputs.
  delay(200);

  // Start with centered servos to ensure defined outputs before deploy
  // Center servo 1.
  servo1.writeMicroseconds(1500);
  // Center servo 2.
  servo2.writeMicroseconds(1500);
  // Center servo 3.
  servo3.writeMicroseconds(1500);

  // Enable interrupts for the flight-controller PWM inputs.
  attachInterrupt(digitalPinToInterrupt(pwmInput1Pin), isrPwm1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInput2Pin), isrPwm2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInput3Pin), isrPwm3, CHANGE);

  // Start the I2C bus used by the BME280.
  Wire.begin();
  // Track whether any BME280 address responds.
  bool found = false;
  // Try both supported I2C addresses.
  for (uint8_t addr : { (uint8_t)BME280_ADDR_1, (uint8_t)BME280_ADDR_2 }) {
    // Probe the bus for the current address.
    Wire.beginTransmission(addr);
    // End the transmission and get the bus status.
    uint8_t err = Wire.endTransmission();
    // If the device responded, try to initialize it.
    if (err == 0) {
      // Mark that at least one address was found.
      found = true;
      // Initialize the sensor at the detected address.
      if (bme.begin(addr)) {
        // Remember that the sensor is ready.
        bmeOk = true;
      } else {
        // The address responded but the sensor did not initialize properly.
        bmeOk = false;
      }
      // Stop after the first successful address.
      break;
    }
  }
  // If no address responded, mark the sensor as unavailable.
  if (!found) {
    bmeOk = false;
  } else {
    // Capture the ground reference altitude before the drone carries the vehicle upward.
    groundAltitude = bme.readAltitude(1013.25f);
  }

  // Configure the rotor RPM sensor pin.
  pinMode(SENSOR1, INPUT);
  // Attach the sensor interrupt on falling edges.
  attachInterrupt(digitalPinToInterrupt(SENSOR1), isr1, FALLING);

  // Clear the moving-average buffer.
  for (int i = 0; i < N_PROMEDIO; i++) buffer1[i] = 0.0;

  // Launch the control task on core 0.
  xTaskCreatePinnedToCore(
    Task1code,
    "Task1",
    4096,
    NULL,
    1,
    &Task1,
    0
  );

  // Give the task time to start before entering the main loop.
  delay(500);
}

// Main sampling loop that reads altitude and RPM and wakes the control task.
void loop() {
  // Read the current time.
  unsigned long t_actual = millis();
  // Run the sampling block at the configured period.
  if (t_actual - t_anterior >= TIEMPO_MUESTREO) {
    // Update the last sample time.
    t_anterior = t_actual;

    // Default altitude to invalid until the sensor is read.
    float altitude = NAN;
    // If the BME280 is ready, read the current altitude.
    if (bmeOk) {
      // Convert pressure into an altitude estimate.
      altitude = bme.readAltitude(1013.25f);
      // Share the altitude with the control task.
      currentAltitude = altitude;
    }

    // Disable interrupts while copying the pulse counter.
    noInterrupts();
    // Snapshot the pulse count.
    unsigned long c1 = pulsos1;
    // Reset the counter for the next sample window.
    pulsos1 = 0;
    // Re-enable interrupts after the shared state has been copied.
    interrupts();

    // Convert pulses per sampling window into rotor RPM.
    float rpm1 = ((c1 * 60000.0f) / (PULSOS_POR_REV * TIEMPO_MUESTREO)) / 2.0f;
    // Store the new sample in the moving-average buffer.
    buffer1[indice] = rpm1;
    // Advance the circular buffer index.
    indice++;
    // Wrap the circular buffer index when it reaches the end.
    if (indice >= N_PROMEDIO) indice = 0;

    // Move the current smoothed RPM into the previous slot.
    lastRpm1Smooth = currentRpm1Smooth;
    // Update the smoothed RPM from the moving average.
    currentRpm1Smooth = promedio(buffer1);

    // Only update the mission logic if the altitude sensor is available.
    if (bmeOk && !isnan(altitude)) {
      // Compute the height above ground from the stored ground reference.
      float heightAboveGroundM = altitude - groundAltitude;
      // Hold the instantaneous altitude difference between samples.
      float altDelta = NAN;
      // Assume the vehicle is not descending until proven otherwise.
      bool isDescending = false;
      // If a previous altitude exists, compare the new sample against it.
      if (!isnan(lastAltitude)) {
        // Compute the altitude change over one sample period.
        altDelta = altitude - lastAltitude;
        // Negative altitude change means descent.
        isDescending = altDelta < 0.0f;
      }
      // Record the first time the vehicle climbs above the deploy threshold.
      if (!deployAltitudeSeenAbove && !isnan(heightAboveGroundM) && heightAboveGroundM >= DEPLOY_AGL_THRESHOLD_M) {
        deployAltitudeSeenAbove = true;
      }

      // Arm the deploy window on the second crossing below 200 m AGL while descending.
      if (!deployWindowActive && deployAltitudeSeenAbove && !isnan(heightAboveGroundM) && heightAboveGroundM <= DEPLOY_AGL_THRESHOLD_M && isDescending) {
        deployWindowActive = true;
        deployStartMs = t_actual;
      }

      // Update the mission state based on height above ground and motion direction.
      missionState = selectMissionState(heightAboveGroundM, t_actual, currentRpm1Smooth, isDescending);

      // Enable Task1 notifications from PWM ISRs once the deploy window is active or descent is confirmed.
      if (deployWindowActive || (!isnan(heightAboveGroundM) && heightAboveGroundM > 0.0f && isDescending)) {
        task1Active = true;
      } else {
        task1Active = false;
      }

      // Print the sampled telemetry.
      Serial.print("Tiempo(ms):"); Serial.print(t_actual);
      Serial.print("\tAltitud(m):"); Serial.print(altitude, 2);
      Serial.print("\tAGL(m):"); Serial.print(heightAboveGroundM, 2);
      Serial.print("\tRPM:"); Serial.print(currentRpm1Smooth, 2);
      Serial.print("\tState:"); Serial.print((int)missionState);
      Serial.print("\tBiasUs:"); Serial.println(collectiveBiasUs);
      // Remember this altitude for the next descent-rate calculation.
      lastAltitude = altitude;
    } else {
      // Print a fallback line when altitude is unavailable.
      Serial.print("Tiempo(ms):"); Serial.print(t_actual);
      Serial.print("\tAltitud(m):N/A\tRPM:"); Serial.print(currentRpm1Smooth, 2);
      Serial.println();
    }

    // Wake the control task so it can recompute servo outputs.
    if (Task1 != NULL) {
      xTaskNotifyGive(Task1);
    }
  }

  // Short idle delay to keep the loop cooperative.
  delay(5);
}