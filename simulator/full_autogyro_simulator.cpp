#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <random>

// This simulator reproduces the high-level logic of autogyro_collective_v1.ino
// - Mission states and deploy window
// - Legacy sweep and adaptive controller
// - Simple RPM dynamics driven toward model optimum
// - Prints telemetry similar to Serial output

enum MissionState { WAIT_FOR_RELEASE = 0, DEPLOY_PULSE = 1, UPPER_SWEEP = 2, LOWER_SWEEP = 3 };

// Constants copied from firmware
const int pwmMinUs = 1000;
const int pwmMaxUs = 2000;
const float pitchStepDeg = 1.0f;
const float pwmDegPerUs = 180.0f / (pwmMaxUs - pwmMinUs);
const long pitchStepUs = (long)(pitchStepDeg / pwmDegPerUs + 0.5f);

const float DEPLOY_AGL_THRESHOLD_M = 200.0f;
const unsigned long DEPLOY_MAX_PITCH_DURATION_MS = 2000UL;
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

// Simple simulator state
struct SimState {
  float groundAltitude = 0.0f;
  float altitude = 0.0f; // absolute
  float heightAGL = NAN;
  float lastAltitude = NAN;
  float rpm = 1300.0f;
  float rpmSmooth = 1300.0f;
  bool deployAltitudeSeenAbove = false;
  bool deployWindowActive = false;
  bool deployReleaseTriggered = false;
  unsigned long deployStartMs = 0;
  MissionState missionState = WAIT_FOR_RELEASE;
  float alpha = 12.0f; // stored pitch command
  long collectiveBiasUs = 0;
};

float clampf(float v, float a, float b) { return std::fmax(a, std::fmin(b, v)); }
long clampCollectiveBiasUs(long req, unsigned long w1, unsigned long w2) {
  long maxPositiveBias = std::min((long)(pwmMaxUs - (long)w1), (long)(pwmMaxUs - (long)w2));
  long maxNegativeBias = std::min((long)w1 - pwmMinUs, (long)w2 - pwmMinUs);
  if (req > 0) return std::min(req, maxPositiveBias);
  if (req < 0) return std::max(req, -maxNegativeBias);
  return 0;
}

float computeOptimalRpmFromAlphaDeg(float alphaDeg) {
  if (std::isnan(alphaDeg)) return NAN;
  if (alphaDeg <= 0.0f) return 1300.0f;
  return OPTIMAL_RPM_ALPHA_CONSTANT / std::sqrt(alphaDeg);
}

float computeLegacySweepAlphaDeg(float heightAboveGroundM) {
  float sweepProgress = (CONTROL_STAGE_MAX_HEIGHT_M - heightAboveGroundM) / (CONTROL_STAGE_MAX_HEIGHT_M - CONTROL_STAGE_MIN_HEIGHT_M);
  sweepProgress = clampf(sweepProgress, 0.0f, 1.0f);
  if (heightAboveGroundM > SPINUP_AGL_M) {
    float highBandProgress = (CONTROL_STAGE_MAX_HEIGHT_M - heightAboveGroundM) / (CONTROL_STAGE_MAX_HEIGHT_M - SPINUP_AGL_M);
    highBandProgress = clampf(highBandProgress, 0.0f, 1.0f);
    return -12.0f * (1.0f - highBandProgress);
  }
  float lowBandProgress = (SPINUP_AGL_M - heightAboveGroundM) / (SPINUP_AGL_M - CONTROL_STAGE_MIN_HEIGHT_M);
  lowBandProgress = clampf(lowBandProgress, 0.0f, 1.0f);
  return 12.0f * lowBandProgress;
}

long computeLegacyCollectiveBiasUs(MissionState state, float heightAboveGroundM, float rpmNow, float rpmPrev, unsigned long nowMs) {
  const long maxCollectiveBiasUs = 12L * pitchStepUs;
  const long minCollectiveBiasUs = -12L * pitchStepUs;
  if (state == WAIT_FOR_RELEASE) return 0;
  if (state == DEPLOY_PULSE) return maxCollectiveBiasUs;
  if (state == UPPER_SWEEP) {
    long biasUs = minCollectiveBiasUs;
    if (!std::isnan(rpmNow) && !std::isnan(rpmPrev) && rpmNow + RPM_BIAS_STEP_EPS < rpmPrev) biasUs = minCollectiveBiasUs;
    return biasUs;
  }
  float landingSpanM = LANDING_RAMP_START_AGL_M - LANDING_RAMP_END_AGL_M;
  if (landingSpanM < 1.0f) landingSpanM = 1.0f;
  float progress = (LANDING_RAMP_START_AGL_M - heightAboveGroundM) / landingSpanM;
  progress = clampf(progress, 0.0f, 1.0f);
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
  if (!std::isnan(rpmNow) && !std::isnan(rpmPrev) && rpmNow + RPM_TREND_EPS < rpmPrev) targetBiasUs = std::min(targetBiasUs, 0L);
  return (long)clampf((float)targetBiasUs, (float)minCollectiveBiasUs, (float)maxCollectiveBiasUs);
}

float computeAdaptiveAlphaDeg(float heightAboveGroundM, float descentRateMps, float rpmNow, float alphaStoredDeg) {
  if (std::isnan(heightAboveGroundM)) return NAN;
  if (heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M) {
    if (std::isnan(rpmNow) || rpmNow < 1300.0f) return NAN;
    return alphaStoredDeg;
  }
  if (heightAboveGroundM > CONTROL_STAGE_MAX_HEIGHT_M) return NAN;
  float rpmOptimal = computeOptimalRpmFromAlphaDeg(alphaStoredDeg);
  if (std::isnan(rpmNow) || std::isnan(rpmOptimal) || std::isnan(descentRateMps)) return NAN;
  if (rpmNow < rpmOptimal) {
    if (descentRateMps > MID_DESCENT_RATE_MPS) return clampf(alphaStoredDeg - 0.5f, -12.0f, 12.0f);
    return clampf(alphaStoredDeg - 1.0f, -12.0f, 12.0f);
  }
  if (rpmNow > rpmOptimal && descentRateMps > MID_DESCENT_RATE_MPS) return clampf(alphaStoredDeg + 1.0f, -12.0f, 12.0f);
  return alphaStoredDeg;
}

MissionState selectMissionStateSim(bool &deployWindowActive, unsigned long &deployStartMs, bool &deployReleaseTriggered, float heightAboveGroundM, unsigned long nowMs, float rpmCurrent, bool isDescending) {
  if (!deployWindowActive) return WAIT_FOR_RELEASE;
  if (deployWindowActive && deployStartMs != 0UL) {
    if ((nowMs - deployStartMs) < DEPLOY_MAX_PITCH_DURATION_MS) return DEPLOY_PULSE;
    deployReleaseTriggered = true;
  }
  if (heightAboveGroundM <= CONTROL_STAGE_MIN_HEIGHT_M) return LOWER_SWEEP;
  if (heightAboveGroundM <= SPINUP_AGL_M) return LOWER_SWEEP;
  if (heightAboveGroundM <= CONTROL_STAGE_MAX_HEIGHT_M) return UPPER_SWEEP;
  return WAIT_FOR_RELEASE;
}

// Simple RPM dynamics: RPM tends toward computeOptimalRpmFromAlphaDeg(alpha) with time constant
void updateRpmDynamics(float &rpm, float alpha, float dtSec) {
  float rpmTarget = computeOptimalRpmFromAlphaDeg(alpha);
  if (std::isnan(rpmTarget)) rpmTarget = 1300.0f;
  // time constant ~ 1s
  float tau = 1.0f;
  rpm += (rpmTarget - rpm) * (dtSec / (tau + dtSec));
}

int main() {
  SimState s;
  // Simulation scenario: ascend to 220m, hold, then descend crossing deploy threshold
  float simTimeMs = 0.0f;
  const float dtMs = 100.0f; // simulation timestep
  const float dtSec = dtMs / 1000.0f;

  // altitude profile: climb at 5 m/s to 220m, hold 2s, then descend at 5 m/s
  float climbRate = 5.0f;
  float descendRate = -5.0f;
  bool climbing = true;
  float holdUntilMs = 3000.0f;

  unsigned long simMs = 0;

  std::cout << "Starting full autogyro simulator\n";
  std::cout << "Tiempo(ms)\tAlt(m)\tAGL(m)\tRPM\tState\tBiasUs\talpha(deg)\n";

  for (int step=0; step<2000; ++step) {
    // update time
    simMs = (unsigned long)simTimeMs;

    // update altitude
    if (climbing) s.altitude += climbRate * dtSec;
    else s.altitude += descendRate * dtSec;

    // switch from climbing to hold then descend
    if (s.altitude >= 220.0f) {
      climbing = false;
    }

    // compute height AGL
    s.heightAGL = s.altitude - s.groundAltitude;

    // detect deploy altitude crossing
    if (!s.deployAltitudeSeenAbove && !std::isnan(s.heightAGL) && s.heightAGL >= DEPLOY_AGL_THRESHOLD_M) {
      s.deployAltitudeSeenAbove = true;
    }
    bool isDescending = false;
    if (!std::isnan(s.lastAltitude)) {
      float altDelta = s.altitude - s.lastAltitude;
      isDescending = altDelta < 0.0f;
    }

    if (!s.deployWindowActive && s.deployAltitudeSeenAbove && !std::isnan(s.heightAGL) && s.heightAGL <= DEPLOY_AGL_THRESHOLD_M && isDescending) {
      s.deployWindowActive = true;
      s.deployStartMs = simMs;
    }

    // update mission state
    s.missionState = selectMissionStateSim(s.deployWindowActive, s.deployStartMs, s.deployReleaseTriggered, s.heightAGL, simMs, s.rpm, isDescending);

    // compute control decisions similar to Task1code
    float alphaStoredDeg = s.alpha;
    float alphaNextDeg = alphaStoredDeg;
    long requestedBiasUs = 0;
    bool usedAdaptiveLogic = false;

    if (s.missionState == DEPLOY_PULSE) {
      // Deploy always wins while the hold timer is active.
      requestedBiasUs = computeLegacyCollectiveBiasUs(s.missionState, s.heightAGL, s.rpm, s.rpm, simMs);
      alphaNextDeg = alphaStoredDeg;
      usedAdaptiveLogic = true;
    } else if (s.deployReleaseTriggered && !std::isnan(s.heightAGL) && s.heightAGL > CONTROL_STAGE_MIN_HEIGHT_M && s.heightAGL <= CONTROL_STAGE_MAX_HEIGHT_M) {
      float descentRateMps = NAN;
      if (!std::isnan(s.lastAltitude)) descentRateMps = (s.lastAltitude - s.altitude) / dtSec; // sample period dtSec
      float adaptiveAlphaDeg = computeAdaptiveAlphaDeg(s.heightAGL, descentRateMps, s.rpm, alphaStoredDeg);
      if (!std::isnan(adaptiveAlphaDeg)) {
        alphaNextDeg = adaptiveAlphaDeg;
        requestedBiasUs = lroundf((alphaNextDeg - alphaStoredDeg) * pitchStepUs);
        usedAdaptiveLogic = true;
      }
    }

    if (!usedAdaptiveLogic) {
      if (s.deployReleaseTriggered && !std::isnan(s.heightAGL) && s.heightAGL > CONTROL_STAGE_MIN_HEIGHT_M && s.heightAGL <= CONTROL_STAGE_MAX_HEIGHT_M) {
        alphaNextDeg = computeLegacySweepAlphaDeg(s.heightAGL);
        requestedBiasUs = lroundf((alphaNextDeg - alphaStoredDeg) * pitchStepUs);
      } else {
        requestedBiasUs = computeLegacyCollectiveBiasUs(s.missionState, s.heightAGL, s.rpm, s.rpm, simMs);
        alphaNextDeg = alphaStoredDeg;
      }
    }

    s.collectiveBiasUs = clampCollectiveBiasUs(requestedBiasUs, 1500, 1500);

    // Apply bias to servo PWMs (simulate)
    unsigned long servo1AdjustedPwmUs = (unsigned long)clampf(1500.0f - (float)s.collectiveBiasUs, (float)pwmMinUs, (float)pwmMaxUs);
    unsigned long servo2AdjustedPwmUs = (unsigned long)clampf(1500.0f + (float)s.collectiveBiasUs, (float)pwmMinUs, (float)pwmMaxUs);

    float servo3OutDeg = (servo1AdjustedPwmUs - pwmMinUs) * pwmDegPerUs;

    // update alpha
    s.alpha = clampf(alphaNextDeg, -12.0f, 12.0f);

    // update RPM dynamics toward optimal from stored alpha
    updateRpmDynamics(s.rpm, s.alpha, dtSec);

    // print telemetry
    std::cout << simMs << "\t" << s.altitude << "\t" << s.heightAGL << "\t" << s.rpm << "\t" << (int)s.missionState << "\t" << s.collectiveBiasUs << "\t" << s.alpha << "\n";

    s.lastAltitude = s.altitude;

    // advance time
    simTimeMs += dtMs;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << "Simulation finished.\n";
  return 0;
}
