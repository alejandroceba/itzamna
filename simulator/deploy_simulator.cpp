#include <iostream>
#include <chrono>
#include <thread>

enum MissionState { WAIT_FOR_RELEASE = 0, DEPLOY_PULSE = 1, UPPER_SWEEP = 2, LOWER_SWEEP = 3 };

const unsigned long DEPLOY_MAX_PITCH_DURATION_MS = 2000UL;
const float CONTROL_STAGE_MIN_HEIGHT_M = 20.0f;
const float CONTROL_STAGE_MAX_HEIGHT_M = 200.0f;
const float SPINUP_AGL_M = 60.0f;

bool deployWindowActive = false;
unsigned long deployStartMs = 0UL;
bool deployReleaseTriggered = false;

MissionState selectMissionStateSim(float heightAboveGroundM, unsigned long nowMs, bool isDescending) {
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

int main() {
  unsigned long now = 0;
  float height = 0.0f;
  bool isDescending = false;

  std::cout << "Simulating deploy sequence:\n";

  // Climb up past 200m
  for (int i = 0; i < 5; ++i) {
    height += 60.0f; now += 1000;
    std::cout << "t=" << now << " ms, height=" << height << "m, state=" << selectMissionStateSim(height, now, false) << "\n";
  }

  // Mark that we saw altitude above deploy threshold and then descend
  // Simulate second crossing: descend from 210 to 190 at t=6000
  isDescending = true;
  height = 210.0f; now += 1000;
  // Arm deploy window (as the firmware does when crossing while descending)
  deployWindowActive = true;
  deployStartMs = now;
  std::cout << "-- Armed deploy window at t=" << now << " ms --\n";

  // Print states for 0..3000ms after arming to show deploy pulse lasts DEPLOY_MAX_PITCH_DURATION_MS
  for (int dt = 0; dt <= 3000; dt += 250) {
    unsigned long t = now + dt;
    float h = 190.0f; // below threshold
    MissionState s = selectMissionStateSim(h, t, isDescending);
    std::cout << "t=" << t << " ms, state=" << s << ((s==DEPLOY_PULSE)?" (DEPLOY_PULSE)":"") << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << "Done. deployReleaseTriggered=" << (deployReleaseTriggered?"true":"false") << "\n";
  return 0;
}
