#include <iostream>
#include <cmath>
#include <vector>

const float OPTIMAL_RPM_ALPHA_CONSTANT = 4810.0544f;

float computeOptimalRpmFromAlphaDeg(float alphaDeg) {
  if (std::isnan(alphaDeg)) return NAN;
  if (alphaDeg <= 0.0f) return 1300.0f;
  return OPTIMAL_RPM_ALPHA_CONSTANT / std::sqrt(alphaDeg);
}

float computeAdaptiveAlphaDeg(float heightAboveGroundM, float descentRateMps, float rpmNow, float alphaStoredDeg) {
  if (std::isnan(heightAboveGroundM)) return NAN;
  if (heightAboveGroundM <= 20.0f) {
    if (std::isnan(rpmNow) || rpmNow < 1300.0f) return 12.0f;
    return alphaStoredDeg;
  }
  if (heightAboveGroundM > 200.0f) return NAN;
  float rpmOptimal = computeOptimalRpmFromAlphaDeg(alphaStoredDeg);
  if (std::isnan(rpmNow) || std::isnan(rpmOptimal) || std::isnan(descentRateMps)) return NAN;
  if (rpmNow < rpmOptimal) {
    if (descentRateMps > 12.0f) return std::fmax(-12.0f, alphaStoredDeg - 0.5f);
    return std::fmax(-12.0f, alphaStoredDeg - 1.0f);
  }
  if (rpmNow > rpmOptimal && descentRateMps > 12.0f) return std::fmin(12.0f, alphaStoredDeg + 1.0f);
  return alphaStoredDeg;
}

int main() {
  std::vector<float> heights = {150.0f, 50.0f, 25.0f, 20.0f, 15.0f};
  std::vector<float> rpms = {1200.0f, 1300.0f, 1400.0f, NAN};
  std::vector<float> descs = {0.0f, 5.0f, 15.0f};
  float alphaStored = 6.0f;

  std::cout << "Adaptive alpha test harness\n";
  std::cout << "h(m), rpm, dHdt(m/s) -> adaptive alpha\n";
  for (float h : heights) {
    for (float r : rpms) {
      for (float d : descs) {
        float a = computeAdaptiveAlphaDeg(h, d, r, alphaStored);
        std::cout << h << ", ";
        if (std::isnan(r)) std::cout << "N/A"; else std::cout << r;
        std::cout << ", " << d << " -> ";
        if (std::isnan(a)) std::cout << "NAN"; else std::cout << a;
        std::cout << "\n";
      }
    }
  }
  return 0;
}
