#include <Arduino.h>
#include <ESP32Servo.h>

const int SERVO1_PIN = D1;
const int SERVO2_PIN = D2;
const int SERVO1_INPUT_PIN = D3;
const int SERVO2_INPUT_PIN = D4;

const int PWM_MIN_US = 1000;
const int PWM_MAX_US = 2000;

// Standard servo PWM-to-angle conversion:
// 1000 us = 0°, 2000 us = 180°
// Linear relationship: 180° span / 1000 us = 0.18°/us
const float PWM_CENTER_US = 1500.0;    // Center position (90°)
const float PWM_RANGE_US = 1000.0;     // Total range 1000-2000 us
const float DEG_PER_US = 180.0 / PWM_RANGE_US;  // 0.18°/us
const int PWM_OFFSET_US = 167;         // 30° offset in microseconds

Servo servo1;
Servo servo2;

unsigned long readPwmWidthUs(int pin) {
  return pulseIn(pin, HIGH, 25000);
}

// Convert PWM pulse width (1000-2000 us) to degrees (0-180)
float pulseToDegreesStandard(unsigned long pulseWidthUs) {
  pulseWidthUs = constrain(pulseWidthUs, PWM_MIN_US, PWM_MAX_US);
  return (pulseWidthUs - PWM_MIN_US) * DEG_PER_US;
}

// Servo 1: standard mapping
float servo1PulseToDegrees(unsigned long pulseWidthUs) {
  return pulseToDegreesStandard(pulseWidthUs);
}

// Servo 2: already reversed from the other device, so use standard mapping
// to preserve the reversal and mirror servo 1
float servo2PulseToDegrees(unsigned long pulseWidthUs) {
  return pulseToDegreesStandard(pulseWidthUs);
}

void setup() {
  Serial.begin(115200);
  pinMode(SERVO1_INPUT_PIN, INPUT);
  pinMode(SERVO2_INPUT_PIN, INPUT);

  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, PWM_MIN_US, PWM_MAX_US);
  servo2.attach(SERVO2_PIN, PWM_MIN_US, PWM_MAX_US);
}

void loop() {
  const unsigned long servo1PulseUs = readPwmWidthUs(SERVO1_INPUT_PIN);
  const unsigned long servo2PulseUs = readPwmWidthUs(SERVO2_INPUT_PIN);

  if (servo1PulseUs > 0) {
    unsigned long servo1PwmWithOffset = servo1PulseUs - PWM_OFFSET_US;
    servo1PwmWithOffset = constrain(servo1PwmWithOffset, PWM_MIN_US, PWM_MAX_US);
    servo1.writeMicroseconds(servo1PwmWithOffset);
    float servo1Deg = servo1PulseToDegrees(servo1PwmWithOffset);
    Serial.print("servo1: ");
    Serial.print(servo1PulseUs);
    Serial.print(" us -167us → ");
    Serial.print(servo1Deg);
    Serial.print("°");
  }

  if (servo2PulseUs > 0) {
    unsigned long servo2PwmWithOffset = servo2PulseUs + PWM_OFFSET_US;
    servo2PwmWithOffset = constrain(servo2PwmWithOffset, PWM_MIN_US, PWM_MAX_US);
    servo2.writeMicroseconds(servo2PwmWithOffset);
    float servo2Deg = servo2PulseToDegrees(servo2PwmWithOffset);
    if (servo1PulseUs > 0) Serial.print(" | ");
    Serial.print("servo2: ");
    Serial.print(servo2PulseUs);
    Serial.print(" us +167us → ");
    Serial.print(servo2Deg);
    Serial.print("°");
  }

  Serial.println();
}
