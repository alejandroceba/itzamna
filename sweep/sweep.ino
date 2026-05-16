#include <Arduino.h>
#include <ESP32Servo.h>

const int SERVO_PIN = D1;
Servo servo;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("sweep.ino: starting single-servo test");

  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  Serial.print("Attaching servo to pin "); Serial.println(SERVO_PIN);
  servo.attach(SERVO_PIN, 500, 2500);
  Serial.println("servo attached");

  // Quick diagnostic moves
  servo.write(0);
  delay(200);
  servo.write(90);
  delay(200);
  servo.write(0);
  delay(200);

  Serial.println("Entering continuous sweep loop");
}

void loop() {
  const int stepDelayMs = 15;
  for (int angle = 0; angle <= 180; ++angle) {
    servo.write(angle);
    delay(stepDelayMs);
  }
  for (int angle = 180; angle >= 0; --angle) {
    servo.write(angle);
    delay(stepDelayMs);
  }
}
