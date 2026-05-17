#include <Arduino.h>
#include <ESP32Servo.h>

const int SERVO1_PIN = D1;
const int SERVO2_PIN = D2;
Servo servo1;
Servo servo2;
int angle = 0;
int alpha = 0;
const int updateDelayMs = 20;

void setup() {
  Serial.begin(115200);
  //delay(200);
  //Serial.println("sweep.ino: starting single-servo test");

  randomSeed(esp_random());

  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  //Serial.print("Attaching servo to pin "); Serial.println(SERVO_PIN);
  servo1.attach(SERVO1_PIN, 500, 2500);
  servo2.attach(SERVO2_PIN, 500, 2500);
  //Serial.println("servo attached");

  // Quick diagnostic moves
  
  //Serial.println("Entering continuous sweep loop");
}

void loop() {
  for (angle = -45; angle <= 45; angle++) {
    alpha = angle + 1;
    servo1.write(alpha + 90);
    servo2.write(90 - alpha);
    delay(updateDelayMs);
  }

  for (angle = 45; angle >= -45; angle--) {
    alpha = angle + 1;
    servo1.write(alpha + 90);
    servo2.write(90 - alpha);
    delay(updateDelayMs);
  }

  //servo.write(angle);

  //Serial.print("Random offset: ");
  //Serial.print(offset);
  //Serial.print(" | Servo angle: ");
  //Serial.println(angle);

  //delay(200);



}
