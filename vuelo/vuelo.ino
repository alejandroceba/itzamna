#include <Wire.h>
#include <MS5xxx.h>

#define MS5607_I2C_ADDR 0x77

MS5607 ms5607(&Wire);

void setup() {
  Serial.begin(115200);
  if(ms5607.connect()>0) {
    Serial.println("Error connecting...");
    delay(500);
    setup();
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
