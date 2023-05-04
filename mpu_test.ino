#include "Adafruit_MPU6050.h"
Adafruit_MPU6050 mpu;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (mpu.begin() == false) {
    Serial.println("failed connction...");
  }
  Serial.println("conncection successful..");
}

void loop() {
  // put your main code here, to run repeatedly:
  /*sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  Serial.println(g.orientation.heading);
  //if (g.orientation.z > 358) { Serial.println("full rotation........"); }*/
}
