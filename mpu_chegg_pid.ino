#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
const int pwm[] = { 6, 13 };
const int in1[] = { 5, 11 };
const int in2[] = { 4, 12 };
double targetAngle = 90.0;  // Target angle in degrees
double currentAngle = 0.0;  // Current angle in degrees
double error, lastError, errorSum, errorDiff;
double kp = 1.0;         // Proportional gain
double ki = 0.0;         // Integral gain
double kd = 0.0;         // Derivative gain
double pidOutput = 0.0;  // PID controller output

void setup() {
  Wire.begin();
  mpu.initialize();
  Serial.begin(9600);
}

void loop() {
  // Read accelerometer data
  //int16_t ax , ay , az;
  //mpu.getAcceleration(&ax ,&ay , &az );
  //double angle = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2)));
  //int16_t gx,gy,gz;
  // mpu.getRotation(&gx,&gy,&gz);
  //Serial.print("z  ==");Serial.println(gz);
  //Serial.print("y  ==");Serial.println(gy);
  //Serial.print("x  ==");Serial.println(gx);
  
  delay(1000);
  /*currentAngle = angle * 180.0 / M_PI;
  // Calculate PID output
  error = targetAngle - currentAngle;
  errorSum += error; //integration part of the pid
  errorDiff = error - lastError; // derivative part of the pid
  pidOutput = kp * error + ki * errorSum + kd * errorDiff;
  lastError = error;

  // Rotate the robot
  if (pidOutput > 0) {
    // Rotate clockwise 
    right();
    // Code to rotate clockwise
  } else {
    // Rotate counterclockwise
    left();
    // Code to rotate counterclockwise
  }

  // Print current angle and PID output
  Serial.print("Current angle: ");
  Serial.print(currentAngle);
  Serial.print(" degrees");
  Serial.print("\t PID output: ");
  Serial.print(pidOutput);
  Serial.println();
  
  delay(10);*/
}
void forward() {
  digitalWrite(in1[1], LOW);
  digitalWrite(in2[1], HIGH);
  digitalWrite(in1[0], LOW);
  digitalWrite(in2[0], HIGH);
  analogWrite(pwm[0], 255);
  analogWrite(pwm[1], 255);
}

void backward() {
  digitalWrite(in1[0], LOW);
  digitalWrite(in2[0], HIGH);
  digitalWrite(in1[1], LOW);
  digitalWrite(in2[1], HIGH);
  analogWrite(pwm[0], 255);
  analogWrite(pwm[1], 255);
}
void right() {
  digitalWrite(in1[0], HIGH);
  digitalWrite(in2[0], LOW);
  digitalWrite(in1[1], LOW);
  digitalWrite(in2[1], HIGH);
  analogWrite(pwm[0], 150);
  analogWrite(pwm[1], 150);
}

void left() {
  digitalWrite(in1[0], HIGH);
  digitalWrite(in2[0], HIGH);
  digitalWrite(in1[1], LOW);
  digitalWrite(in2[1], LOW);
  analogWrite(pwm[0], 150);
  analogWrite(pwm[1], 150);
}

void Stop() {
  digitalWrite(in1[0], LOW);
  digitalWrite(in2[0], LOW);
  digitalWrite(in1[1], LOW);
  digitalWrite(in2[1], LOW);
  analogWrite(pwm[0], 0);
  analogWrite(pwm[1], 0);
}