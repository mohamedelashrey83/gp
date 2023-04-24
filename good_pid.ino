#include <util/atomic.h>
#include <HCSR04.h>
#include <Wire.h>
// A class to compute the control signal
volatile float distance_front, distance_left, distance_right;

const int MPU = 0x68;  // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

class SimplePID {
private:
  float kp, kd, ki, umax;  // Parameters
  float eprev, eintegral;  // Storage

public:
  // Constructor
  SimplePID()
    : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir) {
    // error
    int e = target - value;

    // derivative
    float dedt = (e - eprev) / (deltaT);

    // integral
    eintegral = eintegral + e * deltaT;

    // control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // motor power
    pwr = (int)fabs(u);
    if (pwr > umax) {
      pwr = umax;
    }
    //Serial.println(pwr);

    // motor direction
    dir = 1;
    if (u < 0) {
      dir = -1;
    }

    // store previous error
    eprev = e;
  }
};

// How many motors
#define NMOTORS 2
// Pins
const int enca[] = { 18, 19 };
const int encb[] = { 17, 2 };
const int pwm[] = { 6, 13 };
const int in1[] = { 4, 12 };
const int in2[] = { 5, 11 };
int trigger_front = A0;
int echo_front = A1;
UltraSonicDistanceSensor frontSensor(trigger_front, echo_front);
// Globals
long prevT = 0;
volatile int posi[] = { 0, 0 };
// PID class instances
SimplePID pid[NMOTORS];

void INIT() {
  Serial.begin(9600);
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission

  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  Serial.println("target pos");
}

void setup() {
  INIT();
  //calculate_IMU_error();
  rotate(90.0);
  //Stop();
  //move_dist(10,10);
  //move_single_motor(0,22);
  //setMotor(1, 255, pwm[0], in1[0],in2[0]);
  //Stop();
  //setMotor(1, 35, pwm[0], in1[0], in2[0]);
}

void loop() {
   /*forward();
  float us = 0.0;
  bool b = false;
  us = read_front();
  Serial.println(us);
  if (us < 15) {
  Stop();
  delay(2000);
  rotate(-88);
  Stop();
  delay(2000);
  move_dist(22,22);
  Stop();
  delay(2000);
  rotate(-88);
  Stop();
  delay(2000);
  }*/
  /* Get new sensor events with the readings */
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
template<int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b > 0) {
    posi[j]++;
  } else {
    posi[j]--;
  }
}
void move_dist(int d1, int d2) {
  // set target position
  pid[0].setParams(3, 0.2, 0, 255);
  pid[1].setParams(3, 0.2, 0, 255);
  int target[NMOTORS];
  target[0] = round(d1 * 16.552);  //390 tick per revolution
  target[1] = round(d2 * 16.552);
  posi[0] = 0;
  posi[1] = 0;
  while (1) {
    // time difference
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;
    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS] = { 0, 0 };
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      for (int k = 0; k < NMOTORS; k++) {
        pos[k] = posi[k];
      }
    }
    if ((abs(pos[0]) >= abs(target[0])) && (abs(pos[1]) >= abs(target[1]))) {
      Stop();
      break;
    }
    // evaluate the control signal
    for (int k = 0; k < NMOTORS; k++) {
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);
      pwr = constrain(pwr, 120, 255);
      // signal the motor
      setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
    }
    for (int k = 0; k < NMOTORS; k++) {
      Serial.print(target[k]);
      Serial.print(" ");
      Serial.print(pos[k]);
      Serial.print(" ");
    }
    Serial.println();
    
  }
  delay(300);
}
void move_single_motor(int motor_num, int distance) {
  int target;
  target = round(distance * 16.552114);  //390 tick per revolution
  posi[motor_num] = 0;
  while (1) {
    // time difference
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;
    // Read the position in an atomic block to avoid a potential misread
    int pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = posi[motor_num];
    }
    // evaluate the control signal
    int pwr, dir;
    // evaluate the control signal
    pid[motor_num].evalu(pos, target, deltaT, pwr, dir);
    pwr = constrain(pwr, 120, 255);
    // signal the motor
    setMotor(dir, pwr, pwm[motor_num], in1[motor_num], in2[motor_num]);

    //}

    Serial.print(target);
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.println();
    if (abs(pos) >= abs(target)) {
      break;
    }
  }
}
float read_front() {
  distance_front = frontSensor.measureDistanceCm();
  if (distance_front < 0) { distance_front = 100; }
  Serial.println(distance_front);
  //delay(50);
  return distance_front;
}
void forward() {
  digitalWrite(in1[1], HIGH);
  digitalWrite(in2[1], LOW);
  digitalWrite(in1[0], HIGH);
  digitalWrite(in2[0], LOW);
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
  analogWrite(pwm[0], 0);
  analogWrite(pwm[1], 0);
  digitalWrite(in1[0], LOW);
  digitalWrite(in2[0], LOW);
  digitalWrite(in1[1], LOW);
  digitalWrite(in2[1], LOW);
 
}
bool has_reached() {
  int prev_posi[2];
  for (int i = 0; i < 2; i++) {
    prev_posi[i] = posi[i];
  }
  delay(10);
  if (posi[0] == prev_posi[0]) {
    if (posi[1] == prev_posi[1]) {
      return true;
    }
  };
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  /*while (c < 1000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }*/
  //Divide the sum by 200 to get the error value
  /*AccErrorX = AccErrorX / 1000;
  AccErrorY = AccErrorY / 1000;*/
  c = 0;
  // Read gyro values 200 times
  while (c < 1000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 1000;
  GyroErrorY = GyroErrorY / 1000;
  GyroErrorZ = GyroErrorZ / 1000;
  // Print the error values on the Serial Monitor
  /*Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);*/
}
void rotate(float angle) {
  pid[0].setParams(12, 5, 4, 255);
  pid[1].setParams(12, 5, 4, 255);
  float target = angle;
  yaw = 0;
  GyroErrorZ = 0;
  calculate_IMU_error();
  while (1) {
    readYaw();
    if (abs(yaw) >= abs(target)) {
      Stop();
      break;
    }
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;
    int pwr[2], dir[2];
    pid[1].evalu(yaw, target, deltaT, pwr[1], dir[1]);
    pwr[1] = constrain(pwr[0], 120, 255);
    setMotor(dir[1], pwr[1], pwm[1], in1[1], in2[1]);
    setMotor(-dir[1], pwr[1], pwm[0], in1[0], in2[0]);
    //pid[1].evalu(yaw, target, deltaT, pwr[1], dir[1]);
    //pwr[1] = constrain(pwr[1], 120, 255);
    //setMotor(dir[1], pwr[1], pwm[1], in1[1], in2[1]);
    Serial.print(target);
    Serial.print(" ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.println();
  }
  readYaw();
  Serial.print(target);
  Serial.print(" ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.println();
}
void readYaw() {
  previousTime = currentTime;                         // Previous time is stored before the actual time read
  currentTime = millis();                             // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);                    // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = GyroZ - (GyroErrorZ);  //~ (-0.8)
  yaw = yaw + GyroZ * elapsedTime;
}
void readMPU() {
  /* Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;       // ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;  // ~(-1.58)
    // === Read gyroscope data === //*/
  previousTime = currentTime;                         // Previous time is stored before the actual time read
  currentTime = millis();                             // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);                    // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  // GyroX = GyroX - GyroErrorX;    //~(-0.56)
  //GyroY = GyroY - GyroErrorY;    //~(2)
  GyroZ = GyroZ - (GyroErrorZ);  //~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  //gyroAngleX = gyroAngleX + GyroX * elapsedTime;  // deg/s * s = deg
  //gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw = yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  //roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  //pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  // Print the values on the serial monitor
  /* Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.println(yaw);*/
}