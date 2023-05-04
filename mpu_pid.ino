
#include <Wire.h>
#include "TimerOne.h"
#include "header.h"
#include <MsTimer2.h>
//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;
// Setup timers and temp variables
long loop_timer;
int temp;


const unsigned long interval = 50;  //in millisecond // the sampling frequency of the mpu6050 in default is 250 HZ
float angle_rotated;
void task() {
  //float angle = 0.0;
  read_mpu_6050_data();
  /* gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  angle_pitch += (gyro_x / 65.5) * 0.004;  //500 deg per second gyroscope and periodic task every 4 ms*/
  //delay(500);*/
  Serial.println("mohamed");
  Serial.println(gyro_z);
}
void setup() {
  // put your setup code here, to run once:
  //cli();
  Serial.begin(9600);
  Wire.begin();

  //Read the raw acc and gyro data from the MPU-6050 1000 times
  Serial.println("start calibration...");
  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    read_mpu_6050_data();
    //Add the gyro x offset to the gyro_x_cal variable
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable
    gyro_y_cal += gyro_y;
    //Add the gyro z offset to the gyro_z_cal variable
    gyro_z_cal += gyro_z;
    //Delay 3us to have 250Hz for-loop
    delay(3);
  }

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;
  Serial.println("calibration ended ...");
  setup_mpu_6050_registers();
  delay(1000);
  // sei();
  MsTimer2::set(50, task);
  MsTimer2::start();
  unsigned long prev_time = micros();
  //Timer1.initialize(interval * 1000);  // Set timer interval in microseconds
  //Timer1.attachInterrupt(task);
}

void loop() {
}
void setup_mpu_6050_registers() {

  //Activate the MPU-6050

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x6B);
  //Set the requested starting register
  Wire.write(0x00);
  //End the transmission
  Wire.endTransmission();

  //Configure the accelerometer (+/-8g)

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x1C);
  //Set the requested starting register
  Wire.write(0x10);
  //End the transmission
  Wire.endTransmission();

  //Configure the gyro (500dps full scale)

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x1B);
  //Set the requested starting register
  Wire.write(0x08);
  //End the transmission
  Wire.endTransmission();
}
void read_mpu_6050_data() {

  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x3B);
  //End the transmission
  Wire.endTransmission();
  //Request 14 bytes from the MPU-6050
  Wire.requestFrom(0x68, 14);
  //Wait until all the bytes are received
  while (Wire.available() < 14)
    ;

  //Following statements left shift 8 bits, then bitwise OR.
  //Turns two 8-bit values into one 16-bit value
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void rotate_pid_(float angle) {
  float angle;
  float curr_angle;
  long start_time = micros();
  read_mpu_6050_data();
    gyro_x -= gyro_x_cal;
    gyro_y -= gyro_y_cal;
    gyro_z -= gyro_z_cal;
    angle = (gyro_z / 65.5) * 0.004;
  while (micros() - start_time < 4000) {
  }
}
