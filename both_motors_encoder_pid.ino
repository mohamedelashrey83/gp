#include <util/atomic.h>

// A class to compute the control signal
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
const int enca[] = { 2, 18 };
const int encb[] = { 20, 19 };
const int pwm[] = { 6, 13 };
const int in1[] = { 5, 12 };
const int in2[] = { 4, 11 };

// Globals
long prevT = 0;
volatile int posi[] = { 0, 0 };
volatile int old_pos = 0;
volatile int new_pos = 0;
int num_of_loops = 2;
int l = 0;
// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(9600);

  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);

    pid[k].setParams(1.5, 0.025, 0.0, 255);
  }

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);

  Serial.println("target pos");
}

void loop() {
  // set target position
  int target[NMOTORS];
  //target[0] = 750*sin(prevT/1e6);
  //target[1] = -750*sin(prevT/1e6);
  target[0] = 1200;
  target[1] = 1200;
  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int k = 0; k < NMOTORS; k++) {
      pos[k] = posi[k];
    }
  }

  // loop through the motors
  //for (int k = 0; k < NMOTORS; k++) {
    int pwr, dir;
    //if (k == 0) {
      // evaluate the control signal
      //pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);
      pid[0].evalu(pos[0], target[0], deltaT, pwr, dir);

      // signal the motor
      //setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
      setMotor(dir, pwr, pwm[0], in1[0], in2[0]);
      setMotor(dir, pwr, pwm[1], in1[1], in2[1]);
    /*} else if (k == 1 &&new_pos != old_pos  ) {
      pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);
      // signal the motor
      setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
    }*/
    /*if ( new_pos == old_pos ) {
      //Serial.println("in the check");
      l = 0;
      pid[1].evalu(pos[1], pos[0], deltaT, pwr, dir);
      // signal the motor
      setMotor(dir, pwr, pwm[1], in1[1], in2[1]);
    }*/
  

  
  for (int k = 0; k < NMOTORS; k++) {
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
  }
  Serial.println();
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
  //new_pos = posi[0];
}

float num_rev = (dist * 10) / wheel_c;  // Convert to mm
  unsigned long target_count = num_rev * counts_per_rev;

void rotate_90_right() {
}