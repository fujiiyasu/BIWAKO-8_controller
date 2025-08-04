#include <Wire.h>
#include <Servo.h>

// --- サーボピン定義 ---
byte servoPin1 = 3;
byte servoPin2 = 5;
byte servoPin3 = 6;
byte servoPin4 = 9;

// --- RC入力ピン定義 ---
int rc_pin1 = 2;
int rc_pin2 = 4;
int rc_pin3 = 7;
int rc_pin4 = 8;

// --- グローバル変数 ---
int rc_pin1_duration;
int rc_pin2_duration;
int rc_pin3_duration;
int rc_pin4_duration;
int mode = 0;  // 0: manual(RC), 1: fixed, 2: I2C control
int motor1_dir = 0;
int motor2_dir = 0;
int motor3_dir = 0;
int motor4_dir = 1;

Servo servo1, servo2, servo3, servo4;

// --- セット関数 ---
void set_thrusters(int d1, int d2, int d3, int d4) {
  servo1.writeMicroseconds(d1);
  servo2.writeMicroseconds(d2);
  servo3.writeMicroseconds(d3);
  servo4.writeMicroseconds(d4);
}

void setup() {
  int SLAVE_ADDRESS = 0x10;
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(ReceiveMessage);

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  set_thrusters(1500, 1500, 1500, 1500);

  Serial.println("Wait seven seconds....");
  delay(7000);
  Serial.println("Complete thrusters setup.");
}

void loop() {
  if (mode == 0) {
    rc_pin1_duration = duration_noise_cut(pulseIn(rc_pin1, HIGH));
    rc_pin2_duration = duration_noise_cut(pulseIn(rc_pin2, HIGH));
    rc_pin3_duration = duration_noise_cut(pulseIn(rc_pin3, HIGH));
    rc_pin4_duration = duration_noise_cut(pulseIn(rc_pin4, HIGH));

    if (motor1_dir == 1) rc_pin1_duration = 3000 - rc_pin1_duration;
    if (motor2_dir == 1) rc_pin2_duration = 3000 - rc_pin2_duration;
    if (motor3_dir == 1) rc_pin3_duration = 3000 - rc_pin3_duration;
    if (motor4_dir == 1) rc_pin4_duration = 3000 - rc_pin4_duration;

    set_thrusters(rc_pin1_duration, rc_pin2_duration, rc_pin3_duration, rc_pin4_duration);
  }
}

int duration_noise_cut(int duration) {
  if (1450 < duration && duration < 1550) {
    duration = 1500;
  }
  return duration;
}

void ReceiveMessage(int n) {
  int action = 0;
  int pwm = 0;
  mode = Wire.read();
  action = Wire.read();
  pwm = Wire.read();

  if (mode == 2) {
    thruster_control(action, pwm);
  }
}

// --- アクションに応じたスラスタ制御 ---
void thruster_control(int action, int pwm) {
  int power = map(pwm, 0, 100, 1500, 1900);
  int neutral = 1500;

  int d1 = power;
  int d2 = power;
  int d3 = power;
  int d4 = power;

  if (motor1_dir == 1) d1 = 3000 - d1;
  if (motor2_dir == 1) d2 = 3000 - d2;
  if (motor3_dir == 1) d3 = 3000 - d3;
  if (motor4_dir == 1) d4 = 3000 - d4;

  switch (action) {
    case 0: set_thrusters(neutral, neutral, neutral, neutral); break;
    case 1: set_thrusters(3000 - d1, 3000 - d2, d3, d4); break;
    case 2: set_thrusters(d1, d2, 3000 - d3, 3000 - d4); break;
    case 3: set_thrusters(3000 - d1, d2, 3000 - d3, d4); break;
    case 4: set_thrusters(d1, 3000 - d2, d3, 3000 - d4); break;
    case 5: set_thrusters(d1, 3000 - d2, 3000 - d3, d4); break;
    case 6: set_thrusters(3000 - d1, d2, d3, 3000 - d4); break;
    case 7: set_thrusters(neutral, 3000 - d2, d3, neutral); break;
    case 8: set_thrusters(3000 - d1, neutral, neutral, d4); break;
    case 9: set_thrusters(neutral, d2, 3000 - d3, neutral); break;
    case 10: set_thrusters(d1, neutral, neutral, 3000 - d4); break;
    case 11: set_thrusters(neutral, neutral, d3, d4); break;
    case 12: set_thrusters(d1, d2, neutral, neutral); break;
    case 13: set_thrusters(neutral, d2, neutral, d4); break;
    case 14: set_thrusters(d1, neutral, d3, neutral); break;
    case 15: set_thrusters(3000 - d1, 3000 - d2, neutral, neutral); break;
    case 16: set_thrusters(neutral, neutral, 3000 - d3, 3000 - d4); break;
    case 17: set_thrusters(3000 - d1, neutral, 3000 - d3, neutral); break;
    case 18: set_thrusters(neutral, 3000 - d2, neutral, 3000 - d4); break;
    default: set_thrusters(neutral, neutral, neutral, neutral); break;
  }

  Serial.print("PWM: ");
  Serial.println(pwm);
  Serial.print("Mapped Duration (d1): ");
  Serial.println(d1);
}
