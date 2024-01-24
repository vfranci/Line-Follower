#include <QTRSensors.h>

const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;
int m1Speed = 0;
int m2Speed = 0;
float kp = 4.3;
float ki = 0.000;
float kd = 23.2;
int p = 1;
int i = 0;
int d = 0;
int error = 0;
int lastError = 0;
const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 255;
bool left = false;
QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = {0, 0, 0, 0, 0, 0};

void setup() {
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 300; i++) {
    qtr.calibrate();
    int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

    if (left) {
      setMotorSpeed(-130, 130);
    } else {
      setMotorSpeed(130, -130);
    }

    if (error > 35 && left || error < -35 && !left)
      left = !left;
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);
}

void loop() {
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

  if (abs(error) < 15) {
    error = 0;
  }

  p = error;
  i = i + error;
  d = error - lastError;

  pidControl(kp, ki, kd);

  lastError = error;
}

void pidControl(float kp, float ki, float kd) {
  int motorSpeed = kp * p + ki * i + kd * d;
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  if (error < -20) {
    m1Speed += motorSpeed;
  } else if (error > 20) {
    m2Speed -= motorSpeed;
  }

  m1Speed = constrain(m1Speed, -80, maxSpeed);
  m2Speed = constrain(m2Speed, -80, maxSpeed);
  setMotorSpeed(m1Speed, m2Speed);
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {
  motor2Speed = -motor2Speed;

  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }

  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}
