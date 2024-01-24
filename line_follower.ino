#include <QTRSensors.h>
 
// Motor pins
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;
 
// Motor speeds
int m1Speed = 0;
int m2Speed = 0;
 
// PID controller parameters
float kp = 4.3;
float ki = 0.000;
float kd = 23.2;
int p = 1;
int i = 0;
int d = 0;
int error = 0;
int lastError = 0;
 
// Motor speed constraints
const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 255;
 
// Line following direction flag
bool left = false;
 
// QTR Sensor setup
QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = {0, 0, 0, 0, 0, 0};
 
void setup() {
  // Set pin modes for motors
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
 
  // Configure QTR sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);
  delay(500);
 
  // LED feedback during calibration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
 
  // Calibrate QTR sensors
  for (uint16_t i = 0; i < 300; i++) {
    qtr.calibrate();
    int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);
 
    // Adjust motor speed based on calibration
    if (left) {
      setMotorSpeed(-130, 130);
    } else {
      setMotorSpeed(130, -130);
    }
 
    // Change direction if necessary
    if (error > 35 && left || error < -35 && !left)
      left = !left;
  }
 
  // Turn off LED after calibration
  digitalWrite(LED_BUILTIN, LOW);
 
  // Initialize serial communication for debugging
  Serial.begin(9600);
}
 
void loop() {
  // Read sensor values and map to an error value
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);
 
  // Filter small errors
  if (abs(error) < 15) {
    error = 0;
  }
 
  // Update PID terms
  p = error;
  i = i + error;
  d = error - lastError;
 
  // Apply PID control
  pidControl(kp, ki, kd);
 
  // Update last error for the next iteration
  lastError = error;
}
 
void pidControl(float kp, float ki, float kd) {
  // Calculate motor speed using PID control formula
  int motorSpeed = kp * p + ki * i + kd * d;
 
  // Set base motor speeds
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;
 
  // Adjust motor speeds based on error
  if (error < -20) {
    m1Speed += motorSpeed;
  } else if (error > 20) {
    m2Speed -= motorSpeed;
  }
 
  // Constrain motor speeds within limits
  m1Speed = constrain(m1Speed, -80, maxSpeed);
  m2Speed = constrain(m2Speed, -80, maxSpeed);
 
  // Control motors with adjusted speeds
  setMotorSpeed(m1Speed, m2Speed);
}
 
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // Invert motor2Speed for proper direction
  motor2Speed = -motor2Speed;
 
  // Control motor 1
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
 
  // Control motor 2
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
