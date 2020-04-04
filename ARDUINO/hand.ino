#include "pid.h"

#define A_CTRL_PIN A5
#define A_F5_PIN A0

#define M_F5_SPEED 5 
#define M_F5_DIR 4

const uint8_t ctrlF5Pin = A_CTRL_PIN;
const uint8_t sensorF5Pin = A_F5_PIN;

const uint8_t motorF5SpeedPin = M_F5_SPEED;
const uint8_t motorF5DirPin = M_F5_DIR;

double sensorPositionF5 = 0;
double ctrlPositionF5 = 0;
double ctrlPositionMinF5 = 180;
double ctrlPositionMaxF5 = 700;

double diffPositionF5 = 0;
double diffThresholdF5 = 22;

double motorSpeedF5 = 0;
double motorSpeedMaxF5 = 255;
double motorSpeedMinF5 = 0;

double f5Kp = 2;
double f5Ki = 0.1;
double f5Kd = 0;
double pidSpeedF5 = 0;

PID F5PID(0.1, motorSpeedMaxF5, motorSpeedMinF5, f5Kp, f5Kd, f5Ki);

void setup() {
  Serial.begin(115200);
  pinMode(motorF5SpeedPin, OUTPUT);
  pinMode(motorF5DirPin, OUTPUT);
}

void motorF5Handler() {
  if (motorSpeedF5 >= 0)
    digitalWrite(motorF5DirPin, 1);
  else
    digitalWrite(motorF5DirPin, 0);
  
  analogWrite(motorF5SpeedPin, abs(motorSpeedF5)); 
}

void readCtrlPosition(){
  double _ctrlPositionF5 = analogRead(ctrlF5Pin);
  ctrlPositionF5 = map(_ctrlPositionF5, 0, 1024, ctrlPositionMinF5, ctrlPositionMaxF5);
}

void readSensorPosition(){
  sensorPositionF5 = analogRead(sensorF5Pin);
}

void motorF5Control() {
  diffPositionF5 = ctrlPositionF5 - sensorPositionF5;
//  double m = 0;
//  if (ctrlPositionF5 >= sensorPositionF5) {
//    diffPositionF5 = ctrlPositionF5 - sensorPositionF5;
//    m = -1;
//  }
//  else {
//    diffPositionF5 = sensorPositionF5 - ctrlPositionF5;
//    m = 1;
//  }
    
  if (diffPositionF5 >= -1 * diffThresholdF5 && diffPositionF5 <= diffThresholdF5) {
    diffPositionF5 = 0;
    pidSpeedF5 = 0;
  } else {
  
    
//  if (diffPositionF5 >= 0 && diffPositionF5 <= diffThresholdF5)
//    diffPositionF5 = 0;

  pidSpeedF5 = F5PID.calculate(0, -1 * abs(diffPositionF5));
  }
//  pidSpeedF5 = F5PID.calculate(sensorPositionF5, ctrlPositionF5);

  if (diffPositionF5 >= 0)
    motorSpeedF5 = -1 * pidSpeedF5;
  else
    motorSpeedF5 = 1 * pidSpeedF5;

}

void debugPrint() {
  Serial.print("ctrl: ");
  Serial.print(ctrlPositionF5);
  Serial.print("  sensor: ");
  Serial.print(sensorPositionF5);
  Serial.print("  diff: ");
  Serial.print(diffPositionF5);
  Serial.print("  pid speed:  ");
  Serial.print(pidSpeedF5);
  Serial.print("  m speed:  ");
  Serial.print(motorSpeedF5);
  Serial.println();
}

void loop() {
  readCtrlPosition();
  readSensorPosition();

  motorF5Control();
  motorF5Handler();
  
  debugPrint();
}
