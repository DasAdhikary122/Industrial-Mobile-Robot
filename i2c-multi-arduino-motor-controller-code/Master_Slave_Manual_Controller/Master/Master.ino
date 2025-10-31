#include <Wire.h>

// Master Motor Pins (Right Rear Motor)
#define RPWM 5
#define LPWM 6
#define REN 11
#define LEN 12

// Receiver Channels
#define THROTTLE_PIN A0
#define STEERING_PIN A1
void sendToAllSlaves(String direction, int pwm = 0);

void setup() {
  Wire.begin();  // I2C Master
  Serial.begin(9600);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);
}

void loop() {
  int throttle = pulseIn(THROTTLE_PIN, HIGH, 50000);
  int steering = pulseIn(STEERING_PIN, HIGH, 50000);

  if (throttle < 1000 || throttle > 2000 || steering < 1000 || steering > 2000) {
    stopAllMotors();
    return;
  }

  // Auto-Trigger Left/Right Turn
  if (steering < 1100) {
    autoLeftTurn();
    return;
  } else if (steering > 1900) {
    autoRightTurn();
    return;
  }

  int throttle_offset = throttle - 1500;
  int steering_offset = steering - 1500;

  if (abs(throttle_offset) < 20) throttle_offset = 0;
  if (abs(steering_offset) < 20) steering_offset = 0;

  int left_speed = throttle_offset - (steering_offset * 0.5);
  int right_speed = throttle_offset + (steering_offset * 0.5);

  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

  String left_dir = (left_speed > 10) ? "Forward" : (left_speed < -10 ? "Reverse" : "Stop");
  String right_dir = (right_speed > 10) ? "Forward" : (right_speed < -10 ? "Reverse" : "Stop");

  int left_pwm = abs(left_speed);
  int right_pwm = abs(right_speed);

 
  runMasterMotor(right_dir, right_pwm);


  sendToSlave(1, left_dir, left_pwm);
  sendToSlave(2, left_dir, left_pwm);
  sendToSlave(3, right_dir, right_pwm);

  delay(50);
}

void stopAllMotors() {
  runMasterMotor("Stop", 0);
  sendToAllSlaves("Stop", 0);
}

void runMasterMotor(String direction, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (direction == "Forward") {
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (direction == "Reverse") {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void sendToSlave(int address, String direction, int pwm) {
  String command = direction + "," + String(pwm);
  Wire.beginTransmission(address);
  Wire.write(command.c_str());
  Wire.endTransmission();
  delay(5);
}

void sendToAllSlaves(String direction, int pwm = 0) {
  for (int addr = 1; addr <= 3; addr++) {
    sendToSlave(addr, direction, pwm);
  }
}

void sendToIndividualSlaves(String direction, int addr) {
  Wire.beginTransmission(addr);
  Wire.write(direction.c_str());
  Wire.endTransmission();
  delay(5);
}

// Auto Turn Left Routine 
void autoLeftTurn() {
  Serial.println("Auto LEFT turn");

  sendToAllSlaves("Stop", 0);
  runMasterMotor("Stop", 0);
  delay(2000);

  sendToAllSlaves("Reverse", 30);
  runMasterMotor("Reverse", 30);
  delay(2000);

  sendToAllSlaves("Stop", 0);
  runMasterMotor("Stop", 0);
  delay(2000);

  sendToIndividualSlaves("Reverse,50", 1);
  sendToIndividualSlaves("Reverse,50", 2);
  sendToIndividualSlaves("Forward,50", 3);
  runMasterMotor("Forward", 50);
  delay(1400);

  sendToAllSlaves("Stop", 0);
  runMasterMotor("Stop", 0);
  delay(2000);

  sendToAllSlaves("Forward", 30);
  runMasterMotor("Forward", 30);
  delay(2000);

  sendToAllSlaves("Stop", 0);
  runMasterMotor("Stop", 0);
}

//Auto Turn Right Routine (Same as Left, or modify if needed)
void autoRightTurn() {
  Serial.println("Auto RIGHT turn");

  sendToAllSlaves("Stop", 0);
  runMasterMotor("Stop", 0);
  delay(2000);

  sendToAllSlaves("Reverse", 30);
  runMasterMotor("Reverse", 30);
  delay(2000);

  sendToAllSlaves("Stop", 0);
  runMasterMotor("Stop", 0);
  delay(2000);

  sendToIndividualSlaves("Forward,50", 1);
  sendToIndividualSlaves("Forward,50", 2);
  sendToIndividualSlaves("Reverse,50", 3);
  runMasterMotor("Reverse", 50);
  delay(1400);

  sendToAllSlaves("Stop", 0);
  runMasterMotor("Stop", 0);
  delay(2000);

  sendToAllSlaves("Forward", 30);
  runMasterMotor("Forward", 30);
  delay(2000);

  sendToAllSlaves("Stop", 0);
  runMasterMotor("Stop", 0);
}
