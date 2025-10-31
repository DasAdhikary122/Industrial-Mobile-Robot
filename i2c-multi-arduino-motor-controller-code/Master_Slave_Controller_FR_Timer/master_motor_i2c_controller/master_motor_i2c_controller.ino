#include <Wire.h>


#define RPWM 5
#define LPWM 6
#define REN 11
#define LEN 12

void setup() {
  Wire.begin(); // Master mode
  Serial.begin(9600);

 
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
}

void loop() {
  // 1. Forward 200 PWM
  sendToAllSlaves("Forward,50");
  runMasterMotor("Forward", 50);
  delay(5000);  // Run for 5 sec

  // 2. Reverse 50 PWM
  sendToAllSlaves("Reverse,50");
  runMasterMotor("Reverse", 50);
  delay(4000);  // Run for 4 sec

  // 3. Stop 
  sendToAllSlaves("Stop,0");
  runMasterMotor("Stop", 0);
  delay(3000);  // Wait 3 sec
}

void sendToAllSlaves(String command) {
  for (int addr = 1; addr <= 3; addr++) {
    Wire.beginTransmission(addr);
    Wire.write(command.c_str());
    Wire.endTransmission();
    delay(5); // Small delay between commands
  }
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
