#include <Wire.h>

#define RPWM 5
#define LPWM 6
#define REN 7
#define LEN 8

String command = "";

void setup() {
  Wire.begin(2); 
  Wire.onReceive(receiveCommand);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
}

void loop() {
  // Nothing here needed
}

void receiveCommand(int howMany) {
  command = "";
  while (Wire.available()) {
    char c = Wire.read();
    command += c;
  }

  // Parse command
  String direction = command.substring(0, command.indexOf(','));
  int pwm = command.substring(command.indexOf(',') + 1).toInt();
  pwm = constrain(pwm, 0, 255);

  if (direction == "Forward") {
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (direction == "Reverse") {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  } else { // Stop or any unknown command
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}
