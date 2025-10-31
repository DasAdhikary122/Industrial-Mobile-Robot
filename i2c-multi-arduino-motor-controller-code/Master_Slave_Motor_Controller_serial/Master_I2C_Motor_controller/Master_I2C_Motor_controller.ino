/*
 * I2C Master Motor Controller 
 * 
 * This code controls one motor (Motor 4) directly using BTS7960
 * and communicates with slave Arduinos to control Motor 1, 2, and 3 over I2C.
 * 
 * Commands are sent over Serial in the following format:
 *     M1Forward M2Stop M3Reverse M4Forward
 * 
 * Author: Suman Das Adhikary
 */

#include <Wire.h>

String readString;

// Motor 4 (connected to Master) pin definitions
#define RPWM 5
#define LPWM 6
#define REN 11
#define LEN 12

void setup() {
  Wire.begin();              // Start I2C communication as Master
  Serial.begin(9600);        // Start serial monitor
  Serial.println("Enter commands like: M1Forward M2Stop M3Reverse M4Forward");

  // Setup Motor 4 pins (BTS7960 Driver)
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  digitalWrite(REN, HIGH);   // Enable BTS7960 inputs
  digitalWrite(LEN, HIGH);
}

void loop() {
  // Collect serial input
  while (Serial.available()) {
    delay(2);
    char c = Serial.read();
    readString += c;
  }

  if (readString.length() > 0) {
    readString.trim();
    Serial.println("Received: " + readString);

    // Break the input into individual motor commands
    while (readString.length() > 0) {
      int spaceIndex = readString.indexOf(' ');
      String cmd;

      if (spaceIndex != -1) {
        cmd = readString.substring(0, spaceIndex);
        readString = readString.substring(spaceIndex + 1);
        readString.trim();
      } else {
        cmd = readString;
        readString = "";
      }

      if (cmd.length() > 0) {
        processCommand(cmd);
      }
    }
  }
}

// Processes command string like "M1Forward"
void processCommand(String cmd) {
  int motorNum = 0;
  String action = "";

  if (cmd.startsWith("M1")) { motorNum = 1; action = cmd.substring(2); }
  else if (cmd.startsWith("M2")) { motorNum = 2; action = cmd.substring(2); }
  else if (cmd.startsWith("M3")) { motorNum = 3; action = cmd.substring(2); }
  else if (cmd.startsWith("M4")) { motorNum = 4; action = cmd.substring(2); }

  // Send to slave motors (M1 to M3)
  if (motorNum >= 1 && motorNum <= 3) {
    Wire.beginTransmission(motorNum);
    Wire.write(action.c_str());
    Wire.endTransmission();
    Serial.println("Sent to Slave M" + String(motorNum) + ": " + action);
  }
  // Apply command to Master Motor (M4)
  else if (motorNum == 4) {
    applyM4Command(action);
    Serial.println("Executed on Master M4: " + action);
  }
}

// Execute action (Forward, Reverse, Stop) on Motor 4
void applyM4Command(String command) {
  if (command == "Forward") {
    analogWrite(RPWM, 200);
    analogWrite(LPWM, 0);
  } else if (command == "Reverse") {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 200);
  } else if (command == "Stop") {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}
