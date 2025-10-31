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


   sendToAllSlaves("Reverse,30");
  runMasterMotor("Reverse", 30);
  delay(3000);

   
  sendToAllSlaves("Stop,0");
  runMasterMotor("Stop", 0);
  delay(2000);
 
 
  sendToIndividualSlaves("Forward,50", 1);
  sendToIndividualSlaves("Forward,50", 2); 
  sendToIndividualSlaves("Reverse,50", 3); 
  runMasterMotor("Reverse",50); 
  delay(1400); // Adjust for ~90-degree turn

  sendToAllSlaves("Stop,0");
  runMasterMotor("Stop", 0);
  delay(2000);

    sendToAllSlaves("Reverse,30");
  runMasterMotor("Reverse", 30);
  delay(2000);

sendToAllSlaves("Stop,0");
  runMasterMotor("Stop", 0);
  delay(2000);

   sendToIndividualSlaves("Reverse,50", 1);
  sendToIndividualSlaves("Reverse,50", 2); 
  sendToIndividualSlaves("Forward,50", 3); 
  runMasterMotor("Forward",50); 
  delay(1400);

  
sendToAllSlaves("Stop,0");
  runMasterMotor("Stop", 0);
  delay(2000);

  sendToAllSlaves("Forward,30");
  runMasterMotor("Forward", 30);
  delay(2000);
  
sendToAllSlaves("Stop,0");
  runMasterMotor("Stop", 0);
  

 
}

// Broadcast to all slaves
void sendToAllSlaves(String command) {
  for (int addr = 1; addr <= 3; addr++) {
    Wire.beginTransmission(addr);
    Wire.write(command.c_str());
    Wire.endTransmission();
    delay(5);
  }
}

// Send to specific slave
void sendToIndividualSlaves(String command, int addr) {
  Wire.beginTransmission(addr);
  Wire.write(command.c_str());
  Wire.endTransmission();
  delay(5);
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
