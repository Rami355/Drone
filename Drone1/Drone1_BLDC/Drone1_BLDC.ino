#include <Servo.h>

Servo motorRollM;   // Pin 9
Servo motorRollP;  // Pin 10
Servo motorPitchM;  // Pin 11
Servo motorPitchP;  // Pin 12

float R = 0;
float P = 0;

double throttle=1400;

void setup() {
  motor_setup();
  Serial.begin(500000);
}

void loop() {


motor_drive(throttle, R, P);

}

/**********************************************************************************/
void motor_setup() {
  motorRollM.attach(9, 1000, 2000);
  motorRollP.attach(10, 1000, 2000);
  motorPitchM.attach(11, 1000, 2000);
  motorPitchP.attach(12, 1000, 2000);
}

void motor_drive(double throttle, float R, float P) {
  
// Krav: R+ ska öka M2 & M3 (fram-motorer)
//       P+ ska öka M3 & M4 (höger sida) — byt tecken på P om du vill tvärtom
float MotorInput1 = throttle - R - P; // M1 bak-vänster
float MotorInput2 = throttle + R - P; // M2 fram-vänster
float MotorInput3 = throttle + R + P; // M3 fram-höger
float MotorInput4 = throttle - R + P; // M4 bak-höger

// Begränsa och skriv ut
MotorInput1 = constrain(MotorInput1, 1000, 2000);
MotorInput2 = constrain(MotorInput2, 1000, 2000);
MotorInput3 = constrain(MotorInput3, 1000, 2000);
MotorInput4 = constrain(MotorInput4, 1000, 2000);

motorRollM.writeMicroseconds((int)MotorInput1);   // M1
motorPitchP.writeMicroseconds((int)MotorInput2);  // M2
motorRollP.writeMicroseconds((int)MotorInput3);   // M3
motorPitchM.writeMicroseconds((int)MotorInput4);  // M4

}
/**********************************************************************************/