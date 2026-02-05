/*************************************************
 * Includes
 *************************************************/
#include <Servo.h>

/*************************************************
 * Variables
 *************************************************/
Servo motorM1; //Motor front-left
Servo motorM2; //Motor front-right
Servo motorM3; //Motor back-left
Servo motorM4; //Motor back-right

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

/**
 * @brief Initializes and attaches all four motor outputs (ESC/servo signals).
 *
 * This function configures the PWM/servo output channels used to drive the
 * quadcopter's four ESCs (or any device expecting servo-style pulses).
 * Each Servo channel is attached with a defined pulse range:
 *  - 1000 µs = minimum throttle / idle
 *  - 2000 µs = maximum throttle
 *
 * Motor mapping (matches the quad layout documentation below):
 *  - M1 = Front Left  (pin 5)
 *  - M2 = Front Right (pin 6)
 *  - M3 = Back  Left  (pin 9)
 *  - M4 = Back  Right (pin 10)
 */
void motor_setup() {
  motorM1.attach(5, 1000, 2000);
  motorM2.attach(6, 1000, 2000);
  motorM3.attach(9, 1000, 2000);
  motorM4.attach(10, 1000, 2000);
}


/**
 * @brief Computes motor outputs using a Roll/Pitch mixer and writes PWM pulses.
 *
 *         QUADCOPTER MOTOR LAYOUT & AXES (X-CONFIG)
 *
 *                          P+  
 *                 (Pitch Up / Nose Up)
 *
 *                 M1 (FL)        M2 (FR)
 *                  [ ]------------[ ]
 *                    \            /
 *                     \          /
 *                      \        /
 *                       \      /
 *    R+ (Roll Left)       BODY         R- (Roll Right) 
 *                       /      \
 *                      /        \
 *                     /          \
 *                    /            \
 *                  [ ]------------[ ]
 *                 M3 (BL)        M4 (BR)
 *
 *                          P-  
 *                (Pitch Down / Nose Down)
 *
 *  Motor positions:
 *  M1 = Front Left
 *  M2 = Front Right
 *  M3 = Back Left
 *  M4 = Back Right
 *
 *  Control effects:
 *  Roll  +R → Right motors (M2, M4) increase
 *  Roll  -R → Left motors  (M1, M3) increase
 *
 *  Pitch +P → Back motors  (M3, M4) increase
 *  Pitch -P → Front motors (M1, M2) increase
 *
 *************************************************/
void motor_drive(double throttle, float R, float P) {
  
  float MotorInput1 = throttle - R - P;
  float MotorInput2 = throttle + R - P;
  float MotorInput3 = throttle - R + P;
  float MotorInput4 = throttle + R + P;

  MotorInput1 = constrain(MotorInput1, 1000, 2000);
  MotorInput2 = constrain(MotorInput2, 1000, 2000);
  MotorInput3 = constrain(MotorInput3, 1000, 2000);
  MotorInput4 = constrain(MotorInput4, 1000, 2000);

  motorM1.writeMicroseconds((int)MotorInput1);
  motorM2.writeMicroseconds((int)MotorInput2);
  motorM3.writeMicroseconds((int)MotorInput3);
  motorM4.writeMicroseconds((int)MotorInput4);

}