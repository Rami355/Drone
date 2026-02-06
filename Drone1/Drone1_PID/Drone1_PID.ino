#include <math.h>


/**********************************************************************************/
float angle_roll_constrained = 0.0f;
float angle_pitch_constrained = 0.0f;
int16_t gyro_y = 0;
int16_t gyro_x = 0;
int potFromNano = 0;

float desired_angle = 0.0f;
double throttle=1400;
/**********************************************************************************/


// ==== OUTER RATE LOOP ====
float outerErrorRoll, outerErrorPitch;
float desiredRateRoll = 0, desiredRatePitch = 0;
float outerKpRoll = 3.0f;
float outerKpPitch = 3.0f;

// ==== INNER RATE LOOP ====
float innerErrorRoll, innerErrorPitch;
float innerErrorRollPrev = 0, innerErrorPitchPrev = 0;
float pR = 0, pP = 0;
float iR = 0, iP = 0;
float dR = 0, dP = 0;

float kP_Roll = 0.015f, kI_Roll = 0.0f, kD_Roll = 0.0045f;
float kP_Pitch = 0.035f, kI_Pitch = 0.0f, kD_Pitch = 0.0045f;

float outputR = 0, outputP = 0;

// Begränsningar
const float RATE_I_LIM = 300.0f;     // anti-windup
const float RATE_OUT_LIM = 600.0f;   // motsvarar ungefär ditt gamla PID clamp

void setup() {

}


void loop() {
  static uint32_t tPrev = micros();
  uint32_t tNow = micros();
  float dt = (tNow - tPrev) * 1e-6f;
  tPrev = tNow;
  
  if (dt < 0.002f || dt > 0.01f) dt = 0.004f;

throttle = map(potFromNano, 0, 1023, 1000, 1500);   // snällare intervall först

//Nollställ I-led när throttle är låg
if (throttle < 1100) {
  iR = 0;
  iP = 0;
}

// ======= Calculate outer loop (angle) =======
outerErrorRoll = desired_angle - angle_roll_constrained;
outerErrorPitch = desired_angle - angle_pitch_constrained;

desiredRateRoll = outerKpRoll * outerErrorRoll;
desiredRatePitch = outerKpPitch * outerErrorPitch;

desiredRateRoll = constrain(desiredRateRoll, -120.0f, 120.0f);
desiredRatePitch = constrain(desiredRatePitch, -120.0f, 120.0f);

// ======= Calculate inner loop (acc) =======
float rateRoll_meas  =  (float)gyro_y / 65.5f;
float ratePitch_meas =  (float)gyro_x / 65.5f;

innerErrorRoll = desiredRateRoll - rateRoll_meas;
innerErrorPitch = desiredRatePitch - ratePitch_meas;

pR = kP_Roll * innerErrorRoll;
pP = kP_Pitch * innerErrorPitch;

iR += kI_Roll * innerErrorRoll * dt;
iP += kI_Pitch * innerErrorPitch * dt;
iR = constrain(iR, -RATE_I_LIM, RATE_I_LIM);
iP = constrain(iP, -RATE_I_LIM, RATE_I_LIM);

dR = kD_Roll * (innerErrorRoll - innerErrorRollPrev) / dt;
dP = kD_Pitch * (innerErrorPitch - innerErrorPitchPrev) / dt;

// ======= Calculate output =======
outputR = pR + iR + dR;
outputP = pP + iP + dP;

// Klamp på innerloopens utgång (detta blir din “R” och “P” till mixern)
outputR = constrain(outputR, -RATE_OUT_LIM, RATE_OUT_LIM);
outputP = constrain(outputP, -RATE_OUT_LIM, RATE_OUT_LIM);

// Håll en liten min-gas så att motorerna alltid snurrar lite
if (throttle < 1150) throttle = 1150;

float R = -outputR;   // Roll (fram/bak)
float P = outputP;   // Pitch (vänster/höger)  // byt till P = -PID2; om sidled blir fel

innerErrorRollPrev = innerErrorRoll;
innerErrorPitchPrev = innerErrorPitch;

}
