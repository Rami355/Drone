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
float rateI_R = 0, rateI_P = 0;
float outRateR = 0, outRateP = 0;
float kP_Roll = 0.015f, kI_Roll = 0.0f, kD_Roll = 0.0045f;
float kP_Pitch = 0.035f, kI_Pitch = 0.0f, kD_Pitch = 0.0045f;

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
  rateI_R = 0;
  rateI_P = 0;
}

outerErrorRoll = desired_angle - angle_roll_constrained;   // roll-vinkel fel (°)
outerErrorPitch = desired_angle - angle_pitch_constrained;  // pitch-vinkel fel (°)

// Outer-loop: bara P-led, ger rate-kommando (°/s)
desiredRateRoll = outerKpRoll * outerErrorRoll;
desiredRatePitch = outerKpPitch * outerErrorPitch;

// Begränsa önskade rates
desiredRateRoll = constrain(desiredRateRoll, -120.0f, 120.0f);
desiredRatePitch = constrain(desiredRatePitch, -120.0f, 120.0f);

// MPU 500 dps ⇒ 65.5 LSB/(°/s). Kolla att axlarna stämmer mot din orientering.
float rateRoll_meas  =  (float)gyro_y / 65.5f;  // din kod använder gyro_y för roll
float ratePitch_meas =  (float)gyro_x / 65.5f;  // och gyro_x för pitch

// Rate-fel
innerErrorRoll = desiredRateRoll - rateRoll_meas;
innerErrorPitch = desiredRatePitch - ratePitch_meas;

// P
float pR = kP_Roll * innerErrorRoll;
float pP = kP_Pitch * innerErrorPitch;

// I (trapezoidalt, klamp)
rateI_R += kI_Roll * innerErrorRoll * dt;
rateI_P += kI_Pitch * innerErrorPitch * dt;
rateI_R = constrain(rateI_R, -RATE_I_LIM, RATE_I_LIM);
rateI_P = constrain(rateI_P, -RATE_I_LIM, RATE_I_LIM);

// D på felet (diskret derivata). Med låg DLPF räcker detta.
float dR = kD_Roll * (innerErrorRoll - innerErrorRollPrev) / dt;
float dP = kD_Pitch * (innerErrorPitch - innerErrorPitchPrev) / dt;
innerErrorRollPrev = innerErrorRoll;
innerErrorPitchPrev = innerErrorPitch;

// Rate-utgång
outRateR = pR + rateI_R + dR;
outRateP = pP + rateI_P + dP;

// Klamp på innerloopens utgång (detta blir din “R” och “P” till mixern)
outRateR = constrain(outRateR, -RATE_OUT_LIM, RATE_OUT_LIM);
outRateP = constrain(outRateP, -RATE_OUT_LIM, RATE_OUT_LIM);

// Håll en liten min-gas så att motorerna alltid snurrar lite
if (throttle < 1150) throttle = 1150;

float R = -outRateR;   // Roll (fram/bak)
float P = outRateP;   // Pitch (vänster/höger)  // byt till P = -PID2; om sidled blir fel

}
