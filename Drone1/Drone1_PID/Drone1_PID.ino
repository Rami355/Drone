#include <math.h>


/**********************************************************************************/
float angle_roll_constrained = 0.0f;
float angle_pitch_constrained = 0.0f;
int16_t gyro_y = 0;
int16_t gyro_x = 0;
int potFromNano = 0;

float desired_angle = 0.0f;

/**********************************************************************************/


float error1, error2, previous_error1, previous_error2;
double throttle=1400;

// ==== INNER RATE LOOP (Roll/Pitch) ====
float rateErrR, rateErrP;
float ratePrevErrR = 0, ratePrevErrP = 0;
float rateI_R = 0, rateI_P = 0;
float outRateR = 0, outRateP = 0;   // detta går in i mixern som R,P

// Outer (attitude) -> DesiredRate i °/s
float desiredRateR = 0, desiredRateP = 0;

// Outer-loop gains (vinkel -> rate)
float kP_att_R = 3.0f;  // start: 3–8
float kP_att_P = 3.0f;  // samma som ovan

// Inner rate-loop gains (rate PID)
// Börja försiktigt; höj P först, sedan D; I sist.
float kP_rate_R = 0.015f, kI_rate_R = 0.0f, kD_rate_R = 0.0045f;
float kP_rate_P = 0.035f, kI_rate_P = 0.0f, kD_rate_P = 0.0045f;

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

error1 = desired_angle - angle_roll_constrained;   // roll-vinkel fel (°)
error2 = desired_angle - angle_pitch_constrained;  // pitch-vinkel fel (°)

// Outer-loop: bara P-led, ger rate-kommando (°/s)
desiredRateR = kP_att_R * error1;
desiredRateP = kP_att_P * error2;

// Begränsa önskade rates
desiredRateR = constrain(desiredRateR, -120.0f, 120.0f);
desiredRateP = constrain(desiredRateP, -120.0f, 120.0f);

// MPU 500 dps ⇒ 65.5 LSB/(°/s). Kolla att axlarna stämmer mot din orientering.
float rateRoll_meas  =  (float)gyro_y / 65.5f;  // din kod använder gyro_y för roll
float ratePitch_meas =  (float)gyro_x / 65.5f;  // och gyro_x för pitch

// Rate-fel
rateErrR = desiredRateR - rateRoll_meas;
rateErrP = desiredRateP - ratePitch_meas;

// P
float pR = kP_rate_R * rateErrR;
float pP = kP_rate_P * rateErrP;

// I (trapezoidalt, klamp)
rateI_R += kI_rate_R * rateErrR * dt;
rateI_P += kI_rate_P * rateErrP * dt;
rateI_R = constrain(rateI_R, -RATE_I_LIM, RATE_I_LIM);
rateI_P = constrain(rateI_P, -RATE_I_LIM, RATE_I_LIM);

// D på felet (diskret derivata). Med låg DLPF räcker detta.
float dR = kD_rate_R * (rateErrR - ratePrevErrR) / dt;
float dP = kD_rate_P * (rateErrP - ratePrevErrP) / dt;
ratePrevErrR = rateErrR;
ratePrevErrP = rateErrP;

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
