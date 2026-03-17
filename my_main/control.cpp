#include "control.h"
#include "pendulum_encoder.h" // for jerk
#include "motors.h" // for jerk


LQR::LQR(const float X_des[4]){
  memcpy(x_des, X_des, sizeof(x_des));
}

float LQR::run(const float current_state[4]){

    float u = 0.0;

    // u = -K * current_state
    for (int i=0; i<4; i++){
      u += -K[i] * (current_state[i] - x_des[i]);
    }
    return u;
}
  

// output time - how long to jerk for
void jerk(float angle) {

    float K = 40000;  // 10deg becomes 0.4s = 400ms = 400,000us
    uint32_t jerk_duration_us = (uint32_t)(K * fabs(angle));  

    uint32_t start = micros();
    motors_setSpeedAll(800);

    while (micros() - start < jerk_duration_us) {
        float theta = get_pendulum_angle_rad();
        if (fabs(theta) < radians(2)) break;
    }

    motors_setSpeedAll(0);
}

