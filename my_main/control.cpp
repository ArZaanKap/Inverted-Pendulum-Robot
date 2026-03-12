#include "control.h"

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
  