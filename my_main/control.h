#pragma once
#include <Arduino.h>

class LQR{
private:
  float x_des[4];
  float K[4] = {-91.382, -134.846, -947.433, -121.270}; ///////////// change
  // contin - -99.081, -145.698, -1020.569, -128.843

public:
  LQR(const float X_des[4]);

  float run(const float current_state[4]); 
};


void jerk(float angle);