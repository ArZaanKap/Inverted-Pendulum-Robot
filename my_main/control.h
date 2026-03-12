#pragma once
#include <Arduino.h>

class LQR{
private:
  float x_des[4];
  float K[4] = {-99.081f, -145.698f, -1020.569f, -128.843f}; ///////////// change

public:
  LQR(const float X_des[4]);

  float run(const float current_state[4]); 
};