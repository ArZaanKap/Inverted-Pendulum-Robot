#pragma once
#include <Arduino.h>

class LQR{
private:
  float x_des[4];
  float K[4] = {-104.182, -153.199, -1073.178, -135.522}; ///////////// change
  // contin : -104.182, -153.199, -1073.178, -135.522
  //-96.101, -141.808, -996.418, -127.575

public:
  LQR(const float X_des[4]);

  float run(const float current_state[4]); 
};


void jerk(float angle);