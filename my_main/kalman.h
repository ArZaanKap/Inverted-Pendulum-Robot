#pragma once
#include <Arduino.h>


class KalmanFilter{
private:

  float x[4]; // [x, xdot, theta, thetadot]

  float H[2][4] = {}; // observation mapping

  float P[4][4] = {}; // error covariance
  float Q[4][4] = {}; // process noise (trust in physics model)
  float R[2][2] = {}; // sensor noise (trust in sensors)

  // helper arrays
  float AP[4][4];
  float P_new[4][4];
  float S[2][2];
  float S_inv[2][2];
  float K[4][2];
  float y[2];

public:
  KalmanFilter(float init_state[4]);
  void predict(float u);
  void update(float measured_x, float measured_theta);
  void get_state(float *state);
};



