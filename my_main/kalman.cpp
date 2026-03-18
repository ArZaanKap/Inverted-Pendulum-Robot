#include "kalman.h"
#include "state_space.h"
#include <string.h>
#include <math.h>


KalmanFilter::KalmanFilter(float init_state[4]){

  memcpy(x, init_state, sizeof(x)); // initialise state as init state

  // zero everything
  memset(P, 0, sizeof(P));
  memset(Q, 0, sizeof(Q));
  memset(R, 0, sizeof(R));
  memset(H, 0, sizeof(H));

  // init P - high uncertainty at start
  for (int i=0; i<4; i++) P[i][i] = 1.0f; // identity matrix
  //P[0][0] = 0.01;   // x
  //P[1][1] = 0.01;   // x_dot
  //P[2][2] = 0.1;   // theta
  //P[3][3] = 0.1;   // theta_dot


  // H - x, theta is measured
  H[0][0] = 1.0f;
  H[1][2] = 1.0f;

  // Q PROCESS NOISE -> tune: trust in physics model ///////// higher means less trust in physics model -> more trust in sensors
  Q[0][0] = 0.001f;   // x
  Q[1][1] = 1.0f;   // x_dot
  Q[2][2] = 0.0001f;   // theta
  Q[3][3] = 0.01f;   // theta_dot

  // R: MEASUREMENT NOISE -> higher R means trust sensors less
  R[0][0] = 0.05f;   // x
  R[1][1] = 0.005f;   // theta


}

void KalmanFilter::predict(float u){

  float x_next[4] = {0,0,0,0};

  // 1. x̂ = A*x + B*u
  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      x_next[i] += A[i][j] * x[j]; // Ax
    }
    x_next[i] += B[i] * u;
  }
  memcpy(x, x_next, sizeof(x)); //


  // 2. P = A * P * A.T + Q
  memset(AP, 0, sizeof(AP));  // filled with 0s - temp array

  // step 1: A*P
  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      for (int k=0; k<4; k++){
        AP[i][j] += A[i][k] * P[k][j];  // A_d from state space
      }
    }
  }

  // step 2: AP * A.T
  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      float APA_t = 0;
      for (int k=0; k<4; k++){
        APA_t += AP[i][k] * A[j][k]; // using jk instead of kj since transpose
      }
      P[i][j] = APA_t + Q[i][j];
    }
  }

}

void KalmanFilter::update(float measured_x, float measured_theta){

  // H: (2,4)   P: (4,4)  S: (2,2)   y: (2,)

  // y = z - H*x̂  (innovation)
  y[0] = measured_x - (H[0][0]*x[0] + H[0][1]*x[1] + H[0][2]*x[2] + H[0][3]*x[3]);
  y[1] = measured_theta - (H[1][0]*x[0] + H[1][1]*x[1] + H[1][2]*x[2] + H[1][3]*x[3]);

  // S = H*P*H.T + R

  // step 1: H*P -> (2,4)
  float HP[2][4] = {};
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        HP[i][j] += H[i][k] * P[k][j];

  // step 2: S = HP * H.T -> (2,2)
  memset(S, 0, sizeof(S));
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++){
      for (int k = 0; k < 4; k++){
        S[i][j] += HP[i][k] * H[j][k];  // kj switched since transpose
      }
      S[i][j] += R[i][j];
    }

  // S_inv -> (2,2)
  float det = S[0][0] * S[1][1] - S[1][0] * S[0][1];
  float inv_det = 1.0f / det;
  S_inv[0][0] = inv_det * S[1][1];
  S_inv[0][1] = inv_det * -S[0][1];
  S_inv[1][0] = inv_det * -S[1][0];
  S_inv[1][1] = inv_det * S[0][0];

  // K = P * H.T * S_inv   ->  (4,2)

  // step 1: P*H.T
  float PHt[4][2] = {};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 2; j++)
      for (int k = 0; k < 4; k++)
        PHt[i][j] += P[i][k] * H[j][k]; // kj switched since transpose

  // step 2: K = PH.T * S_inv   -> (4,2)
  memset(K, 0, sizeof(K));
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 2; j++)
      for (int k = 0; k < 2; k++)
        K[i][j] += PHt[i][k] * S_inv[k][j];

  // x̂ = x̂ + K*y  -> (4,)
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 2; j++)
      x[i] += K[i][j] * y[j];
   
  // P = (I - K*H) * P

  // step 1: K*H  -> (4,4)
  float KH[4][4] = {};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 2; k++)
        KH[i][j] += K[i][k] * H[k][j];
      
  // step 2:  P -> (4,4)
  memset(P_new, 0, sizeof(P_new));
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++){
        float I_minus_KH = (i==k ? 1.0f : 0.0f) - KH[i][k];
        P_new[i][j] += I_minus_KH * P[k][j];
      }
  memcpy(P, P_new, sizeof(P));
}   

// get kalman filter to overwrite state
void KalmanFilter::get_state(float* state){
  memcpy(state, x, 4 * sizeof(float));
}






