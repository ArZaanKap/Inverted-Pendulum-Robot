#include "control.h"
#include "kalman.h"
#include "motor_encoders.h"
#include "pendulum_encoder.h"
#include "motors.h"


float d1, d2, d3, d4; // motor encoder distance readings
float avg_dist;

float desired_state[4] = {0.5, 0, 0, 0}; // [x, x_dot, theta, theta_dot]
float state[4] = {0.0, 0.0, 0.0, 0.0};

LQR controller(desired_state);
KalmanFilter kalman(state);
float u = 0.0f;

unsigned long last_time = 0;
const unsigned long dt_us = 10000;  // 10,000us = 10ms -> 100Hz

void setup() {
  Serial.begin(115200);
  
  pendulumEncoder_init();
  pendulumEncoder_forceZero();// new
  Serial.println("Pendulum Encoders Ready");

  motorEncoders_init();
  Serial.println("Motor Encoders Ready");

  motors_init();
  Serial.println("Motors Ready");

}

void loop() {

  // 100Hz loop
  unsigned long current_time = micros(); //
  if (current_time - last_time >= dt_us){

    last_time = current_time;

    // read motor encoders - dist (x)
    encoders_getDistance(&d1, &d2, &d3, &d4); // use nullptr to not read an encoder
    avg_dist = (d1 + d2 + d3 + d4) / 4.0;
    //Serial.println(String(d1) + ", " + String(d2) + ", " + String(d3) + ", " + String(d4) + "  avg: " + String(avg_dist));
    //Serial.println(avg_dist);
    
    // read pendulum encoder
    float theta = get_pendulum_angle_rad();
    Serial.println(theta);

    kalman.predict(u);
    kalman.update(avg_dist, theta);
    kalman.get_state(state); // [x, x_dot, theta, theta_dot]
    
    u = controller.run(state);

    motors_setCommand(u);

  }
  

}