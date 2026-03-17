#include "control.h"
#include "kalman.h"
#include "motor_encoders.h"
#include "pendulum_encoder.h"
#include "motors.h"



float d1, d2, d3, d4; // motor encoder distance readings
float avg_dist;

float desired_state[4] = {0.0, 0.0, 0.0, 0.0}; // [x, x_dot, theta, theta_dot]
float state[4] = {0.0, 0.0, 0.0, 0.0};

LQR controller(desired_state);
KalmanFilter kalman(state);
float u = 0.0f;

unsigned long last_time = 0;
const unsigned long dt_us = 2500;  // 2500us = 2.5ms -> 400Hz

// modes
bool ANGLED_START = false;
bool has_jerked = false;
float start_angle = radians(10); // angle in rad

bool SPRINT = false;


void setup() {
  delay(1000);

  Serial.begin(115200);
  
  pendulumEncoder_init();
  pendulumEncoder_forceZero();// new
  Serial.println("Pendulum Encoders Ready");

  motorEncoders_init();
  Serial.println("Motor Encoders Ready");

  motors_init();
  Serial.println("Motors Ready");
  delay(200);

  last_time = micros();
}

void loop() {

  // 400Hz loop
  unsigned long current_time = micros(); //

  if (current_time - last_time >= dt_us){

    //last_time = current_time;
    last_time += dt_us;

    // ANGLED START MODE
    if (ANGLED_START==true && has_jerked==false) {
      jerk(start_angle);       // blocking, but only runs once
      has_jerked = true;

      last_time = micros();
      return;                  // let kalman run from next iter
    }


    // read motor encoders - dist (x)
    encoders_getDistance(&d1, &d2, &d3, &d4); // use nullptr to not read an encoder
    avg_dist = (d1 + d2 + d3 + d4) / 4.0;
    //float median_dist = median_filter(d1, d2, d3, d4);
    //Serial.println(String(d1) + ", " + String(d2) + ", " + String(d3) + ", " + String(d4) + "  avg: " + String(median_dist));
    //Serial.println(avg_dist);
    
    // read pendulum encoder
    float theta = get_pendulum_angle_rad();
    Serial.println(degrees(theta));
    //Serial.println("Invalid counts: " + String(pendulumEncoder_getInvalidCount()));

    kalman.predict(u);
    kalman.update(avg_dist, theta); //median_dist - TEST
    kalman.get_state(state); // [x, x_dot, theta, theta_dot]
    
    u = controller.run(state);

    //motors_setCommand(u);

  }
  

}

