#include "control.h"
#include "kalman.h"
#include "motor_encoders.h"
#include "pendulum_encoder.h"
#include "motors.h"

#define START_BUTTON_PIN  10

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

bool system_running = false; // Starts paused
bool button_pressed = false; // Tracks physical button state

void setup() {
  delay(1000);

  Serial.begin(115200);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP); // button setup
  
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
  // 1. Check for a button press to toggle Run/Stop
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    if (!button_pressed) {
      button_pressed = true;
      system_running = !system_running; // Flip the state (Pause <-> Run)
      
      if (system_running) {
        //Serial.println("System STARTED");
      } else {
        //Serial.println("System STOPPED");
        motors_setCommand(0.0); // SAFETY: Instantly kill motor power when paused
      }
      
      delay(200);           // A quick 200ms delay to debounce the physical button
      last_time = micros(); // CRITICAL: Reset the timer right before returning to the loop
                            // so the Kalman filter doesn't get a massive 'dt' spike.
    }
  } else {
    button_pressed = false; // Reset when button is released
  }

  // 2. Only execute the 400Hz control math if the system is running
  if (system_running) {
    unsigned long current_time = micros();

    if (current_time - last_time >= dt_us) {
      last_time += dt_us;  // or last_time = current_time??

      // ANGLED START MODE
      if (ANGLED_START == true && has_jerked == false) {
        jerk(start_angle);       // blocking, but only runs once
        has_jerked = true;

        last_time = micros();
        return;                  // let kalman run from next iter
      }

      // read motor encoders - dist (x)
      encoders_getDistance(&d1, &d2, &d3, &d4); 
      avg_dist = (d1 + d2 + d3 + d4) / 4.0;
      
      // read pendulum encoder
      float theta = get_pendulum_angle_rad();
      if (fabs(theta) > radians(40.0)){ // break if angle too large
        motors_setSpeedAll(0);
        system_running = false;
        return;
      }

      kalman.predict(u);
      kalman.update(avg_dist, theta); 
      kalman.get_state(state); // [x, x_dot, theta, theta_dot]
      
      u = controller.run(state);

      motors_setCommand(u);
    }
  }
}
