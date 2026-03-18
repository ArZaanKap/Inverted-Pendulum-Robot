#include "motors.h"


MotoronI2C mc1(16); 
MotoronI2C mc2(17);


void motors_init()
{
  Wire1.begin();

  // Set the I2C bus for both motor controllers
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);

  //
  mc1.reinitialize();
  mc2.reinitialize();

  //
  mc1.clearResetFlag();
  mc2.clearResetFlag();

  mc1.setMaxAcceleration(1, 800);
  mc1.setMaxDeceleration(1, 800);
  mc1.setMaxAcceleration(2, 800);
  mc1.setMaxDeceleration(2, 800);
  mc1.clearMotorFaultUnconditional();///////

  mc2.setMaxAcceleration(1, 800);
  mc2.setMaxDeceleration(1, 800);
  mc2.setMaxAcceleration(2, 800);
  mc2.setMaxDeceleration(2, 800);
  mc2.clearMotorFaultUnconditional();//////

}

void motors_setSpeed(int16_t speed1, int16_t speed2, int16_t speed3, int16_t speed4){
  mc1.setSpeed(1, speed1);  
  mc1.setSpeed(2, speed2);    
  mc2.setSpeed(1, speed3);  
  mc2.setSpeed(2, speed4);  
}

void motors_setSpeedAll(int16_t speed){
  mc1.setSpeed(1, speed);  
  mc1.setSpeed(2, speed);    
  mc2.setSpeed(1, speed);  
  mc2.setSpeed(2, speed); 
}

// scale u from lqr to 800 to -800 range
void motors_setCommand(float u){

  static constexpr float K_u = 10.0; // TUNE.... 15?
  //static constexpr int16_t FRICTION    = 100;     // tune: min command to overcome static friction
  //static constexpr int16_t DEADBAND    = 40;  

  int16_t cmd = (int16_t)constrain(K_u * u, -800, 800);

  /*
  if (abs(cmd) < DEADBAND) {
    cmd = 0;
  } else if (cmd > 0) {
    cmd = max(cmd, (int16_t)FRICTION);
  } else {
    cmd = min(cmd, (int16_t)-FRICTION);
  }
  */
  motors_setSpeedAll(cmd);
  
}


