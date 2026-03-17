#include "motor_encoders.h"

// Constructor: Saves the pin numbers when you create the motor
MotorEncoder::MotorEncoder(int a, int b) : pinA(a), pinB(b) {}

// Initialize pins and read starting state
void MotorEncoder::init() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  lastStateA = digitalRead(pinA);
  lastStateB = digitalRead(pinB);
}

// The Logic: Runs every time the wheel moves
void MotorEncoder::update() {
  int stateA = digitalRead(pinA);
  int stateB = digitalRead(pinB);

  if (stateA != lastStateA) {
    if (stateA != stateB) count++; else count--;
  }
  else if (stateB != lastStateB) {
    if (stateA == stateB) count++; else count--;
  }
  lastStateA = stateA;
  lastStateB = stateB;
}

// updated
MotorEncoder ME1(27,28);  // front left
MotorEncoder ME2(22,23);  // front right
MotorEncoder ME3(29,30);  // back left
MotorEncoder ME4(49,50);  // back right

// --- INTERRUPT WRAPPERS ---
static void isr1() { ME1.update(); }
static void isr2() { ME2.update(); }
static void isr3() { ME3.update(); }
static void isr4() { ME4.update(); }


void motorEncoders_init(){

  attachInterrupt(digitalPinToInterrupt(ME1.pinA), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ME1.pinB), isr1, CHANGE);
  ME1.init();

  attachInterrupt(digitalPinToInterrupt(ME2.pinA), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ME2.pinB), isr2, CHANGE);
  ME2.init();

  attachInterrupt(digitalPinToInterrupt(ME3.pinA), isr3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ME3.pinB), isr3, CHANGE);
  ME3.init();

  attachInterrupt(digitalPinToInterrupt(ME4.pinA), isr4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ME4.pinB), isr4, CHANGE);
  ME4.init();
  
}

void encoders_getDistance(float *d1, float *d2, float *d3, float *d4){  // requires pointers

  // clockwise spin is positive count
  noInterrupts();
  long c1 = ME1.count, c2 = ME2.count, c3 = ME3.count, c4 = ME4.count;
  interrupts();

  if(d1) *d1 = c1 * DIST_PER_COUNT;
  if(d2) *d2 = c2 * DIST_PER_COUNT;
  if(d3) *d3 = c3 * DIST_PER_COUNT;
  if(d4) *d4 = c4 * DIST_PER_COUNT;
  
}

void compare_swap(float &x, float &y){ // pass by reference
  if (x > y){
    float temp = x;
    x = y;
    y = temp;
  }
}

// sort sensor readings in ascending order
void sort(float &r1, float &r2, float &r3, float &r4){  // pass by reference
  compare_swap(r1, r2);
  compare_swap(r3, r4);
  compare_swap(r1, r3);
  compare_swap(r2, r4);
  compare_swap(r2, r3);
}

float median_filter(float &r1, float &r2, float &r3, float &r4){  // pass by reference
  sort(r1, r2, r3, r4);
  return (r2+r3) / 2.0;
}

