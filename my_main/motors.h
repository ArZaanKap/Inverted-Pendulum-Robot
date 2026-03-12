#pragma once
#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

extern MotoronI2C mc1;
extern MotoronI2C mc2;

void motors_init();
void motors_setSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void motors_setSpeedAll(int16_t speed);
void motors_setCommand(float u);