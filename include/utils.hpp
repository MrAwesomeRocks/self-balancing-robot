#ifndef _UTILS_
#define _UTILS_

// Libs
#include <Arduino.h>
#include "L298N.h"

void drive(L298N &motor, int speed);
void drive(L298N &motor1, L298N &motor2, int speed);
void computePIDConsts(double &Kp, double &Ki, double &Kd, int pinKp, int pinKi, int pinKd);
#endif
