#ifndef _UTILS_
#define _UTILS_

// Libs
#include <Arduino.h>
#include "L298N.h"

void drive(L298N &motor, int speed);
void drive(L298N &motor1, L298N &motor2, int speed);
float floatmap(float x, float in_min, float in_max, float out_min, float out_max);

#endif
