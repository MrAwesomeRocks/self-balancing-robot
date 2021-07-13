#ifndef _MOTOR_UTILS_
#define _MOTOR_UTILS_

// Libs
#include <Arduino.h>
#include "L298N.h"

void drive(L298N motor, int speed);
void drive(L298N motor1, L298N motor2, int speed);
#endif
