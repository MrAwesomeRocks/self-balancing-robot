#ifndef _UTILS_
#define _UTILS_

// Libs
#include <Arduino.h>
#include "L298N.h"

void drive(L298N &motor, int speed);
void drive(L298N &motor1, L298N &motor2, int speed);

template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max);

#endif
