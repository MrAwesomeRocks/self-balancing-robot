#ifndef _UTILS_
#define _UTILS_

// Libs
#include <Arduino.h>
#include "L298N.h"

// Utils
namespace utils
{
    /*
     * Drive a motor at a certain speed.
     * If speed is negative, the motors will move backwards.
     *
     * Precondition: -255 <= speed <= 255
     */
    void drive(L298N &motor, int speed);

    /*
     * Drive two motors at a certain speed in the same direction.
     * If speed is negative, the motors will move backwards.
     *
     * Precondition: -255 <= speed <= 255
     */
    void drive(L298N &motor1, L298N &motor2, int speed);

    /*
     * Map a numerical value to a range.
     *
     * See the map() function for more details.
     */
    template <class T_IN, class T_OUT>
    T_OUT map(const T_IN &x, const T_IN &in_min, const T_IN &in_max, const T_OUT &out_min, const T_OUT &out_max)
    {
        return static_cast<T_OUT>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
    }
}
#endif
