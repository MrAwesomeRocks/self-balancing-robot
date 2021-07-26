#include "utils.hpp"

/*
 * Drive a motor at a certain speed.
 * If speed is negative, the motors will move backwards.
 *
 * Precondition: -255 <= speed <= 255
 */
void drive(L298N &motor, int speed)
{
    if (speed > 0)
    {
        motor.setSpeed(speed);
        motor.forward();
    }
    else if (speed < 0)
    {
        motor.setSpeed(-speed);
        motor.backward();
    }
    else
    {
        motor.stop();
    }
}

/*
 * Drive two motors at a certain speed in the same direction.
 * If speed is negative, the motors will move backwards.
 *
 * Precondition: -255 <= speed <= 255
 */
void drive(L298N &motor1, L298N &motor2, int speed)
{
    drive(motor1, speed);
    drive(motor2, speed);
}

/*
 * Map a numerical value to a range.
 *
 * See the map() function for more details.
 */
template <class T>
T map(T &x, T &in_min, T &in_max, T &out_min, T &out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
