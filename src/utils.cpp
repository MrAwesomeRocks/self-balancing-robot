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
 * Map a float to a range.
 *
 * See the map() function for more details.
 */
float floatmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
