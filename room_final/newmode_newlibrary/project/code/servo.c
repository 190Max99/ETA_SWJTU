#include "servo.h"

float limit(float smotor_duty, float limit)
{
    if (smotor_duty > limit)
    {
        smotor_duty = limit;
    }
    else if (smotor_duty < -limit)
    {
        smotor_duty = -limit;
    }
    return smotor_duty;
}