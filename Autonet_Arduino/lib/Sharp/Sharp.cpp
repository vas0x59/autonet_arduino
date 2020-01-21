#include "Sharp.h"

Sharp::Sharp(int p)
{
    pin = p;
}

float Sharp::read_cm()
{

    float volts = analogRead(pin) * 0.0048828125;
    float distance = 13 * powf(volts, -1);
    return distance;
}