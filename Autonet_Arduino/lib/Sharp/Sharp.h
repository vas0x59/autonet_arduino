
#pragma once 
#include <Arduino.h>

class Sharp
{
public:
    Sharp(int p);
    float read_cm();

private:
    int pin = A0;
};