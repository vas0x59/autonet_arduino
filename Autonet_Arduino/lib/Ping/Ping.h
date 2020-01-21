
#pragma once 
#include <Arduino.h>

class Ping
{
public:
    Ping(int p);
    float read_cm();

private:
    int pingPin = 2;
};