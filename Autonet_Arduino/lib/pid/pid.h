#ifndef PID_h
#define PID_h
#include <Arduino.h>

class PID
{
public:
    PID();
    PID(float kp, float ki, float kd);
    float calc(float value, float set_point);
    float calc(float error);
    float kP = 0;
    float kI = 0;
    float kD = 0;
    bool first_i = true;
    float prev_error = 0;
    unsigned long prev_time = 0;
    float _calc(float error);
    float integral = 0;

    
};
#endif