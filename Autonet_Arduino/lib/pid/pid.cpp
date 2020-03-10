#include "pid.h"

PID::PID()
{
    kP = 0;
    kI = 0;
    kD = 0;
    prev_error = 0;
}
PID::PID(float kp, float ki, float kd){
    kP = kp;
    kI = ki;
    kD = kd;
    prev_error = 0;
}
float PID::_calc(float error)
{
    // float error =
    float v = 0;
    unsigned long now = millis();
    float d_time = (now - prev_time)/1000;
    integral += error * d_time;
    if (first_i == true)
    {
        first_i = false;
        v = error * kP;
    }
    else
    {
        float vP = kP * error;
        float vI = kI * integral;
        float vD = kD * ((error - prev_error) / d_time);
        v = vP + vI + vD;
        // v = kP*error + kI*() + kD*();
    }

    prev_error = error;
    prev_time = now;
    return v;
}
float PID::calc(float error)
{
     // float error =
    float v = 0;
    unsigned long now = millis();
    float d_time = (now - prev_time)/1000;
    integral += error * d_time;
    if (first_i == true)
    {
        first_i = false;
        v = error * kP;
        prev_error = error;
    }
    else
    {
        float vP = kP * error;
        float vI = kI * integral;
        float vD = kD * ((error - prev_error) / d_time);
        v = vP + vI;
        // v = kP*error + kI*() + kD*();
    }

    prev_error = error;
    prev_time = now;
    return v;
}
float PID::calc(float value, float set_point)
{
    float error = set_point - value;
    return _calc(error);
}