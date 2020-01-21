#pragma once
#include <Arduino.h>
#include <Wire.h>
class MotorDriver
{
public:
    MotorDriver(byte addres);
    void init();
    void set_speed_m1(byte speed);
    void set_speed_m2(byte speed);
    long get_encoder_m1();
    long get_encoder_m2();
    int read_bat();
    void reset_encoder();

private:
    long enc_m1 = 0;
    long enc_m2 = 0;
    byte addr = 0x02;
};