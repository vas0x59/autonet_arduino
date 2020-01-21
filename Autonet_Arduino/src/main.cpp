#include <Arduino.h>
#include <Wire.h>
#include <Tetrix_Motor.cpp>
#include <Sharp.h>
#include <Ping.h>

#define SHARP1_PIN A0
#define SHARP2_PIN A1
#define SHARP3_PIN A2
#define SHARP4_PIN A3

#define PING1_PIN 2
#define PING2_PIN 3

MotorDriver driver(2);

Sharp sharp1(SHARP1_PIN);
Sharp sharp2(SHARP2_PIN);
Sharp sharp3(SHARP3_PIN);
Sharp sharp4(SHARP4_PIN);

Ping ping1(PING1_PIN);
Ping ping2(PING2_PIN);

float ping1_cm = 0;
float ping2_cm = 0;

float sharp1_cm = 0;
float sharp2_cm = 0;
float sharp3_cm = 0;
float sharp4_cm = 0;

long enc1 = 0;
long enc2 = 0;
float bat = 0;

void read_sensors()
{
    //Sharps
    sharp1_cm = sharp1.read_cm();
    sharp2_cm = sharp2.read_cm();
    sharp3_cm = sharp3.read_cm();
    sharp4_cm = sharp4.read_cm();

    //Pings
    ping1_cm = ping1.read_cm();
    ping2_cm = ping2.read_cm();
}

void read_driver()
{
    enc1 = driver.get_encoder_m1();
    enc2 = driver.get_encoder_m2();
    bat = driver.read_bat();
}

void setup()
{
    Wire.begin();
    driver.init();
    Serial.begin(115200);
}

void loop()
{
}