#include <Arduino.h>
#include <Wire.h>
#include <Tetrix_Motor.cpp>
#include <Sharp.h>
#include <Ping.h>
#include <Adafruit_PWMServoDriver.h>

//CONFIG
#define SHARP1_PIN A0
#define SHARP2_PIN A1
#define SHARP3_PIN A2
#define SHARP4_PIN A3

#define PING1_PIN 2
#define PING2_PIN 3

#define DRIVER_ADDRES 2

// #define SERVO1_PIN

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

MotorDriver driver(DRIVER_ADDRES);

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

float m1 = 0;
float m2 = 0;

bool emergency = false;

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

void send_to_motors()
{
    if (!emergency)
    {
        driver.set_speed_m1(m1);
        driver.set_speed_m2(m2);
    }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    driver.init();
    pwm.begin();

    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(1600); // This is the maximum PWM frequency

    // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
    // some i2c devices dont like this so much so if you're sharing the bus, watch
    // out for this!
    // Wire.setClock(400000);
}

void loop()
{
}