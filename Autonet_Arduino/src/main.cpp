#include <Arduino.h>
#include <Wire.h>
#include <Tetrix_Motor.cpp>
#include <Sharp.h>
#include <Ping.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

using namespace std_msgs;
using namespace sensor_msgs;
//CONFIG
#define SHARP_MAX 1
#define SHARP_MIN 0.01

#define PING_MAX 3
#define PING_MIN 0.01

#define SHARP1_PIN A0
#define SHARP2_PIN A1
#define SHARP3_PIN A2
#define SHARP4_PIN A3

#define PING1_PIN 2
#define PING2_PIN 3

#define DRIVER_ADDRES 2

#define EMERGENCY_OUT 12
#define EMERGENCY_IN 11

#define POWER_LINE1 A13
#define POWER_LINE2 A14
#define POWER_LINE3 A15

// #define STD_SERVO1

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


float bat = 0;
float power_line1 = 0;
float power_line2 = 0;
float power_line3 = 0;
bool emergency_1 = false;
bool emergency_2 = false;

float m1 = 0;
float m2 = 0;
long enc1 = 0;
long enc2 = 0;

int servo1 = 0;
bool servo1_first = true;
int servo2 = 0;
bool servo2_first = true;
int servo3 = 0;
bool servo3_first = true;
int servo4 = 0;
bool servo4_first = true;

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

    power_line1 = analogRead(POWER_LINE1);
    power_line2 = analogRead(POWER_LINE2);
    power_line3 = analogRead(POWER_LINE3);
}

void read_driver()
{
    enc1 = driver.get_encoder_m1();
    enc2 = driver.get_encoder_m2();
    bat = driver.read_bat();
}

void send_to_motors()
{
    if (!emergency_1 && !emergency_2)
    {
        driver.set_speed_m1(m1);
        driver.set_speed_m2(m2);
    }
    else
    {
        float m1 = 0;
        float m2 = 0;
        driver.set_speed_m1(0);
        driver.set_speed_m2(0);
    }
}

void send_to_servos(){

}
// OUT
Header h;
Range range_sharp1;
Range range_sharp2;
Range range_sharp3;
Range range_sharp4;

Range range_ping1;
Range range_ping2;

Bool emergency_arduino_msg;
Float32 bat_msg;
Float32 power_line1_msg;
Float32 power_line2_msg;
Float32 power_line3_msg;

Int32 enc1_msg;
Int32 enc2_msg;

// IN
Bool emergency_main_msg;
Int16 servo1_msg;
Int16 servo2_msg;
Int16 servo3_msg;
Int16 servo4_msg;

Int16 m1_msg;
Int16 m2_msg;

ros::Publisher range_sharp1_pub("range_sharp1", &range_sharp1);
ros::Publisher range_sharp2_pub("range_sharp2", &range_sharp2);
ros::Publisher range_sharp3_pub("range_sharp3", &range_sharp3);
ros::Publisher range_sharp4_pub("range_sharp4", &range_sharp4);

ros::Publisher range_ping1_pub("range_ping1", &range_ping1);
ros::Publisher range_ping2_pub("range_ping2", &range_ping2);



void communicate()
{

    h.stamp.sec = millis() / 1000;

    range_sharp1.header = h;
    range_sharp2.header = h;
    range_sharp3.header = h;
    range_sharp4.header = h;
    range_sharp1.range = sharp1_cm / 100;
    range_sharp2.range = sharp2_cm / 100;
    range_sharp3.range = sharp3_cm / 100;
    range_sharp4.range = sharp3_cm / 100;

    range_ping1.header = h;
    range_ping2.header = h;
    range_ping1.range = ping1_cm / 100;
    range_ping2.range = ping2_cm / 100;

    emergency_arduino_msg.data = emergency_1;
    bat_msg.data = bat;
    power_line1_msg.data = power_line1;
    power_line2_msg.data = power_line2;
    power_line3_msg.data = power_line3;

    enc1_msg.data = enc1;
    enc2_msg.data = enc2;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    driver.init();
    pinMode(EMERGENCY_IN, INPUT);
    pinMode(EMERGENCY_OUT, OUTPUT);
    range_sharp1.radiation_type = range_sharp1.INFRARED;
    range_sharp1.max_range = SHARP_MAX;
    range_sharp1.min_range = SHARP_MIN;
    range_sharp2.radiation_type = range_sharp2.INFRARED;
    range_sharp2.max_range = SHARP_MAX;
    range_sharp2.min_range = SHARP_MIN;
    range_sharp3.radiation_type = range_sharp3.INFRARED;
    range_sharp3.max_range = SHARP_MAX;
    range_sharp3.min_range = SHARP_MIN;
    range_sharp3.radiation_type = range_sharp4.INFRARED;
    range_sharp4.max_range = SHARP_MAX;
    range_sharp4.min_range = SHARP_MIN;

    range_ping1.radiation_type = range_ping1.ULTRASOUND;
    range_ping1.max_range = PING_MAX;
    range_ping1.min_range = PING_MIN;
    range_ping2.radiation_type = range_ping2.ULTRASOUND;
    range_ping2.max_range = PING_MAX;
    range_ping2.min_range = PING_MIN;
    // pwm.begin();

    // pwm.setOscillatorFrequency(27000000);
    // pwm.setPWMFreq(1600); // This is the maximum PWM frequency

    // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
    // some i2c devices dont like this so much so if you're sharing the bus, watch
    // out for this!
    // Wire.setClock(400000);
}

void loop()
{
    if (digitalRead(EMERGENCY_IN) == 1)
    {
        emergency_1 = true;
    }
    read_sensors();
    read_driver();
    communicate();

    if (emergency_1 || emergency_2)
    {
        digitalWrite(EMERGENCY_OUT, 1);
    }
    else
    {
        digitalWrite(EMERGENCY_OUT, 0);
    }

    send_to_motors();
}