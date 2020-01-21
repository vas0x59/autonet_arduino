#include <Arduino.h>
#include <Wire.h>
#include <Tetrix_Motor.cpp>

MotorDriver driver(0x02);

void setup()
{
    Wire.begin();
    driver.init();
    Serial.begin(115200);
}

void loop()
{
    driver.set_speed_m1(10);
    delay(1000);
    Serial.print(driver.get_encoder_m1());
    driver.set_speed_m1(-10);
    delay(1000);
    Serial.print(driver.get_encoder_m1());
    driver.set_speed_m1(0);
    delay(1000);
    // Serial.print(driver.get_encoder_m1());
    
}