#include <Tetrix_Motor.h>

MotorDriver::MotorDriver(byte addres)
{
    addr = addres;
}
void MotorDriver::init()
{
    // set gear ratio
    Wire.beginTransmission(addr); // transmit to device #44 (0x2c)
    Wire.write(0x56);
    Wire.write(60);
    Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(addr); // transmit to device #44 (0x2c)
    Wire.write(0x5A);
    Wire.write(60);
    Wire.endTransmission();
    delay(10);
    // set
}

void MotorDriver::set_speed_m1(byte speed)
{
    Wire.beginTransmission(addr); // transmit to device #44 (0x2c)
    Wire.write(0x44);
    Wire.write(1);
    Wire.endTransmission(); // stop transmitting
    delay(1);
    Wire.beginTransmission(addr); // transmit to device #44 (0x2c)
    Wire.write(0x45);
    Wire.write(speed);
    Wire.endTransmission(); // stop transmitting
    delay(1);
}

void MotorDriver::set_speed_m2(byte speed)
{
    Wire.beginTransmission(addr); // transmit to device #44 (0x2c)
    Wire.write(0x47);
    Wire.write(1);
    Wire.endTransmission(); // stop transmitting
    delay(1);
    Wire.beginTransmission(addr); // transmit to device #44 (0x2c)
    Wire.write(0x46);
    Wire.write(speed);
    Wire.endTransmission(); // stop transmitting
    delay(1);
}

long MotorDriver::get_encoder_m1()
{
    unsigned long eCount; // return value variable. We have to pass this an unsigned into Arduino.

    byte byte1;
    byte byte2;
    byte byte3;
    byte byte4;

    Wire.beginTransmission(addr);
    Wire.write(0x4C);
    Wire.endTransmission();
    delay(1);

    Wire.requestFrom(addr, 1);
    byte1 = Wire.read();
    Wire.requestFrom(addr, 1);
    byte2 = Wire.read();
    Wire.requestFrom(addr, 1);
    byte3 = Wire.read();
    Wire.requestFrom(addr, 1);
    byte4 = Wire.read();

    eCount = byte1;
    eCount = (eCount * 256) + byte2;
    eCount = (eCount * 256) + byte3;
    eCount = (eCount * 256) + byte4;
    delay(1);
    return eCount;
}

long MotorDriver::get_encoder_m2()
{
    unsigned long eCount; // return value variable. We have to pass this an unsigned into Arduino.

    byte byte1;
    byte byte2;
    byte byte3;
    byte byte4;

    Wire.beginTransmission(addr);
    Wire.write(0x50);
    Wire.endTransmission();
    delay(1);

    Wire.requestFrom(addr, 1);
    byte1 = Wire.read();
    Wire.requestFrom(addr, 1);
    byte2 = Wire.read();
    Wire.requestFrom(addr, 1);
    byte3 = Wire.read();
    Wire.requestFrom(addr, 1);
    byte4 = Wire.read();

    eCount = byte1;
    eCount = (eCount * 256) + byte2;
    eCount = (eCount * 256) + byte3;
    eCount = (eCount * 256) + byte4;
    delay(1);
    return eCount;
}

int MotorDriver::read_bat()
{

    int Bvoltage;

    byte byte1;
    byte byte2;

    Wire.beginTransmission(addr);
    Wire.write(0x54);
    Wire.endTransmission();
    // delay(10);

    Wire.requestFrom(addr, 2);
    byte1 = Wire.read();
    byte2 = Wire.read();
    Bvoltage = byte1;
    Bvoltage = (Bvoltage * 256) + byte2;
    delay(1);

    return Bvoltage;
}

void MotorDriver::reset_encoder()
{
    Wire.beginTransmission(addr);
    Wire.write(0x44);
    Wire.write(3);
    Wire.endTransmission();
    delay(5);
    Wire.beginTransmission(addr);
    Wire.write(0x47);
    Wire.write(3);
    Wire.endTransmission();
    delay(20);

    Wire.beginTransmission(addr);
    Wire.write(0x44);
    Wire.write(1);
    Wire.endTransmission();
    delay(5);
    Wire.beginTransmission(addr);
    Wire.write(0x47);
    Wire.write(1);
    Wire.endTransmission();
    delay(10);
}