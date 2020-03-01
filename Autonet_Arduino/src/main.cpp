#include <Arduino.h>
#include <Ping.h>
#include <Sharp.h>
#include <Tetrix_Motor.cpp>
#include <Wire.h>
#include <ros.h>

#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

#include <Adafruit_PWMServoDriver.h>
#include <Encoder.h>
// #include <Servo.h>
#include <pid.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// создаём объекты для управления сервоприводами
// Servo s1;
// Servo s2;
// Servo s3;
// Servo s4;

using namespace std_msgs;
using namespace sensor_msgs;

// #include <ros.h>

ros::NodeHandle nh;
#define PI 3.141592653589793
#define MOTORS_V_UPDATERATE 7
#define ONE_ROTATE_CONST 1426
#define WHEEL_D 0.076
#define ADD_MOTOR_CONST 40
// CONFIG
#define SHARP_MAX 1
#define SHARP_MIN 0.01

#define PING_MAX 3
#define PING_MIN 0.01

#define SHARP1_PIN A8
#define SHARP2_PIN A9
#define SHARP3_PIN A10
#define SHARP4_PIN A11

#define PING1_PIN 30
#define PING2_PIN 32

#define DRIVER_ADDRES 0x05
#define DRIVER_ADDRES2 0x06

#define EMERGENCY_OUT 12
#define EMERGENCY_IN 33

#define POWER_DATA_RATE 1.5
#define POWER_LINE1 A13
#define POWER_LINE2 A14
#define POWER_LINE3 A15
#define POWER_LINE_K 1

#define ANALOGIN_1 A4
#define ANALOGIN_2 A5
#define ANALOGIN_3 A6
#define ANALOGIN_4 A7

#define SERVO1_PIN 0
#define SERVO2_PIN 1
#define SERVO3_PIN 2
#define SERVO4_PIN 3
#define SERVO5_PIN 4
#define SERVO6_PIN 5

#define SERVOMIN 150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 900 
#define USMAX 2100
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
// #define STD_SERVO1

// #define SERVO1_PIN

// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Encoder encoder1(2, 3);
Encoder encoder2(18, 19);

MotorDriver driver(DRIVER_ADDRES);
MotorDriver driver2(DRIVER_ADDRES2);
PID m1_v_pid(10, 0.1, 0);
PID m2_v_pid(10, 0.1, 0);


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

float analogin_1 = 0;
float analogin_2 = 0;
float analogin_3 = 0;
float analogin_4 = 0;

int m1 = 0;
int m2 = 0;
float target_v_m1 = 0;
float target_v_m2 = 0;
int m3 = 0;
// Float32
float enc1 = 0;
float enc2 = 0;
float enc1v = 0;
float enc2v = 0;
float prev_enc1 = 0;
float prev_enc2 = 0;

float m1_v = 0;
float m2_v = 0;

unsigned long prev_time_m_v = 0;


int servo1 = 0;
int servo1_first = -1;
int servo2 = 0;
int servo2_first = -1;
int servo3 = 0;
int servo3_first = -1;
int servo4 = 0;
int servo4_first = -1;
int servo5 = 0;
int servo5_first = -1;
int servo6 = 0;
int servo6_first = -1;


void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;     // 1,000,000 us per second
  pulselength /= SERVO_FREQ; // Analog servos run at ~60 Hz updates
  Serial.print(pulselength);
  Serial.println(" us per period");
  pulselength /= 4096; // 12 bits of resolution
  Serial.print(pulselength);
  Serial.println(" us per bit");
  pulse *= 1000000; // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void read_sensors() {
  // Sharps
  sharp1_cm = sharp1.read_cm();
  sharp2_cm = sharp2.read_cm();
  sharp3_cm = sharp3.read_cm();
  sharp4_cm = sharp4.read_cm();

  // Pings
  // ping1_cm = ping1.read_cm();
  // ping2_cm = ping2.read_cm();

  power_line1 = analogRead(POWER_LINE1) * 0.0048828125 * POWER_LINE_K;
  power_line2 = analogRead(POWER_LINE2) * 0.0048828125 * POWER_LINE_K;
  power_line3 = analogRead(POWER_LINE3) * 0.0048828125 * POWER_LINE_K;

  analogin_1 = analogRead(ANALOGIN_1);
  analogin_2 = analogRead(ANALOGIN_2);
  analogin_3 = analogRead(ANALOGIN_3);
  analogin_4 = analogRead(ANALOGIN_4);
}

void read_driver() {
  // enc1 = driver.get_encoder_m1();
  // enc2 = driver.get_encoder_m2();
  enc1 = (2.0*PI / ONE_ROTATE_CONST)*WHEEL_D*encoder1.read();
  enc2 = (2.0*PI / ONE_ROTATE_CONST)*WHEEL_D*encoder2.read();
  if ((prev_time_m_v + 1000/MOTORS_V_UPDATERATE) >= millis()){
    enc1v = (enc1 - prev_enc1) / ((millis() - prev_time_m_v) / 1000.0);
    enc2v = (enc2 - prev_enc2) / ((millis() - prev_time_m_v) / 1000.0);
    
    prev_time_m_v = millis();
    prev_enc1 = enc1;
    prev_enc2 = enc2;
  }
  // bat = driver.read_bat();
}

void send_to_motors() {
  if (!emergency_1 && !emergency_2) {
    m1 = (int)(m1_v_pid.calc(target_v_m1 - enc1v) + target_v_m1*ADD_MOTOR_CONST);
    m2 = (int)(m2_v_pid.calc(target_v_m2 - enc2v) + target_v_m2*ADD_MOTOR_CONST);

    if (m1 >= 70)
      m1 = 70;
    if (m1 <= -70)
      m1 = -70;
    if (m2 >= 70)
      m2 = 70;
    if (m2 <= -70)
      m2 = 70;
    
    driver.set_speed_m1(m1);
    driver.set_speed_m2(m2);
    driver2.set_speed_m1(m3);
  } else {
    m1 = 0;
    m2 = 0;
    m3 = 0;
    driver.set_speed_m1(0);
    driver.set_speed_m2(0);
    driver2.set_speed_m1(0);
  }
}

void send_to_servos() {
  if (!emergency_1 && !emergency_2) {
    //   if (servo1_first)
    // s1.write(servo1);
    // s2.write(servo2);
    // s3.write(servo3);
    // s4.write(servo4);
    // pwm.writeMicroseconds(servonum, microsec);
    pwm.writeMicroseconds(SERVO1_PIN, map(servo1, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO2_PIN, map(servo2, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO3_PIN, map(servo3, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO4_PIN, map(servo4, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO5_PIN, map(servo5, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO6_PIN, map(servo6, 0, 180, USMIN, USMAX));

  } else {
    // s1.write(servo1_first);
    // s2.write(servo2_first);
    // s3.write(servo3_first);
    // s4.write(servo4_first);
    pwm.writeMicroseconds(SERVO1_PIN, map(servo1_first, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO2_PIN, map(servo2_first, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO3_PIN, map(servo3_first, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO4_PIN, map(servo4_first, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO5_PIN, map(servo5_first, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(SERVO6_PIN, map(servo6_first, 0, 180, USMIN, USMAX));
  }
}
// OUT
Header h;
Range range_sharp1; //
Range range_sharp2; //
Range range_sharp3; //
Range range_sharp4; //

Range range_ping1; //
Range range_ping2; //

Bool emergency_arduino_msg; //
Float32 bat_msg;            //
Float32 power_line1_msg;    //
Float32 power_line2_msg;    //
Float32 power_line3_msg;    //

Float32 enc1_msg; //
Float32 enc2_msg; //
Float32 enc1v_msg; //
Float32 enc2v_msg; //

Int16 analogin_1_msg; //
Int16 analogin_2_msg; //
Int16 analogin_3_msg; //
Int16 analogin_4_msg; //

// IN
void emergency_main_cb(const std_msgs::Bool &emg_msg) {
  emergency_2 = emg_msg.data;
}
void servo1_cb(const std_msgs::Int16 &msg) {
  servo1 = msg.data;
  if (servo1_first == -1)
    servo1_first = servo1;
}
void servo2_cb(const std_msgs::Int16 &msg) {
  servo2 = msg.data;
  if (servo2_first == -1)
    servo2_first = servo2;
}
void servo3_cb(const std_msgs::Int16 &msg) {
  servo3 = msg.data;
  if (servo3_first == -1)
    servo3_first = servo3;
}
void servo4_cb(const std_msgs::Int16 &msg) {
  servo4 = msg.data;
  if (servo4_first == -1)
    servo4_first = servo4;
}
void servo5_cb(const std_msgs::Int16 &msg) {
  servo5 = msg.data;
  if (servo5_first == -1)
    servo5_first = servo5;
}
void servo6_cb(const std_msgs::Int16 &msg) {
  servo6 = msg.data;
  if (servo6_first == -1)
    servo6_first = servo6;
}

void m1_cb(const std_msgs::Float32 &msg) { target_v_m1 = msg.data; }
void m2_cb(const std_msgs::Float32 &msg) { target_v_m2 = msg.data; }
void m3_cb(const std_msgs::Int16 &msg) { m3 = msg.data; }

// Publisher
ros::Publisher range_sharp1_pub("arduino/range_sharp1", &range_sharp1);
ros::Publisher range_sharp2_pub("arduino/range_sharp2", &range_sharp2);
ros::Publisher range_sharp3_pub("arduino/range_sharp3", &range_sharp3);
ros::Publisher range_sharp4_pub("arduino/range_sharp4", &range_sharp4);

ros::Publisher range_ping1_pub("arduino/range_ping1", &range_ping1);
ros::Publisher range_ping2_pub("arduino/range_ping2", &range_ping2);

// ros::Publisher enc1_pub("arduino/enc1", &enc1_msg);
// ros::Publisher enc2_pub("arduino/enc2", &enc1_msg);
ros::Publisher enc1_pub("encoder1", &enc1_msg);
ros::Publisher enc2_pub("encoder2", &enc2_msg);
ros::Publisher enc1v_pub("encoder1_v", &enc1v_msg);
ros::Publisher enc2v_pub("encoder2_v", &enc2v_msg);

ros::Publisher bat_pub("arduino/bat", &bat_msg);
ros::Publisher emergency_arduino_pub("arduino/emergency_arduino",
                                     &emergency_arduino_msg);
ros::Publisher power_line1_pub("arduino/power_line1", &power_line1_msg);
ros::Publisher power_line2_pub("arduino/power_line2", &power_line2_msg);
ros::Publisher power_line3_pub("arduino/power_line3", &power_line3_msg);

ros::Publisher analogin_1_pub("arduino/analogin_1", &analogin_1_msg);
ros::Publisher analogin_2_pub("arduino/analogin_2", &analogin_2_msg);
ros::Publisher analogin_3_pub("arduino/analogin_3", &analogin_3_msg);
ros::Publisher analogin_4_pub("arduino/analogin_4", &analogin_4_msg);

// Subscriber
ros::Subscriber<std_msgs::Bool> emergency_main_sub("emergency_main",
                                                   &emergency_main_cb);

ros::Subscriber<std_msgs::Int16> servo1_sub("arduino/servo1", &servo1_cb);
ros::Subscriber<std_msgs::Int16> servo2_sub("arduino/servo2", &servo2_cb);
ros::Subscriber<std_msgs::Int16> servo3_sub("arduino/servo3", &servo3_cb);
ros::Subscriber<std_msgs::Int16> servo4_sub("arduino/servo4", &servo4_cb);
ros::Subscriber<std_msgs::Int16> servo5_sub("arduino/servo5", &servo5_cb);
ros::Subscriber<std_msgs::Int16> servo6_sub("arduino/servo6", &servo6_cb);

ros::Subscriber<std_msgs::Float32> m1_sub("motor1", &m1_cb);
ros::Subscriber<std_msgs::Float32> m2_sub("motor2", &m2_cb);
ros::Subscriber<std_msgs::Int16> m3_sub("arduino/m3", &m3_cb);

unsigned long prev_t = 0;

void communicate() {

  // h.stamp.sec = millis() / 1000;
  unsigned long tm = millis() / 1000;

  range_sharp1.header.stamp.sec = tm;
  range_sharp2.header.stamp.sec = tm;
  range_sharp3.header.stamp.sec = tm;
  range_sharp4.header.stamp.sec = tm;
  range_sharp1.range = sharp1_cm / 100;
  range_sharp2.range = sharp2_cm / 100;
  range_sharp3.range = sharp3_cm / 100;
  range_sharp4.range = sharp3_cm / 100;

  range_ping1.header.stamp.sec = tm;
  range_ping2.header.stamp.sec = tm;
  range_ping1.range = ping1_cm / 100;
  range_ping2.range = ping2_cm / 100;

  emergency_arduino_msg.data = emergency_1;
  bat_msg.data = bat;
  power_line1_msg.data = power_line1;
  power_line2_msg.data = power_line2;
  power_line3_msg.data = power_line3;

  enc1_msg.data = enc1;
  enc2_msg.data = enc2;
  enc1v_msg.data = enc1v;
  enc2v_msg.data = enc2v;

  analogin_1_msg.data = analogin_1;
  analogin_2_msg.data = analogin_2;
  analogin_3_msg.data = analogin_3;
  analogin_4_msg.data = analogin_4;

  range_sharp1_pub.publish(&range_sharp1);
  range_sharp2_pub.publish(&range_sharp2);
  range_sharp3_pub.publish(&range_sharp3);
  range_sharp4_pub.publish(&range_sharp4);

  range_ping1_pub.publish(&range_ping1);
  range_ping1_pub.publish(&range_ping2);

  enc1_pub.publish(&enc1_msg);
  enc2_pub.publish(&enc2_msg);

  analogin_1_pub.publish(&analogin_1_msg);
  analogin_2_pub.publish(&analogin_2_msg);
  analogin_3_pub.publish(&analogin_3_msg);
  analogin_4_pub.publish(&analogin_4_msg);
  emergency_arduino_pub.publish(&emergency_arduino_msg);
  if ((millis() - prev_t) > (1.0 / POWER_DATA_RATE) * 1000) {

    bat_pub.publish(&bat_msg);
    power_line1_pub.publish(&power_line1_msg);
    power_line2_pub.publish(&power_line2_msg);
    power_line3_pub.publish(&power_line3_msg);
    prev_t = millis();
  }
}

void setup() {
  Wire.begin();
  driver.init();
  driver2.init();
  pinMode(EMERGENCY_IN, INPUT_PULLUP);
  pinMode(EMERGENCY_OUT, OUTPUT);
  range_sharp1.radiation_type = range_sharp1.INFRARED;
  range_sharp1.max_range = SHARP_MAX;
  range_sharp1.min_range = SHARP_MIN;
  range_sharp1.header.frame_id = "range_sharp1";
  range_sharp2.radiation_type = range_sharp2.INFRARED;
  range_sharp2.max_range = SHARP_MAX;
  range_sharp2.min_range = SHARP_MIN;
  range_sharp2.header.frame_id = "range_sharp2";
  range_sharp3.radiation_type = range_sharp3.INFRARED;
  range_sharp3.max_range = SHARP_MAX;
  range_sharp3.min_range = SHARP_MIN;
  range_sharp3.header.frame_id = "range_sharp3";
  range_sharp3.radiation_type = range_sharp4.INFRARED;
  range_sharp4.max_range = SHARP_MAX;
  range_sharp4.min_range = SHARP_MIN;
  range_sharp4.header.frame_id = "range_sharp4";

  range_ping1.radiation_type = range_ping1.ULTRASOUND;
  range_ping1.max_range = PING_MAX;
  range_ping1.min_range = PING_MIN;
  range_ping1.header.frame_id = "range_ping1";
  range_ping2.radiation_type = range_ping2.ULTRASOUND;
  range_ping2.max_range = PING_MAX;
  range_ping2.min_range = PING_MIN;
  range_ping2.header.frame_id = "range_ping2";
  // range_ping2.alignas

  nh.initNode();
  Serial.begin(115200);
  nh.subscribe(m1_sub);
  nh.subscribe(m2_sub);
  nh.subscribe(m3_sub);
  nh.advertise(range_sharp1_pub);
  nh.advertise(range_sharp2_pub);
  nh.advertise(range_sharp3_pub);
  nh.advertise(range_sharp4_pub);

  nh.advertise(range_ping1_pub);
  nh.advertise(range_ping2_pub);

  nh.advertise(enc1_pub);
  nh.advertise(enc2_pub);

  nh.advertise(bat_pub);
  nh.advertise(emergency_arduino_pub);
  nh.advertise(power_line1_pub);
  nh.advertise(power_line2_pub);
  nh.advertise(power_line3_pub);

  nh.advertise(analogin_1_pub);
  nh.advertise(analogin_2_pub);
  nh.advertise(analogin_3_pub);
  nh.advertise(analogin_4_pub);

  nh.subscribe(emergency_main_sub);
  nh.subscribe(servo1_sub);
  nh.subscribe(servo2_sub);
  nh.subscribe(servo3_sub);
  nh.subscribe(servo4_sub);
  nh.subscribe(servo5_sub);
  nh.subscribe(servo6_sub);
  pwm.begin();
  // s1.attach(SERVO1_PIN);
  // s2.attach(SERVO2_PIN);
  // s3.attach(SERVO2_PIN);
  // s4.attach(SERVO2_PIN);

  // pwm.begin();

  // pwm.setOscillatorFrequency(27000000);
  // pwm.setPWMFreq(1600); // This is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C'
  // mode some i2c devices dont like this so much so if you're sharing the bus,
  // watch out for this! Wire.setClock(400000);
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {
  if (!digitalRead(EMERGENCY_IN) == 1) {
    emergency_1 = true;
  }
  else {
    emergency_1 = false;
  }
  read_sensors();
  read_driver();
  communicate();

  if (emergency_1 || emergency_2) {
    digitalWrite(EMERGENCY_OUT, 1);
  } else {
    digitalWrite(EMERGENCY_OUT, 0);
  }
  nh.spinOnce();
  delay(1);
  send_to_motors();
  send_to_servos();
}

void scan_i2c() {
  while (true) {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        Serial.println("  !");

        nDevices++;
      } else if (error == 4) {
        Serial.print("Unknown error at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.println(address, HEX);
      }
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");

    delay(5000); // wait 5 seconds for next scan
  }
}
