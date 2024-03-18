#ifndef blockManipulation_h
#define blockManipulation_h

#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_TCS34725.h"
#include "Servo.h"
#include "DFRobot_VL53L0X.h"
#include "Adafruit_MotorShield.h"

#define RED 0
#define BLACK 1


class BlockManipulation 
{
public:
  BlockManipulation(Adafruit_DCMotor * arm_lifting_motor);
  void pick_up_block();
  void drop_block();
  void get_colour(bool* colour);
  DFRobot_TCS34725 tcs = DFRobot_TCS34725(&Wire, TCS34725_ADDRESS, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  uint8_t lowering_switch = 5;


private:
  void change_servo(int angle, Servo servo);
  uint8_t _servo_pin = 4;
  Servo _my_servo;
  unsigned int _open_angle = 175;
  unsigned int _closed_angle = 75;
  
  Adafruit_DCMotor *_arm_lifting_motor;
  unsigned int _arm_lifting_time = 700;
};

#endif