// Movement.h
#ifndef Movement_h
#define Movement_h

#include "Arduino.h"
#include "Adafruit_MotorShield.h"
#include "DFRobot_VL53L0X.h"
#include "Timer.h"
#include "Wire.h"


#define LEFT 0
#define RIGHT 1
#define STRAIGHT 2

#define END 1

#define JUNCTION 1
#define BLOCK 2
#define PLATFORM 3
#define DELAY 4
#define ENDSTOP 1


class Movement {
public:
  Movement(Adafruit_DCMotor * left_drive_motor, Adafruit_DCMotor * right_drive_motor); // Constructor with motor control pins
  // movement functions that will be used
  void move_forward(int stop_condition = JUNCTION, int wait_time = 800);
  void move_backward(int stop_condition = JUNCTION, int time_delay = 0);
  void turn_90(bool direction);
  void turn_adjust(bool direction);

  // helper functions
  void release_drive_motors();
  void return_to_start_box();

  const uint8_t left_junction_sensor = 6;
  const uint8_t right_junction_sensor = 7;
  const uint8_t left_line_sensor = 8;
  const uint8_t right_line_sensor = 9;
  const uint8_t platform_sensor = 2;
  const uint8_t blue_led = 10;
  DFRobot_VL53L0X sensor;


private:
// internal utility functions for movement
  void set_drive_motors_to_base_speed();
  void start_drive_motors();
  void adjust_direction(int direction);
  void update_drive_motor_speed();
  bool check_stop_condition(int condition, int time_delay=0);
  void set_motors_to_turn(int direction);
  bool check_for_block();
  void toggle_movement_led();
  void flash_movement_led();

  
// different base speed since motors have different calibration
  uint8_t _left_base_speed = 255;
  uint8_t _right_base_speed = 200;

  uint8_t _left_drive_motor_speed = 0;
  uint8_t _right_drive_motor_speed = 0;

  uint8_t _new_left_drive_motor_speed = 0;
  uint8_t _new_right_drive_motor_speed = 0;

  uint8_t _movement_direction = FORWARD;
  uint8_t _new_movement_direction = FORWARD;
  uint8_t _default_stop_condition = JUNCTION;

  Adafruit_DCMotor *_left_drive_motor;
  Adafruit_DCMotor *_right_drive_motor;

  bool _movement_led_state = 0;
  unsigned long _last_led_change; 
  const uint32_t _led_time_period = 250;


};

#endif
