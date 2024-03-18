#ifndef ROBOT_H
#define ROBOT_H

#include "blockManipulation.h"
#include "movement.h"



class Robot: public BlockManipulation, public Movement 
{
public:

    Robot(void);

    void drop_sequence(bool colour, bool end = false);
    void pickup_sequence(bool *colour, int time_delay = 2100, bool turn_around = false, bool turn_direction = RIGHT);
    
    

private:
    void detect_motors();
    void wait_for_start_signal();
    void switch_on_block_colour_led(bool* colour);
    void switch_off_block_colour_leds();
    static void restart();

    static const uint8_t start_button = 3;
    static const uint8_t reset_pin = 13;
    static const uint8_t green_led = 11;
    static const uint8_t red_led = 12;

    const uint8_t left_drive_motor_location = 1;
    const uint8_t right_drive_motor_location = 2;
    const uint8_t arm_lifting_motor_location = 3;

    Adafruit_MotorShield AFMS = Adafruit_MotorShield();

    Adafruit_DCMotor *left_drive_motor;
    Adafruit_DCMotor *right_drive_motor;
    Adafruit_DCMotor *arm_lifting_motor;


};

#endif
