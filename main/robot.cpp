#include "robot.h"

Robot::Robot(void):Movement(left_drive_motor, right_drive_motor), BlockManipulation(arm_lifting_motor)
{
  // initialise motors
  left_drive_motor = AFMS.getMotor(left_drive_motor_location);
  right_drive_motor = AFMS.getMotor(right_drive_motor_location);
  arm_lifting_motor = AFMS.getMotor(arm_lifting_motor_location);
  arm_lifting_motor -> setSpeed(255);
  detect_motors();
  // initialise LEDs and button
  pinMode(start_button, INPUT);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(lowering_switch, INPUT);

  digitalWrite(red_led, LOW);
  digitalWrite(green_led, LOW);
  // initialise color sensor
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous,sensor.eHigh); //Laser rangefinder begins to work
  sensor.start();
  while(!tcs.begin())
  {
    Serial.println("No TCS34725 found .. check your connections");
    delay(1000);
  }

  // set pincer to starting state

  arm_lifting_motor -> run(FORWARD);
  while(digitalRead(lowering_switch) == HIGH);
  arm_lifting_motor -> run(RELEASE);

  // adding reset button functionality

  wait_for_start_signal();
  attachInterrupt(digitalPinToInterrupt(start_button),restart, RISING);
  

}

void Robot::pickup_sequence(bool *colour, int time_delay, bool turn_around, bool turn_direction) 
{
  move_forward(DELAY, time_delay);
  get_colour(colour);
  switch_on_block_colour_led(colour);
  pick_up_block();
  delay(5000);
  move_backward(JUNCTION); 

}
void Robot::drop_sequence(bool colour, bool end) 
{
  if (colour == RED)
  {
    turn_adjust(LEFT);
  }
  else
  {
    turn_adjust(RIGHT);
  }
  move_forward(PLATFORM);
  drop_block();
  switch_off_block_colour_leds();
  move_backward(DELAY, 400);

  if (colour == RED) {
    turn_90(RIGHT);
    move_forward(JUNCTION, 0);
    if (end)
    {
      
      turn_90(LEFT);
      move_forward();
      turn_90(LEFT);
      move_forward(ENDSTOP);
      return_to_start_box();
    }
    else
    {
      move_forward();
    }
  } 
  else 
  {
    turn_90(LEFT);
    move_forward(JUNCTION, 0);
    turn_90(RIGHT);
    move_forward();
    move_forward();
    if (end)
    {
      turn_90(RIGHT);
      move_forward(ENDSTOP);
      return_to_start_box();
    }
    else
    {
      move_forward();
      turn_90(LEFT);
      move_forward();
    }
  }
}
static void Robot::restart()
{
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, LOW);
}
void Robot::wait_for_start_signal()
{
  while (!digitalRead(start_button));
  delay(500);
}
void Robot::detect_motors()
{
  while (!AFMS.begin())
  {
    Serial.println("Motors not connected");
  }
  Serial.println("Motors Detected");
}

void Robot::switch_on_block_colour_led(bool* colour)
{
  digitalWrite(red_led, HIGH);
  digitalWrite(green_led, HIGH);
  switch_off_block_colour_leds();
  if (*colour == RED)
  {
    digitalWrite(red_led, HIGH);
  }
  else
  {
    digitalWrite(green_led, HIGH);
  }
}
void Robot::switch_off_block_colour_leds()
{
  digitalWrite(red_led, LOW);
  digitalWrite(green_led, LOW);
}