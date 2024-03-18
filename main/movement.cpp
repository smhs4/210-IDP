#include "movement.h" // Include the Movement class header.

// Constructor initializing the left and right drive motors.
Movement::Movement(Adafruit_DCMotor * left_drive_motor, Adafruit_DCMotor * right_drive_motor): 
  _left_drive_motor(left_drive_motor), _right_drive_motor(right_drive_motor)
{
  // Initialize sensor and LED pins.
  pinMode(left_junction_sensor, INPUT);
  pinMode(right_junction_sensor, INPUT);
  pinMode(left_line_sensor, INPUT);
  pinMode(right_line_sensor, INPUT);
  pinMode(platform_sensor, INPUT);
  pinMode(blue_led, OUTPUT);

  // Ensure sensors and LED are turned off initially.
  digitalWrite(left_junction_sensor, LOW);
  digitalWrite(right_junction_sensor, LOW);
  digitalWrite(left_line_sensor, LOW);
  digitalWrite(right_line_sensor, LOW);
  digitalWrite(blue_led, LOW);

  _last_led_change = millis(); // Track the last time the LED state was changed.
}

// Function to move the robot forward until a specific stop condition is met.
void Movement::move_forward(int stop_condition, int wait_time) 
{
  bool stop_condition_found = false;
  bool left_sensor_value, right_sensor_value;
  int turn_direction;
  unsigned long start_time = millis();

  _new_movement_direction = STRAIGHT; // Assume straight movement initially.

  set_drive_motors_to_base_speed(); // Set motors to their base speed.
  start_drive_motors(); // Start both drive motors in the forward direction.

  while(!stop_condition_found)
  {
    _new_movement_direction = STRAIGHT; // Reset movement direction to straight.
    flash_movement_led(); // Blink the movement LED as an indicator.
    
    // Read the line sensor values.
    left_sensor_value = digitalRead(left_line_sensor);
    right_sensor_value = digitalRead(right_line_sensor);
    
    // Determine turn direction based on line sensor readings.
    if (left_sensor_value && right_sensor_value || !(left_sensor_value || right_sensor_value))
    {
      turn_direction = STRAIGHT;
    }
    else
    {
      turn_direction = left_sensor_value ? RIGHT : LEFT;
    }
    
    adjust_direction(turn_direction); // Adjust the robot's direction based on sensor input.
    // Check if the stop condition is met and the wait time has elapsed.
    stop_condition_found = check_stop_condition(stop_condition, 0) && ((millis() - start_time) > wait_time);
  }

  release_drive_motors(); // Stop the motors once the stop condition is found.
}

// Function to move the robot backward for a specified time or until a stop condition is met.
void Movement::move_backward(int stop_condition, int time_delay) 
{
  set_drive_motors_to_base_speed(); // Set motors to their base speed.
  _movement_direction = BACKWARD; // Set the direction to backward.
  start_drive_motors(); // Start the motors in the backward direction.

  // Wait until the stop condition is met.
  while(!check_stop_condition(stop_condition, time_delay));

  release_drive_motors(); // Stop the motors once the condition is met.
}

// Function to turn the robot 90 degrees in the specified direction.
void Movement::turn_90(bool direction) 
{
  unsigned long start_time = millis();

  _movement_direction = BACKWARD; // Set direction for alignment.
  set_drive_motors_to_base_speed(); // Set motors to base speed for the turn.

  set_motors_to_turn(direction); // Configure motors for turning.

  // Turn until both line sensors detect a line or a minimum time has elapsed.
  while (!(digitalRead(left_line_sensor) && digitalRead(right_line_sensor)) || (millis() < (start_time + 600)))
  {
    flash_movement_led(); // Blink the movement LED as an indicator during the turn.
  }

  release_drive_motors(); // Stop the motors once the turn is completed.
}

// Function to slightly adjust the robot's direction.
void Movement::turn_adjust(bool direction) 
{
  _movement_direction = BACKWARD; // Set direction for alignment.
  set_drive_motors_to_base_speed(); // Set motors to base speed for adjustment.

  set_motors_to_turn(direction); // Configure motors for turning.

  // Turn until either of the line sensors detects a line.
  while (!(digitalRead(left_line_sensor) || digitalRead(right_line_sensor)));

  release_drive_motors(); // Stop the motors once adjustment is done.
}

// Helper function to set the motor directions for turning.
void Movement::set_motors_to_turn(int direction)
{
  if (direction == RIGHT)
  {
    // Turn right by setting opposite directions for left and right motors.
    _right_drive_motor->run(FORWARD);
    _left_drive_motor->run(BACKWARD);
  }
  else
  {
    // Turn left by setting opposite directions for left and right motors.
    _left_drive_motor->run(FORWARD);
    _right_drive_motor->run(BACKWARD);
  }
}

// Function to set the base speed for the drive motors.
void Movement::set_drive_motors_to_base_speed()
{
  _new_right_drive_motor_speed = _right_base_speed; // Set the right motor to its base speed.
  _new_left_drive_motor_speed = _left_base_speed; // Set the left motor to its base speed.
  update_drive_motor_speed(); // Apply the new speeds to the motors.
}

// Function to stop both drive motors.
void Movement::release_drive_motors()
{
  _left_drive_motor->run(RELEASE); // Stop the left motor.
  _right_drive_motor->run(RELEASE); // Stop the right motor.
}

// Function to start the drive motors in the set movement direction.
void Movement::start_drive_motors()
{
  _left_drive_motor->run(_movement_direction); // Start the left motor in the movement direction.
  _right_drive_motor->run(_movement_direction); // Start the right motor in the movement direction.
  _movement_direction = FORWARD; // Reset the movement direction to forward after starting.
}

// Function to adjust the robot's direction based on sensor input.
void Movement::adjust_direction(int towards_direction)
{
  if (towards_direction == STRAIGHT)
  {
    // If direction is straight, ensure motors are running and set to base speed.
    if (_new_movement_direction != _movement_direction)
    {
      start_drive_motors();
    }
    set_drive_motors_to_base_speed();
  }
  else
  {
    // If turning, adjust motor directions for turning.
    set_motors_to_turn(towards_direction);
  }
}

// Function to update the speed of the drive motors.
void Movement::update_drive_motor_speed()
{
  // Apply new speed to left motor if it has changed.
  if (_new_left_drive_motor_speed != _left_drive_motor_speed)
  {
    _left_drive_motor_speed = _new_left_drive_motor_speed;
    _left_drive_motor->setSpeed(_left_drive_motor_speed);
  }
  // Apply new speed to right motor if it has changed.
  if (_new_right_drive_motor_speed != _right_drive_motor_speed)
  {
    _right_drive_motor_speed = _new_right_drive_motor_speed;
    _right_drive_motor->setSpeed(_right_drive_motor_speed);
  }
}

// Function to check if the robot has met the specified stop condition.
bool Movement::check_stop_condition(int stop_condition, int time_delay)
{
  switch (stop_condition)
  {
    case JUNCTION:
      // Stop if either junction sensor is triggered.
      return (digitalRead(right_junction_sensor) || digitalRead(left_junction_sensor));
    case PLATFORM:
      // Stop if the platform sensor is triggered.
      return (!digitalRead(platform_sensor));
    case DELAY:
      // Wait for the specified time delay, then stop.
      delay(time_delay);
      return true;
    case ENDSTOP:
      // Stop if neither line sensor detects a line.
      return (!(digitalRead(right_line_sensor) || digitalRead(left_line_sensor)));
    default:
      return false;
  }
}

// Function to guide the robot back to the starting box.
void Movement::return_to_start_box()
{
  int time_to_get_into_box = 500; // Time to drive into the start box.
  unsigned long start_time;

  start_drive_motors(); // Start motors to drive back to the box.
  // Wait until a junction sensor is triggered.
  while (!(digitalRead(right_junction_sensor) || digitalRead(left_junction_sensor)));

  delay(15); // Short delay for alignment.
  release_drive_motors(); // Stop motors temporarily.

  // Align with the junction line.
  if (digitalRead(right_junction_sensor))
  {
    // If the right sensor was triggered, align using the left motor.
    while (!digitalRead(left_junction_sensor))
    {
      _right_drive_motor->run(FORWARD);
    }
  }
  else
  {
    // If the left sensor was triggered, align using the right motor.
    _left_drive_motor->run(FORWARD);
    while (!digitalRead(right_junction_sensor));
  }

  start_drive_motors(); // Start motors again to enter the start box.
  start_time = millis();
  // Drive for a specified time to ensure the robot is fully inside the box.
  while (millis() < (start_time + time_to_get_into_box));

  release_drive_motors(); // Stop motors once inside the start box.
  digitalWrite(blue_led, LOW); // Turn off the movement LED.
  while (true); // Halt the program to prevent further movement.
}

// Function to toggle the state of the movement LED.
void Movement::toggle_movement_led()
{
  _movement_led_state = !_movement_led_state;
  digitalWrite(blue_led, _movement_led_state);
}
void Movement::flash_movement_led()
{
  if ((millis() - _last_led_change) > _led_time_period)
      {
        toggle_movement_led();
        _last_led_change = millis();
      }
}