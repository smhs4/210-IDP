#include "blockManipulation.h"


BlockManipulation::BlockManipulation(Adafruit_DCMotor * arm_lifting_motor): _arm_lifting_motor(arm_lifting_motor)
{
  
  _my_servo.attach(_servo_pin);
  _my_servo.write(_open_angle);
}

void BlockManipulation::pick_up_block()
{
  /* Close claw around block */
  change_servo(_closed_angle, _my_servo);
  delay(300);
  /* Lift block */
  _arm_lifting_motor -> run(BACKWARD);
  while(digitalRead(lowering_switch) == LOW);
  Serial.println(_arm_lifting_time);
  delay(_arm_lifting_time);
  _arm_lifting_motor -> run(RELEASE);

}

void BlockManipulation::drop_block() 
{
  /* Open claw from around block */
  change_servo(_open_angle, _my_servo);
  delay(500);

  /* Lower arm */
  _arm_lifting_motor -> run(FORWARD);
  unsigned long start_time = millis();
  while(digitalRead(lowering_switch) == HIGH);
  delay(100);
  _arm_lifting_motor -> run(RELEASE);

}
void BlockManipulation::get_colour(bool *colour)
{
  uint16_t clear, red, green, blue;
  tcs.getRGBC(&red, &green, &blue, &clear);
  // reads color data until sensor gives realistic values
  while((red == 0) || ((red + blue + green + clear) > 30000))
  {
    tcs.getRGBC(&red, &green, &blue, &clear);
  }

  tcs.lock();

  // compute color ratios

  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;

  // check if red percentage is above 55%
  if (red > 0.55*clear)
  {
    
    Serial.println("RED");
    *colour = RED;
  } 
  else 
  {
    Serial.println("BLACK");
    *colour = BLACK;
  }

  delay(1000);
  Serial.println("");
  
}
void BlockManipulation::change_servo(int angle, Servo servo)
{
  for (int i=0; i<5; i++)
  {
    servo.write(angle);
    delay(15);
  }
}