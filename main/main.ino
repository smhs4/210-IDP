#include "movement.h"           // Include the Movement library
#include "blockManipulation.h"  // Include the BlockManipulation library
#include "robot.h"
bool colour;

void setup() 
{
  Serial.begin(9600);
  Robot myRobot = Robot();
  // First Block
  myRobot.move_forward(JUNCTION, 0);
  myRobot.move_forward();
  myRobot.turn_90(LEFT);
  myRobot.move_forward(); 
  myRobot.turn_90(RIGHT);
  myRobot.pickup_sequence(&colour, 1700);
  
  if (colour == RED) 
  {
    myRobot.turn_90(RIGHT);
    myRobot.move_forward();
    myRobot.move_forward();
    myRobot.turn_90(RIGHT);
    
  }
  else 
  {

    myRobot.turn_90(LEFT);
    myRobot.move_forward();
    myRobot.turn_90(LEFT);
  } 
  myRobot.drop_sequence(colour);

  // Second Block
  myRobot.turn_90(LEFT);
  myRobot.move_forward();
  myRobot.turn_90(LEFT);

  myRobot.pickup_sequence(&colour, 1300);

  if (colour==RED) 
  {
    myRobot.turn_90(LEFT);
    myRobot.move_forward();
    myRobot.turn_90(RIGHT);
    myRobot.move_forward();
  } 
  else 
  {

    myRobot.turn_90(RIGHT);
    myRobot.move_forward();
    myRobot.move_forward();
    myRobot.turn_90(LEFT);
    myRobot.move_forward();
  }
  myRobot.drop_sequence(colour);

  //Third Block
  myRobot.move_forward();
  myRobot.turn_90(LEFT);
  myRobot.pickup_sequence(&colour, 1900, true, LEFT);

  if (colour==RED) 
  {
    myRobot.turn_90(LEFT);
    myRobot.move_forward();
    myRobot.move_forward();
  }
  else 
  {
    myRobot.turn_90(RIGHT);
    myRobot.move_forward();
    myRobot.move_forward();
  }
  myRobot.drop_sequence(colour);

  //Fourth Block
  myRobot.turn_90(LEFT);
  myRobot.move_forward();
  myRobot.move_forward();
  myRobot.turn_90(RIGHT);
  myRobot.move_forward();
  myRobot.turn_90(LEFT);

  myRobot.pickup_sequence(&colour, 2700, true, LEFT);
  myRobot.turn_90(LEFT);
  myRobot.move_forward();

  if (colour==RED) 
  {
    myRobot.turn_90(LEFT);
    myRobot.move_forward();
    myRobot.move_forward();
    myRobot.turn_90(RIGHT);
    myRobot.move_forward();
    


  }
  else 
  {
    myRobot.turn_90(RIGHT);
    myRobot.move_forward();
    myRobot.turn_90(LEFT);
    myRobot.move_forward();
  }
  myRobot.drop_sequence(colour,END);

}

void loop(){}


