#include "stepper_motor.h"

// STEPPER MOTORS
const int DIR = 10;  // SD3 
const int STEP = 9;  // SD2

#define motorInterfaceType 1

void stepper_motor_setup(){
  stepper_motor.setMaxSpeed(100); // Set maximum speed to 100 steps per second
  stepper_motor.setAcceleration(50); // Set acceleration to 50 steps per second per second
  stepper_motor.setSpeed(50); // Set current speed to 50 steps per second
  stepper_motor.move(360 / 1.8 * 2); // Move 2 cm (360 / 1.8 = 200 steps per revolution)
}