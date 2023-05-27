#include <AccelStepper.h>

const int DIR = 10;
const int STEP = 9;

#define motorInterfaceType 1
AccelStepper stepper_motor(motorInterfaceType, STEP, DIR);

void setup() {
  Serial.begin(115200);
  stepper_motor.setMaxSpeed(100); // Set maximum speed to 100 steps per second
  stepper_motor.setAcceleration(50); // Set acceleration to 50 steps per second per second
  stepper_motor.setSpeed(50); // Set current speed to 50 steps per second
  stepper_motor.move(360 / 1.8 * 2); // Move 2 cm (360 / 1.8 = 200 steps per revolution)
}

void loop() {
  
  // Move 2 cm in the clockwise direction
  Serial.println("Moving in anticlockwise directrion");
  stepper_motor.move(360 / 1.8 * 4);
  while (stepper_motor.distanceToGo() != 0) {
    stepper_motor.run();
  }
  delay(1000); // Wait for 1 second
}

