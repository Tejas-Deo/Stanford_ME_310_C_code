#include <AccelStepper.h>
#include <Encoder.h>
#include <Servo.h>

// Motor configuration
const int motorPin1 = 9;      // Step pin for stepper motor
const int motorPin2 = 10;     // Direction pin for stepper motor
const int stepsPerRevolution = 200; // Number of steps per revolution

const int ENB = D7;           // Enable pin for DC motor
const int IN3 = D1;           // IN3 pin for DC motor
const int IN4 = D2;           // IN4 pin for DC motor

const int encoderPinA = D5;   // Encoder pin A
const int encoderPinB = D6;   // Encoder pin B

const int servoPin = 15;      // Servo motor pin
const int targetPos = 360; // Target position in degrees (one full revolution)

AccelStepper stepper(AccelStepper::DRIVER, motorPin1, motorPin2);
Encoder encoder(encoderPinA, encoderPinB);
Servo servo_motor;

bool clockwise = true;

void setup() {
  // Stepper motor setup
  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(500.0);

  // DC motor setup
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Servo motor setup
  servo_motor.attach(servoPin);
  servo_motor.write(115);
}

void loop() {
  // Stepper motor movement
  //int targetPos = 560; // Target position in degrees (one full revolution)
  delay(1);
  stepper.moveTo( 360/1.8 * 3);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(1000);

  // DC motor forward movement
  moveDCMotorForward(6);
  delay(1000);

  // Servo motor clockwise rotation
  rotateServoClockwise();
  delay(1000);

  // DC motor forward movement
  moveDCMotorForward(6);
  delay(1000);

  servo_motor.write(115);
  delay(1000);
  // Servo motor counterclockwise rotation
  rotateServoCounterclockwise();
  delay(1000);

  // DC motor forward movement
  moveDCMotorForward(6);
  delay(1000);

  stepper.moveTo(-targetPos * 360 / stepsPerRevolution);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(1000);
}

void moveDCMotorForward(float rotations) {
  analogWrite(ENB, 100);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(500);

  int targetEncoderValue = encoder.read() + (rotations * 360);

  while (encoder.read() > targetEncoderValue) {
    analogWrite(ENB, 100);
    delay(20);
  }

  stopDCMotor();
}

void stopDCMotor() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(1000);
}

void rotateServoClockwise() {
  // servo_motor.write(115);  // Move the servo to base position
  // delay(1000);
  servo_motor.write(20);   // Move the servo clockwise to 0 degrees
  delay(1000);
  // servo_motor.write(115);  // Move the servo to base position
  // delay(1000);
}

void rotateServoCounterclockwise() {
  // servo_motor.write(115);  // Move the servo to base position
  // delay(1000);
  servo_motor.write(180);  // Move the servo counterclockwise to 180 degrees
  delay(1000);
  // servo_motor.write(115);  // Move the servo to base position
  // delay(1000);
}