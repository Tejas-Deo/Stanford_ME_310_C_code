#include <Servo.h>

Servo myservo;  // Create a servo object

void setup() {
  myservo.attach(2);  // Attach the servo to D4
}

void loop() {
  myservo.write(90);  // Move the servo to 90 degrees
  delay(1000);       // Wait for 1 second
  myservo.write(0);   // Move the servo to 0 degrees
  delay(1000);       // Wait for 1 second
}
