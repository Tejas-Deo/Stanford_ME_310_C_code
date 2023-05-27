#include "ultrasonic_sensors.h"

#define triggerPin_sensor_1 12
#define echoPin_sensor_1 14
#define triggerPin_sensor_2 4
#define echoPin_sensor_2 5

void ultrasonicSetup() {
  pinMode(triggerPin_sensor_1, OUTPUT);
  pinMode(echoPin_sensor_1, INPUT);
  pinMode(triggerPin_sensor_2, OUTPUT);
  pinMode(echoPin_sensor_2, INPUT);
}

bool checkUltrasonicSensors() {
  digitalWrite(triggerPin_sensor_1, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin_sensor_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin_sensor_1, LOW);
  duration_sensor_1 = pulseIn(echoPin_sensor_1, HIGH);
  distance_sensor_1 = duration_sensor_1 * 0.034 / 2;

  Serial.print("Distance from ultrasonic sensor 1 is: ");
  Serial.println(distance_sensor_1);

  digitalWrite(triggerPin_sensor_2, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin_sensor_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin_sensor_2, LOW);
  duration_sensor_2 = pulseIn(echoPin_sensor_2, HIGH);
  distance_sensor_2 = duration_sensor_2 * 0.034 / 2;

  Serial.print("Distance from ultrasonic sensor 2 is: ");
  Serial.println(distance_sensor_2);

  if (distance_sensor_1 > 60 && distance_sensor_2 > 60) {
    return true;
  } else {
    return false;
  }
}
