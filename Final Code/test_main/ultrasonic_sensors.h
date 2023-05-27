#ifndef ULTRASONIC_H
#define ULTRASONIC_H

// to define the variables
extern long duration_sensor_1;
extern int distance_sensor_1;
extern long duration_sensor_2;
extern int distance_sensor_2;

void ultrasonicSetup();
bool check_ultrasonic_sensors();

#endif // ULTRASONIC_H