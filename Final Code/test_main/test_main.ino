#include  <ESP8266WiFi.h>   // Enables the ESP8266 to connect to the local network (via WiFi)
#include <PubSubClient.h> //Connect and publish to the MQTT broker
#include <AccelStepper.h>
#include <Encoder.h>
#include <Servo.h>

#include "config.h"
#include "ultrasonic_sensors.h"
#include "stepper_motor.h"
#include "dc_motor.h"
#include "servo_motor.h"


// Initialize the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the broker
PubSubClient client(mqtt_server, 1883, wifiClient);

// Stepper Motor
AccelStepper stepper_motor(motorInterfaceType, STEP, DIR);

// DC Motor
Encoder encoder(encoderPinA, encoderPinB);

// Servo Motor
Servo servo_motor;

// SETTING FLAGS
bool recvd_mqtt_voice_cmd = false;     // flag: when true indicates that a received voice cmd is being executed
bool recvd_mqtt_UI_cmd = false;        // flag: when true indicates that a received UI cmd is being executed

// Interrupt Service Function for the DC Motor
void ICACHE_RAM_ATTR DCMotorISR(){
  encoderValue = encoder.read();
  abs_encoderValue = abs(encoderValue);      // taking the absolute value as I am not concerned about the direction of rotation and I am using this function for forward and reverse motion
}



void setup() {
  Serial.begin(115200);

  connect_MQTT();   // connect to MQTT

  Serial.setTimeout(2000);
  client.setCallback(callback);

  ultrasonicSetup();   // setup the ultrasonic sensors

  servo_motor.attach(servo_in);       // attaching the servo motor pin to D4



}

void loop() {
  
  bool result = checkUltrasonicSensors();

}
