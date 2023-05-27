//Code for the ESP8266

#include <ESP8266WiFi.h>   // Enables the ESP8266 to connect to the local network (via WiFi)
#include <PubSubClient.h> //Connect and publish to the MQTT broker
#include <AccelStepper.h>
#include <Servo.h>
#include <Encoder.h>


//WiFi
const char* ssid = "IIIPA";  // my network's SSID
const char* wifi_password = "ME310Printer";   // the password of my network


//MQTT
const char* mqtt_server = "192.168.30.3";
const char* mqtt_username = "avatar"; 
const char* mqtt_password = "avatar";
const char* clientID = "microcontroller_client";

//defined the MQTT topic to which I need to subscribe and then later publish as well
const char* mqtt_voice_command_topic = "V";
const char* mqtt_UI_command_topic = "UI";


// Initialize the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the broker
PubSubClient client(mqtt_server, 1883, wifiClient);


//custom function to connect to the MQTT broker via WiFi
void connect_MQTT(){
  Serial.println("Connecting to");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection has been confimed before continuing
  while (WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  // Debugging - Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  // client.connect returns a Boolean value to let us know if the connection was successful
  if (client.connect(clientID, mqtt_username, mqtt_password)){
    Serial.print(clientID);
    Serial.println(" Connected to MQTT Broker");
  }
  else{
    Serial.println("Connection to MQTT Broker failed...");
  }
}


// GLOBAL VARIABLES FOR DC AND SERVO MOTORS
#define DC_LEFT 0
#define DC_RIGHT 1
#define DC_STRAIGHT 2
#define DC_REVERSE 3
#define DC_STOP 4
#define SERVO_LEFT 5
#define SERVO_RIGHT 6
#define SERVO_RESET 7


// ULTRASONIC SENSORS
#define triggerPin_sensor_1 12   // D6
#define echoPin_sensor_1 14      // D5
#define triggerPin_sensor_2 4   // D2
#define echoPin_sensor_2 5   // D1

//define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034
#define CM_TO_INCH 0.393701

// to define the variables
long duration_sensor_1;
int distance_sensor_1;
long duration_sensor_2;
int distance_sensor_2;


// STEPPER MOTORS
const int DIR = 10;  // SD3 
const int STEP = 9;  // SD2

#define motorInterfaceType 1
AccelStepper stepper_motor(motorInterfaceType, STEP, DIR);


// DC MOTOR
int ENA = D7;
int IN1 = D1;
int IN2 = D2;
int encoderPinA = D5;
int encoderPinB = D6;

long encoder_value = 0;
long encoder_counts_per_rev = 2443;
int wheel_rev = 2;
int wheel_rev_turn = 1;
int encoder_subtraction_value = 750;     // as i just want to move the walker by 50 cms

Encoder encoder(encoderPinA, encoderPinB);
long encoderValue;
unsigned long abs_encoderValue;


// SERVO MOTOR
Servo servo_motor;     // creating a servo motor object
int servo_pin = 2;    // D4



// SETTING FLAGS
bool recvd_mqtt_voice_cmd = false;     // flag: when true indicates that a received voice cmd is being executed
bool recvd_mqtt_UI_cmd = false;        // flag: when true indicates that a received UI cmd is being executed


// Interrupt Service Function for the DC Motor
void ICACHE_RAM_ATTR DCMotorISR(){
  encoderValue = encoder.read();
  abs_encoderValue = abs(encoderValue);      // taking the absolute value as I am not concerned about the direction of rotation and I am using this function for forward and reverse motion
}




// TO setup all the sensors
void setup() {
  Serial.begin(115200); // Starts the serial communication
  Serial.println("In the void setup block");
  
  connect_MQTT();   // connect to the MQTT broker
  
  Serial.setTimeout(2000);
  client.setCallback(callback);
  
  // ULTRASONIC SENSORS
  pinMode(triggerPin_sensor_1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_sensor_1, INPUT); // Sets the echoPin as an Input
  pinMode(triggerPin_sensor_2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_sensor_2, INPUT); // Sets the echoPin as an Input

  // STEPPER MOTOR
  stepper_motor.setMaxSpeed(100); // Set maximum speed to 100 steps per second
  stepper_motor.setAcceleration(50); // Set acceleration to 50 steps per second per second
  stepper_motor.setSpeed(50); // Set current speed to 50 steps per second
  stepper_motor.move(360 / 1.8 * 2); // Move 2 cm (360 / 1.8 = 200 steps per revolution)

  // SERVO MOTOR
  servo_motor.attach(servo_pin);      // attaching servo motor to D4

  // DC MOTOR
  //Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(encoderPinA, DCMotorISR, CHANGE);  // to enable interrupt on encoder PinA

  // SUBSCRIBE TO TOPICS
  if (client.connected()){
    client.subscribe(mqtt_voice_command_topic);
    client.subscribe(mqtt_UI_command_topic);
    Serial.println("Have subscribed to the topics!!");
    delay(100);
  }
}



// RETURN BOOLEAN VALUE BASED ON DISTANCES RETURNED BY THE SENSORS - 60 CMS IS THE THRESHOLD
bool check_ultrasonic_sensors(){

  // FOR ULTRASONIC SENSOR 1
  digitalWrite(triggerPin_sensor_1, LOW); //set trigger signal low for 2us
  delayMicroseconds(2);
  
  /*send 10 microsecond pulse to trigger pin of HC-SR04 */
  digitalWrite(triggerPin_sensor_1, HIGH);  // make trigger pin active high
  delayMicroseconds(10);            // wait for 10 microseconds
  digitalWrite(triggerPin_sensor_1, LOW);   // make trigger pin active low
  
  /*Measure the Echo output signal duration or pulss width */
  duration_sensor_1 = pulseIn(echoPin_sensor_1, HIGH); // save time duration value in "duration variable
  distance_sensor_1= duration_sensor_1*0.034/2; //Convert pulse duration into distance

  Serial.print("Distance from ultrasonic sensor 1 is: ");
  Serial.println(distance_sensor_1);

  
  // FOR ULTRASONIC SENSOR 2
  digitalWrite(triggerPin_sensor_2, LOW); 
  delayMicroseconds(2);
  
  /*send 10 microsecond pulse to trigger pin of HC-SR04 */
  digitalWrite(triggerPin_sensor_2, HIGH);  
  delayMicroseconds(10);            
  digitalWrite(triggerPin_sensor_2, LOW);   
  
  /*Measure the Echo output signal duration or pulss width */
  duration_sensor_2 = pulseIn(echoPin_sensor_2, HIGH); 
  distance_sensor_2 = duration_sensor_2*0.034/2; 

  Serial.print("Distance from ultrasonic sensor 2 is: ");
  Serial.println(distance_sensor_2);

  // RETURN a bool value of True if the distances returned by the sensors is greater than 50 cms
  if (distance_sensor_1 > 60 && distance_sensor_2 > 60){
    return true;
  }
  else{
    return false;
  }
}


// STOPPING THE DC MOTORS
void stop_DC_motor(){
  analogWrite(ENA, 0);       // setting the PWM pin to 0% Duty cycle    
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(1);
}


// MOVING THE SERVO MOTOR
void move_servo_motor(int SERVO_motion_type){
  if (SERVO_motion_type == 5){
    servo_motor.write(180);       // LEFT
  }
  if (SERVO_motion_type == 6){    // RIGHT
    servo_motor.write(20);
  }
  if (SERVO_motion_type == 7){    // RESET
    servo_motor.write(115);
  }
}


// MOVING THE DC MOTOR
void move_DC_motor(int DC_motion_type){

  // STRAIGHT MOTION
  if (DC_motion_type == DC_STRAIGHT){
    //rotate the DC motors clockwise; move 50 cms forward   // assuming clokwise take the walker forward

    analogWrite(ENA, 100);   // to rotate the DC motor at 65 rpm
    digitalWrite(IN1, HIGH);  // this is the clockwise direction
    digitalWrite(IN2, LOW);

    long target_encoder_counts = wheel_rev * encoder_counts_per_rev;    // 4886 = 2 * 2443; (75 encoder steps = walker moving 1cm forward)
    target_encoder_counts = target_encoder_counts - encoder_subtraction_value;                  // to scan the ultrasonic distances for 60 cms but just moving forward for 50 cms
    encoder.write(0);     // to reset the encoder

    int counter = 0;    // defining the counter to scan ultrasonic sensors after 75 encoder steps i.e. after every 1 cm

    long next_target = 75;

    // using interrupts to monitor the encoder steps and exit the while loop once the encoder value exceeds the target encoder value
    while (abs_encoderValue < target_encoder_counts){     
      analogWrite(ENA, 100);
      counter++;

      // if (abs_encoderValue >= next_target){      // to use encoder values i.e. to scan ultrasonic sensors on the amount of distance moved instead of time
      //   next_target += 75;
      //   if (!check_ultrasonic_sensors()){
      //     stop_DC_motor();
      //   }
      //   else{
          
      //   }
      // }

      if (counter >= 150){        // to scan the ultrasonic sensors after every 75 encoder steps i.e. after every 1 cm that the walker has moved
        counter = 0;             // reset the counter value as I have to scan the sensors again after 75 encoder steps i.e. after 1 cm
        if (!check_ultrasonic_sensors()){
          stop_DC_motor();         // stop the motors if the ultrasonic sensor def return False
        }
        else{  // else continue moving forward
        }
      }
    }
    stop_DC_motor();
    move_servo_motor(SERVO_RESET);
  }


  // REVERSE MOTION
  if (DC_motion_type == DC_REVERSE){
    // rotate the DC motors anticlockwise; move 50 cms backward      (assuming anticlokwise takes the walker backward)

    analogWrite(ENA, 100);    // rotate the DC motor at 65 rpm
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    long target_encoder_counts = wheel_rev * encoder_counts_per_rev;    // 4886 = 2 * 2443
    target_encoder_counts = target_encoder_counts - encoder_subtraction_value;
    encoder.write(0);    // to reset the enocder

    while (abs_encoderValue < target_encoder_counts){
      analogWrite(ENA, 100);
    }
    stop_DC_motor();
    move_servo_motor(SERVO_RESET);
  }


  // LEFT MOTION
  if (DC_motion_type == DC_LEFT){

    // rotating the servo motor by some angle and then moving the walker ahead by 30 cms

    // actuating the servo motor first
    move_servo_motor(SERVO_LEFT);

    // actuating the DC motor
    analogWrite(ENA, 100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    long target_encoder_counts = wheel_rev_turn * encoder_counts_per_rev;      // 2443 = 1 * 2443  // to just move ahead by 30 cms
    encoder.write(0);     // to reset the encoders

    int counter = 0;

    // using interrupts to monitor the encoder steps and exit the while loop once the encoder value exceeds the target encoder value
    while (abs_encoderValue < target_encoder_counts){     
      analogWrite(ENA, 100);
      counter++;

      if (counter >= 75){        // to scan the ultrasonic sensors after every 75 encoder steps i.e. after every 1 cm that the walker has moved
        counter = 0;             // reset the counter value as I have to scan the sensors again after 75 encoder steps i.e. after 1 cm
        if (!check_ultrasonic_sensors()){
          stop_DC_motor();         // stop the motors if the ultrasonic sensor def return False
        }
        else{  // else continue moving forward
        }
      }
    }
    stop_DC_motor();
    move_servo_motor(SERVO_RESET);
  }


  // RIGHT MOTION
  if (DC_motion_type == DC_RIGHT){

    // rotating the servo motor by some angle and then moving the walker ahead by 30 cms

    // actuating the servo motor first
    move_servo_motor(SERVO_RIGHT);

    // actuating the DC motor
    analogWrite(ENA, 100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    long target_encoder_counts = wheel_rev_turn * encoder_counts_per_rev;      // 2443 = 1 * 2443
    encoder.write(0);     // to reset the encoders

    int counter = 0;

    // using interrupts to monitor the encoder steps and exit the while loop once the encoder value exceeds the target encoder value
    while (abs_encoderValue < target_encoder_counts){     
      analogWrite(ENA, 100);
      counter++;

      if (counter >= 75){        // to scan the ultrasonic sensors after every 75 encoder steps i.e. after every 1 cm that the walker has moved
        counter = 0;             // reset the counter value as I have to scan the sensors again after 75 encoder steps i.e. after 1 cm
        if (!check_ultrasonic_sensors()){
          stop_DC_motor();         // stop the motors if the ultrasonic sensor def return False
        }
        else{  // else continue moving forward
        }
      }
    }
    stop_DC_motor();
    move_servo_motor(SERVO_RESET);
  }


  // STOPPING THE DC MOTOR
  if (DC_motion_type == DC_STOP){
    // stopping the DC motor based on the user's commands (different from the stop function but essentially does the same thing)

    analogWrite(ENA, 0);       // setting the PWM pin to 0% Duty cycle    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}



// FUNCTION WHICH LISTENS TO MQTT MESSAGES
void callback(char* topic, byte* payload, unsigned int length) {
  
  // to ensure that the user is not sending voice and UI commands simulataneously and just execute the one that is received first
  String topicString = String(topic);

  if (topicString == mqtt_voice_command_topic){
    recvd_mqtt_voice_cmd = true;
    recvd_mqtt_UI_cmd = false;
  }
  if (topicString == mqtt_voice_command_topic){
    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = true;
  }
  
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);

  // Convert payload to a string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Payload: ");
  Serial.println(message);



  // GO Straight
  if ((strcmp(message.c_str(), "straight") == 0 && recvd_mqtt_voice_cmd && check_ultrasonic_sensors()) || (strcmp(message.c_str(), "straight") == 0 && recvd_mqtt_UI_cmd && check_ultrasonic_sensors())){     // to start turning the wheels to the left if any other MQTT or UI commands are not being executed

    Serial.println("Going Straight!");

    // moving the motors according to the specified command
    move_DC_motor(DC_STRAIGHT);       // assuming that clockwise motion moves the walker forward
    client.publish(mqtt_voice_command_topic, "Have executed the Left command!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }
  else {
    client.publish(mqtt_voice_command_topic, "Waiting for 3 seconds to check if the object in front of me moves away!");
    delay(3000);

    if (check_ultrasonic_sensors()){

      Serial.println("Going Straight!");

      // moving the motors according to the specified command
      move_DC_motor(DC_STRAIGHT);       // assuming that clockwise motion moves the walker forward
      client.publish(mqtt_voice_command_topic, "Have executed the Straight command!");

      recvd_mqtt_voice_cmd = false;
      recvd_mqtt_UI_cmd = false;
    }
    else{
      client.publish(mqtt_voice_command_topic, "Object is still in front of me and so I cannot go straight! Please try something else!");
    }
  }


  // START THE AUTONOMOUS SYSTEM MODE
  if ((strcmp(message.c_str(), "StartA") == 0 && recvd_mqtt_voice_cmd && check_ultrasonic_sensors()) || (strcmp(message.c_str(), "StartA") == 0 && recvd_mqtt_UI_cmd && check_ultrasonic_sensors())){       // to execute this command only if any other MQTT or UI commands are not being executed

    Serial.println("Starting the autonomous system mode!");

    stepper_motor.move(360 / 1.8 * 3);   // it should be * 2 if we have to move it 2 cm. Also assuming that CLOCKWISE DIRECTION IS THE ENGAGEMENT
    while (stepper_motor.distanceToGo() != 0){
      stepper_motor.run();
    }
    delay(100);

    client.publish(mqtt_voice_command_topic, "Have activated the autonomous system mode!");
    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }


  // STOP THE AUTONOMOUS SYSTEM MODE  
  if ((strcmp(message.c_str(), "StopA") == 0 && recvd_mqtt_voice_cmd && check_ultrasonic_sensors()) || (strcmp(message.c_str(), "StopA") == 0 && recvd_mqtt_UI_cmd && check_ultrasonic_sensors())){       // to execute this command only if any other MQTT or UI commands are not being executed

    Serial.println("Stopping the autonomous system mode!");

    stepper_motor.move(- 360 / 1.8 * 3);   // it should be * 2 if wehave to move it 2 cm. Also assuming that ANTICLOCKWISE DIRECTION IS THE DISENGAGEMENT
    while (stepper_motor.distanceToGo() != 0){
      stepper_motor.run();
    }
    delay(100); 

    client.publish(mqtt_voice_command_topic, "Have deactivated the autonomous system mode!");
    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }


  // STOP THE WALKER FROM MOVING
  if ((strcmp(message.c_str(), "SM") == 0 && recvd_mqtt_voice_cmd && check_ultrasonic_sensors()) || (strcmp(message.c_str(), "SM") == 0 && recvd_mqtt_UI_cmd && check_ultrasonic_sensors())){     // to execute this command only if any other MQTT or UI commands are not being executed

    Serial.println("Stopping now!");
    move_DC_motor(DC_STOP);
    client.publish(mqtt_voice_command_topic, "Have stopped moving!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }


  // GO LEFT
  if ((strcmp(message.c_str(), "left") == 0 && recvd_mqtt_voice_cmd && check_ultrasonic_sensors()) || (strcmp(message.c_str(), "left") == 0 && recvd_mqtt_UI_cmd && check_ultrasonic_sensors())){     // to start turning the wheels to the left if any other MQTT or UI commands are not being executed

    Serial.println("Going Left!");
    move_DC_motor(DC_LEFT);
    client.publish(mqtt_voice_command_topic, "Have executed the Left command!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }
  else {
    client.publish(mqtt_voice_command_topic, "Waiting for 3 seconds to check if the object in front of me moves away!");
    delay(3000);

    if (check_ultrasonic_sensors()){

      Serial.println("Going Left!");
      move_DC_motor(DC_LEFT);
      client.publish(mqtt_voice_command_topic, "Have executed the Left command!");

      recvd_mqtt_voice_cmd = false;
      recvd_mqtt_UI_cmd = false;
    }
    else{
      client.publish(mqtt_voice_command_topic, "Object is still in front of me and so I cannot turn left!");
    }
  }


  // GO RIGHT
  if ((strcmp(message.c_str(), "right") == 0 && recvd_mqtt_voice_cmd && check_ultrasonic_sensors()) || (strcmp(message.c_str(), "right") == 0 && recvd_mqtt_UI_cmd && check_ultrasonic_sensors())){     // to start turning the wheels to the left if any other MQTT or UI commands are not being executed

    Serial.println("Going Right!");
    move_DC_motor(DC_RIGHT);
    client.publish(mqtt_voice_command_topic, "Have executed the Right command!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }
  else {
    client.publish(mqtt_voice_command_topic, "Waiting for 3 seconds to check if the object in front of me moves away!");
    delay(3000);

    if (check_ultrasonic_sensors()){

      Serial.println("Going Right!");
      move_DC_motor(DC_RIGHT);
      client.publish(mqtt_voice_command_topic, "Have executed the Left command!");

      recvd_mqtt_voice_cmd = false;
      recvd_mqtt_UI_cmd = false;
    }
    else{
      client.publish(mqtt_voice_command_topic, "Object is still in front of me and so I cannot turn Right!");
    }
  }


  // GO REVERSE
  if ((strcmp(message.c_str(), "reverse") == 0 && recvd_mqtt_voice_cmd) || (strcmp(message.c_str(), "reverse") == 0 && recvd_mqtt_UI_cmd)){     // to start turning the wheels to the left if any other MQTT or UI commands are not being executed

    Serial.println("Going Reverse!");
    move_DC_motor(DC_REVERSE);
    client.publish(mqtt_voice_command_topic, "Have executed the Reverse command!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }
  else {
    client.publish(mqtt_voice_command_topic, "Waiting for 3 seconds to check if the object in front of me moves away!");
    delay(3000);

    if (check_ultrasonic_sensors()){

      Serial.println("Going Reverse!");
      move_DC_motor(DC_REVERSE);
      client.publish(mqtt_voice_command_topic, "Have executed the Reverse command!");

      recvd_mqtt_voice_cmd = false;
      recvd_mqtt_UI_cmd = false;
    }
    else{
      client.publish(mqtt_voice_command_topic, "Object is still in front of me and so I cannot Reverse!");
    }
  }


  // GO TO THE CHARGING DOCK
  // YET TO BE EXECUTED

}



void loop() {

  delay(1);
  client.loop();
  
}
