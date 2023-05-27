#include <ESP8266WiFi.h>   // Enables the ESP8266 to connect to the local network (via WiFi)
#include <PubSubClient.h> //Connect and publish to the MQTT broker
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
    // delay(500);
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
long encoderValue = 0;
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




void setup(){
  Serial.begin(115200); // Starts the serial communication
  Serial.println("In the void setup block");
  
  connect_MQTT();   // connect to the MQTT broker
  
  Serial.setTimeout(2000);
  client.setCallback(callback);

  // DC MOTOR
  Serial.begin(115200);
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


// STOPPING THE DC MOTORS
void stop_DC_motor(){
  analogWrite(ENA, 0);       // setting the PWM pin to 0% Duty cycle    
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // delay(1);
}


// MOVING THE SERVO MOTOR
void move_servo_motor(int SERVO_motion_type){
  if (SERVO_motion_type == 5){   // LEFT
    servo_motor.write(180);
  }
  if (SERVO_motion_type == 6){   // RIGHT
    servo_motor.write(20);
  }
  if (SERVO_motion_type == 7){
    servo_motor.write(115);     // RESET
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

    // using interrupts to monitor the encoder steps and exit the while loop once the encoder value exceeds the target encoder value
    while (abs_encoderValue < target_encoder_counts){     
      analogWrite(ENA, 100);

      // if (abs_encoderValue >= next_target){      // to use encoder values i.e. to scan ultrasonic sensors on the amount of distance moved instead of time
      //   next_target += 75;
      //   if (!check_ultrasonic_sensors()){
      //     stop_DC_motor();
      //   }
      //   else{
          
      //   }
      // }
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

    move_servo_motor(SERVO_LEFT);

    // actuating the DC motor
    analogWrite(ENA, 100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    long target_encoder_counts = wheel_rev_turn * encoder_counts_per_rev;      // 2443 = 1 * 2443  // to just move ahead by 30 cms
    encoder.write(0);     // to reset the encoders

    // using interrupts to monitor the encoder steps and exit the while loop once the encoder value exceeds the target encoder value
    while (abs_encoderValue < target_encoder_counts){     
      analogWrite(ENA, 100);
    }
    stop_DC_motor();
    move_servo_motor(SERVO_RESET);
  }


  // RIGHT MOTION
  if (DC_motion_type == DC_RIGHT){

    // actuating the servo motor first
    move_servo_motor(SERVO_RIGHT);

    // actuating the DC motor
    analogWrite(ENA, 100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    long target_encoder_counts = wheel_rev_turn * encoder_counts_per_rev;      // 2443 = 1 * 2443
    encoder.write(0);     // to reset the encoders

    // using interrupts to monitor the encoder steps and exit the while loop once the encoder value exceeds the target encoder value
    while (abs_encoderValue < target_encoder_counts){     
      analogWrite(ENA, 100);
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
  if ((strcmp(message.c_str(), "straight") == 0 && recvd_mqtt_voice_cmd) || (strcmp(message.c_str(), "straight") == 0 && recvd_mqtt_UI_cmd)){     // to start turning the wheels to the left if any other MQTT or UI commands are not being executed

    Serial.println("Going Straight!");

    // moving the motors according to the specified command
    move_DC_motor(DC_STRAIGHT);       // assuming that clockwise motion moves the walker forward
    client.publish(mqtt_voice_command_topic, "Have executed the Left command!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }


  // STOP THE WALKER FROM MOVING
  if ((strcmp(message.c_str(), "SM") == 0 && recvd_mqtt_voice_cmd) || (strcmp(message.c_str(), "SM") == 0 && recvd_mqtt_UI_cmd)){     // to execute this command only if any other MQTT or UI commands are not being executed

    Serial.println("Stopping now!");
    move_DC_motor(DC_STOP);
    client.publish(mqtt_voice_command_topic, "Have stopped moving!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }


  // GO LEFT
  if ((strcmp(message.c_str(), "left") == 0 && recvd_mqtt_voice_cmd) || (strcmp(message.c_str(), "left") == 0 && recvd_mqtt_UI_cmd)){     // to start turning the wheels to the left if any other MQTT or UI commands are not being executed

    Serial.println("Going Left!");
    move_DC_motor(DC_LEFT);
    client.publish(mqtt_voice_command_topic, "Have executed the Left command!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }


  // GO RIGHT
  if ((strcmp(message.c_str(), "right") == 0 && recvd_mqtt_voice_cmd) || (strcmp(message.c_str(), "right") == 0 && recvd_mqtt_UI_cmd)){     // to start turning the wheels to the left if any other MQTT or UI commands are not being executed

    Serial.println("Going Right!");
    move_DC_motor(DC_RIGHT);
    client.publish(mqtt_voice_command_topic, "Have executed the Right command!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }


  // GO REVERSE
  if ((strcmp(message.c_str(), "reverse") == 0 && recvd_mqtt_voice_cmd) || (strcmp(message.c_str(), "reverse") == 0 && recvd_mqtt_UI_cmd)){     // to start turning the wheels to the left if any other MQTT or UI commands are not being executed

    Serial.println("Going Reverse!");
    move_DC_motor(DC_REVERSE);
    client.publish(mqtt_voice_command_topic, "Have executed the Reverse command!");

    recvd_mqtt_voice_cmd = false;
    recvd_mqtt_UI_cmd = false;
  }

}


void loop(){
  
  // delay(1);
  client.loop();
}