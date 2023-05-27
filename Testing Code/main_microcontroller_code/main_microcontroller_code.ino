//Code for the ESP8266

#include <ESP8266WiFi.h>   // Enables the ESP8266 to connect to the local network (via WiFi)
#include <PubSubClient.h> //Connect and publish to the MQTT broker
#include <AccelStepper.h>

// Serial.println("Have imported the required documents");

//WiFi
const char* ssid = "IIIPA";  // my network's SSID
const char* wifi_password = "ME310Printer";   // the password of my network


//MQTT
const char* mqtt_server = "192.168.30.3";
const char* mqtt_username = "avatar"; 
const char* mqtt_password = "avatar";
const char* clientID = "ultrasonic_sensor_ids";

const char* ultrasonic_sensor_1_topic = "ultrasonic_sensor/sensor_1";
const char* ultrasonic_sensor_2_topic = "ultrasonic_sensor/sensor_2";
const char* start_autonomous_system_topic = "AutoSystem/Start";
const char* stop_autonomous_system_topic = "AutoSystem/Stop";
const char* charging_dock_topic = "AutoSystem/ChargingDock";
const char* go_straight_topic = "AutoSystem/GoStraight";
const char* go_right_topic = "AutoSystem/GoRight";
const char* go_left_topic = "AutoSystem/GoLeft";
const char* go_reverse_topic = "AutoSystem/GoReverse";
const char* stop_moving_topic = "AutoSystem/StopMoving";



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



// TO define the GPIOs for the ultrasonics sensor 1
#define triggerPin_sensor_1 12   // D6
#define echoPin_sensor_1 14      // D5
#define triggerPin_sensor_2 4   // D2
#define echoPin_sensor_2 5   // D1

const int DIR = 10;  // SD3 
const int STEP = 9;  // SD2

#define motorInterfaceType 1
AccelStepper stepper_motor(motorInterfaceType, STEP, DIR);


//define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034
#define CM_TO_INCH 0.393701


/* two variables to store duraion and distance value */
long duration_sensor_1;
int distance_sensor_1;
long duration_sensor_2;
int distance_sensor_2;





// TO setup all the sensors
void setup() {
  Serial.begin(115200); // Starts the serial communication
  Serial.println("In the void setup block");
  
  connect_MQTT();   // connect to the MQTT broker
  
  Serial.setTimeout(2000);

  client.setCallback(callback);
  
  pinMode(triggerPin_sensor_1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_sensor_1, INPUT); // Sets the echoPin as an Input

  pinMode(triggerPin_sensor_2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_sensor_2, INPUT); // Sets the echoPin as an Input


  stepper_motor.setMaxSpeed(100); // Set maximum speed to 100 steps per second
  stepper_motor.setAcceleration(50); // Set acceleration to 50 steps per second per second
  stepper_motor.setSpeed(50); // Set current speed to 50 steps per second
  stepper_motor.move(360 / 1.8 * 2); // Move 2 cm (360 / 1.8 = 200 steps per revolution)
  

  if (client.connected()){
    client.subscribe(start_autonomous_system_topic);
    client.subscribe(stop_autonomous_system_topic);
    client.subscribe(charging_dock_topic);
    client.subscribe(go_straight_topic);
    client.subscribe(go_left_topic);
    client.subscribe(go_right_topic);
    client.subscribe(go_reverse_topic);
    client.subscribe(stop_moving_topic);
    Serial.println("Have subscribed to the topics!!");
    delay(5000);
  }
}



void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  

  // TO START THE AUTONOMOUS SYSTEM MODE
  if (String(topic) == start_autonomous_system_topic) {
    Serial.println("Starting the autonomous system mode!!!");
//    delay(2000);

    // currently I am assuming that the I am actuating the assembly my spinning the stepper motor in the clockwise direction

    stepper_motor.move(360 / 1.8 * 3);   // it should be * 2 if wehave to move it 2 cm
    while (stepper_motor.distanceToGo() != 0){
      stepper_motor.run();
    }
    delay(1000); // Wait for 1 second
      
    }


  // TO STOP THE AUTONOMOUS SYSTEM MODE
  if (String(topic) == stop_autonomous_system_topic){
    Serial.println("Stopping the autonomous system mode!!!");
    //delay(2000);

    // to move the motor in the anticlockwise direction
    stepper_motor.move(- 360 / 1.8 * 3);   // it should be * 2 if wehave to move it 2 cm
    while (stepper_motor.distanceToGo() != 0){
      stepper_motor.run();
    }
    delay(1000); 
    
  }


  // TO GO TO THE CHARGING DOCK
  if (String(topic) == charging_dock_topic){
    Serial.println("Going to the charging dock!!!");
    delay(2000);

    // execute this block of code. Assuming that the walker is going in a straight line, continuously take feedback from the ultrasonic sensors and stop the motor if the distances returned by the ultrasonic sensors are less than 20cms
  }

  
  
  // TO GO STRAIGHT
  if (String(topic) == go_straight_topic){
    Serial.println("Going straight!!");
    delay(2000);

    // execute this code and make use of the distance values from the ultrasonic sensors; using a DC motor. Want to move the walker forward by 70 cms if the user gives the command to go straight
  }

  
  
  // TO GO LEFT
  if (String(topic) == go_left_topic){
    Serial.println("Going Left!");
    delay(2000);

    // execute this code and make use of the distance values from the ultrasonic sensors; using a DC motor for actuating and a servo motor for turning. First turn the wheel to a certain angle and then move for a total distances of 70 cms
  }

  
  
  // TO GO RIGHT
  if (String(topic) == go_right_topic){
    Serial.println("Going Right");
    delay(2000);

    // execute this code and make use of the distance values from the ultrasonic sensors; using a DC motor for actuating and a servo motor for turning. First turn the wheel to a certain angle and then move for a total distances of 70 cms
  }

  
  
  
  // TO GO REVERSE
  if (String(topic) == go_reverse_topic){
    Serial.println("Going reverse");
    delay(2000);

    // execute this code and make use of the distance values from the ultrasonic sensors; using a DC motor. Want to move the walker backward by 70 cms if the user gives the command to go straight
  }

  
  
  // TO STOP MOVING
  if (String(topic) == stop_moving_topic){
    Serial.println("Stopping to move now");
    delay(2000);

    // execute this code to stop the main DC motor from moving, simulataneously also give commands to the servo motor
  }


}





void loop() {

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
  digitalWrite(triggerPin_sensor_2, LOW); //set trigger signal low for 2us
  delayMicroseconds(2);
  
  /*send 10 microsecond pulse to trigger pin of HC-SR04 */
  digitalWrite(triggerPin_sensor_2, HIGH);  // make trigger pin active high
  delayMicroseconds(10);            // wait for 10 microseconds
  digitalWrite(triggerPin_sensor_2, LOW);   // make trigger pin active low
  
  /*Measure the Echo output signal duration or pulss width */
  duration_sensor_2 = pulseIn(echoPin_sensor_2, HIGH); // save time duration value in "duration variable
  distance_sensor_2 = duration_sensor_2*0.034/2; //Convert pulse duration into distance

  Serial.print("Distance from ultrasonic sensor 2 is: ");
  Serial.println(distance_sensor_2);

  delay(500);



  // PUBLISH ULTRASONIC SENSOR 1 INFORMATION TO THE BROKER
  if (client.publish(ultrasonic_sensor_1_topic, String(distance_sensor_1).c_str())){
    Serial.println("Distance from SENSOR 1 sent!");
  }
  
  //The client.publish will return a boolean value depedning on whether it succeded or not
  //If the message failed to send, we will try again, as the connection may have broken
  else{
    Serial.println("Failed to send the distance to the broker. Reconnecting to the MQTT broker and tryinng again!");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // to make sure that the client.publish() does not clash with the client.connect() call
    client.publish(ultrasonic_sensor_1_topic, String(distance_sensor_1).c_str());
  }



  // PUBLISH ULTRASONIC SENSOR 2 INFORMATION TO THE BROKER
  if (client.publish(ultrasonic_sensor_2_topic, String(distance_sensor_2).c_str())){
    Serial.println("Distance from SENSOR 2 sent!");
  }
  
  //The client.publish will return a boolean value depedning on whether it succeded or not
  //If the message failed to send, we will try again, as the connection may have broken
  else{
    Serial.println("Failed to send the distance to the broker. Reconnecting to the MQTT broker and tryinng again!");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // to make sure that the client.publish() does not clash with the client.connect() call
    client.publish(ultrasonic_sensor_2_topic, String(distance_sensor_2).c_str());
  }


  client.loop();
  



//  client.disconnect(); // disocnnect from the MQTT broker
//  delay(5000);
  
}
