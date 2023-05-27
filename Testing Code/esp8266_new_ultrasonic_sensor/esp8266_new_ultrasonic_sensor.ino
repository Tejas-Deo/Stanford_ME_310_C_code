//Code for the ESP8266

#include <ESP8266WiFi.h>   // Enables the ESP8266 to connect to the local network (via WiFi)
#include <PubSubClient.h> //Connect and publish to the MQTT broker



//WiFi
const char* ssid = "IIIPX";  // my network's SSID
const char* wifi_password = "ME310Printer";   // the password of my network


//MQTT
const char* mqtt_server = "192.168.10.9";
const char* ultrasonic_sensor_1_topic = "ultrasonic_sensor/sensor_1";
const char* ultrasonic_sensor_2_topic = "ultrasonic_sensor/sensor_2";
const char* mqtt_username = "avatar"; 
const char* mqtt_password = "avatar";
const char* clientID = "front_sensor_1";



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
  
  pinMode(triggerPin_sensor_1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_sensor_1, INPUT); // Sets the echoPin as an Input

  pinMode(triggerPin_sensor_2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_sensor_2, INPUT); // Sets the echoPin as an Input
  
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

  
  
//  // Prints the distance on the Serial Monitor
//  Serial.print("Distance (cm) from Sensor 1 and 2 are: ");
//  Serial.println(distance_sensor_1, distance_sensor_2);

  delay(500);


//  // MQTT can only transmit strings
//  String distanceCmS_1="Sensor_1: "+String((float)distanceCm_1)+"cm";
//  String distanceCmS_2="Sensor_2: "+String((float)distanceCm_2)+"cm";


  //Publish info from SENSOR 1 to the MQTT Broker
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



  //Publish info from SENSOR 2 to the MQTT Broker
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



//  client.disconnect(); // disocnnect from the MQTT broker
  delay(5000);
  
}
