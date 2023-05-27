#include "config.h"


//WiFi
const char* ssid = "IIIPA";  // my network's SSID
const char* wifi_password = "ME310Printer";   // the password of my network


//MQTT
const char* mqtt_server = "192.168.30.3";
const char* mqtt_username = "avatar"; 
const char* mqtt_password = "avatar";
const char* clientID = "microcontroller_client";

//defined the MQTT topic to which I need to subscribe and then later publish as well
const char* mqtt_voice_command_topic = "MQTTCommand/Voice";
const char* mqtt_UI_command_topic = "MQTTCommand/UI";




// GLOBAL VARIABLES FOR DC AND SERVO MOTORS
#define DC_LEFT 0
#define DC_RIGHT 1
#define DC_STRAIGHT 2
#define DC_REVERSE 3
#define DC_STOP 4
#define SERVO_LEFT 5
#define SERVO_RIGHT 6


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

  // Connect to MQTT Broker. Client.connect returns a Boolean value to let us know if the connection was successful
  if (client.connect(clientID, mqtt_username, mqtt_password)){
    Serial.print(clientID);
    Serial.println(" Connected to MQTT Broker");
  }
  else{
    Serial.println("Connection to MQTT Broker failed...");
  }
}

