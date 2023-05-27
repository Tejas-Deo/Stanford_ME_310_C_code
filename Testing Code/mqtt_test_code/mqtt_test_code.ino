//Code for the ESP8266

#include <ESP8266WiFi.h>   // Enables the ESP8266 to connect to the local network (via WiFi)
#include <PubSubClient.h> //Connect and publish to the MQTT broker
#include <AccelStepper.h>



//WiFi
const char* ssid = "IIIPA";  // my network's SSID
const char* wifi_password = "ME310Printer";   // the password of my network


//MQTT
const char* mqtt_server = "192.168.30.3";
const char* mqtt_username = "avatar"; 
const char* mqtt_password = "avatar";
const char* clientID = "ultrasonic_sensor_ids";

// defined the MQTT topic to which I need to subscribe and then later publish as well
const char* mqtt_voice_command_topic = "MQTTCommand/Voice";
const char* mqtt_UI_command_topic = "MQTTCommand/UI";


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




// TO setup all the sensors
void setup() {
  Serial.begin(115200); // Starts the serial communication
  Serial.println("In the void setup block");

  client.setCallback(callback);
  
  connect_MQTT();   // connect to the MQTT broker
  
  if (client.connected()){
    client.subscribe(mqtt_voice_command_topic);
    client.subscribe(mqtt_UI_command_topic);
    Serial.println("Have subscribed to the topics!!");
    delay(1000);
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);

  // Convert payload to a string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Payload: ");
  Serial.println(message);

  
  // if (String(topic) == mqtt_voice_command_topic){
  //   Serial.println("Starting the autonomous system mode!");
  //   delay(1000);
  //   client.publish(mqtt_voice_command_topic, "Have activated the autonomous system mode!");
  //   Serial.println("Have send the message to the specified topic...");
  // }
  
  if (strcmp(message.c_str(), "StartAuto") == 0){
    Serial.println("Starting the autonomous system mode!");
    client.publish(mqtt_voice_command_topic, "Have completed the autonomous system mode!");
    Serial.println("Have send the message to the specified topic...");
  }
  }



void loop() {

  // delay(500);

  client.loop();
   
}


