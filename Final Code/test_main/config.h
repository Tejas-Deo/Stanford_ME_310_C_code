#ifndef CONFIG_H
#define CONFIG_H


//WiFi
extern const char* ssid
extern const char* wifi_password 


//MQTT
extern const char* mqtt_server 
extern const char* mqtt_username
extern const char* mqtt_password
extern const char* clientID

//defined the MQTT topic to which I need to subscribe and then later publish as well
extern const char* mqtt_voice_command_topic
extern const char* mqtt_UI_command_topic


// GLOBAL VARIABLES FOR DC AND SERVO MOTORS
extern #define DC_LEFT
extern #define DC_RIGHT 
extern #define DC_STRAIGHT 
extern #define DC_REVERSE 
extern #define DC_STOP 
extern #define SERVO_LEFT 
extern #define SERVO_RIGHT 

void connect_MQTT()

#endif //CONFIG_H