#include <SoftwareSerial.h>
//Above should select ESPsoftwareserial when selecting ESP8266 boards
#include <ESP8266WiFi.h>  //I need wifi, despite I have removed these pieces from the code sample below
#include <ESP8266WiFiMulti.h>

#define MAXLINELENGTH 255
char txt[MAXLINELENGTH];
#define SERIAL_RX 14      //GPIO14 = D5 pin for SoftwareSerial RX
#define SERIAL_TX 12      //GPIO12 = D6 pin for SoftwareSerial TX
SoftwareSerial mySerial;  //(SERIAL_RX, SERIAL_TX, false, MAXLINELENGTH); // (RX, TX, inverted, buffer)

char compareChar = 't';

void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  //delay(2000);
  mySerial.begin(9600, SWSERIAL_8N1, SERIAL_RX, SERIAL_TX, false, MAXLINELENGTH);
  Serial.println("\nBegin 9600");
};

void loop() {
  char output = mySerial.read();
  Serial.println(output);
  delay(2000);

  if (output == compareChar){
    Serial.println("Recieved the character t and starting the servo motor......");
  }
}

