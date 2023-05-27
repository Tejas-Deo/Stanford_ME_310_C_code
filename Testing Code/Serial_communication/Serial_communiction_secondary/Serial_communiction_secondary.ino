#include <SoftwareSerial.h>

// SoftwareSerial configuration
#define TX_PIN 1   // Transmit pin of NodeMCU 1 connected to RX pin of NodeMCU 2
#define RX_PIN 3   // Receive pin of NodeMCU 1 connected to TX pin of NodeMCU 2
SoftwareSerial espSerial(RX_PIN, TX_PIN);

void setup(){
  Serial.begin(9600);
  espSerial.begin(9600);
}

void loop(){
  char output = espSerial.read();
  delay(2000);
  Serial.print("`Received: ");
  Serial.println(output);
}   