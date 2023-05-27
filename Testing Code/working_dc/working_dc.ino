  #include <Encoder.h>

int ENA = D7;
int IN1 = D1;
int IN2 = D2;
int encoderPinA = D5;
int encoderPinB = D6;

long encoder_value = 0;
long encoder_counts_per_rev = 2443;
int wheel_rev = 2;

Encoder encoder(encoderPinA, encoderPinB);
long encoderValue = 0;
unsigned long abs_encoderValue;
unsigned int rpm = 0;
unsigned long timeold;

bool clockwise = true;



void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), isr_fn, CHANGE);   //to enable interrupt on encoder pinA
}


void stop_motor(){
  Serial.println("Inside stop motor function....");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(1000);
}


void ICACHE_RAM_ATTR DCMotorISR(){
  encoderValue = encoder.read();
  abs_encoderValue = abs(encoderValue);
}


void move_motor(String command){
  if (command == "forward"){        // assuming that forward motion is clockwise for the DC motor
    
    analogWrite(ENA, 100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    long target_encoder_counts = wheel_rev * encoder_counts_per_rev;    // 4886 = 2 * 2443
    encoder.write(0);    // to reset the encoders

    int counter = 0; 

    // using interrupts to monitor the encoder steps and exit the while loop once the encoder value exceeds the target encoder value
    while(abs_encoderValue < target_encoder_counts){     
      analogWrite(ENA, 100);
      counter++;

      if (counter == 75){        // to scane the ultrasonic sensors after every 75 encoder steps i.e. after every 1 cm that the walker has moved
        counter = 0;             // reset the counter value as I have to scan the sensors again after 75 encoder steps
        if (!check_ultrasonic_sensors()){
          stop_motors();         // stop the motors if the ultrasonic sensor def return False
        }
        else{  // else continue moving forward
        }
      }
      
    }

    stop_motors();

  }


  if (command == "reverse"){
    analogWrite(ENA, 100);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    int target_encoder_counts = wheel_rev * encoder_counts_per_rev;
    encoder.wirte(0);    // to reset the encoders

    while(encoderValue<target_encoder_counts){
      analogWrite(ENA, 100);
      encoderValue = encoder.read();
      delay(1);
    }

    stop_motors();

  }
}



void loop() {
  move_motors("forward");
  delay(10000);
  move_motors("reverse");
}
