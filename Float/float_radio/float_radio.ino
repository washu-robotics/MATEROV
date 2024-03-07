#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN pins
const byte addresses[][6] = {"00001", "00002"};
// motor control
int motorPin1 = 2;  // AIN1 on the motor driver
int motorPin2 = 3; // AIN2 on the motor driver
int standbyPin = 4; // STBY on the motor driver
int pwm = 5;

int hall_sensorPin1 = A5;
int hall_sensorPin2 = A4;
int counter = 0;
int sensorValue = 0;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(addresses[1]); // Use the first address to write
  radio.openReadingPipe(1, addresses[0]); // Use the second address to read
  radio.startListening();

  // motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(standbyPin, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(standbyPin, HIGH); 

  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  // magnet pins
  pinMode(hall_sensorPin1, INPUT);
  pinMode(hall_sensorPin2, INPUT);

  // initial movement
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin1, LOW);
}


//use radio input
void loop() {

  int hall_sensorValue1 = digitalRead(hall_sensorPin1);
  int hall_sensorValue2 = digitalRead(hall_sensorPin2);

  if (radio.available()){

    radio.stopListening();
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);

    if (String(text)=="up\n"){

      while(hall_sensorValue1==1){
        hall_sensorValue1 = digitalRead(hall_sensorPin1);
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(pwm, 255);
      }

    }else if (String(text)=="down\n"){

      while(hall_sensorValue2==1){
        hall_sensorValue2 = digitalRead(hall_sensorPin2);
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(pwm, 255);
      }

    }else{

      Serial.println("Invalid Input");

    }
  }

//use serial input
    if (Serial.available()){

    String input = Serial.readStringUntil("\n");
    Serial.println(input);

    if (input=="up\n"){

      while(hall_sensorValue1==1){
        hall_sensorValue1 = digitalRead(hall_sensorPin1);
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(pwm, 255);
      }

    }else if (input=="down\n"){

      while(hall_sensorValue2==1){
        hall_sensorValue2 = digitalRead(hall_sensorPin2);
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(pwm, 255);
      }

    }else{

      Serial.println("Invalid Input");

    }
  }


  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  radio.startListening();

}



