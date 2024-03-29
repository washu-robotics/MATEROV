#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN pins
const byte addresses[][6] = {"00001", "00002"};
// motor control
// int motorPin1 = 2;  // AIN1 on the motor driver
// int motorPin2 = 3; // AIN2 on the motor driver
// int standbyPin = 4; // STBY on the motor driver
// int pwm = 5;
int motorPin1 = 2;  // AIN1 on the motor driver
int motorPin2 = 3; // AIN2 on the motor driver
int standbyPin = 4; // STBY on the motor driver
int pwm = 5;

int hall_sensorPin1 = A5; //green
int hall_sensorPin2 = A4; //blue
int counter = 0;
int sensorValue = 0;


//pressure sensor setup
#include <Adafruit_LPS35HW.h>

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

// For SPI mode, we need a CS pin, Yellow
#define LPS_CS  10
//blue
#define LPS_SCK  34
//green
#define LPS_MISO 12
//pink
#define LPS_MOSI 11



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

  if (!lps35hw.begin_SPI(LPS_CS, LPS_SCK, LPS_MISO, LPS_MOSI)) {
  Serial.println("Couldn't find LPS35HW chip");
  delay(500);
  }
  Serial.println("Found LPS35HW chip");


}

int *dynamicList = NULL; 
int listSize = 0;  

void addElement(int element) {
  int *temp = (int *)realloc(dynamicList, (listSize + 1) * sizeof(int));
  if (temp != NULL) {
    dynamicList = temp;
    dynamicList[listSize] = element;
    listSize++;
  } else {
    Serial.println("Failed to allocate memory.");
  }
}

void printList() {
  Serial.println("List Contents:");
  for (int i = 0; i < listSize; i++) {
    Serial.print(dynamicList[i]);
    Serial.print(", ");
  }
}

void freeList() {
  free(dynamicList);
  dynamicList = NULL;
  listSize = 0;
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

    if (String(text)=="up"){

      while(hall_sensorValue1==1){
        hall_sensorValue1 = digitalRead(hall_sensorPin1);
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(pwm, 255);
      }

    }else if (String(text)=="down"){

      while(hall_sensorValue2==1){
        hall_sensorValue2 = digitalRead(hall_sensorPin2);
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(pwm, 255);
      }

    }
    else if (String(text)=="test"){
      float pres = 0;

      while(hall_sensorValue2==1){
        hall_sensorValue2 = digitalRead(hall_sensorPin2);
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(pwm, 255);
        pres = lps35hw.readPressure();
        addElement(pres);
        Serial.print("Pressure: ");
        Serial.print(lps35hw.readPressure());
        Serial.println(" hPa");
        delay(100);
      }

      for (int i = 0; i<10; i++){
        pres = lps35hw.readPressure();
        addElement(pres);
        Serial.print("Pressure: ");
        Serial.print(lps35hw.readPressure());
        Serial.println(" hPa");
        delay(100);
      }

      hall_sensorValue1 = digitalRead(hall_sensorPin1);
      while(hall_sensorValue1==1){
        hall_sensorValue1 = digitalRead(hall_sensorPin1);
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(pwm, 255);
        pres = lps35hw.readPressure();
        addElement(pres);
        Serial.print("Pressure: ");
        Serial.print(lps35hw.readPressure());
        Serial.println(" hPa");
        delay(100);
      }
      printList();
      freeList();
      radio.startListening();

    }
    
    else{

      Serial.println("Invalid Radio Input");

    }

  }

//use serial input
    if (Serial.available()){

    String input = Serial.readStringUntil("\n");
    input.trim();
    Serial.println(input);

    if (input=="up"){

      while(hall_sensorValue1==1){
        hall_sensorValue1 = digitalRead(hall_sensorPin1);
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(pwm, 255);
      }

    }else if (input=="down"){

      while(hall_sensorValue2==1){
        hall_sensorValue2 = digitalRead(hall_sensorPin2);
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(pwm, 255);
      }

    }else if (input=="test"){

      while(hall_sensorValue2==1){
        hall_sensorValue2 = digitalRead(hall_sensorPin2);
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(pwm, 255);
      }

      hall_sensorValue1 = digitalRead(hall_sensorPin1);
      while(hall_sensorValue1==1){
        hall_sensorValue1 = digitalRead(hall_sensorPin1);
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(pwm, 255);
      }
    }
    else{

      Serial.println("Invalid Serial Input");

    }
  }

digitalWrite(motorPin1, LOW);
digitalWrite(motorPin2, LOW);
radio.startListening();



}



