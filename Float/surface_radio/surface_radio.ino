#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN pins
const byte addresses[][6] = {"00001", "00002"};

int hall_sensorPin1 = A5;
int hall_sensorPin2 = A4;
int counter = 0;
int sensorValue = 0;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(addresses[0]); // Use the first address to write
  radio.openReadingPipe(1, addresses[1]); // Use the second address to read
  radio.startListening();

  pinMode(hall_sensorPin1, INPUT);
  pinMode(hall_sensorPin2, INPUT);

}
void loop() {

  int hall_sensorValue1 = digitalRead(hall_sensorPin1);
  int hall_sensorValue2 = digitalRead(hall_sensorPin2);

  // Check if there is a reading available
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }
  // Check if there is a Serial input (from keyboard)
  if (Serial.available()) {
    radio.stopListening(); // Stop listening to start writing

    String input = Serial.readString();
    input.trim();
    const char text[32] = "";
    input.toCharArray((char*)text, 32);
    radio.write(&text, sizeof(text));
    radio.startListening(); // Listen for incoming messages again
    delay(100); // Short delay to ensure message is sent
  }
  // else{
  //   radio.stopListening(); // Stop listening to start writing

  //   String input = "goodbye";
  //   const char text[32] = "";
  //   input.toCharArray((char*)text, 32);
  //   radio.write(&text, sizeof(text));
  //   radio.startListening(); // Listen for incoming messages again
  //   delay(100); // Short delay to ensure message is sent
  // }
}