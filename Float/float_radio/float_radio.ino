#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN pins
const byte addresses[][6] = {"00001", "00002"};

int hall_sensorPin1 = A5;
int hall_sensorPin2 = A4;
int counter = 0;
int prevSensorValue1 = 1;
int prevSensorValue2 = 1;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(addresses[1]); // Use the first address to write
  radio.openReadingPipe(1, addresses[0]); // Use the second address to read
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

    // Send confirmation receipt
    if (String(text) == "start"){
      radio.stopListening(); // Stop listening to start writing
      String input = "\nCommand Received\n";

      const char response[32] = "";
      input.toCharArray((char*)response, 32);
      radio.write(&response, sizeof(response));
      radio.startListening(); // Listen for incoming messages again
      delay(100); // Short delay to ensure message is sent
    }

    else {
      radio.stopListening(); // Stop listening to start writing
      String input = "\nInvalid Command\n";

      const char response[32] = "";
      input.toCharArray((char*)response, 32);
      radio.write(&response, sizeof(response));
      radio.startListening(); // Listen for incoming messages again
      delay(100); // Short delay to ensure message is sent

    }
  }

  radio.stopListening(); // Stop listening to start writing

  String packet = String(hall_sensorValue1) + String(hall_sensorValue2);
  const char text[32] = "";
  packet.toCharArray((char*)text, 32);
  radio.write(&text, sizeof(text));
  radio.startListening(); // Listen for incoming messages again
  delay(100); // Short delay to ensure message is sent
}


    // // Check if there is a signal change from the hall sesnsors
  // if (hall_sensorValue1 != prevSensorValue1 || hall_sensorValue2 != prevSensorValue2) {
  //   radio.stopListening(); // Stop listening to start writing

  //   String input = String(hall_sensorValue1) + String(hall_sensorValue2);
  //   const char text[32] = "";
  //   input.toCharArray((char*)text, 32);
  //   radio.write(&text, sizeof(text));
  //   radio.startListening(); // Listen for incoming messages again
  //   delay(100); // Short delay to ensure message is sent
  // }
  // else{
  //   radio.stopListening(); // Stop listening to start writing

  //   String input = String(hall_sensorValue1) + String(hall_sensorValue2);
  //   const char text[32] = "";
  //   input.toCharArray((char*)text, 32);
  //   radio.write(&text, sizeof(text));
  //   radio.startListening(); // Listen for incoming messages again
  //   delay(100); // Short delay to ensure message is sent
  // }

  // prevSensorValue1 = hall_sensorValue1;
  // prevSensorValue2 = hall_sensorValue2;