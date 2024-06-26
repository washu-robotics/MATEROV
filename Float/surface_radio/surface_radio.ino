#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN pins
const byte addresses[][6] = {"00001", "00002"};
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(addresses[0]); // Use the first address to write
  radio.openReadingPipe(1, addresses[1]); // Use the second address to read
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(80);
  radio.startListening();
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void loop() {
  // Check if there is a reading available
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    float depth = (String(text).toInt() - 995)*0.10197;
    Serial.println(depth);
  }
  // Check if there is a Serial input (from keyboard)
  if (Serial.available()) {
    radio.stopListening(); // Stop listening to start writing
    String input = Serial.readStringUntil("\n");
    input.trim();
    Serial.println(input);
    const char text[32] = "";
    input.toCharArray((char*)text, 32);
    radio.write(&text, sizeof(text));
    radio.startListening(); // Listen for incoming messages again
    delay(10); // Short delay to ensure message is sent
  }
}