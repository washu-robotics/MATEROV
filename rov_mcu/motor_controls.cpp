#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>

Servo motor1;

void setup() {
  motor1.attach(2);
  // motor1.writeMicroseconds(1000);
  // delay(5000);
  motor1.writeMicroseconds(1500); // Initialize motor1
  // delay(5000);
  Serial.begin(57600); // Start serial communication
  // setup onboard led
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    // Read a string in format "x\n" over serial
    String input = Serial.readStringUntil('\n');
    
    // pharse json string
    // {"contorls": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}

    // Parse the JSON object
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, input);
    JsonObject controls = doc["controls"];
    JsonObject linear = controls["linear"];
    float x = linear["x"];
    // add more code to parse y and z

    // map -1 ~ 1 to 1100 ~ 1900 without map()
    int x_mapped = (int)(x * 500 + 1500);
    

    // display x raw value on onboard led
    digitalWrite(LED_BUILTIN, x > 0 ? HIGH : LOW);
    
    // Write the mapped value to motor1
    motor1.writeMicroseconds(2000);

  }
}
  //else {
  //  Serial.println("no incoming data or no serial connection established");
  //}
  //delay(3000);
//}