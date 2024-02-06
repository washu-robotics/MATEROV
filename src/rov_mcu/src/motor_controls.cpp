#include <Arduino.h>
#include <Servo.h>

Servo motor1;

void setup() {
  motor1.attach(3);
  motor1.writeMicroseconds(1500); // Initialize motor1
  Serial.begin(9600); // Start serial communication
}

void loop() {
  if (Serial.available()) {
    // Read a string in format "x\n" over serial
    String input = Serial.readStringUntil('\n');
    int x = input.toInt();
    
    // Map the incoming speed to motor compatible speeds and write to motor
    //int speed1 = map(x, -1, 1, 1100, 1900);
    if (x < 1100  || x > 1900)
    {
      Serial.println("bad input");
    }
    else
    {
    motor1.writeMicroseconds(x);
    Serial.print("wrote speed1: ");
    Serial.print(x);
    Serial.println(" to motor1");
    }

  }
}
  //else {
  //  Serial.println("no incoming data or no serial connection established");
  //}
  //delay(3000);
//}