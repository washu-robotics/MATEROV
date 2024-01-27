#include <Arduino.h>

// Define motor pins
const int leftMotorForwardPin = 9;   // Define left motor forward pin
const int leftMotorBackwardPin = 8;  // Define left motor backward pin
const int rightMotorForwardPin = 10; // Define right motor forward pin
const int rightMotorBackwardPin = 7; // Define right motor backward pin

void setup() {
  Serial.begin(9600);     // Initialize serial communication
  
  // Set motor pins as outputs
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);
}

void loop() {
  if (Serial.available() >= 8) {
    float linearVelocity, angularVelocity;
    byte buffer[8];

    // Read 8 bytes from serial input buffer
    Serial.readBytes(buffer, 8);

    // Extract linear and angular velocities from the buffer
    memcpy(&linearVelocity, buffer, sizeof(float));
    memcpy(&angularVelocity, buffer + sizeof(float), sizeof(float));

    // Calculate motor speeds based on velocities
    float leftSpeed = linearVelocity - angularVelocity;
    float rightSpeed = linearVelocity + angularVelocity;

    // Drive the motors based on calculated speeds
    driveMotor(leftMotorForwardPin, leftMotorBackwardPin, leftSpeed);
    driveMotor(rightMotorForwardPin, rightMotorBackwardPin, rightSpeed);
  }
}

void driveMotor(int forwardPin, int backwardPin, float speed) {
  if (speed > 0) {
    analogWrite(forwardPin, constrain(speed * 255, 0, 255));
    analogWrite(backwardPin, 0);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, constrain(-speed * 255, 0, 255));
  }
}