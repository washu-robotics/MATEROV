// motor control
int motorPin1 = 9;  // AIN1 on the motor driver
int motorPin2 = 10; // AIN2 on the motor driver
int standbyPin = 8; // STBY on the motor driver
int pwm = 3;



void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(standbyPin, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(standbyPin, HIGH); // Activate the motor driver

  // Forward rotation
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  
  analogWrite(pwm, 255);
  delay(1000); // Run the motor for 5 seconds

}

void loop() {

  
  // Forward rotation
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  
  analogWrite(pwm, 255);
  delay(5000); // Run the motor for 5 seconds


  // Stop the motor
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  delay(1000); // Wait 

  // Reverse rotation
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  delay(5000); // Run the motor for 5 seconds

  // Stop the motor
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  delay(500); // Wait for 1/2 second

  //exit(0); //STOPPPPP!!!!!!
  
}