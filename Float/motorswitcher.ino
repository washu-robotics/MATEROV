// motor control
int motorPin1 = 6;  // AIN1 on the motor driver
int motorPin2 = 5; // AIN2 on the motor driver
int standbyPin = 7; // STBY on the motor driver
int pwm = 12;

int hall_sensorPin1 = A5;
int hall_sensorPin2 = A6;
int counter = 0;
int sensorValue = 0;

void setup() 
{
  // motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(standbyPin, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(standbyPin, HIGH); // Activate the motor driver

  // Forward rotation
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  // magnet pins
  // setup serial - diagnostics - port
  Serial.begin(9600);
  pinMode(hall_sensorPin1, INPUT);
  pinMode(hall_sensorPin2, INPUT);

  // initial movement
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin1, LOW);
}

void loop() 
{

  int hall_sensorValue1 = digitalRead(hall_sensorPin1);
  int hall_sensorValue2 = digitalRead(hall_sensorPin2);

  Serial.print(hall_sensorValue1);
  Serial.print(" ");
  Serial.println(hall_sensorValue2);
  // delay(250);


  analogWrite(pwm, 255);

  // Hall sensor 1
  if (hall_sensorValue1 == 0) {
    // Stop the motor
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    //delay(1000);

    // reverse
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(pwm, 255);
    delay(1000);
  }

  // Hall sensor 2
  if (hall_sensorValue2 == 0) {

    // Stop the motor
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    //delay(1000);

    // reverse
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(pwm, 255);
    delay(1000);
  }
  

  // // Forward rotation
  // digitalWrite(motorPin1, LOW);
  // digitalWrite(motorPin2, HIGH);

  // analogWrite(pwm, 255);
  // // delay(5000); // Run the motor for 5 seconds

  // int hall_sensorValue1 = digitalRead(hall_sensorPin1);
  // int hall_sensorValue2 = digitalRead(hall_sensorPin2);

  // if (hall_sensorValue1 == 0) {
  //   // Stop the motor
  //   digitalWrite(motorPin1, LOW);
  //   digitalWrite(motorPin2, LOW);

  //   delay(3000); // Wait
  //   digitalWrite(motorPin1, HIGH);
  //   digitalWrite(motorPin2, LOW); 

  //   return 0;
  // }
  return 0;
}


