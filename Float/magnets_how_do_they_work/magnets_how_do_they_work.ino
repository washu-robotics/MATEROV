
int sensorPin = A5;
int counter = 0;
int sensorValue = 0;

void setup() 
{
  // setup serial - diagnostics - port
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
}

void loop() 
{
  // int sensorValue = digitalRead(sensorPin);
  sensorValue = analogRead(sensorPin);

  Serial.println(sensorValue);
  delay(250);
}