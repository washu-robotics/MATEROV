#include <Servo.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
Servo motor_0;
Servo motor_1;
Servo motor_2;
Servo motor_3;

int SEND_INTERVAL = 200;

BLA::Matrix<3> Wrench;
BLA:: Matrix<4> N;
BLA:: Matrix<4> PWM;
BLA::Matrix<4, 3> TwoDMixer = { 46.6198, 46.6198, 46.6198, 46.6198, -46.6198, -46.6198, 46.6198, 46.6198, -49.8387, 49.8387, -49.8387, 49.8387 };
char separator[]= ” “;

void setup() {
  motor_0.attach(2);
  motor_0.writeMicroseconds(1500);  // Initialize motor1
  motor_1.attach(3);
  motor_1.writeMicroseconds(1500);  // Initialize motor1
  motor_2.attach(4);
  motor_2.writeMicroseconds(1500);  // Initialize motor1
  motor_3.attach(5);
  motor_3.writeMicroseconds(1500);  // Initialize motor1
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  if (Serial.available()) {
    Serial.println("begin");
    String inputString = Serial.readStringUntil(‘\n’);
    // Split the string and convert to doubles
    double Fx, Fy, Mz;
    int firstSpaceIndex = inputString.indexOf(' ‘);
    int secondSpaceIndex = inputString.lastIndexOf(’ ‘);
    Fx = inputString.substring(0, firstSpaceIndex).toDouble();
    Fy = inputString.substring(firstSpaceIndex + 1, secondSpaceIndex).toDouble();
    Mz = inputString.substring(secondSpaceIndex + 1).toDouble();
    Wrench = {Fx,Fy,Mz};
    N = TwoDMixer * Wrench;
    PWM={map(N(0),-50,50,1100,1900),map(N(1),-50,50,1100,1900),map(N(2),-50,50,1100,1900),map(N(3),-50,50,1100,1900)};
    Serial << “PWM Values: ” << PWM << ‘\n’;
    if(PWM(0)<1100||PWM(1)<1100||PWM(2)<1100||PWM(3)<1100||PWM(0)>1900||PWM(1)>1900||PWM(2)>1900||PWM(3)>1900){
      Serial.println(“Requesting speeds that are too fast.“);
      Serial.println(“All motors set to 1500.“);
      motor_0.writeMicroseconds(1500);
      motor_1.writeMicroseconds(1500);
      motor_2.writeMicroseconds(1500);
      motor_3.writeMicroseconds(1500);
    }
    else{
    motor_0.writeMicroseconds(PWM(0));
    Serial.print(“wrote speed: “);
    Serial.print(PWM(0));
    Serial.println(” to motor 1");
    motor_1.writeMicroseconds(PWM(1));
    Serial.print(“wrote speed: “);
    Serial.print(PWM(1));
    Serial.println(” to motor 1");
    motor_2.writeMicroseconds(PWM(2));
    Serial.print(“wrote speed: “);
    Serial.print(PWM(2));
    Serial.println(” to motor 1");
    motor_3.writeMicroseconds(PWM(3));
    Serial.print(“wrote speed: “);
    Serial.print(PWM(3));
    Serial.println(” to motor 1");
    }
  }
  else {
    //Serial.println(“no incoming data or no serial connection established”);
  }
  delay(SEND_INTERVAL);
}