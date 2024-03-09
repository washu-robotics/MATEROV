#include <Servo.h>
#include <BasicLinearAlgebra.h> 
using namespace BLA;

#include <Math.h>

Servo motor_0;
Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;
Servo motor_5;
int SEND_INTERVAL = 200;

BLA::Matrix<6> Wrench;
BLA::Matrix<6> N;
BLA::Matrix<6> PWM;
BLA::Matrix<6, 6> Mixer = {
    25, -25,   0,    0,  -35355, -29,
    25,  25,   0,    0,  -35355,  29,
    0,    0, -36,   71,       0,   0,
    25, -25,      0, 0,   35355,  29,
    25,  25,      0, 0,   35355, -29,
    0,  0,   -36,  -71,       0,   0 };
char separator[]= " ";


int nSquared_to_PWM_CCW(float nSquared){
  int PWM_Duty_Cycle;
  if(abs(nSquared)<33){
    PWM_Duty_Cycle=1500;
  }
  else if(nSquared>0){
    PWM_Duty_Cycle=int(1564+0.1385*abs(nSquared));
  }
  else{
    PWM_Duty_Cycle=int(1439-0.1409*abs(nSquared));
  }
  return PWM_Duty_Cycle;
}

int nSquared_to_PWM_CW(float nSquared){
  int PWM_Duty_Cycle;
  if(abs(nSquared)<33){
    PWM_Duty_Cycle=1500;
  }
  else if(nSquared>0){
    PWM_Duty_Cycle=int(1436-0.1385*abs(nSquared));
  }
  else{
    PWM_Duty_Cycle=int(1561+0.1409*abs(nSquared));
  }
  return PWM_Duty_Cycle;
}

int* parseString(String str) {
    static int values[6]; // Static array to hold the parsed integers
    int index = 0; // Index to keep track of the current integer being parsed
    int lastIndex = 0; // Index to keep track of the last position where a number ended
    for (int i = 0; i < str.length(); i++) {
        if (str[i] == ' ' || i == str.length() - 1) { // If space or end of string is encountered
            String numberString = str.substring(lastIndex, i); // Extract the substring representing the number
            values[index++] = numberString.toInt(); // Convert the substring to an integer and store it in the array
            lastIndex = i + 1; // Update the last index to the position after the space
        }
    }
    return values; // Return the array of parsed integers
}

void setup() {
    motor_0.attach(3);
    motor_0.writeMicroseconds(1500);  // Initialize motor0
    motor_1.attach(4);
    motor_1.writeMicroseconds(1500);  // Initialize motor1
    motor_2.attach(5);
    motor_2.writeMicroseconds(1500);  // Initialize motor2
    motor_3.attach(6);
    motor_3.writeMicroseconds(1500);  // Initialize motor3
    motor_4.attach(7);
    motor_4.writeMicroseconds(1500);  // Initialize motor4
    motor_5.attach(8);
    motor_5.writeMicroseconds(1500);  // Initialize motor5
    Serial.begin(9600);  // Start serial communication
}
void loop() {
    if (Serial.available()>0) {
        double Fx, Fy, Fz, Mx, My, Mz;
         // Check if there's data available to read
          String inputString = Serial.readStringUntil('\n'); // Read the incoming string until newline character
          int* Wrench_vector=parseString(inputString); // Parse the string into integers

          Wrench = {Wrench_vector[0],Wrench_vector[1],Wrench_vector[2], Wrench_vector[3], Wrench_vector[4], Wrench_vector[5]};
          Serial << "Wrench: " << Wrench << '\n';
      
        /**initializing the wrench with the double values create above and using them
        to multiply by the ThreeDMixer to create a matrix that will convert the wrench
        values into PWM signals**/
        // Wrench = {Fx,Fy,Mz, Fx2, Fy2, Mz2};
        N = Mixer*Wrench;
        Serial << "NSquared" << N << '\n';
        /**need to map matrix values into PWM signals**/
        PWM={nSquared_to_PWM_CW(N(0)),nSquared_to_PWM_CCW(N(1)),nSquared_to_PWM_CW(N(2)),
        nSquared_to_PWM_CCW(N(3)),nSquared_to_PWM_CW(N(4)),nSquared_to_PWM_CCW(N(5))};
        //reading in the PWM signals
        Serial << "PWM Values: " << PWM << '\n';
        //checking the boundaries of the PWM signls before reading them into speeds
        if(PWM(0)<1100||PWM(1)<1100||PWM(2)<1100||PWM(3)<1100||PWM(4)<1100||PWM(5)<1100||PWM(0)>1900||PWM(1)>1900||PWM(2)>1900||PWM(3)>1900||PWM(4)>1900||PWM(5)>1900){
            Serial.println("Requesting speeds that are too fast.");
            Serial.println("All motors set to 1500.");
            motor_0.writeMicroseconds(1500);
            motor_1.writeMicroseconds(1500);
            motor_2.writeMicroseconds(1500);
            motor_3.writeMicroseconds(1500);
            motor_4.writeMicroseconds(1500);
            motor_5.writeMicroseconds(1500);
        }
            //read in PWMs into serial inputs
        else{
            motor_0.writeMicroseconds(PWM(0));
            Serial.print("wrote speed: ");
            Serial.print(PWM(0));
            Serial.println(" to motor 0");
            motor_1.writeMicroseconds(PWM(1));
            Serial.print("wrote speed: ");
            Serial.print(PWM(1));
            Serial.println(" to motor 1");
            motor_2.writeMicroseconds(PWM(2));
            Serial.print("wrote speed: ");
            Serial.print(PWM(2));
            Serial.println(" to motor 2");
            motor_3.writeMicroseconds(PWM(3));
            Serial.print("wrote speed: ");
            Serial.print(PWM(3));
            Serial.println(" to motor 3");
            motor_4.writeMicroseconds(PWM(4));
            Serial.print("wrote speed: ");
            Serial.print(PWM(4));
            Serial.println(" to motor 4");
            motor_5.writeMicroseconds(PWM(5));
            Serial.print("wrote speed: ");
            Serial.print(PWM(5));
            Serial.println(" to motor 5");
        }
    }
    delay(SEND_INTERVAL);
}

