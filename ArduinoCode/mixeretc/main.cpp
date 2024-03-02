#include <Servo.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
Servo motor_0;
Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;
Servo motor_5;

int SEND_INTERVAL = 200;

BLA::Matrix<6,1> Wrench;
BLA:: Matrix<6, 1> N;
BLA:: Matrix<6,1> PWM;
BLA::Matrix<6, 6> ThreeDMixer = { 84.0533, -84.0533,  78.3353,  0,  -96.4157, -89.8567,
                                  84.0533, 84.0533,-78.3353, 0,  -96.4157,   89.8567, 0, 0,  152.2571, -141.3602, 0 -131.7437,
                                  -84.0533,-84.0533,  -78.3353,         0,  -96.4157,   89.8567, -84.0533, 84.0533, 78.3353, 0,
                                  -96.4157, -89.8567, 0, 0, 56.5643, -141.3602, 0, -131.7437};

char separator[]= " ";

void setup() {
    motor_0.attach(2);
    motor_0.writeMicroseconds(1500);  // Initialize motor0
    motor_1.attach(3);
    motor_1.writeMicroseconds(1500);  // Initialize motor1
    motor_2.attach(4);
    motor_2.writeMicroseconds(1500);  // Initialize motor2
    motor_3.attach(5);
    motor_3.writeMicroseconds(1500);  // Initialize motor3
    motor_4.attach(6);
    motor_4.writeMicroseconds(1500);  // Initialize motor4
    motor_5.attach(7);
    motor_5.writeMicroseconds(1500);  // Initialize motor5
    Serial.begin(9600);  // Start serial communication
}

void loop() {
    if (Serial.available() >0) {
        //indicating the beginning of the communication
        Serial.println("begin");
        //reading the serial string until the end of the line
        String inputString = Serial.readStringUntil('\n');
        // Split the string and convert to doubles
        double Fx, Fy, Mz, Fx2, Fy2, Mz2;
        if(Serial)
        int firstSpaceIndex = inputString.indexOf(' ');
        int secondSpaceIndex = inputString.lasindexOf(' ');
        Fx = inputString.substring(0, firstSpaceIndex).toDouble();
        Fy = inputString.substring(firstSpaceIndex, secondSpaceIndex).toDouble();
        Mz = inputString.substring(secondSpaceIndex).toDouble();
//        Fx2 = inputString.substring(thirdSpaceIndex).toDouble();
//        Fy2 = inputString.substring(firstSpaceIndex + 2, secondSpaceIndex).toDouble();
//        Mz2 = inputString.substring(secondSpaceIndex + 2).toDouble();

//potential new parser
//        if (Serial.available()>0) {
//            //indicating the beginning of the communication
//            Serial.println("begin");
//            //reading the serial string until the end of the line
//            char inputString = Serial.read();
//            // Split the string and convert to doubles
//            double Fx, Fy, Mz, Fx2, Fy2, Mz2;
//            char buffer[20];
//            int bufferIndex = 0;
//            // int firstSpaceIndex = inputString.indexOf(' ');
//            // int secondSpaceIndex = inputString.lastIndexOf(' ');
//            if(inputString != " "){
//                buffer[bufferIndex++] = inputString;
//
//                if(bufferIndex >= sizeof(buffer)){
//                    bufferIndex = 0;
//                }
//            } else if (bufferIndex>0) {
//                buffer[bufferIndex] = "\0";
//                double parsedValue = atof(buffer);
//                if(isnan(Fx)){
//                    Fx = parsedValue;
//                } else if (isnan(Fy)) {
//                    Fy = parsedValue;
//                } else if (isnan(Mz)) {
//                    Mz = parsedValue;
//                } else if (isnan(Fx2)) {
//                    Fx2 = parsedValue;
//                } else if (isnan(Fy2)) {
//                    Fy2 = parsedValue;
//                } else {
//                    Mz2 = parsedValue;
//                }
//            }
//            bufferIndex = 0;
//            Wrench = {Fx,Fy,Mz, Fx2, Fy2, Mz2};
//        }
//        ;
//    }
        /**initializing the wrench with the double values create above and using them
        to multiply by the ThreeDMixer to create a matrix that will convert the wrench
        values into PWM signals**/
        Wrench = {Fx,Fy,Mz, Fx2, Fy2, Mz2};
        N = ThreeDMixer * Wrench;
        /**need to map matrix values into PWM signals**/
        PWM={map(N(0),-50,50,1100,1900),map(N(1),-50,50,1100,1900),
             map(N(2),-50,50,1100,1900),map(N(3),-50,50,1100,1900),
             map(N(4),-50,50,1100,1900),map(N(5),-50,50,1100,1900)};
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
            motor_3.writeMicroseconds(PWM(5));
            Serial.print("wrote speed: ");
            Serial.print(PWM(5));
            Serial.println(" to motor 5");

        }
    }
    delay(SEND_INTERVAL);
    Serial.println("working");
}