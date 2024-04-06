#include <ArduinoJson.h>
#include <Servo.h>
#include <Adafruit_LPS35HW.h>
#include <BasicLinearAlgebra.h> 
#include <math.h>

// Thrusters
Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
Servo motor5;

Servo motors[6] = {motor0, motor1, motor2, motor3, motor4, motor5};

// Mixer
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


// Weird mixer stuff
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

// Pressure & Temperature Sensor
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

#define LPS_CS  10
#define LPS_SCK  13
#define LPS_MISO 12
#define LPS_MOSI 11

double mapValue(float inVal, float inLow, float inHigh, float outLow, float outHigh)
{
  double outVal = (inVal - inLow) * (outHigh - outLow) / (inHigh - inLow) + outLow;

  return outVal;
}


void motorSetup() {
  for (int i = 0; i < 6; i++) {
    motors[i].attach(i);
    motors[i].writeMicroseconds(1500);
  }
}

void sensorSetup() {
  if (!lps35hw.begin_SPI(LPS_CS, LPS_SCK, LPS_MISO, LPS_MOSI)) {
    while (1);
  }
}

void setup() {
  Serial.begin(38400);
  Serial.setTimeout(50);

  motorSetup();
  //sensorSetup();
  pinMode(LED_BUILTIN, OUTPUT);
}

int processThrusterCmd(double x_vel, double y_vel, double z_vel) {
  int x_thruster = (int)(x_vel * 500 + 1500); // 1000 - 2000
  if (x_thruster > 2000) {
    x_thruster = 2000;
  } else if (x_thruster < 1000) {
    x_thruster = 1000;
  }

  motor1.writeMicroseconds(x_thruster);
  return x_thruster;
}

void loop() {
  if (Serial.available()) {
    return;
  }

  StaticJsonDocument<256> request;
  String receivedData = Serial.readStringUntil('\n');
  DeserializationError error = deserializeJson(request, receivedData);

  StaticJsonDocument<256> response;
  response["error"] = "";
  response["info"] = "";
  response["timestamp"] = millis();

  response["temp"] = lps35hw.readTemperature();
  response["pressure"] = lps35hw.readPressure();

  if (error) {
    response["error"] = "Failed to parse request: " + String(error.f_str());
    // processThrusterCmd(0, 0, 0);
    Wrench = {0, 0, 0, 0, 0, 0};
  } else {
    double x_vel = request["controls"]["linear"]["x"];
    double y_vel = request["controls"]["linear"]["y"];
    double z_vel = request["controls"]["linear"]["z"];
    double roll = request["controls"]["angular"]["x"];
    double pitch = 0;//request["controls"]["angular"]["y"];
    double yaw = request["controls"]["angular"]["z"];

    
  //Serial.println(x_vel);

    int MAX_XYZ = 30;
    int MAX_ROLL = 10;
    int MAX_YAW = 10;
    int MAX_PITCH = 0;
    int MIN_XYZ = -30;
    int MIN_ROLL = -10;
    int MIN_YAW = -10;
    int MIN_PITCH = 0;

    // Wrench = {x_vel, y_vel, z_vel, roll, pitch, yaw};
    // Wrench = {map(x_vel, -1, 1, MIN_XYZ, MAX_XYZ), map(y_vel, -1, 1, MIN_XYZ, MAX_XYZ), map(z_vel, -1, 1, MIN_XYZ, MAX_XYZ), 0, 0, map(yaw, -1.5, 1.5, MIN_YAW, MAX_YAW)};
    Wrench = {mapValue(x_vel, -0.5, 0.5, MIN_XYZ, MAX_XYZ), mapValue(y_vel, -0.5, 0.5, MIN_XYZ, MAX_XYZ), mapValue(z_vel, -0.5, 0.5, MIN_XYZ, MAX_XYZ), 0, 0, mapValue(yaw, -0.5, 0.5, MIN_YAW, MAX_YAW)};
  }


  N = Mixer * Wrench;
  PWM = {nSquared_to_PWM_CW(N(0)),nSquared_to_PWM_CCW(N(1)),nSquared_to_PWM_CW(N(2)), nSquared_to_PWM_CCW(N(3)),nSquared_to_PWM_CW(N(4)),nSquared_to_PWM_CCW(N(5))};
  for (int i = 0; i < 6; i++) {
    // motors[i].writeMicroseconds(PWM(i));
    int MAX_PWM = 1900;
    int MIN_PWM = 1100;
    if (PWM(i) > MAX_PWM) {
      PWM(i) = MAX_PWM;
    } else if (PWM(i) < MIN_PWM) {
      PWM(i) = MIN_PWM;
    }
    // motors[i].writeMicroseconds(PWM(i));

    response["motors"][i] = PWM(i);
  }

  char buffer[256];
  serializeJson(response, buffer);
  Serial.println(buffer);
}
