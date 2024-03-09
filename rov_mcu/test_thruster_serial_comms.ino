#include <ArduinoJson.h>
#include <Servo.h>
#include <Adafruit_LPS35HW.h>

Servo motor1;

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

// For SPI mode, we need a CS pin
#define LPS_CS  10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LPS_SCK  13
#define LPS_MISO 12
#define LPS_MOSI 11

void motorSetup() {
  motor1.attach(3);
  motor1.writeMicroseconds(1500);
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
  sensorSetup();
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
    processThrusterCmd(0, 0, 0);
  } else {
    double x_vel = request["controls"]["linear"]["x"];
    double y_vel = request["controls"]["linear"]["y"];
    double z_vel = request["controls"]["linear"]["z"];
    int x_thruster = processThrusterCmd(x_vel, y_vel, z_vel);
    response["info"] = "Moving at " + String(x_thruster);
  }

  char buffer[256];
  serializeJson(response, buffer);
  Serial.println(buffer);
}

