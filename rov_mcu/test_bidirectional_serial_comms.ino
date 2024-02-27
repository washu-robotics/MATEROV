#include <ArduinoJson.h>

void setup() {
  Serial.begin(38400);
  Serial.setTimeout(50);
  Serial.println("Hello world!");
}

void loop() {
  if (Serial.available()) {
    StaticJsonDocument<200> doc;
    String receivedData = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(doc, receivedData);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    double x_vel = doc["controls"]["linear"]["x"];
    if (x_vel > 0) {
      Serial.println("{\"info\": \"Moving forward\"}");
    } else if (x_vel < 0) {
      Serial.println("{\"info\": \"Moving backward\"}");
    } else {
      Serial.println("{\"info\": \"Staying still\"}");
    }
  }
}

