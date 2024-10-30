#include <Arduino.h>
#include <Preferences.h>

Preferences preferences;

unsigned long previousTime = 0;
unsigned long previousTime2 = 0;
unsigned long previousTime3 = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Waiting 10 seconds...");

  // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  

  // Remove all preferences under the opened namespace
  // preferences.clear();

  // float y_offset = preferences.putFloat("y_offset", 0.0);
  // float z_offset = preferences.putFloat("z_offset", 0.0);
  // float x_scale = preferences.putFloat("x_scale", 0.0);
  // float y_scale = preferences.putFloat("y_scale", 0.0);
  // float z_scale = preferences.putFloat("z_scale", 0.0);
}

void loop() {

  unsigned long currentTime = millis();

  if (currentTime - previousTime >= 100) {
    preferences.begin("compass", false);
  
    if (currentTime - previousTime2 >= 10000) {
      Serial.println("10 seconds are over");
      preferences.putFloat("x_offset", float_t(10.0));
      previousTime2 = currentTime;

    }

    if (currentTime - previousTime3 >= 20000) {
      Serial.println("20 seconds are over");
      preferences.putFloat("x_offset", float_t(20.0));
      previousTime3 = currentTime;
    }

    float x_offset = preferences.getFloat("x_offset", 5.0);

    Serial.printf("Current x_offset value: %.2f\n", x_offset);

    preferences.end();

    previousTime = currentTime;
  }
}

