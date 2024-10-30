#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <QMC5883LCompass.h>
#include <calibration.h>

QMC5883LCompass compass;

bool debug = true;

void setup() {
  Serial.begin(115200);
  compass.init();
  float offsets[3];
  float scale[3];

  Serial.println("Waiting 3 seconds to start calibration");
  delay(3000);
  Serial.println("Calibrating...");

  int* x = new int[CALIBRATION_LENGTH];
  int* y = new int[CALIBRATION_LENGTH];
  int* z = new int[CALIBRATION_LENGTH];

  if (debug){
    Serial.println("x,y,z");
  }
  for (int i = 0; i < CALIBRATION_LENGTH; i++) {
    compass.read();
    x[i] = compass.getX();
    y[i] = compass.getY();
    z[i] = compass.getZ();
    if (debug){
      Serial.printf("%d,%d,%d\n", x[i],y[i],z[i]);
    }
    delay(CALIBRATION_DELAY);
  }
  
  calibrate(x, y, z, offsets, scale, debug=debug);
  
  // Print results
  Serial.print("Offsets: ");
  Serial.print(offsets[0]);
  Serial.print(", ");
  Serial.print(offsets[1]);
  Serial.print(", ");
  Serial.println(offsets[2]);
  
  Serial.print("Scale: ");
  Serial.print(scale[0]);
  Serial.print(", ");
  Serial.print(scale[1]);
  Serial.print(", ");
  Serial.println(scale[2]);
}

void loop() {
  delay(1000);
}
