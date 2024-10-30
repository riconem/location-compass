#include <QMC5883LCompass.h>
#include <math.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  compass.init();
  // Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  // Serial.println("Calibration will begin in 5 seconds.");
  // delay(5000);

  // Serial.println("CALIBRATING. Keep moving your sensor...");
  // compass.calibrate();
  // Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
  // Serial.println();
  // Serial.print("compass.setCalibrationOffsets(");
  // Serial.print(compass.getCalibrationOffset(0));
  // Serial.print(", ");
  // Serial.print(compass.getCalibrationOffset(1));
  // Serial.print(", ");
  // Serial.print(compass.getCalibrationOffset(2));
  // Serial.println(");");
  // Serial.print("compass.setCalibrationScales(");
  // Serial.print(compass.getCalibrationScale(0));
  // Serial.print(", ");
  // Serial.print(compass.getCalibrationScale(1));
  // Serial.print(", ");
  // Serial.print(compass.getCalibrationScale(2));
  // Serial.println(");");

  // compass.setCalibrationOffsets(compass.getCalibrationOffset(0), compass.getCalibrationOffset(1), compass.getCalibrationOffset(2));
  // compass.setCalibrationScales(compass.getCalibrationScale(0), compass.getCalibrationScale(1), compass.getCalibrationScale(2));

  Serial.println();
  Serial.println("x,y,z");
}

void loop() {
  int x, y, z;
  float x_offset, y_offset, x_scale, y_scale, x_c, y_c;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  x_offset = -158.92800950219117;
  y_offset = 254.23134906871306;
  x_scale = 1352.4144258849028;
  y_scale = 1405.1253106284073;

  x_c = (x - x_offset) / x_scale;
  y_c = (y - y_offset) / y_scale;

  // byte a = compass.getAzimuth();

  // char myArray[3];
  // compass.getDirection(myArray, a);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(y_c, x_c);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.068;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // // Print xyz in csv style
  Serial.printf("%d,%d,%d\n", x,y,z);

  // Output heading degrees
  // Serial.printf("Heading: %f degrees", headingDegrees);
  // Serial.println("");

  // Serial.print(myArray[0]);
  // Serial.print(myArray[1]);
  // Serial.print(myArray[2]);
  // Serial.println();

  // delay(100);
}