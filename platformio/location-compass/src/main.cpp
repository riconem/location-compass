#include <Arduino.h>
#include <math.h>
#include <ezButton.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <AccelStepper.h>
#include <QMC5883LCompass.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <locations.h>
#include <helpers.h>
#include <calibration.h>
#include <Preferences.h>
#include <vector>  // Include the vector header
#include <algorithm> // Include algorithm for std::find

/////////////////////// General ///////////////////////

#define BAUD 115200

bool DEBUG = true;

/////////////////////// Preferences ///////////////////////

Preferences preferences;

/////////////////////// Stepper Motor ///////////////////////

// Stepper Motor: ULN2003 Motor Driver Pins
#define STEPPER_IN1 19
#define STEPPER_IN2 18
#define STEPPER_IN3 5
#define STEPPER_IN4 4

AccelStepper stepper (AccelStepper::FULL4WIRE, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

const int stepsPerRevolution = 2048;

const int stepperSpeed=500;

const int constantStepperSpeed=250;

int last_step_position = 0;
int target_step_position = 0;
int relative_steps = 0;

int steps_current_home, current_step_position;

bool positionZeroIsSet = false;

/////////////////////// Compass ///////////////////////

QMC5883LCompass compass;

// calibration state
bool calibrationRunning = false;

// calibration delay in milliseconds between each measurement
const int calibrationDelay = 100;

const int max_degrees = 360;

// current calibration count
int calibrationCount;

int* xData = new int[CALIBRATION_LENGTH];
int* yData = new int[CALIBRATION_LENGTH];
int* zData = new int[CALIBRATION_LENGTH];

// last calibrated: 07.05.2024
// const float x_offset = -582.755582319061;
// const float y_offset = 738.1362679188125;
// const float z_offset = 73.60810541927806;
// const float x_scale = 1371.5674045885469;
// const float y_scale = 1364.1750003047318;
// const float z_scale = 1443.0382165087578;

// Offsets: [59.539940758999236, -82.84971840031805, -46.68735595203899]
// Scale: [1037.0762660821954, 1726.1447295456271, 1495.5345049276914]

float x_offset = 59.539940758999236;
float y_offset = -82.84971840031805;
float z_offset = -46.68735595203899;
float x_scale = 1037.0762660821954;
float y_scale = 1726.1447295456271;
float z_scale = 1495.5345049276914;

// Set to true when compass module is upside down
bool invertHeading = false;

// You must add your Declination Angle, which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
// If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
float declinationAngle = 0.068;

// compass calibrated axis
float x_c, y_c;

// compass axis
int x, y, z;

// heading to north
float heading, headingDegrees;

/////////////////////// GPS Module ///////////////////////

#define GPS_SERIAL 2
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600
#define GPS_CONFIG SERIAL_8N1

// GPS Timeout in milliseconds
int gpsTimeout = 5000;

int gpsCharsProcessedMax = 10;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial gpsSerial(GPS_SERIAL);

// Gps Location
double currentLat, currentLon, desiredLat, desiredLon, homeLat, homeLon;

bool gpsValid = false;

bool locationFound = false;

// location not found error intervall in milliseconds
const unsigned long locationNotFoundErrorInterval = 1000;

/////////////////////// Buttons ///////////////////////

#define POSITION_ZERO_BUTTON 26
#define VARIABLE_BUTTON 25
#define LOCATION_MODE_BUTTON 27
#define LOG_MODE_BUTTON 12
#define COMPASS_MODE_BUTTON 14

// Set debounce time for all buttons
const unsigned long defaultButtonDebounceTime = 100, variableButtonDebounceTime = 3000, positionZeroButtonDebounceTime = 100;

// Button to set current stepper position to 0
ezButton positionZeroButton(POSITION_ZERO_BUTTON);

// Variable button on different modes - hold for 3 seconds
// compass mode: calibrate compass
// location mode: throw location and show next location
// home mode: set current position to home
ezButton variableButton(VARIABLE_BUTTON);

// When the location search mode is active
ezButton locationMode(LOCATION_MODE_BUTTON);

// When the log mode is active
ezButton logMode(LOG_MODE_BUTTON);

// When the compass mode is active
ezButton compassMode(COMPASS_MODE_BUTTON);

/////////////////////// Time ///////////////////////

unsigned long previousTime = 0;

// Interval to repeat the set a new stepper position in milliseconds
const unsigned long stepperInterval = 500;

// Shows if the the time is set up
bool initialTimeIsSet = false;

// Central European Time (Frankfurt, Berlin, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};  // Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};   // Central European Standard Time

Timezone localTimezone(CEST, CET);
time_t localTime;
TimeChangeRule *tcr; 

/////////////////////// LEDs ///////////////////////

// Define the LED pin (for external LED or built-in LED)
#define LED_PIN 32

// default blinking interval in milliseconds
const unsigned long defaultBlinkingInterval = 100;

/////////////////////// Skip Locations ///////////////////////

std::vector<int> skippedLocations; // To store the indices of skipped locations
int nearestLocationIndex = -1;  // Track the index of the nearest location
unsigned long lastSkipLocationButtonPressTime = 0;
const unsigned long skipLocationInterval = 5000; // 5 second delay before skipping another location

/////////////////////// Logging ///////////////////////

const long logInterval = 100; // logging interval in milliseconds
unsigned long previousLogMillis = 0;

/////////////////////// Testing ///////////////////////

bool testLocationCompass = false;

double testLat;
double testLon;

// utc+0
int testHour = 10;
int testMinute = 24; 
int testSecond = 00; 
int testDay = 21;
int testMonth = 9;
int testYear = 2024;

void setup()
{
    Serial.begin(BAUD);
    
    gpsSerial.begin(GPS_BAUD, GPS_CONFIG, GPS_RX, GPS_TX);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    compass.init();

    positionZeroButton.setDebounceTime(positionZeroButtonDebounceTime);
    variableButton.setDebounceTime(variableButtonDebounceTime);
    locationMode.setDebounceTime(defaultButtonDebounceTime);
    logMode.setDebounceTime(defaultButtonDebounceTime);
    compassMode.setDebounceTime(defaultButtonDebounceTime);

    // Set constant speed on startup to find position zero
    stepper.setMaxSpeed(constantStepperSpeed);
    stepper.setSpeed(constantStepperSpeed);
}

void loop()
{
  unsigned long currentTime = millis();
  positionZeroButton.loop();
  variableButton.loop();
  locationMode.loop();
  logMode.loop();

  // Set initial position Zero on startup
  // The compass is rotating constantly now
  if (!positionZeroIsSet && positionZeroButton.isPressed())
  {
      Serial.println("Setting current stepper position to 0.");
      stepper.setCurrentPosition(0);
      stepper.setSpeed(stepperSpeed);
      stepper.setMaxSpeed(stepperSpeed);
      stepper.setAcceleration(stepperSpeed);
      stepper.move(target_step_position);
      positionZeroIsSet = true;
  }

  if (variableButton.isPressed() && locationMode.getState() == 0 && nearestLocationIndex != -1 && (millis() - lastSkipLocationButtonPressTime > skipLocationInterval))
  {
    Serial.println("Throwing out latest location");
    lastSkipLocationButtonPressTime = millis();
    skippedLocations.push_back(nearestLocationIndex);
  }

  if (logMode.getState() == 0){
    unsigned long currentLogMillis = millis();
    if (currentLogMillis - previousLogMillis >= logInterval) {
      previousLogMillis = currentLogMillis; 
      float time = micros() / 1e6;
      compass.read();
      x = compass.getX();
      y = compass.getY();
      z = compass.getZ();
      Serial.printf("%f,%d,%d,%d\n", time,x,y,z);
    }
  }
  // Triggers Compass Calibration
  else if (variableButton.isPressed() && logMode.getState() == 1 && locationMode.getState() == 1)
  {
    digitalWrite(LED_PIN, HIGH);
    Serial.printf("Starting compass calibration. Takes %d ms.\n", calibrationDelay*CALIBRATION_LENGTH);
    calibrationRunning = true;
    calibrationCount = 0;
    if (DEBUG){
      Serial.println("x,y,z");
    }
    digitalWrite(LED_PIN, LOW);

    // Set constant speed during calibration to simulate running compass 
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setSpeed(stepperSpeed);
  }
  // Calibration Loop
  if (calibrationRunning && currentTime - previousTime >= calibrationDelay){
    digitalWrite(LED_PIN, HIGH);
    compass.read();
    xData[calibrationCount] = compass.getX();
    yData[calibrationCount] = compass.getY();
    zData[calibrationCount] = compass.getZ();
    if (DEBUG){
      Serial.printf("%d,%d,%d\n", xData[calibrationCount],yData[calibrationCount],zData[calibrationCount]);
    }
    digitalWrite(LED_PIN, LOW);
    if (calibrationCount == CALIBRATION_LENGTH-1){
      if (DEBUG){
        Serial.println("Current Data before calibration");
        for (int i = 0; i < CALIBRATION_LENGTH; i++) {
          Serial.printf("%d,%d,%d\n", xData[i],yData[i],zData[i]);
        }
        Serial.println("End Of Data");
      }
      float offsets[3];
      float scale[3];
      calibrate(xData, yData, zData, offsets, scale);

      // Save calibration data in memory
      Serial.println("Done. Saving calibration data in memory.");
      preferences.begin("compass", false);
      preferences.putFloat("x_offset", offsets[0]);
      preferences.putFloat("y_offset", offsets[1]);
      preferences.putFloat("z_offset", offsets[2]);
      preferences.putFloat("x_scale", scale[0]);
      preferences.putFloat("y_scale", scale[1]);
      preferences.putFloat("z_scale", scale[2]);
      preferences.end();

      Serial.printf("offset: %f, %f, %f\n", offsets[0], offsets[1], offsets[2]);
      Serial.printf("scale: %f, %f, %f\n", scale[0], scale[1], scale[2]);
      calibrationRunning = false;
      positionZeroIsSet = false;
      stepper.setCurrentPosition(0);
      stepper.setMaxSpeed(constantStepperSpeed);
      stepper.setSpeed(constantStepperSpeed);
    }

    calibrationCount++;
    previousTime = currentTime;
  }

  // Getting GPS Data
  while (gpsSerial.available() > 0){
    if (gps.encode(gpsSerial.read()))
    {
      // Set current Lat, Lon and Time
      if (gps.location.isValid() && gps.time.isValid() && gps.date.isValid())
      {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
        localTime = localTimezone.toLocal(now());
        currentLat = gps.location.lat();
        currentLon = gps.location.lng();
        gpsValid = true;
      }
      else if (testLocationCompass)  // TODO BUG: does set the values from defined test
      {
        setTime(testHour, testMinute, testSecond, testDay, testMonth, testYear);
        localTime = localTimezone.toLocal(now());
        currentLat = testLat;
        currentLon = testLon;
        gpsValid = true;
      }
      else
      {
        gpsValid = false;
      }
    }
  }
  if (currentTime > gpsTimeout && gps.charsProcessed() < gpsCharsProcessedMax)
  {
    Serial.println("No GPS detected: check wiring.");
  }


  // Location Compass Loop
  else if (currentTime - previousTime >= stepperInterval && positionZeroIsSet && !calibrationRunning)
  {
    compass.read();

    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();

    // Get latest calibration data from memory
    preferences.begin("compass", false);
    x_offset = preferences.getFloat("x_offset", 0.0);
    y_offset = preferences.getFloat("y_offset", 0.0);
    z_offset = preferences.getFloat("z_offset", 0.0);
    x_scale = preferences.getFloat("x_scale", 0.0);
    y_scale = preferences.getFloat("y_scale", 0.0);
    z_scale = preferences.getFloat("z_scale", 0.0);
    preferences.end();

    x_c = (x - x_offset) / x_scale;
    y_c = (y - y_offset) / y_scale;

    if (invertHeading) {
      heading = atan2(-y_c, x_c)+PI;
    }
    else {
      heading = atan2(y_c, x_c);
    }

    heading += declinationAngle;
    
    if(heading < 0)
      heading += 2*PI;
    if(heading > 2*PI)
      heading -= 2*PI;
    
    // heading in degrees to north or desired location
    headingDegrees = heading * 180/M_PI;

    if (locationMode.getState() == 0){
      Serial.println("Mode: Location\n");
      if(gpsValid)
      {
        // Convert from Sunday 1 - Saturday 7 to Monday 0 - Sunday is 6
        int currentWeekDay = (weekday(localTime)%7+5)%7;
        int currentHour = hour(localTime);
        int currentMinute = minute(localTime);
        
        // Calculate distance to each location and find nearest open location
        float minDistance = INFINITY;
        Location nearestLocation;
        int numLocations = sizeof(locations) / sizeof(locations[0]); // Calculate number of locations
        for (int i = 0; i < numLocations; i++) {
          if (std::find(skippedLocations.begin(), skippedLocations.end(), i) != skippedLocations.end()) {
            continue; // Skip this location
          }
          const auto& location = locations[i];
          // Check if location is open
          if (isOpen(location.openingHours, currentWeekDay, currentHour, currentMinute))
          {
            float distance = calculateDistance(currentLat, currentLon, location.latitude, location.longitude);

            // Check if new minimum is found
            if (distance < minDistance)
            {
              minDistance = distance;
              nearestLocation = location;
              nearestLocationIndex = i;
            }
          }
        }
        if (minDistance == INFINITY)
        {
          desiredLat = 0.0;
          desiredLon = 0.0;
          locationFound = false;
        }
        else
        {
          locationFound = true;
          desiredLat = nearestLocation.latitude;
          desiredLon = nearestLocation.longitude;

          int openHour = nearestLocation.openingHours[currentWeekDay][0];
          int openMinute = nearestLocation.openingHours[currentWeekDay][1];
          int closeHour = nearestLocation.openingHours[currentWeekDay][2];
          int closeMinute = nearestLocation.openingHours[currentWeekDay][3];
          float desiredDirection = calculateDirection(currentLat, currentLon ,desiredLat, desiredLon);

          Serial.printf("Nearest Open Location:\n Lat: %f\n Long: %f\n", nearestLocation.latitude, nearestLocation.longitude);
          Serial.printf(" Distance: %f meters\n", minDistance);
          Serial.print(" Opening Time: ");
          Serial.print(openHour);
          Serial.print(":");
          if (openMinute < 10) 
          {
              Serial.print("0");
          }
          Serial.print(openMinute);
          Serial.println();
          Serial.print(" Closing Time: ");
          Serial.print(closeHour);
          Serial.print(":");
          if (closeMinute < 10)
          {
              Serial.print("0");
          }
          Serial.print(closeMinute);
          Serial.println();
          Serial.printf("Current Week Day: %d\n", currentWeekDay);
          Serial.printf("Current Hour: %d\n", currentHour);
          Serial.printf("Current Minute: %d\n", currentMinute);
          Serial.printf("Current Position:\n Lat: %f\n Long: %f\n", currentLat, currentLon);
          Serial.printf(" Direction: %f degrees\n", desiredDirection);
          Serial.printf(" Heading To North: %f degrees\n", headingDegrees);

          headingDegrees = fmod((headingDegrees + desiredDirection), (float)max_degrees);
        }
      }
    }
    else if (logMode.getState() == 1 && locationMode.getState() == 1)
    {
      Serial.println("Mode: Compass\n");
    }

    if (logMode.getState() == 1){
      Serial.printf("Heading: %f degrees\n", headingDegrees);
      Serial.printf("Last Step Position: %d\n", last_step_position);
    }

    if (stepper.distanceToGo() == 0){
      target_step_position = int(round((headingDegrees/max_degrees)*stepsPerRevolution));
      relative_steps = calculate_shortest_relative_step(last_step_position, target_step_position, stepsPerRevolution);
      stepper.move(relative_steps);
      last_step_position = target_step_position;
    }
    if (logMode.getState() == 1){
      Serial.printf("Target Step Position: %d\n", target_step_position);
      Serial.printf("Relative Steps: %d\n", relative_steps);
      Serial.printf("Position Zero Button State: %d\n", positionZeroButton.getState());
      Serial.printf("Variable Button State: %d\n", variableButton.getState());
      Serial.printf("Location Mode Button State: %d\n", locationMode.getState());
      Serial.printf("Compass Mode Button State: %d\n", compassMode.getState());
      Serial.printf("Log Mode Button State: %d\n", logMode.getState());
      Serial.printf("Compass Offset: %f, %f, %f\n", x_offset, y_offset, z_offset);
      Serial.printf("Compass Scale: %f, %f, %f\n", x_scale, y_scale, z_scale);
      Serial.printf("GPS Valid: %s, %d\n", gpsValid ? "true" : "false", gpsValid);
      Serial.printf("Location Found: %s, %d\n", locationFound ? "true" : "false", locationFound);
      Serial.print("--------------------------------------------------------\n\n");
    }

    // Shows gps status
    if (gpsValid){
      digitalWrite(LED_PIN, LOW);
    }
    else {
      digitalWrite(LED_PIN, HIGH);
    }

    // Shows location found status
    if (locationFound)
    {
      digitalWrite(LED_PIN, LOW);
    }
    else
    {
      digitalWrite(LED_PIN, HIGH);
    }
    previousTime = currentTime;
  }

  if (positionZeroIsSet && !calibrationRunning){
    stepper.run();
  }
  else
  {
    stepper.runSpeed();
  }
}
