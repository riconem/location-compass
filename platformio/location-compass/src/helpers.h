#include "Arduino.h"
#include <TinyGPSPlus.h>
#include <math.h>

const int MAX_DEGREES = 360;

// Function to convert degrees to radians
float degreesToRadians(float degrees) {
    return degrees * M_PI / 180.0;
}

// Function to convert radians to degrees
float radiansToDegrees(float radians) {
    return radians * (180.0 / M_PI);
}

// Main logic to calculate shortest relative path to new stepper position
int calculate_shortest_relative_step(int last_step_position, int target_step_position, int max_steps){

    int clockwise_steps = (target_step_position - last_step_position + max_steps) % max_steps;
    int counterclockwise_steps = (last_step_position - target_step_position + max_steps) % max_steps;
    
    if (clockwise_steps <= counterclockwise_steps)
    {
        return -clockwise_steps;
    }
    else 
    {
        return counterclockwise_steps;
    }
}


// Calculate distance between two coordinates (in meters)
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float R = 6371000; // Earth's radius in meters
    float phi1 = lat1 * M_PI / 180;
    float phi2 = lat2 * M_PI / 180;
    float deltaPhi = (lat2 - lat1) * M_PI / 180;
    float deltaLambda = (lon2 - lon1) * M_PI / 180;

    float a = sin(deltaPhi/2) * sin(deltaPhi/2) +
              cos(phi1) * cos(phi2) *
              sin(deltaLambda/2) * sin(deltaLambda/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));

    return R * c;
}

// Calculate direction between two coordinates
float calculateDirection(float lat1, float lon1, float lat2, float lon2) {

    float phi1 = degreesToRadians(lat1);
    float phi2 = degreesToRadians(lat2);
    float deltaLambda = degreesToRadians(lon2 - lon1);

    float y = sin(deltaLambda * cos(phi2));
    float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
    float c = atan2(y, x);
    float c_degrees = radiansToDegrees(c);
    float bearing =  fmod((c_degrees + (float)MAX_DEGREES), (float)MAX_DEGREES);

    return bearing;
}

// Function to check if a location is currently open
bool isOpen(const int openingHours[7][4], int currentDay, int currentHour, int currentMinute) {
    int openHour = openingHours[currentDay][0];
    int openMinute = openingHours[currentDay][1];
    int closeHour = openingHours[currentDay][2];
    int closeMinute = openingHours[currentDay][3];

    if (openHour == 0 && closeHour == 0) {
      return false;
    }

    // redefine close hour when after 24 h
    if (closeHour < openHour) {
      closeHour+=24;
    }

    if (currentHour > openHour || (currentHour == openHour && currentMinute >= openMinute)) {
        if (currentHour < closeHour || (currentHour == closeHour && currentMinute < closeMinute)) {
            return true;
        }
    }
    return false;
}

void displayGpsInfo(TinyGPSPlus gps) {
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) {
      Serial.print(F("0"));
    }
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) {
      Serial.print(F("0"));
    }
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) { 
      Serial.print(F("0"));
    }
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) {
      Serial.print(F("0"));
    }
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}