# Location Compass

- [Location Compass](#location-compass)
- [Overview](#overview)
- [Hardware Components](#hardware-components)
- [Software Components](#software-components)
  - [Functionality](#functionality)
  - [Constants and Variables](#constants-and-variables)
- [Main Files](#main-files)
  - [main.cpp](#maincpp)
  - [calibration.h](#calibrationh)
  - [helpers.h](#helpersh)
  - [locations.h](#locationsh)
- [Research](#research)
  - [Compass Calibration](#compass-calibration)
  - [Calculate Direction](#calculate-direction)

# Overview

This project is a location-compass system running on an ESP32 microcontroller. It integrates several modules including a stepper motor, GPS module, and a compass module. The system is designed to point to specific locations using the stepper motor, guided by GPS coordinates and compass direction. It was designed for a Kiosktour in Hannover, Germany.

# Hardware Components

- **ESP32 Microcontroller**: The main controller for the project.
- **Stepper Motor**: Used to point the compass needle towards the target location.
- **GPS Module**: Provides the current geographical coordinates.
- **Compass Module**: Determines the current heading direction.
- **LED**: Indicates system status.

# Software Components

## Functionality
- **Finding Nearby Location**: Finds the nearest location based on the current GPS coordinates.
- **Location Skipping**: Allows the user to skip location.
- **Calibration**: Allows the user to calibrate the compass over a button (max 290 data points).
- **Logging**: Logs data at regular intervals (100 milliseconds).
- **Testing**: Provides a testing mode with predefined GPS coordinates and time.

## Constants and Variables
- **defaultBlinkingInterval**: Default interval for LED blinking.
- **skippedLocations**: Vector to store indices of skipped locations.
- **nearestLocationIndex**: Index of the nearest location.
- **lastSkipLocationButtonPressTime**: Timestamp of the last skip location button press.
- **skipLocationInterval**: Delay before skipping another location (5 seconds).
- **logInterval**: Interval for logging data (100 milliseconds).
- **previousLogMillis**: Timestamp of the previous log.
- **testLocationCompass**: Flag for testing the location compass.
- **testLat, testLon**: Test latitude and longitude coordinates.
- **testHour, testMinute, testSecond, testDay, testMonth, testYear**: Test date and time in UTC+0.

# Main Files

## [main.cpp](platformio/location-compass/src/main.cpp)
Manages the core functionality of the location-compass system on the ESP32, including handling GPS data, compass calibration, controlling the stepper motor, managing skipped locations, logging, and testing configurations.

## [calibration.h](platformio/location-compass/src/calibration.h)
Contains functions and definitions for calibrating the compass module on the ESP32, including dynamically allocating matrices for calibration data and calculating offsets and scales for accurate readings. The calibration algorithm is based on [calibration.py](scripts/compass-calibration/calibration.py) from Fabio Varesano and converted to C.

## [helpers.h](platformio/location-compass/src/helpers.h)
Contains utility functions for the location-compass system, including functions for printing GPS time, calculating distances, and managing time-related operations.

## [locations.h](platformio/location-compass/src/locations.h)
Defines a Location structure to store geographical coordinates and opening hours, and initializes an array of Location instances with specific latitude, longitude, and weekly opening hours data.

# Research
## Compass Calibration
The compass module used in the location-compass system requires calibration to provide accurate heading directions. The calibration process involves collecting data points in different orientations and calculating offsets and scales to correct the raw readings. The calibration algorithm is based on the [calibration.py](scripts/compass-calibration/calibration.py) script by Fabio Varesano, which uses a least-squares method to estimate the calibration parameters.

You can find a visual representation of the calibration process in [compass-calibration.ipynb](scripts/compass-calibration/compass-calibration.ipynb). The notebook demonstrates how to collect calibration data, visualize the data points, and calculate the calibration parameters for the compass module.

## Calculate Direction
The location-compass system uses GPS coordinates to determine the direction to a specific location. The direction calculation is based on the Haversine formula, which calculates the initial bearing between two points on the Earth's surface. The formula takes into account the latitude and longitude of the current location and the target location to determine the compass direction.

You can find a visual representation of the direction calculation in [calculate-direction.ipynb](scripts/location-data/calculate-direction.ipynb). The notebook provides an overview of the Haversine formula, demonstrates how to calculate the initial bearing between two points, and shows how to convert the bearing to a compass direction.

