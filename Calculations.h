/*
 * Provides a class for handling calculations related to navigation, sensors, etc.
 */

#ifndef CALCULATIONS_H
#define CALCULATIONS_H
#define declinationAngle = 0.22

#include "Arduino.h"

// Class Definitions
class Calculations
{
  public:
    Calculations();
    float radiansBetween(float degrees1, float degrees2);
    float sensorToDegrees(float magneticX, float magneticY );
};

#endif