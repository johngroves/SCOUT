/*
 * Provides a class for handling calculations related to navigation, sensors, etc.
 */

#ifndef CALCULATIONS_H
#define CALCULATIONS_H
#include "Arduino.h"

// Class Definitions
class Calculations
{
  public:
    Calculations();
    static void  tcaselect(uint8_t i);
    static float radiansBetween(float degrees1, float degrees2);
    static float sensorToDegrees(float magneticX, float magneticY );
    static float degreesBetween(float angleOne, float angleTwo);
    static float degreesToRadians(float degrees);
};

#endif