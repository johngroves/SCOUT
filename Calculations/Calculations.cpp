#include "Arduino.h"
#include "Calculations.h"
#include <Wire.h>
#define TCAADDR 0x70

float declinationAngle = 0.22;

float Calculations::degreessBetween(float angleOne, float angleTwo) {
/*
    Calculates the shortest distance (in radians) between two headings (in degrees)
*/
    float headingDelta = angleOne - angleTwo;
    if (headingDelta < 0) {
        headingDelta += 360;
    }
    if (headingDelta > 180) {
        headingDelta = (360.0 - headingDelta);
    } else {
        headingDelta = (headingDelta);
    }
    return headingDelta;
}

float Calculations::sensorToDegrees( float magneticX, float magneticY ) {
/*
    Accepts magnetic x and magnetic y, returns heading in degrees.
*/
    float angle = atan2( magneticY, magneticX );
    angle += declinationAngle;
    if (angle < 0) {
        angle += 2*PI;
    }
    if (angle > 2*PI) {
        angle -= 2*PI;
    }
    float degs = angle * 180/M_PI;
    return degs;
}


void Calculations::tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}