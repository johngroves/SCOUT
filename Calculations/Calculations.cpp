#include "Arduino.h"
#include "Calculations.h"

float degreessBetween(float angleOne, float angleTwo) {
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

float sensorToDegrees( float magneticX, float magneticY ) {
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

