/*
 *
 */

#ifndef RUDDER_H
#define RUDDER_H

#include "Arduino.h"

struct rudderPosition {
    float angle;
    char direction;
};

// Class Definitions
class RudderController;
class RudderPositionSensor;

class Rudder
{
  public:
    Rudder();
    rudderPosition getAngle(float boatHeading);
    float turnTo(float angle, char side);
};

#endif