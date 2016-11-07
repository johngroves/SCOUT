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
class Boat;

class Rudder
{
  public:
    Rudder();
    rudderPosition getAngle();
    rudderPosition turnTo(float angle, char side);
};

#endif