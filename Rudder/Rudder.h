/*
 *
 */

#ifndef RUDDER_H
#define RUDDER_H

#include "Arduino.h"

// Class Definitions
class RudderController;
class RudderPositionSensor;

class Rudder
{
  public:
    Rudder();
    float getAngle();
    float turnTo(float angle, char side);
};

#endif