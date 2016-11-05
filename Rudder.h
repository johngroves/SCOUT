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
    void turnTo(float angle, char side);
    float getAngle();
}

#endif