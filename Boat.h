/*
 *
 */

#ifndef BOAT_H
#define BOAT_H

#include "Arduino.h"
#include "Rudder.h"
#include "Calculations.h"

// Class Definitions
class Rudder;
class Motor;

class Boat
{
  public:
    Boat();
    ~Boat();
    float getHeading();

    Rudder* rudder;
};

#endif