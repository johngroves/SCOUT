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
class Navigation;
class Motor;

class Boat
{
  public:
    Boat();
    ~Boat();
    float getHeading();
    Navigation* navigation;
    Rudder* rudder;

};

#endif