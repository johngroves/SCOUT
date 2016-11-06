/*
 * Navigation class - provides location services, navigation, waypoints.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "Arduino.h"

struct Coordinate {
    float latitude;
    float longitude;
};

// Class Definitions
class Navigation
{
  public:
    Navigation();
    Coordinate getCurrentLocation();
};

#endif