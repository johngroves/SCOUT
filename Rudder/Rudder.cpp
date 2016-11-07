#include <Adafruit_HMC5883_U.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"
using namespace std;

extern Adafruit_HMC5883_Unified rudderCompass;
int rudderAddr = 6;

float getCompass()
{
    sensors_event_t event;
    Calculations::tcaselect(rudderAddr);
    rudderCompass.getEvent(&event);
    float degs = Calculations::sensorToDegrees(event.magnetic.x, event.magnetic.y);
    return degs;
}

Rudder::Rudder()
{
    Serial.print("Rudder Initialized.");
}

float Rudder::turnTo(float angle, char side)
{

}

rudderPosition Rudder::getAngle(float boatHeading)
{
    rudderPosition position;

    float rudderHeading = getCompass();
    position.angle = Calculations::degreesBetween(boatHeading,rudderHeading);

    float rudderRads = Calculations::degreesToRadians(rudderHeading);
    float boatRads = Calculations::degreesToRadians(boatHeading);
    float direction = sin((boatRads - rudderRads));

    if (direction > 0) {
        position.direction = 'p';
    } else {
        position.direction = 's';
    }
    return position;
}
