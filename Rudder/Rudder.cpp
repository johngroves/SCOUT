#include <Adafruit_HMC5883_U.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"

extern Adafruit_HMC5883_Unified rudderCompass;
int rudderAddr = 6;


Rudder::Rudder()
{
    Serial.print("Rudder Initialized.");
}

float Rudder::turnTo(float angle, char side)
{

}

float Rudder::getAngle()
{
    sensors_event_t event;
    Calculations::tcaselect(rudderAddr);
    rudderCompass.getEvent(&event);
    float degs = Calculations::sensorToDegrees(event.magnetic.x, event.magnetic.y);
    return degs;
}
