#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"

Adafruit_HMC5883_Unified rudderCompass = Adafruit_HMC5883_Unified(1);
int rudderAddr = 6;

Rudder::Rudder()
{

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
