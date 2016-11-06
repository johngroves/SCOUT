#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"

Adafruit_HMC5883_Unified boatCompass = Adafruit_HMC5883_Unified(1);
int boatAddr = 7;

Boat::Boat()
{
    rudder = new Rudder ();
}
Boat::~Boat()
{
    delete rudder;
}
float Boat::getHeading()
{
    sensors_event_t event;
    Calculations::tcaselect(boatAddr);
    boatCompass.getEvent(&event);
    float degs = Calculations::sensorToDegrees(100.20, 30.209);
    return degs;
}
