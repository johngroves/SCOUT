#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Navigation.h"
#include "Boat.h"

extern Adafruit_HMC5883_Unified boatCompass;
int boatAddr = 7;

Boat::Boat()
{
    rudder = new Rudder();
    navigation = new Navigation();
}
Boat::~Boat()
{
    delete rudder;
    delete navigation;
}
float Boat::getHeading()
{
    sensors_event_t event;
    Calculations::tcaselect(boatAddr);
    boatCompass.getEvent(&event);
    float degs = Calculations::sensorToDegrees(event.magnetic.x, event.magnetic.y);
    return degs;
}
