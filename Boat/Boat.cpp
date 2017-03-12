#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Navigation.h"
#include "Boat.h"

extern Adafruit_Simple_AHRS  boatCompass;

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
    sensors_vec_t   orientation;
    Calculations::tcaselect(boatAddr);
    boatCompass.getOrientation(&orientation);
    float degs = orientation.heading;
    if(degs <0){
        degs+=360;
    }
    return degs;
}