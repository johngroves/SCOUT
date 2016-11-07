#include <Adafruit_HMC5883_U.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"
using namespace std;

extern Adafruit_HMC5883_Unified rudderCompass;

// i2C Address of Rudder Sensor
int rudderAddr = 6;
// Relay Pins
int POS1 = 46;
int NEG1 = 45;
int POS2 = 44;
int NEG2 = 47;


float getCompass() {

    sensors_event_t event;
    Calculations::tcaselect(rudderAddr);
    rudderCompass.getEvent(&event);
    float degs = Calculations::sensorToDegrees(event.magnetic.x, event.magnetic.y);
    return degs;
}

void turnOff() {

    // High is off for this relay
    digitalWrite(POS1, HIGH);
    digitalWrite(NEG1, HIGH);
    digitalWrite(POS2, HIGH);
    digitalWrite(NEG2, HIGH);
}

void toPort() {

    digitalWrite(POS1, LOW);
    digitalWrite(NEG1, HIGH);
    digitalWrite(POS2, HIGH);
    digitalWrite(NEG2, LOW);
}

void toStarboard() {

    digitalWrite(POS1, HIGH);
    digitalWrite(NEG1, LOW);
    digitalWrite(POS2, LOW);
    digitalWrite(NEG2, HIGH);
}

Rudder::Rudder() {

    // Initialize Rudder Control Digital Pins
    pinMode(POS1, OUTPUT);
    pinMode(NEG1, OUTPUT);
    pinMode(POS2, OUTPUT);
    pinMode(NEG2, OUTPUT);

    Serial.print("Rudder Initialized.");
}

float Rudder::turnTo(float angle, char side) {

    delay(500);
    return getCompass();
}

rudderPosition Rudder::getAngle(float boatHeading) {

    rudderPosition position;

    float rudderHeading = getCompass();
    position.angle = Calculations::degreesBetween(boatHeading,rudderHeading);

    float rudderRads = Calculations::degreesToRadians(rudderHeading);
    float boatRads = Calculations::degreesToRadians(boatHeading);
    float direction = sin((boatRads - rudderRads));

    if (direction > 0) {
        position.direction = 's';
    } else {
        position.direction = 'p';
    }
    return position;
}
