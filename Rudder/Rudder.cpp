#include <Adafruit_Simple_AHRS.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"
using namespace std;



extern Boat db1;
extern double encoder;

// i2C Address of Rudder Sensor
int rudderAddr = 6;
// Relay Pins
int POS1 = 46;
int NEG1 = 45;
int POS2 = 44;
int NEG2 = 47;


float getCompass() {
    return encoder;
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

void turnToSide(char side, bool same) {
    if (side = 'p') {
        if (same) {
            toPort();
        } else {
            toStarboard();
        }
    } else {
        if (same) {
            toStarboard();
        } else {
            toPort();
        }
    }
}

Rudder::Rudder() {

    // Initialize Rudder Control Digital Pins
    pinMode(POS1, OUTPUT);
    pinMode(NEG1, OUTPUT);
    pinMode(POS2, OUTPUT);
    pinMode(NEG2, OUTPUT);

    turnOff();
}

rudderPosition Rudder::turnTo(float angle, char side) {

    if (angle > 40)
        angle = 40;
    float boatHeading = db1.getHeading();
    rudderPosition currentPosition = this->getAngle();
    if ( side == currentPosition.direction ) {
        if ( angle > currentPosition.angle ) {
            while (angle > currentPosition.angle && side == currentPosition.direction) {
                turnToSide(side,true);
                currentPosition = this->getAngle();
            }
            turnOff();
        } else {
            while (angle < currentPosition.angle && side == currentPosition.direction) {
                turnToSide(side,false);
                currentPosition = this->getAngle();
            }
            turnOff();
        }
    } else {
            while (currentPosition.direction != side) {
                turnToSide(side,false);
                currentPosition = this->getAngle();
            }
            turnOff();
    }
    return this->getAngle();
}

rudderPosition Rudder::getAngle() {

    rudderPosition position;
    float rudderHeading = getCompass();
    position.angle = rudderHeading;

    if (rudderHeading >= 0) {
        position.direction = 's';
    } else {
        position.direction = 'p';
    }

    return position;
}
