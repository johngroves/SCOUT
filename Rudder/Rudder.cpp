#include <Adafruit_Simple_AHRS.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"
using namespace std;

extern Boat db1;
extern volatile double encoder;

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

Rudder::Rudder() {

    // Initialize Rudder Control Digital Pins
    pinMode(POS1, OUTPUT);
    pinMode(NEG1, OUTPUT);
    pinMode(POS2, OUTPUT);
    pinMode(NEG2, OUTPUT);

    turnOff();
}

void Rudder::turnTo(float angle, char side) {

    if (side == 'p') {
        angle = angle * -1.0;
    } else {
        angle = angle * 1.0;
    }

    if (angle > 40.0) {
        angle = 40.0;
    }


    if (angle < -40.0) {
        angle = -40.0;
    }


     rudderPosition currentPosition = this->getAngle();
        if ( angle > currentPosition.angle ) {
            toStarboard();
            while (angle > currentPosition.angle) {
                currentPosition = this->getAngle();
            }
        } else {
            toPort();
            while (angle < currentPosition.angle) {
                currentPosition = this->getAngle();
            }
        }
    turnOff();
}

rudderPosition Rudder::getAngle() {

    rudderPosition position;
    float rudderHeading = getCompass();
    position.angle = rudderHeading;

    if (rudderHeading >= 0) {
        position.direction = 'p';
    } else {
        position.direction = 's';
    }
    return position;
}
