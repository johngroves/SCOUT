#include <Adafruit_Simple_AHRS.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"
using namespace std;

extern Boat db1;
extern volatile int encoder;
double encoderDeg = 0.0;
bool turning = false;

// i2C Address of Rudder Sensor
int rudderAddr = 6;
// Relay Pins
int POS1 = 46;
int NEG1 = 45;
int POS2 = 44;
int NEG2 = 47;


float getCompass() {
    return encoder * .15;
}


void motorOff() {

    // High is off for this relay
    digitalWrite(POS1, HIGH);
    digitalWrite(NEG1, HIGH);
    digitalWrite(POS2, HIGH);
    digitalWrite(NEG2, HIGH);

    turning = false;
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

    motorOff();
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
        turning = true;
    }

    if (angle < currentPosition.angle) {
        toPort();
        turning = true;

    }
}

rudderPosition Rudder::turnOff() {
    motorOff();
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

bool Rudder::getStatus() {
    return turning;
}
