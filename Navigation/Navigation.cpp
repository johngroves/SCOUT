#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include "Arduino.h"
#include "Boat.h"
#include "Calculations.h"
#include "Navigation.h"


extern Adafruit_GPS GPS;

Navigation::Navigation()
{
    Serial.println("GPS Initialized.");
}

Coordinate Navigation::getCurrentLocation()
{
    Coordinate coord;
    GPS.read();
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
            ;
    }
    if (GPS.fix) {
        coord.latitude = GPS.lat;
        coord.longitude = GPS.lon;
    } else {
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        coord.latitude = 0.000;
        coord.longitude = 0.000;
    }
    return coord;
}
