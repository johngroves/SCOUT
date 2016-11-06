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
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        coord.latitude = 0.000;
        coord.longitude = 0.000;
    }
    return coord;
}
