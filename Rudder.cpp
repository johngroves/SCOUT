#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Rudder.h"
#include "Boat.h"

#define TCAADDR 0x70

Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(1);

int addr = 6
float declinationAngle = 0.22;


void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

Rudder::Rudder()
{

}

float turnTo(float angle, char side)
{

}

float getAngle()
{

}

float readCompass()
{
    sensors_event_t event;
    tcaselect(6);
    compass.getEvent(&event);
    float angle = atan2(event.magnetic.y, event.magnetic.x);
    angle += declinationAngle;
    if(angle < 0)
        angle += 2*PI;
    if(angle > 2*PI)
        angle -= 2*PI;
    float degs = angle * 180/M_PI;
    return degs;
}