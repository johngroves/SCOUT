#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Calculations.h"
#include "Rudder.h"
#include "Boat.h"

#define TCAADDR 0x70

Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(1);
int addr = 6;
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
    return 0;
}

float readCompass()
{
    sensors_event_t event;
    tcaselect(addr);
    compass.getEvent(&event);
    float degs = Calculations.sensorToDegrees(event.magnetic.x, event.magnetic.y);
    return degs;
}