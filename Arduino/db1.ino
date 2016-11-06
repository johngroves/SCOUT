#include <Boat.h>
#include <Rudder.h>
#include <Calculations.h>

Boat db1 = Boat();

void setup() {
  Serial.begin(115200);  
  Serial.println("ATmega1280 Started. Serial Comms Succesful.");
}

void loop() {
  // put your main code here, to run repeatedly:
  float rudderAngle = db1.rudder->getAngle();
  delay(200);
  float boatHeading = db1.getHeading();
  Serial.print("Rudder Angle: ");
  Serial.println(rudderAngle);
  Serial.print("Boat Heading: ");
  Serial.println(boatHeading);
}

