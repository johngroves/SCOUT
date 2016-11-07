#include <Boat.h>
#include <Rudder.h>
#include <Navigation.h>
#include <Calculations.h>
#include <Adafruit_GPS.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define TCAADDR 0x70


void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

Adafruit_GPS GPS(&Serial3);
HardwareSerial mySerial = Serial3;

Adafruit_HMC5883_Unified rudderCompass = Adafruit_HMC5883_Unified(1);
Adafruit_HMC5883_Unified boatCompass = Adafruit_HMC5883_Unified(2);

Boat db1 = Boat(); 

void printStats() {
  float boatHeading = db1.getHeading();
  rudderPosition rudderPos = db1.rudder->getAngle();
  float rudderAngle = rudderPos.angle; 
  char rudderSide = rudderPos.direction;
  Coordinate coord = db1.navigation->getCurrentLocation();
  Serial.println("");
  Serial.print("Rudder Angle: ");
  Serial.print(rudderAngle); 
  if(rudderSide == 'p'){
    Serial.println("° Port");
  } else {
    Serial.println("° Starboard");
  }
  Serial.print("Boat Heading: ");
  Serial.println(boatHeading);
  Serial.print("Latitude: ");
  Serial.println(coord.latitude);
  Serial.print("Longitude: ");
  Serial.println(coord.longitude);
}
 
void setup() {
  // Start Processes at Serial Ports
  Serial.begin(115200);
  Wire.begin();   
  GPS.begin(9600);   

  // Start GPS w/ External Antenna Support (update @ 1hz)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  // Upon Success
  Serial.println("ATmega1280 Started. Serial Comms Succesful.");

  // Initialize Sensors
  
  tcaselect(6);
  if(!rudderCompass.begin())
  {
    Serial.println("Error: Rudder Sensor Did Not Initialize.");
    while(1);
  }

  tcaselect(7);
  if(!boatCompass.begin())
  {
    Serial.println("Error: Rudder Sensor Did Not Initialize.");
    while(1);
  }

}

void loop() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))  
          return;  
  }  
  printStats();  
  db1.rudder->turnTo(2.290,'s');
  delay(1000);
}


