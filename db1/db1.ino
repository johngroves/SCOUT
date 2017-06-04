#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_LSM9DS0.h>
#include <Boat.h>
#include <Rudder.h>
#include <Navigation.h>
#include <Calculations.h>
#include <Adafruit_GPS.h>
#include <RotaryEncoder.h>

#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <CmdMessenger.h>
#include <Wire.h>

#define TCAADDR 0x70

// TODO: Move serial messaging to new file
enum {
    get_telemetry_data,
    telemetry_data,
    turn_to,
    new_rudder_position,
    error,
};

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


Adafruit_GPS GPS(&Serial3);
HardwareSerial mySerial = Serial3;

Adafruit_LSM9DS0  blsm(2000);
Adafruit_Simple_AHRS boatCompass (&blsm.getAccel(), &blsm.getMag());

Boat db1 = Boat();

const int encoder_a = 2; // Green - pin 2 - Digital
const int encoder_b = 3; // White - pin 3 - Digital
RotaryEncoder rudderEncoder(encoder_a, encoder_b);
int encoder = 0;

rudderPosition desired;
rudderPosition starting;

CmdMessenger c = CmdMessenger(Serial,',',';','/');

void return_telemetry(void) {
    // Fetch latest telemetry data
    float boatHeading = db1.getHeading();
    rudderPosition rudderPos = db1.rudder->getAngle();
    float rudderAngle = rudderPos.angle;
    char rudderSide = rudderPos.direction;
    Coordinate coord = db1.navigation->getCurrentLocation();

    // Return results over serial
    c.sendCmdStart(telemetry_data);
    c.sendCmdBinArg(boatHeading);
    c.sendCmdBinArg(rudderAngle);
    c.sendCmdBinArg(rudderSide);
    c.sendCmdBinArg(coord.latitude);
    c.sendCmdBinArg(coord.longitude);
    c.sendCmdEnd();
}

void onTurnCommand(void){

    // Accept angle (float), side (char), returns new position
    float desiredAngle = c.readBinArg<float>();
    char desiredSide = c.readBinArg<char>();

    rudderPosition rudderPos = db1.rudder->getAngle();

    if (desired.direction == 'p') {
        desired.angle = desiredAngle * -1.0;
    } else {
        desired.angle = desiredAngle * 1.0;
    }
    if (desired.angle > 40.0) {
        desired.angle = 40.0;
    }
    if (desired.angle < -40.0) {
        desired.angle = -40.0;
    }
    
    starting = rudderPos;     
    desired.direction = desiredSide;

    // Turn rudder
    db1.rudder->turnTo(desiredAngle,desiredSide);    
}

void sendPosition(rudderPosition pos){
      c.sendCmdStart(new_rudder_position);
      c.sendCmdBinArg(pos.angle);
      c.sendCmdBinArg(pos.direction);
      c.sendCmdEnd();
}    

void checkRudder() {
  // If motor is turning, check to see if reached correct position
  if (db1.rudder->getStatus()) {
    rudderPosition rudderPos = db1.rudder->getAngle();
            
    if ( desired.angle  > starting.angle ) {
       if ( desired.angle >= rudderPos.angle) {
        sendPosition(db1.rudder->turnOff());
       }
    } else {
      if (desired.angle <= rudderPos.angle) {
        sendPosition(db1.rudder->turnOff());
      }
    }

  }
         
}

void on_unknown_command(void){
    c.sendCmd(error,"Command without callback.");
}

void attach_callbacks(void) {
    c.attach(get_telemetry_data,return_telemetry);
    c.attach(turn_to,onTurnCommand);
    c.attach(on_unknown_command);
}

void setup() {

    // Start Processes at Serial Ports
    Serial.begin(115200);
    Wire.begin();
    GPS.begin(9600);
    attach_callbacks();

    // Start GPS w/ External Antenna Support (update @ 1hz)
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);

    // Upon Success
    //Serial.println("ATmega1280 Started. Serial Comms Succesful.");

    // Initialize Magnetometer
    tcaselect(7);
    if(!blsm.begin())
    {
        //Serial.println("Error: Rudder Sensor Did Not Initialize.");        
    }      
    configureBoatAccel();    

}

void configureBoatAccel(void)
{  
  blsm.setupAccel(blsm.LSM9DS0_ACCELRANGE_2G);  
  blsm.setupMag(blsm.LSM9DS0_MAGGAIN_8GAUSS);  
  blsm.setupGyro(blsm.LSM9DS0_GYROSCALE_2000DPS);
}

void loop() {
  static int pos = 0;  
  rudderEncoder.tick();
  int newPos = rudderEncoder.getPosition();
  
  if (pos != newPos) {    
    pos = newPos;
  }
  
  checkRudder();
  
  GPS.read();
  if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
          return;
  }
  c.feedinSerialData();  
}



