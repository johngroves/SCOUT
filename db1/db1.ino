#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_LSM9DS0.h>
#include <Boat.h>
#include <Rudder.h>
#include <Navigation.h>
#include <Calculations.h>
#include <Adafruit_GPS.h>

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
volatile double encoder = 0.0;


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

    // Turn rudder
    rudderPosition newPosition = db1.rudder->turnTo(desiredAngle,desiredSide);

    // Return new position
    c.sendCmdStart(new_rudder_position);
    c.sendCmdBinArg(newPosition.angle);
    c.sendCmdBinArg(newPosition.direction);
    c.sendCmdEnd();
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

    // Encoder
    pinMode(encoder_a, INPUT_PULLUP);
    pinMode(encoder_b, INPUT_PULLUP);    
    attachInterrupt(0, encoderPinChangeA, CHANGE);
    attachInterrupt(1, encoderPinChangeB, CHANGE);

}

void configureBoatAccel(void)
{  
  blsm.setupAccel(blsm.LSM9DS0_ACCELRANGE_2G);  
  blsm.setupMag(blsm.LSM9DS0_MAGGAIN_8GAUSS);  
  blsm.setupGyro(blsm.LSM9DS0_GYROSCALE_2000DPS);
}

void loop() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
          return;
  }
  c.feedinSerialData();

}

void encoderPinChangeA() {
encoder += digitalRead(encoder_a) == digitalRead(encoder_b) ? 0.15 : -0.15;
}

void encoderPinChangeB() {
encoder += digitalRead(encoder_a) != digitalRead(encoder_b) ? 0.15 : -0.15;
}


