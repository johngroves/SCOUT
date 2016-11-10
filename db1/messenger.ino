/* -----------------------------------------------------------------------------
 * Example .ino file for arduino, compiled with CmdMessenger.h and
 * CmdMessenger.cpp in the sketch directory.
 *----------------------------------------------------------------------------*/

#include "CmdMessenger.h"


enum {
    get_telemetry_data,
    telemetry_data,
    turn_to,
    new_rudder_position,
    error,
};

const int BAUD_RATE = 28800;
CmdMessenger c = CmdMessenger(Serial,',',';','/');

float boatHeading = 180.123122;
float rudderAngle = 30.3;
char rudderSide = 's';
float latitude = 76.342342;
float longitude = -128.33342;

void return_telemetry(void) {
  c.sendCmdStart(telemetry_data);
  c.sendCmdBinArg(boatHeading);
  c.sendCmdBinArg(rudderAngle);
  c.sendCmdBinArg(rudderSide);
  c.sendCmdBinArg(latitude);
  c.sendCmdBinArg(longitude);
  c.sendCmdEnd();
}

void onTurnCommand(void){
    float desiredAngle = c.readBinArg<float>();
    char desiredSide = c.readBinArg<char>();
    c.sendCmdStart(new_rudder_position);
    c.sendCmdBinArg(rudderAngle);
    c.sendCmdBinArg(rudderSide);
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
    Serial.begin(BAUD_RATE);
    attach_callbacks();
}

void loop() {
    c.feedinSerialData();
}