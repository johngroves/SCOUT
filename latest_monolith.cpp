
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_GPS.h>
#include <PID_v1.h>

#define TCAADDR 0x70
#define GPSECHO  false


// Assign GPS to Serial 3
Adafruit_GPS GPS(&Serial3);
HardwareSerial mySerial = Serial3;

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// When false, relays are not activated
bool annoyRachel = true;

float diff;
// Lat long values
float lat2 = 37.528618;
float lon2 = -122.257563;
float lat1 = GPS.lat;
float lon1 = GPS.lon;


// Relay Pins
int POS1 = 46;
int NEG1 = 45;
int POS2 = 44;
int NEG2 = 47;

Adafruit_HMC5883_Unified mag1 = Adafruit_HMC5883_Unified(1);
Adafruit_HMC5883_Unified mag2 = Adafruit_HMC5883_Unified(2);

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
float getMagneticHeading(){
    sensors_event_t event;
    float declinationAngle = 0.22;
    tcaselect(7);
    mag2.getEvent(&event);
    float angle = atan2(event.magnetic.y, event.magnetic.x);
    angle += declinationAngle;
    if(angle < 0)
        angle += 2*PI;
    if(angle > 2*PI)
        angle -= 2*PI;
    float degs = angle * 180/M_PI;
    return degs;
}

float getMotorAngle(){
    sensors_event_t event;
    float declinationAngle = 0.22;
    tcaselect(6);
    mag2.getEvent(&event);
    float angle = atan2(event.magnetic.y, event.magnetic.x);
    angle += declinationAngle;
    if(angle < 0)
        angle += 2*PI;
    if(angle > 2*PI)
        angle -= 2*PI;
    float degs = angle * 180/M_PI;
    return degs;
}

float degsBetween(float angleOne, float angleTwo){ // ignore variable names. Returns distance in degrees between two angles.
    float headingDelta = angleOne - angleTwo;

    if(headingDelta <0){
        headingDelta+=360;
    }

    if(headingDelta > 180){
        headingDelta = (360.0 - headingDelta);
    } else {
        headingDelta = (headingDelta);
    }

    return headingDelta;

}

bool directionToTurn(float heading, float motor){

   float headingRadians = (heading * 3.1415926535) / 180.0;
   float motorRadians = (motor * 3.1415926535) / 180.0;
   float holder = sin((headingRadians - motorRadians));

   if (holder > 0){
    return true;
   }
   else {
    return false;
   }

}

float getBearing(float lat1, float lon1, float lat2, float lon2){ // Calculates bearing from location 1 to waypoint 1

  float heading;
  lon1 = radians(lon1);
  lon2 = radians(lon2);

  heading = atan2(sin(lon2-lon1)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1)),2*3.1415926535;
  heading = heading*180/3.1415926535;  // convert from radians to degrees

  int head =heading;

    if(head<0){
      heading+=360;   //if the heading is negative then add 360 to make it positive
    }

    return heading;

}


//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpoint2, Input2, Output2;

//Specify the links and initial tuning parameters
PID motorPID(&Input, &Output, &Setpoint ,.1,1,1, DIRECT);
PID navPID(&Input2, &Output2, &Setpoint2,.1,1,1, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;
void setup()
{
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  Wire.begin();

  // Initialize Relays Controlling Motor Angle
  pinMode(POS1, OUTPUT);
  pinMode(NEG1, OUTPUT);
  pinMode(POS2, OUTPUT);
  pinMode(NEG2, OUTPUT);


  // High is OFF on this relay
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, HIGH);



  tcaselect(7);
  if(!mag1.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  tcaselect(6);
  if(!mag2.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
    windowStartTime = millis();
    Serial.begin(115200);
    //initialize the variables we're linked to
    Setpoint = 0.0;
    Setpoint2 = 0.0;

    //tell the PID to range between 0 and the full window size
    motorPID.SetOutputLimits(0, WindowSize);
    navPID.SetOutputLimits(0, 45);
    //turn the PID on
    motorPID.SetMode(AUTOMATIC);

    Serial.print("Motor Angle: ");
    Serial.println(getMotorAngle());
    Serial.print("Boat Angle: ");
    Serial.println(getMagneticHeading());
    Serial.print("Difference: ");
    Serial.println(degsBetween(getMagneticHeading(),getMotorAngle()));
    Serial.print("Direction to turn: ");
    Serial.println(directionToTurn(getMagneticHeading(),getMotorAngle()));


}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
    // Writing direct to UDR0 is much much faster than Serial.print
#endif
}
uint32_t timer = millis();


void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }

}

void loop()
{
    if (! usingInterrupt) {
        char c = GPS.read(); // read data from the GPS in the 'main loop'
        if (GPSECHO) // Debug using GPSECHO
          if (c) Serial.print(c);
    }
    // If NMEA sentence received, parse it
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
          return;  // If fails to parse a NMEA sentence, wait for another
    }

    if (GPS.fix) {
        /* Main Loop w/ Fix */
        Input2 = degsBetween(getMagneticHeading(),getBearing(GPS.lat,GPS.lon,lat2,lon2));
        Input = degsBetween(getMagneticHeading(),getMotorAngle()); // Get Motor Angle

        /*************************************************
        * TODO: Update Setpoint w/ magnetic bearing      *
        **************************************************/
        navPID.Compute();
        Setpoint = Output2;
        motorPID.Compute();
        Serial.print("Heading - Motor Angle ");
        Serial.println(getMagneticHeading() - getMotorAngle());
        Serial.print("Direction to turn: ");
        bool clockwise = directionToTurn(getMagneticHeading()+Setpoint,getMotorAngle());
        Serial.println(clockwise);

        /************************************************
         * turn the output pin on/off based on pid output
         ************************************************/
        unsigned long now = millis();
        if(now - windowStartTime>WindowSize)
        { //time to shift the Relay Window
            windowStartTime += WindowSize;
        }
        if(Output > now - windowStartTime){

          if(clockwise){
            digitalWrite(POS1, LOW);
            digitalWrite(NEG1, HIGH);
            digitalWrite(POS2, HIGH);
            digitalWrite(NEG2, LOW);

          }
          else{
            digitalWrite(POS1, HIGH);
            digitalWrite(NEG1, LOW);
            digitalWrite(POS2, LOW);
            digitalWrite(NEG2, HIGH);
          }

        }
        else {
            digitalWrite(POS1, HIGH);
            digitalWrite(NEG1, HIGH);
            digitalWrite(POS2, HIGH);
            digitalWrite(NEG2, HIGH);
        }
        Serial.print("Output: ");
        Serial.println(Output);
    }
    else{
        Serial.print("No GPS Connection.");
    }

}

