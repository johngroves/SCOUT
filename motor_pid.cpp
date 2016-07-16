
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <PID_v1.h>

#define TCAADDR 0x70

// When false, relays are not activated
bool annoyRachel = true;

float diff;

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



//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint ,.1,1,1, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;
void setup()
{
  Wire.begin();
  pinMode(POS1, OUTPUT);
  pinMode(NEG1, OUTPUT);
  pinMode(POS2, OUTPUT);
  pinMode(NEG2, OUTPUT);


  // With this relay, HIGH is OFF
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

    //tell the PID to range between 0 and the full window size
    myPID.SetOutputLimits(0, WindowSize);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    Serial.print("Motor Angle: ");
    Serial.println(getMotorAngle());
    Serial.print("Boat Angle: ");
    Serial.println(getMagneticHeading());
    Serial.print("Difference: ");
    Serial.println(degsBetween(getMagneticHeading(),getMotorAngle()));
    Serial.print("Direction to turn: ");
    Serial.println(directionToTurn(getMagneticHeading(),getMotorAngle()));


}

void loop()
{
    Input = degsBetween(getMagneticHeading(),getMotorAngle()); // Get Motor Angle
    Serial.print("Input: ");
    Serial.println(Input);
    myPID.Compute();
    Serial.print("Heading - Motor Angle ");
    Serial.println(getMagneticHeading() - getMotorAngle());
    Serial.print("Direction to turn: ");
    bool clockwise = directionToTurn(getMagneticHeading(),getMotorAngle());
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
