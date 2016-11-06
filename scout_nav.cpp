#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

#define GPSECHO  false
#define TCAADDR 0x70

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag1 = Adafruit_HMC5883_Unified(1);
Adafruit_HMC5883_Unified mag2 = Adafruit_HMC5883_Unified(2);

Adafruit_GPS GPS(&Serial3);
HardwareSerial mySerial = Serial3;

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

int POS1 = 46;
int NEG1 = 45;
int POS2 = 44;
int NEG2 = 47;
bool annoyRachel = true;

int POW1 = 42;
int POW2 = 43;
int delayValue = 500;
int turnDelay = 500;
bool closenuf;
bool clockwise;

float lat2 = 37.528618;
float lon2 = -122.257563;
float lat1 = GPS.lat;
float lon1 = GPS.lon;

float getheading(float lat1, float lon1, float lat2, float lon2){ // Calculates heading from location to waypoint

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

  float degsToTurn(float magneticBearing, float magneticHeading){ // Calculates bearing from location to waypoint
    float rudadj = 0.1;
    float degsToTurn = 0.0;
    float relativeHeading = magneticBearing - magneticHeading;

    if(relativeHeading <0){
        relativeHeading+=360;
    }

    if(relativeHeading > 180){
      degsToTurn = rudadj * (0.5 * (360.0 - relativeHeading));
    } else {
      degsToTurn = rudadj * (0.5 * (relativeHeading));
    }

    return degsToTurn;

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

float directionToTurn(float delta){
  bool clockwise = true;

  if(delta <-180){
    clockwise = true;
  }
  else if(180 < delta <= 0){
    clockwise = true;
  }
  else if(-180 <= delta < 0){
    clockwise = false;
  }
  else if(delta > 180){
    clockwise = false;
  }
  return clockwise;
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

 void printStats(){
  float magneticHeading = getMagneticHeading();
  delay(200);
  float motorAngle = getMotorAngle();
  delay(200);
  float magneticBearing = getheading(lat1,lon1,lat2,lon2);
  float headingCorrection = degsToTurn(magneticBearing,magneticHeading);
  float turningDirection = directionToTurn(getMagneticHeading());

  Serial.print("Boat Heading: ");
  Serial.println(magneticHeading);
  Serial.print("Motor Angle: ");
  Serial.println(motorAngle);
  Serial.print("Desired Heading: ");
  Serial.println(magneticBearing);
  Serial.print("Degrees To Turn: ");
  Serial.println(headingCorrection);
 }

void turnClockwise(){
  if(annoyRachel){
    digitalWrite(POS1, LOW);
    digitalWrite(NEG1, HIGH);
    digitalWrite(POS2, HIGH);
    digitalWrite(NEG2, LOW);
    delay(turnDelay);
    digitalWrite(POS1, HIGH);
    digitalWrite(NEG1, HIGH);
    digitalWrite(POS2, HIGH);
    digitalWrite(NEG2, HIGH);
  }
  else{

    delay(turnDelay);
  }

}

void turnCClockwise(){
  if(annoyRachel){
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, LOW);
  digitalWrite(POS2, LOW);
  digitalWrite(NEG2, HIGH);
  delay(turnDelay);

  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, HIGH);
  }
  else{

    delay(turnDelay);
  }

}

void setup() {
  // put your setup code here, to run once:
  // Assign names to analog 0-3 for relay control
  // Initialize analog pins
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  Wire.begin();
  useInterrupt(true);
  pinMode(POS1, OUTPUT);
  pinMode(NEG1, OUTPUT);
  pinMode(POS2, OUTPUT);
  pinMode(NEG2, OUTPUT);
  pinMode(POW1, OUTPUT);
  pinMode(POW2, OUTPUT);

  // With this relay, HIGH is OFF
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, HIGH);
  digitalWrite(POW1, HIGH);
  digitalWrite(POW2, HIGH);

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

}
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
#endif
}

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
uint32_t timer = millis();


void loop() {
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000){

  bool closeEnough = true;
  float oldDiff  = 1;
  int min_accuracy = 5;
  float newDiff = 0;
  float magneticHeading = getMagneticHeading(); // Compass direction of boat
  delay(200);
  float motorAngle = getMotorAngle(); // Angle of boat motor
  delay(200);
  float magneticBearing = getheading(GPS.lat,GPS.lon,lat2,lon2); // Direction from current GPS to desired location
  float headingCorrection = degsToTurn(magneticBearing,magneticHeading); // Amount to turn the motor
  float turningDirection = directionToTurn(getMagneticHeading()); // Direction to turn the motor
  printStats();
  closeEnough = degsBetween((magneticHeading + headingCorrection),motorAngle) <= min_accuracy;
  Serial.print("Close Enough: ");
  Serial.println((magneticHeading + headingCorrection)-motorAngle);
  Serial.print("Direction to Turn: ");
  Serial.println(directionToTurn(getMagneticHeading()));
  if (GPS.fix) {
  while(!closeEnough){
    if(directionToTurn(getMagneticHeading())){
      while(oldDiff > newDiff){
        oldDiff = degsBetween((magneticHeading + headingCorrection),motorAngle);
        motorAngle = getMotorAngle();
        Serial.print("Turning Counter Clockwise");
        Serial.print("Motor Angle: ");
        Serial.println(motorAngle);
        Serial.print("Difference: ");
        Serial.println((magneticHeading + headingCorrection)-motorAngle);
        Serial.print("");
        Serial.print("Close E: ");
        Serial.println(closeEnough);
        turnCClockwise();
        newDiff = degsBetween((magneticHeading + headingCorrection),motorAngle);
        closeEnough = degsBetween((magneticHeading + headingCorrection),motorAngle) <= min_accuracy;
        Serial.print("Old Difference: ");
        Serial.println(oldDiff);
        Serial.print("New Difference: ");
        Serial.println(newDiff);
      }
      break;
    }
    if(!directionToTurn(getMagneticHeading())){
      while(oldDiff > newDiff){
        oldDiff = degsBetween((magneticHeading + headingCorrection),motorAngle);
        motorAngle = getMotorAngle();
        Serial.print("Turning Clockwise");
        Serial.print("Motor Angle: ");
        Serial.println(motorAngle);
        Serial.print("Difference: ");
        Serial.println((magneticHeading + headingCorrection)-motorAngle);
        Serial.print("Close Enough: ");
        Serial.println(closeEnough);
        turnClockwise();
        newDiff = degsBetween((magneticHeading + headingCorrection),motorAngle);
        closeEnough = degsBetween((magneticHeading + headingCorrection),motorAngle) <= min_accuracy;
        Serial.print("Old Difference: ");
        Serial.println(oldDiff);
        Serial.print("New Difference: ");
        Serial.println(newDiff);
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.longitude,4);

      }
      break;

    }
    closeEnough = degsBetween((magneticHeading + headingCorrection),motorAngle) <= min_accuracy;
  }


}
else{
  Serial.print("Location: ");
      timer = millis(); // reset the timer
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 4);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
}
}
}


