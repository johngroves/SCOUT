// SCOUT Navigation and Logic

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
 
// Assign a unique ID to compass / accelerometer
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Pass data to USB serial via digital 3,2
SoftwareSerial mySerial(3, 2);

// Assign names to analog 0-3 for relay control
int POS1 = A0;
int NEG1 = A1;
int POS2 = A2;
int NEG2 = A3;
int delayValue = 5000;

Adafruit_GPS GPS(&mySerial);

// Listen to the raw GPS sentences. 
#define GPSECHO  true

// Using interrupt between statements
boolean usingInterrupt = false;

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()  
{

  // Initialise compass
  if(!mag.begin())
  {
    // Verify compass / accel configured 
    Serial.println("No Accelerometer Detected");
    while(1);
  }
  
  // connect at 115200 baud to read GPS quickly
  Serial.begin(115200);
  Serial.println("GPS Initializing");

  // Initialize analog pins
  pinMode(POS1, OUTPUT);
  pinMode(NEG1, OUTPUT);
  pinMode(POS2, OUTPUT);
  pinMode(NEG2, OUTPUT);

  // With this relay, HIGH is OFF
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, HIGH);

  Serial.println("Initialized With High.");

  // Turn Clockwise
  Serial.println("Turning Clockwise");
  digitalWrite(POS1, LOW);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, LOW);

  delay(delayValue);

  // Turn Off Power to Steering Motor
  Serial.println("Turning OFF");
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, HIGH);

  delay(delayValue);

  // Turn Counter Clockwise
  Serial.println("Turning Counter Clockwise");
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, LOW);
  digitalWrite(POS2, LOW);
  digitalWrite(NEG2, HIGH);

  delay(delayValue);

  // Turn Off Power to Steering Motor
  Serial.println("Turning OFF");
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, HIGH);

  delay(delayValue);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  GPS.begin(9600);
  
  // Read RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  
  // Request updates on antenna status
  GPS.sendCommand(PGCMD_ANTENNA);

  // 1 ms interrupt
  useInterrupt(true);

  delay(1000);
  
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();

#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 
#endif
}

void useInterrupt(boolean v) {
  if (v) {  
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
void loop()                     
{
  sensors_event_t event; 
  mag.getEvent(&event);
  float Pi = 3.14159;
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }

  if (! usingInterrupt) {
    // read data from the GPS 
    char c = GPS.read();
      
  }
  
  // Determine if new statement received, if so, check the checksum, parse
  if (GPS.newNMEAreceived()) {
    
    if (!GPS.parse(GPS.lastNMEA()))   // this sets the newNMEAreceived() flag to false
      return;  // if miss statement, wait for
  }

  // if millis() or timer wraps around, reset it
  if (timer > millis())  timer = millis();

  //  Every 2 seconds, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Compass Heading: ");
      Serial.println(heading);
  
    }
  }

}
