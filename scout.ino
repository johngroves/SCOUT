// SCOUT Navigation and Logic

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Pass data to USB serial via digital 3,2
SoftwareSerial mySerial(3, 2);

// Assign names to analog 0-3 for relay
int POS1 = A0;
int NEG1 = A1;
int POS2 = A2;
int NEG2 = A3;
int delayValue = 5000;

Adafruit_GPS GPS(&mySerial);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()  
{
    
  // connect at 115200 baud to read GPS quickly
  // also spit it out
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

  Serial.println("Turning Clockwise");
  digitalWrite(POS1, LOW);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, LOW);

  delay(delayValue);

  Serial.println("Turning OFF");
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, HIGH);

  delay(delayValue);

  Serial.println("Turning Counter Clockwise");
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, LOW);
  digitalWrite(POS2, LOW);
  digitalWrite(NEG2, HIGH);

  delay(delayValue);

  Serial.println("Turning OFF");
  digitalWrite(POS1, HIGH);
  digitalWrite(NEG1, HIGH);
  digitalWrite(POS2, HIGH);
  digitalWrite(NEG2, HIGH);

  delay(delayValue);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // 1 ms interrupt
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  //mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 
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
void loop()                     
{
  
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
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
    }
  }
}
